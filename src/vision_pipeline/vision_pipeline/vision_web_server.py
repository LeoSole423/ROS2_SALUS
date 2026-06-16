#!/usr/bin/env python3

from __future__ import annotations  # compatibilidad con anotaciones en Python 3.8

# ── Librerías estándar ────────────────────────────────────────────────────────
import json       # para serializar detecciones y estado como JSON en /data y /health
import threading  # para el servidor HTTP en hilo separado y el lock de estado compartido
import time       # para calcular la antigüedad de frames y detecciones
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # servidor HTTP multi-hilo
from pathlib import Path      # para leer el archivo HTML del dashboard
from typing import Optional, Sequence

# ── OpenCV ────────────────────────────────────────────────────────────────────
import cv2   # para codificar frames como JPEG y dibujar los bboxes sobre la imagen

# ── ROS 2 ────────────────────────────────────────────────────────────────────
import rclpy
from ament_index_python.packages import get_package_share_directory  # para encontrar el HTML del paquete
from cv_bridge import CvBridge                  # convierte sensor_msgs/Image ↔ array OpenCV
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,  # QoS predefinido BEST_EFFORT para sensores
)
from sensor_msgs.msg import Image            # imágenes de cámara
from std_msgs.msg import String              # texto de debug de YOLO
from vision_msgs.msg import Detection2DArray # array de detecciones YOLO


# ── Función auxiliar: convierte timestamp ROS a float ─────────────────────────
def _stamp_to_float(stamp) -> Optional[float]:
    """Convierte el timestamp ROS (sec + nanosec) a segundos float Unix.

    Devuelve None si el stamp es None (estado inicial antes de recibir mensajes).
    """
    if stamp is None:
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9  # nanosec → segundos


# ══════════════════════════════════════════════════════════════════════════════
#  NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
class VisionWebServerNode(Node):
    """Nodo ROS 2 que expone el stream de video y detecciones YOLO via HTTP.

    Se suscribe a:
      - /camera/image_raw    → imágenes de la cámara
      - /objeto_detectado    → texto de debug de YOLO
      - /detections          → array de detecciones para dibujar bboxes

    Expone endpoints HTTP en puerto 8088:
      GET /            → página HTML del dashboard
      GET /stream.mjpg → stream MJPEG en vivo con bboxes
      GET /snap.jpg    → snapshot del frame actual
      GET /data        → JSON con estado completo (cámara + IA)
      GET /health      → JSON simple de health check
    """

    def __init__(self) -> None:
        super().__init__('vision_web_server')

        # ── Declaración de parámetros ─────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')        # fuente de video
        self.declare_parameter('debug_topic', '/objeto_detectado')         # texto debug de YOLO
        self.declare_parameter('detections_topic', '/detections')          # detecciones para overlay
        self.declare_parameter('http_host', '0.0.0.0')                    # escucha en todas las interfaces
        self.declare_parameter('http_port', 8088)                          # puerto del servidor web
        self.declare_parameter('html_path', '')                            # ruta al dashboard HTML (vacío = automático)
        self.declare_parameter('jpeg_quality', 90)                         # calidad JPEG del stream [40-95]
        self.declare_parameter('overlay_enabled', True)                    # dibujar bboxes sobre el video
        self.declare_parameter('detections_overlay_timeout_s', 1.0)        # segundos antes de ocultar overlay si no hay detecciones

        # ── Lectura de parámetros ─────────────────────────────────────────────
        image_topic = str(self.get_parameter('image_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        http_host = str(self.get_parameter('http_host').value)
        http_port = int(self.get_parameter('http_port').value)
        html_path = str(self.get_parameter('html_path').value)
        # Clamp entre 40 y 95: por debajo de 40 la imagen es inutilizable
        self._jpeg_quality = min(95, max(40, int(self.get_parameter('jpeg_quality').value)))
        self._overlay_enabled = bool(self.get_parameter('overlay_enabled').value)
        self._detections_overlay_timeout_s = max(
            0.1, float(self.get_parameter('detections_overlay_timeout_s').value)
        )

        # ── Carga del HTML del dashboard ──────────────────────────────────────
        if not html_path:
            # Si no se especificó ruta, usa el HTML instalado con el paquete ROS
            share_dir = Path(get_package_share_directory('vision_pipeline'))
            html_path = str(share_dir / 'web' / 'index.html')

        self._html_content = self._load_html(html_path)  # lee el archivo HTML a memoria
        self._bridge = CvBridge()                         # bridge OpenCV ↔ ROS

        # ── Estado compartido entre hilos ROS y HTTP ──────────────────────────
        # Un Condition combina un Lock y un Event: permite notificar a múltiples waiters
        self._state_lock = threading.Lock()
        self._frame_condition = threading.Condition(self._state_lock)

        # Frame actual como bytes JPEG (listo para enviar por HTTP sin recodificar)
        self._latest_jpeg: Optional[bytes] = None
        self._latest_frame_stamp: Optional[float] = None       # timestamp ROS del frame
        self._latest_frame_wall_time: Optional[float] = None   # tiempo real de recepción
        self._latest_frame_shape = {'width': 0, 'height': 0}   # dimensiones del frame
        self._latest_frame_seq = 0                             # contador de frames (para detectar frames nuevos)

        # Último texto de debug de YOLO
        self._latest_ai_text = 'Esperando inferencia...'
        self._latest_ai_wall_time: Optional[float] = None

        # Últimas detecciones YOLO en formato dict (para /data y overlay)
        self._latest_detections_wall_time: Optional[float] = None
        self._latest_detections: list[dict] = []

        # ── QoS para debug: BEST_EFFORT (puede perder mensajes, no crítico) ───
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscripciones a tópicos ROS ─────────────────────────────────────
        self.create_subscription(Image, image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(String, debug_topic, self._debug_cb, debug_qos)
        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)

        # ── Inicia el servidor HTTP en un hilo daemon ─────────────────────────
        self._httpd = self._start_http_server(http_host, http_port)
        self.get_logger().info(
            f'vision_web_server running at http://{http_host}:{http_port} '
            f'(image_topic={image_topic}, debug_topic={debug_topic}, detections_topic={detections_topic})'
        )

    # ── Carga del archivo HTML ────────────────────────────────────────────────
    def _load_html(self, html_path: str) -> str:
        """Lee el archivo HTML del dashboard. Devuelve página de error si falla."""
        try:
            return Path(html_path).read_text(encoding='utf-8')
        except Exception as exc:
            self.get_logger().error(f'failed to load HTML {html_path}: {exc}')
            return '<html><body>Missing dashboard HTML.</body></html>'

    # ── Callback: procesa cada frame de la cámara ─────────────────────────────
    def _image_cb(self, msg: Image) -> None:
        """Recibe un frame de la cámara, le dibuja el overlay y lo codifica como JPEG.

        El JPEG se guarda en memoria para enviarlo a todos los clientes del stream
        sin necesidad de recodificar por cada cliente conectado.
        """
        try:
            # Convierte mensaje ROS Image → array OpenCV BGR
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cannot decode image frame: {exc}')
            return

        # ── Dibuja overlay de detecciones si está habilitado ──────────────────
        if self._overlay_enabled:
            overlay_text = self._latest_ai_text  # texto de debug de YOLO
            preview = frame.copy()               # copia para no modificar el original
            self._draw_overlay(preview, overlay_text)  # dibuja bboxes + texto
        else:
            preview = frame  # sin overlay: usa el frame tal cual

        # ── Codifica a JPEG para transmisión HTTP ─────────────────────────────
        ok, encoded = cv2.imencode(
            '.jpg',
            preview,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)],  # calidad configurable
        )
        if not ok:
            self.get_logger().warning('cannot encode JPEG preview')
            return

        # ── Actualiza el estado compartido y notifica a los clientes MJPEG ───
        now = time.time()
        with self._frame_condition:  # adquiere lock + permite notify
            self._latest_jpeg = encoded.tobytes()                          # bytes JPEG listos para enviar
            self._latest_frame_stamp = _stamp_to_float(msg.header.stamp)   # timestamp ROS
            self._latest_frame_wall_time = now                             # tiempo real
            self._latest_frame_shape = {
                'width': int(preview.shape[1]),
                'height': int(preview.shape[0]),
            }
            self._latest_frame_seq += 1   # incrementa secuencia: notifica que es un frame nuevo
            self._frame_condition.notify_all()  # despierta a todos los clientes MJPEG esperando

    # ── Callback: recibe texto de debug de YOLO ──────────────────────────────
    def _debug_cb(self, msg: String) -> None:
        """Actualiza el texto de overlay con el resumen textual de YOLO."""
        with self._state_lock:
            # Ejemplo: "top=person score=0.92 total=3 [person:0.92, car:0.85]"
            self._latest_ai_text = str(msg.data).strip() or 'sin_detecciones'
            self._latest_ai_wall_time = time.time()

    # ── Callback: recibe array de detecciones YOLO ────────────────────────────
    def _detections_cb(self, msg: Detection2DArray) -> None:
        """Convierte las detecciones ROS a lista de dicts para el overlay y el endpoint /data."""
        detections: list[dict] = []
        for detection in msg.detections:
            top_label = ''
            top_score = 0.0
            if detection.results:
                top_result = detection.results[0]            # hipótesis con mayor probabilidad
                top_label = str(top_result.hypothesis.class_id)  # nombre de la clase
                top_score = float(top_result.hypothesis.score)   # confianza

            # Serializa la detección como dict (fácil de convertir a JSON)
            detections.append(
                {
                    'id': str(detection.id),       # identificador único ("person_0")
                    'label': top_label,             # clase ("person", "car", etc.)
                    'score': top_score,             # confianza [0,1]
                    'bbox': {
                        'cx': float(detection.bbox.center.position.x),  # centro X en píxeles
                        'cy': float(detection.bbox.center.position.y),  # centro Y en píxeles
                        'width': float(detection.bbox.size_x),           # ancho en píxeles
                        'height': float(detection.bbox.size_y),          # alto en píxeles
                    },
                }
            )

        with self._state_lock:
            self._latest_detections = detections            # actualiza lista de detecciones
            self._latest_detections_wall_time = time.time() # registra cuándo llegaron

    # ── Dibuja overlay sobre el frame ─────────────────────────────────────────
    def _draw_overlay(self, frame, overlay_text: str) -> None:
        """Dibuja bboxes y texto de IA sobre el frame. Modifica el frame in-place."""
        now = time.time()
        with self._state_lock:
            detections = list(self._latest_detections)  # copia para no bloquear con el lock
            detections_age = (
                None
                if self._latest_detections_wall_time is None
                else now - self._latest_detections_wall_time
            )

        # Solo dibuja los bboxes si las detecciones son recientes (no expiró el timeout)
        if detections_age is not None and detections_age <= self._detections_overlay_timeout_s:
            self._draw_detection_boxes(frame, detections)

        # ── Dibuja el banner de texto de IA en la parte superior ──────────────
        text = f'IA: {overlay_text or "sin_datos"}'
        margin = 14    # margen desde el borde de la imagen
        box_height = 42  # alto del banner en píxeles
        # Fondo oscuro para el texto (más legible sobre cualquier fondo)
        cv2.rectangle(
            frame,
            (margin, margin),                                    # esquina superior-izquierda
            (frame.shape[1] - margin, margin + box_height),      # esquina inferior-derecha
            (18, 30, 44),                                        # color fondo: azul muy oscuro
            thickness=-1,                                        # -1 = relleno sólido
        )
        # Texto blanco sobre el fondo oscuro
        cv2.putText(
            frame,
            text[:96],                       # limita a 96 chars para que quepa en pantalla
            (margin + 12, margin + 27),      # posición del texto
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,                             # tamaño de fuente
            (242, 247, 251),                 # color texto: blanco ligeramente frío
            2,                               # grosor del trazo
            cv2.LINE_AA,                     # antialiasing para texto más suave
        )

    # ── Dibuja los bboxes de cada detección ──────────────────────────────────
    def _draw_detection_boxes(self, frame, detections: list[dict]) -> None:
        """Dibuja un rectángulo y etiqueta para cada objeto detectado."""
        frame_h, frame_w = frame.shape[:2]
        box_color = (34, 197, 94)    # verde (BGR): color del rectángulo del bbox
        label_bg = (20, 83, 45)      # verde oscuro: fondo de la etiqueta
        label_fg = (236, 253, 245)   # verde muy claro: texto de la etiqueta

        for detection in detections:
            bbox = detection.get('bbox', {})
            try:
                # Extrae las coordenadas del centro y tamaño del bbox
                cx = float(bbox.get('cx', 0.0))
                cy = float(bbox.get('cy', 0.0))
                width = float(bbox.get('width', 0.0))
                height = float(bbox.get('height', 0.0))
            except (TypeError, ValueError):
                continue  # coordenadas inválidas, salta esta detección

            if width <= 0 or height <= 0:
                continue  # bbox degenerado, nada que dibujar

            # Convierte de (cx, cy, w, h) a coordenadas de esquina (x1, y1, x2, y2)
            x1 = int(round(cx - width / 2.0))
            y1 = int(round(cy - height / 2.0))
            x2 = int(round(cx + width / 2.0))
            y2 = int(round(cy + height / 2.0))

            # Clamp: asegura que el bbox no salga de los bordes de la imagen
            x1 = max(0, min(frame_w - 1, x1))
            y1 = max(0, min(frame_h - 1, y1))
            x2 = max(0, min(frame_w - 1, x2))
            y2 = max(0, min(frame_h - 1, y2))

            if x2 <= x1 or y2 <= y1:
                continue  # bbox degenerado después del clamp

            # Prepara el texto de la etiqueta: "person 92%"
            label = str(detection.get('label') or 'objeto')
            try:
                score = float(detection.get('score', 0.0))
            except (TypeError, ValueError):
                score = 0.0
            text = f'{label} {score * 100:.0f}%'

            # ── Dibuja el rectángulo del bbox ─────────────────────────────────
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, thickness=3)

            # ── Dibuja la etiqueta encima del bbox ───────────────────────────
            # Calcula el tamaño del texto para posicionar el fondo
            (text_w, text_h), baseline = cv2.getTextSize(
                text,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,  # tamaño de fuente
                2,     # grosor
            )
            # Posiciona la etiqueta justo encima del bbox
            label_y1 = max(0, y1 - text_h - baseline - 8)
            label_y2 = label_y1 + text_h + baseline + 8
            label_x2 = min(frame_w - 1, x1 + text_w + 12)

            # Fondo de la etiqueta
            cv2.rectangle(frame, (x1, label_y1), (label_x2, label_y2), label_bg, thickness=-1)

            # Texto de la etiqueta
            cv2.putText(
                frame,
                text,
                (x1 + 6, label_y2 - baseline - 4),  # posición dentro del fondo
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                label_fg,
                2,
                cv2.LINE_AA,
            )

    # ── Servidor HTTP ─────────────────────────────────────────────────────────
    def _start_http_server(self, host: str, port: int) -> ThreadingHTTPServer:
        """Crea y arranca el servidor HTTP en un hilo daemon.

        ThreadingHTTPServer maneja cada petición en un hilo separado,
        permitiendo múltiples clientes simultáneos del stream MJPEG.
        """
        node = self  # captura referencia al nodo para uso en el closure

        class Handler(BaseHTTPRequestHandler):
            """Manejador HTTP que enruta las peticiones GET a los métodos del nodo."""

            def do_GET(self):  # noqa: N802
                """Enruta peticiones GET según la ruta de la URL."""
                clean_path = self.path.split('?', 1)[0]  # elimina query string (?foo=bar)

                if clean_path in ('/', '/index.html'):
                    # Sirve la página HTML del dashboard
                    self._send_response(200, 'text/html; charset=utf-8', node._html_content)
                    return
                if clean_path in ('/snapshot.jpg', '/snap.jpg'):
                    # Sirve el último frame como imagen JPEG estática
                    node._send_snapshot(self)
                    return
                if clean_path == '/stream.mjpg':
                    # Inicia el stream MJPEG en vivo (bloquea hasta que el cliente desconecte)
                    node._send_mjpeg_stream(self)
                    return
                if clean_path == '/data':
                    # Sirve el estado actual como JSON (cámara + IA + detecciones)
                    self._send_response(200, 'application/json', node._get_snapshot())
                    return
                if clean_path == '/health':
                    # Health check: simple JSON con ok=true/false
                    self._send_response(200, 'application/json', node._get_health())
                    return
                if clean_path == '/favicon.ico':
                    # Responde vacío al favicon para evitar logs de error
                    self.send_response(204)
                    self.end_headers()
                    return
                # Cualquier otra ruta: 404
                self._send_response(404, 'text/plain; charset=utf-8', 'Not Found')

            def do_OPTIONS(self):  # noqa: N802
                """Maneja preflight CORS: permite peticiones desde el cockpit en otro puerto."""
                self.send_response(204)
                self.send_header('Access-Control-Allow-Origin', '*')   # permite cualquier origen
                self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()

            def _send_response(self, code: int, content_type: str, body) -> None:
                """Helper para enviar respuesta HTTP completa con headers correctos."""
                if isinstance(body, str):
                    body = body.encode('utf-8')  # convierte string a bytes
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')           # no cachear: siempre datos frescos
                self.send_header('Access-Control-Allow-Origin', '*')    # CORS: accesible desde el cockpit
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):  # noqa: A003
                """Silencia los logs de acceso HTTP (demasiado verbosos para producción)."""
                return

        # Crea el servidor y lo arranca en un hilo daemon
        httpd = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)  # daemon: muere con el proceso
        thread.start()
        return httpd

    # ── Endpoint /snap.jpg: snapshot estático ────────────────────────────────
    def _send_snapshot(self, handler: BaseHTTPRequestHandler) -> None:
        """Envía el último frame JPEG disponible como imagen estática."""
        with self._state_lock:
            payload = self._latest_jpeg  # toma el JPEG actual

        if payload is None:
            # Aún no hay frame disponible
            body = b'No image available yet.'
            handler.send_response(503)  # Service Unavailable
            handler.send_header('Content-Type', 'text/plain; charset=utf-8')
            handler.send_header('Content-Length', str(len(body)))
            handler.send_header('Cache-Control', 'no-store')
            handler.send_header('Access-Control-Allow-Origin', '*')
            handler.end_headers()
            handler.wfile.write(body)
            return

        # Envía el JPEG como imagen
        handler.send_response(200)
        handler.send_header('Content-Type', 'image/jpeg')
        handler.send_header('Content-Length', str(len(payload)))
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('Access-Control-Allow-Origin', '*')
        handler.end_headers()
        handler.wfile.write(payload)

    # ── Endpoint /stream.mjpg: stream MJPEG en vivo ───────────────────────────
    def _send_mjpeg_stream(self, handler: BaseHTTPRequestHandler) -> None:
        """Transmite video en vivo usando el protocolo MJPEG (multipart/x-mixed-replace).

        MJPEG: el servidor envía frames JPEG consecutivos separados por un boundary.
        El cliente (browser/cockpit) los muestra uno tras otro → video en vivo.
        Este método bloquea hasta que el cliente desconecta (BrokenPipeError).
        """
        boundary = b'frame'  # string que separa frames en el stream multipart

        # Cabecera HTTP inicial del stream MJPEG
        handler.send_response(200)
        handler.send_header('Age', '0')
        handler.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, private')
        handler.send_header('Pragma', 'no-cache')
        handler.send_header('Access-Control-Allow-Origin', '*')
        # Content-Type especial: indica que es un stream de partes separadas por 'frame'
        handler.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        handler.end_headers()

        last_seq = -1  # último número de secuencia enviado (para no enviar el mismo frame dos veces)
        try:
            while rclpy.ok():  # continúa mientras el nodo ROS esté activo
                with self._frame_condition:
                    if self._latest_jpeg is None:
                        # Sin frames aún: espera hasta 100ms
                        self._frame_condition.wait(timeout=0.1)
                        continue
                    if self._latest_frame_seq == last_seq:
                        # No hay frame nuevo: espera hasta ~33ms (30 FPS máx de envío)
                        self._frame_condition.wait(timeout=0.033)
                        continue
                    payload = self._latest_jpeg          # toma el frame actual
                    last_seq = self._latest_frame_seq    # registra el número enviado

                # ── Envía el frame como parte MJPEG ──────────────────────────
                handler.wfile.write(b'--' + boundary + b'\r\n')          # boundary de inicio
                handler.wfile.write(b'Content-Type: image/jpeg\r\n')     # tipo de la parte
                handler.wfile.write(f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii'))  # tamaño
                handler.wfile.write(payload)                              # bytes del JPEG
                handler.wfile.write(b'\r\n')                             # fin de la parte
                handler.wfile.flush()                                    # envía inmediatamente (sin buffer)

        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # El cliente cerró la conexión: termina el stream limpiamente
            return

    # ── Endpoint /data: estado completo como JSON ─────────────────────────────
    def _get_snapshot(self) -> str:
        """Construye un JSON con el estado completo del sistema de visión.

        Incluye: estado de cámara, texto de IA, edad de datos, lista de detecciones.
        """
        now = time.time()
        with self._state_lock:
            payload = {
                'camera': {
                    'available': self._latest_jpeg is not None,   # ¿hay video disponible?
                    'stamp': self._latest_frame_stamp,             # timestamp ROS del frame
                    'age_sec': None if self._latest_frame_wall_time is None  # cuántos segundos hace
                               else now - self._latest_frame_wall_time,
                    'width': self._latest_frame_shape['width'],
                    'height': self._latest_frame_shape['height'],
                },
                'ai': {
                    'text': self._latest_ai_text,                  # "top=person score=0.92 ..."
                    'age_sec': None if self._latest_ai_wall_time is None
                               else now - self._latest_ai_wall_time,  # cuántos segundos hace
                    'detections_age_sec': None
                    if self._latest_detections_wall_time is None
                    else now - self._latest_detections_wall_time,
                    'count': len(self._latest_detections),          # número de objetos detectados
                    'detections': list(self._latest_detections),    # lista completa de detecciones
                },
                'server_time': now,  # timestamp del servidor para sincronización del cliente
            }
        return json.dumps(payload)  # serializa a JSON string

    # ── Endpoint /health: health check simple ─────────────────────────────────
    def _get_health(self) -> str:
        """Devuelve un JSON simple indicando si el servidor tiene video disponible."""
        now = time.time()
        with self._state_lock:
            ok = self._latest_jpeg is not None and self._latest_frame_wall_time is not None
            age_sec = None if self._latest_frame_wall_time is None else now - self._latest_frame_wall_time
        return json.dumps({'ok': ok, 'camera_age_sec': age_sec})

    # ── Destrucción limpia ────────────────────────────────────────────────────
    def destroy_node(self) -> bool:
        """Apaga el servidor HTTP antes de cerrar el nodo ROS 2."""
        try:
            self._httpd.shutdown()     # detiene el loop serve_forever
            self._httpd.server_close() # cierra el socket del servidor
        except Exception:
            pass  # si ya estaba cerrado, ignorar el error
        return super().destroy_node()


# ── Punto de entrada ──────────────────────────────────────────────────────────
def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = VisionWebServerNode()
    try:
        rclpy.spin(node)        # bloquea hasta Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
