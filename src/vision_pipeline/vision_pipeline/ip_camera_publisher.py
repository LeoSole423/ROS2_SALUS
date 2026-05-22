#!/usr/bin/env python3

from __future__ import annotations  # compatibilidad con anotaciones en Python 3.8

# ── Librerías estándar ────────────────────────────────────────────────────────
import os          # para leer/escribir variables de entorno (opciones FFMPEG)
import threading   # para separar el hilo de lectura de cámara del de publicación
import time        # para controlar FPS y timeouts de reconexión
from typing import Optional, Sequence
from urllib.request import (
    HTTPDigestAuthHandler,          # autenticación HTTP Digest (cámaras IP con usuario/contraseña)
    HTTPPasswordMgrWithDefaultRealm,  # gestor de credenciales para Digest Auth
    build_opener,                   # construye un opener HTTP personalizado con autenticación
)
from urllib.error import URLError  # excepción de error de red en urllib

# ── Librerías de visión ───────────────────────────────────────────────────────
import cv2    # OpenCV: decodifica video RTSP/MJPEG y redimensiona frames
import numpy as np  # para convertir bytes JPEG a array decodificable por OpenCV

# ── ROS 2 ────────────────────────────────────────────────────────────────────
import rclpy
from cv_bridge import CvBridge                    # convierte array OpenCV ↔ sensor_msgs/Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data    # QoS predefinido para sensores
from sensor_msgs.msg import CameraInfo, Image     # mensajes estándar de cámara


class IpCameraPublisherNode(Node):
    """Nodo ROS 2 que lee un stream de cámara (RTSP, MJPEG, USB) y publica imágenes.

    Arquitectura de dos hilos:
      - _reader_thread: lee frames de la cámara tan rápido como los entrega
      - _publisher_thread: publica en ROS 2 exactamente a target_fps

    Esto desacopla la velocidad de la cámara de la frecuencia de publicación ROS,
    evitando que latencia de red o decodificación bloquee el loop de ROS.
    """

    def __init__(self) -> None:
        super().__init__('ip_camera_publisher')  # nombre del nodo en el grafo ROS 2

        # ── Declaración de parámetros configurables ───────────────────────────
        self.declare_parameter('stream_url', '')             # URL del stream: rtsp://... o http://... (obligatorio)
        self.declare_parameter('image_topic', '/camera/image_raw')  # tópico donde publicar las imágenes
        self.declare_parameter('frame_id', 'camera_optical_frame')  # frame de referencia para TF
        self.declare_parameter('target_fps', 15.0)           # FPS de publicación en ROS (puede diferir de la cámara)
        self.declare_parameter('width', 640)                 # ancho deseado del frame publicado (0 = sin cambio)
        self.declare_parameter('height', 480)                # alto deseado
        self.declare_parameter('reconnect_interval_sec', 2.0)  # segundos entre intentos de reconexión
        self.declare_parameter('read_timeout_sec', 3.0)      # segundos sin frames antes de reconectar
        self.declare_parameter('use_mjpeg', False)           # True si el stream es MJPEG (no RTSP)
        self.declare_parameter('rtsp_transport', 'tcp')      # protocolo RTSP: tcp|udp|auto
        self.declare_parameter('snapshot_user', '')          # usuario para cámaras con auth HTTP Digest
        self.declare_parameter('snapshot_pass', '')          # contraseña para HTTP Digest 
        # CameraInfo — parámetros intrínsecos de la cámara para fusión con LiDAR.
        # Defaults asumen 90° de FOV horizontal en el tamaño de imagen configurado.
        # Para fusión precisa, reemplazar con valores de calibración real.
        self.declare_parameter('publish_camera_info', False)          # publicar CameraInfo
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_fx', 0.0)   # focal length X en píxeles (0 → estimado desde hfov)
        self.declare_parameter('camera_fy', 0.0)   # focal length Y
        self.declare_parameter('camera_cx', 0.0)   # punto principal X (0 → width/2)
        self.declare_parameter('camera_cy', 0.0)   # punto principal Y (0 → height/2)
        self.declare_parameter('camera_hfov_deg', 90.0)  # FOV horizontal en grados para estimar fx/fy
        self.declare_parameter('camera_k1', 0.0)   # coeficiente de distorsión radial k1
        self.declare_parameter('camera_k2', 0.0)   # coeficiente de distorsión radial k2
        self.declare_parameter('camera_p1', 0.0)   # coeficiente de distorsión tangencial p1
        self.declare_parameter('camera_p2', 0.0)   # coeficiente de distorsión tangencial p2
        self.declare_parameter('camera_k3', 0.0)   # coeficiente de distorsión radial k3

        # ── Lectura de parámetros ─────────────────────────────────────────────
        self._stream_url = str(self.get_parameter('stream_url').value) #viene del .sh para lanzar la transmicion 
        self._image_topic = str(self.get_parameter('image_topic').value) #viene de ymal 
        self._frame_id = str(self.get_parameter('frame_id').value) # id del topico 
        self._target_fps = max(1.0, float(self.get_parameter('target_fps').value))  # mínimo 1 FPS 
        self._width = max(0, int(self.get_parameter('width').value))
        self._height = max(0, int(self.get_parameter('height').value))
        self._reconnect_interval_sec = max(0.5, float(self.get_parameter('reconnect_interval_sec').value))
        self._read_timeout_sec = max(0.5, float(self.get_parameter('read_timeout_sec').value))
        self._use_mjpeg = bool(self.get_parameter('use_mjpeg').value)
        self._rtsp_transport = self._normalize_rtsp_transport(
            str(self.get_parameter('rtsp_transport').value)
        )
        self._snapshot_user = str(self.get_parameter('snapshot_user').value).strip()
        self._snapshot_pass = str(self.get_parameter('snapshot_pass').value).strip()

        # Detecta si el stream es una URL de snapshot (foto individual) vs stream continuo
        self._use_snapshot = self._stream_url.startswith('http') and (
            '/ISAPI/' in self._stream_url     # cámaras Hikvision con API ISAPI
            or '/snap.jpg' in self._stream_url  # endpoint de snapshot genérico
            or bool(self._snapshot_user)        # cualquier URL http con credenciales
        )
        # Detecta si es un stream de red (RTSP/HTTP) vs dispositivo local (USB)
        self._is_network_stream = self._stream_url.startswith(
            ('rtsp://', 'rtsps://', 'http://', 'https://')
        )
        # Detecta específicamente RTSP para aplicar opciones de latencia baja
        self._is_rtsp_stream = self._stream_url.startswith(('rtsp://', 'rtsps://'))

        if not self._stream_url:
            raise ValueError(
                'stream_url is required. Example: '
                'rtsp://admin:PASS@192.168.1.64:554/Streaming/Channels/101'
            )

        # ── Configuración de CameraInfo (parámetros intrínsecos) ──────────────
        self._publish_camera_info = bool(self.get_parameter('publish_camera_info').value)
        # Solo construye el mensaje si se va a publicar (ahorra memoria)
        self._camera_info_msg = self._build_camera_info() if self._publish_camera_info else None

        # ── Bridge OpenCV ↔ ROS e inicialización de publicadores ─────────────
        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, self._image_topic, qos_profile_sensor_data)
        if self._publish_camera_info:
            _ci_topic = str(self.get_parameter('camera_info_topic').value)
            self._camera_info_pub = self.create_publisher(CameraInfo, _ci_topic, qos_profile_sensor_data)

        # ── Estado interno de la captura ──────────────────────────────────────
        self._capture: Optional[cv2.VideoCapture] = None  # objeto de captura OpenCV
        self._capture_lock = threading.Lock()              # mutex para acceder a _capture desde dos hilos
        self._stop_event = threading.Event()               # señal de cierre del nodo

        # Frame compartido entre hilo lector y hilo publicador
        self._latest_frame = None                          # último frame decodificado
        self._latest_frame_lock = threading.Lock()         # mutex para acceder a _latest_frame
        self._latest_frame_event = threading.Event()       # señal: "hay un frame nuevo"

        # Marcas de tiempo para control de timeout y reconexión
        self._last_frame_monotonic = 0.0                   # cuándo llegó el último frame
        self._last_connect_attempt_monotonic = 0.0         # cuándo fue el último intento de conexión

        # ── Creación de los dos hilos ─────────────────────────────────────────
        # Hilo lector: lee frames de la cámara (puede ser lento por la red)
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name='ip-camera-reader',
            daemon=True,  # se cierra automáticamente con el proceso
        )
        # Hilo publicador: publica en ROS exactamente a target_fps
        self._publisher_thread = threading.Thread(
            target=self._publisher_loop,
            name='ip-camera-publisher',
            daemon=True,
        )

        # ── Inicialización según tipo de stream ───────────────────────────────
        if self._use_snapshot:
            # Modo snapshot: hace peticiones HTTP individuales para obtener cada frame
            self._snapshot_opener = self._make_snapshot_opener()  # configura autenticación HTTP
            self.get_logger().info(
                f'ip_camera_publisher snapshot mode '
                f'(topic={self._image_topic}, fps={self._target_fps}, url={self._stream_url})'
            )
        else:
            # Modo stream continuo: abre la conexión RTSP/MJPEG antes de iniciar hilos
            self._snapshot_opener = None
            self._connect()  # establece la conexión inicial

        # Inicia los hilos después de configurar todo
        self._reader_thread.start()
        self._publisher_thread.start()

    # ── Construcción del mensaje CameraInfo ───────────────────────────────────
    def _build_camera_info(self) -> CameraInfo:
        """Construye el mensaje CameraInfo con parámetros intrínsecos de la cámara.

        Los parámetros intrínsecos definen la geometría de proyección de la cámara:
          - fx, fy: longitudes focales (cómo se escala la profundidad a píxeles)
          - cx, cy: punto principal (centro óptico en píxeles)
          - k1..k3, p1, p2: coeficientes de distorsión radial y tangencial

        Si fx/fy no se configuran, se estiman a partir del FOV horizontal:
          fx = (width/2) / tan(hfov/2)
        """
        import math
        w = self._width if self._width > 0 else 640    # usa 640 si no se configuró width
        h = self._height if self._height > 0 else 360
        hfov = float(self.get_parameter('camera_hfov_deg').value)

        fx_param = float(self.get_parameter('camera_fx').value)
        fy_param = float(self.get_parameter('camera_fy').value)
        cx_param = float(self.get_parameter('camera_cx').value)
        cy_param = float(self.get_parameter('camera_cy').value)

        # Si no se proporcionaron fx/fy, estima desde el FOV horizontal
        fx = fx_param if fx_param > 0 else (w / 2.0) / math.tan(math.radians(hfov / 2.0))
        fy = fy_param if fy_param > 0 else fx   # asume píxeles cuadrados (fx ≈ fy)
        cx = cx_param if cx_param > 0 else w / 2.0  # punto principal en el centro
        cy = cy_param if cy_param > 0 else h / 2.0

        # Coeficientes de distorsión
        k1 = float(self.get_parameter('camera_k1').value)
        k2 = float(self.get_parameter('camera_k2').value)
        p1 = float(self.get_parameter('camera_p1').value)
        p2 = float(self.get_parameter('camera_p2').value)
        k3 = float(self.get_parameter('camera_k3').value)

        msg = CameraInfo()
        msg.header.frame_id = self._frame_id
        msg.width = w
        msg.height = h
        msg.distortion_model = 'plumb_bob'   # modelo estándar de distorsión de lente
        msg.d = [k1, k2, p1, p2, k3]        # vector de distorsión: [k1, k2, p1, p2, k3]
        # Matriz intrínseca K (3×3 aplanada en row-major):
        # [fx,  0, cx]
        # [ 0, fy, cy]
        # [ 0,  0,  1]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        # Matriz de rectificación R (identidad: cámara mono no necesita rectificación)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Matriz de proyección P (3×4): igual a K pero con columna 0 extra para cámara mono
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.get_logger().info(
            f'camera_info: {w}x{h}, fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} '
            f'(hfov={hfov}°, {"estimated" if fx_param == 0 else "from params"})'
        )
        return msg

    # ── Construcción del opener HTTP con autenticación Digest ─────────────────
    def _make_snapshot_opener(self):
        """Crea un opener HTTP con autenticación Digest para cámaras IP protegidas.

        HTTP Digest Auth es más seguro que Basic Auth: el password nunca viaja en claro.
        Si no hay usuario/contraseña, devuelve un opener anónimo estándar.
        """
        if self._snapshot_user and self._snapshot_pass:
            mgr = HTTPPasswordMgrWithDefaultRealm()
            # Registra las credenciales para cualquier realm en esta URL
            mgr.add_password(None, self._stream_url, self._snapshot_user, self._snapshot_pass)
            return build_opener(HTTPDigestAuthHandler(mgr))  # opener con Digest Auth
        return build_opener()  # opener sin autenticación

    # ── Normalización del protocolo de transporte RTSP ────────────────────────
    def _normalize_rtsp_transport(self, raw_value: str) -> str:
        """Valida y normaliza el valor de rtsp_transport. Fallback a 'tcp' si inválido.

        TCP: más confiable (retransmisión), mayor latencia
        UDP: menor latencia pero puede perder paquetes
        auto: OpenCV decide según la cámara
        """
        value = raw_value.strip().lower()
        if value in {'tcp', 'udp', 'auto'}:
            return value
        self.get_logger().warning(
            f"invalid rtsp_transport='{raw_value}', falling back to tcp"
        )
        return 'tcp'  # TCP es el más compatible con cámaras IP

    # ── Configuración de opciones FFMPEG para latencia baja ──────────────────
    def _merge_ffmpeg_capture_options(self) -> Optional[str]:
        """Combina opciones FFMPEG del entorno con defaults de baja latencia para RTSP.

        Las opciones se pasan a OpenCV via la variable de entorno
        OPENCV_FFMPEG_CAPTURE_OPTIONS con formato 'clave;valor|clave;valor'.

        Los defaults para RTSP:
          - fflags=nobuffer: deshabilita el buffer del demuxer (menos latencia)
          - flags=low_delay: activa modo de bajo delay en el decodificador
          - max_delay=200000: limita el delay acumulado a 200ms
        """
        raw_options = os.environ.get('OPENCV_FFMPEG_CAPTURE_OPTIONS', '')  # lee opciones del entorno
        parsed_options: dict[str, str] = {}
        option_order: list[str] = []

        # Parsea el formato 'key;value|key;value'
        for chunk in raw_options.split('|'):
            key, _, value = chunk.partition(';')
            key = key.strip()
            value = value.strip()
            if not key or not value:
                continue
            if key not in parsed_options:
                option_order.append(key)
            parsed_options[key] = value  # las opciones del usuario tienen prioridad

        if self._is_rtsp_stream:
            # Aplica defaults de baja latencia solo para streams RTSP
            # Las opciones del usuario sobrescriben estos defaults
            low_latency_defaults = {
                'fflags': 'nobuffer',    # desactiva buffer del demuxer → menos latencia
                'flags': 'low_delay',    # modo bajo delay en el decodificador
                'max_delay': '200000',   # máximo 200ms de delay acumulado
            }
            for key, value in low_latency_defaults.items():
                if key not in parsed_options:  # no sobreescribe opciones del usuario
                    option_order.append(key)
                    parsed_options[key] = value

            # Añade el protocolo de transporte RTSP al inicio de las opciones
            if self._rtsp_transport != 'auto':
                if 'rtsp_transport' not in parsed_options:
                    option_order.insert(0, 'rtsp_transport')
                parsed_options['rtsp_transport'] = self._rtsp_transport

        if not option_order:
            return None  # sin opciones: no modifica la variable de entorno
        # Reconstruye el formato 'key;value|key;value'
        return '|'.join(f'{key};{parsed_options[key]}' for key in option_order)

    # ── Conexión al stream de video ───────────────────────────────────────────
    def _connect(self) -> None:
        """Abre o reabre la conexión al stream de video.

        Respeta el intervalo mínimo entre intentos para no saturar la red.
        Usa FFMPEG como backend para mejor soporte de RTSP y H.264.
        """
        now = time.monotonic()
        # Evita reconectar demasiado rápido (espera reconnect_interval_sec)
        if now - self._last_connect_attempt_monotonic < self._reconnect_interval_sec:
            return
        self._last_connect_attempt_monotonic = now

        backend = cv2.CAP_FFMPEG  # usa FFMPEG para decodificar H.264/H.265
        capture = cv2.VideoCapture()
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # buffer interno de OpenCV mínimo (1 frame)
        capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 8000.0)  # timeout de conexión: 8 segundos
        capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000.0)  # timeout de lectura: 5 segundos

        # Aplica opciones FFMPEG para latencia baja antes de abrir el stream
        ffmpeg_capture_options = self._merge_ffmpeg_capture_options()
        if ffmpeg_capture_options:
            os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = ffmpeg_capture_options

        capture.open(self._stream_url, backend)  # abre el stream (puede tardar varios segundos)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # vuelve a forzar buffer=1 (open puede resetearlo)

        # Para dispositivos locales (USB): configura resolución y FPS antes de leer
        if not self._is_network_stream:
            if self._width > 0:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            if self._height > 0:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            capture.set(cv2.CAP_PROP_FPS, self._target_fps)

        # Verifica que la conexión fue exitosa
        if not capture.isOpened():
            self.get_logger().error(f'cannot open stream: {self._stream_url}')
            capture.release()
            with self._capture_lock:
                if self._capture is not None:
                    self._capture.release()
                self._capture = None  # marca como sin conexión
            return

        # Reemplaza la captura anterior con la nueva (thread-safe)
        with self._capture_lock:
            if self._capture is not None:
                self._capture.release()   # libera la conexión anterior
            self._capture = capture       # activa la nueva conexión

        self.get_logger().info(
            'ip_camera_publisher connected '
            f'(topic={self._image_topic}, fps={self._target_fps}, '
            f'url={self._stream_url}, rtsp_transport={self._rtsp_transport})'
        )

    # ── Normalización del frame (color y resolución) ──────────────────────────
    def _normalize_frame(self, frame):
        """Convierte el frame al formato y resolución correctos.

        Cámaras MJPEG a veces entregan frames en escala de grises.
        Streams de red pueden entregar resolución distinta a la configurada.
        """
        if self._use_mjpeg and frame.ndim == 2:
            # Frame MJPEG en escala de grises (2D) → convierte a BGR (3 canales)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif (
            self._is_network_stream
            and self._width > 0
            and self._height > 0
            and (frame.shape[1] != self._width or frame.shape[0] != self._height)
        ):
            # El stream de red entrega resolución diferente a la configurada: redimensiona
            # Se hace aquí en lugar de configurar RTSP porque muchas cámaras ignoran esa propiedad
            frame = cv2.resize(frame, (self._width, self._height), interpolation=cv2.INTER_AREA)
        return frame

    # ── Hilo lector: elige entre modo snapshot y modo stream ──────────────────
    def _reader_loop(self) -> None:
        """Punto de entrada del hilo lector. Delega al modo correcto según el tipo de stream."""
        if self._use_snapshot:
            self._snapshot_reader_loop()  # peticiones HTTP individuales
        else:
            self._rtsp_reader_loop()      # lectura continua del stream

    # ── Modo snapshot: una petición HTTP por frame ────────────────────────────
    def _snapshot_reader_loop(self) -> None:
        """Lee frames haciendo peticiones HTTP individuales al endpoint de snapshot.

        Útil para cámaras que no soportan RTSP pero tienen endpoint /snap.jpg.
        Respeta el FPS configurado para no saturar la cámara.
        """
        min_interval = 1.0 / self._target_fps  # tiempo mínimo entre peticiones
        errors = 0
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                # Hace la petición HTTP con timeout de 3 segundos
                with self._snapshot_opener.open(self._stream_url, timeout=3) as resp:
                    data = resp.read()  # descarga los bytes de la imagen JPEG

                # Verifica que los datos comienzan con el magic bytes de JPEG (FF D8)
                if data and data[:2] == b'\xff\xd8':
                    arr = np.frombuffer(data, dtype=np.uint8)         # bytes → array NumPy
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)        # decodifica JPEG → BGR
                    if frame is not None:
                        # Redimensiona si es necesario
                        if self._width > 0 and self._height > 0:
                            if frame.shape[1] != self._width or frame.shape[0] != self._height:
                                frame = cv2.resize(frame, (self._width, self._height), interpolation=cv2.INTER_AREA)
                        errors = 0  # resetea contador de errores si tuvo éxito
                        self._last_frame_monotonic = time.monotonic()
                        with self._latest_frame_lock:
                            self._latest_frame = frame  # actualiza el frame más reciente
                        self._latest_frame_event.set()  # notifica al publicador
            except (URLError, OSError):
                errors += 1
                if errors <= 3:
                    self.get_logger().warning(f'snapshot fetch error, retrying...')
                self._stop_event.wait(0.5)  # espera medio segundo antes de reintentar
                continue

            # Controla la frecuencia: espera el tiempo restante del período
            elapsed = time.monotonic() - t0
            wait = min_interval - elapsed
            if wait > 0:
                self._stop_event.wait(wait)

    # ── Modo stream: lectura continua RTSP/MJPEG ──────────────────────────────
    def _rtsp_reader_loop(self) -> None:
        """Lee frames del stream RTSP tan rápido como los entrega la cámara.

        No limita la velocidad aquí: el publicador se encarga de regular el FPS.
        Maneja reconexiones automáticas si el stream se interrumpe.
        """
        while not self._stop_event.is_set():
            with self._capture_lock:
                capture = self._capture  # obtiene la captura actual (puede ser None)

            if capture is None:
                # Sin conexión: intenta reconectar y espera un poco
                self._connect()
                self._stop_event.wait(0.05)
                continue

            # Lee el siguiente frame del stream (puede bloquear hasta que llegue)
            ok, frame = capture.read()
            now = time.monotonic()

            if not ok or frame is None:
                # Error de lectura: puede ser pérdida de conexión o fin de stream
                if now - self._last_frame_monotonic >= self._read_timeout_sec:
                    # Pasó demasiado tiempo sin frames: reconecta
                    if now - self._last_connect_attempt_monotonic >= self._reconnect_interval_sec:
                        self.get_logger().warning('stream read timeout, reconnecting')
                    self._connect()
                else:
                    # Error transitorio: espera un poco antes de reintentar
                    self._stop_event.wait(0.01)
                continue

            # Frame válido: actualiza el estado y notifica al publicador
            self._last_frame_monotonic = now
            with self._latest_frame_lock:
                self._latest_frame = frame  # sobreescribe: siempre el más fresco
            self._latest_frame_event.set()  # despierta al publicador

    # ── Hilo publicador: publica a frecuencia exacta ──────────────────────────
    def _publisher_loop(self) -> None:
        """Publica el último frame disponible exactamente a target_fps.

        Corre independiente del lector para que el overhead de encode+publicación
        no bloquee la lectura de frames de la cámara.
        """
        min_period = 1.0 / self._target_fps  # período entre publicaciones

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            # Espera hasta un período por un frame nuevo
            if not self._latest_frame_event.wait(timeout=min_period):
                continue  # timeout: no hay frame nuevo, vuelve a esperar
            self._latest_frame_event.clear()  # limpia la señal

            with self._latest_frame_lock:
                frame = self._latest_frame  # toma el frame más reciente

            if frame is None:
                continue  # no hay frame disponible aún

            # Normaliza el frame (color y resolución)
            frame = self._normalize_frame(frame)

            # Construye el timestamp actual
            stamp = self.get_clock().now().to_msg()
            # Convierte array OpenCV BGR → mensaje sensor_msgs/Image
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = stamp          # timestamp para sincronización
            msg.header.frame_id = self._frame_id  # frame de referencia para TF

            self._publisher.publish(msg)  # publica en /camera/image_raw

            # Publica CameraInfo sincronizado con la imagen (mismo timestamp)
            if self._publish_camera_info and self._camera_info_msg is not None:
                self._camera_info_msg.header.stamp = stamp
                self._camera_info_pub.publish(self._camera_info_msg)

            # Controla el FPS: duerme el tiempo restante del período
            elapsed = time.monotonic() - loop_start
            remaining = min_period - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)

    # ── Destrucción limpia del nodo ───────────────────────────────────────────
    def destroy_node(self) -> bool:
        """Detiene los hilos y libera la captura de video antes de cerrar el nodo."""
        self._stop_event.set()          # señaliza a todos los hilos que paren
        self._latest_frame_event.set()  # desbloquea el publicador si estaba esperando
        # Espera a que ambos hilos terminen (con timeout para no bloquearse)
        for thread in (self._reader_thread, self._publisher_thread):
            if thread.is_alive():
                thread.join(timeout=self._read_timeout_sec + 1.0)
        # Libera el objeto de captura OpenCV
        with self._capture_lock:
            if self._capture is not None:
                self._capture.release()
                self._capture = None
        return super().destroy_node()


# ── Punto de entrada ──────────────────────────────────────────────────────────
def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = IpCameraPublisherNode()
    try:
        rclpy.spin(node)        # bloquea hasta Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
