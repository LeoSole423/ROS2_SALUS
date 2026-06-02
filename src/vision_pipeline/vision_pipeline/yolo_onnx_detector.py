#!/usr/bin/env python3

from __future__ import annotations  # permite anotar tipos como string para compatibilidad con Python 3.8

# ── Librerías estándar ────────────────────────────────────────────────────────
import platform    # para bloquear inferencia en arquitecturas no permitidas
import threading   # para correr la inferencia en un hilo separado sin bloquear ROS
import time        # para medir tiempos de inferencia y controlar FPS
from dataclasses import dataclass       # para crear clases de datos simples (DetectionResult)
from pathlib import Path                # para manejar rutas de archivos de forma segura
from typing import List, Optional, Sequence, Tuple  # tipos para anotaciones

# ── Librerías de visión y numéricas ──────────────────────────────────────────
import cv2          # OpenCV: preprocesamiento de imágenes y NMS (supresión de cuadros duplicados)
import numpy as np  # NumPy: operaciones matriciales sobre tensores del modelo

# ── ROS 2 ────────────────────────────────────────────────────────────────────
import rclpy
from ament_index_python.packages import get_package_share_directory  # para encontrar archivos del paquete ROS
from cv_bridge import CvBridge                    # convierte mensajes Image de ROS ↔ arrays de OpenCV
from rclpy.executors import MultiThreadedExecutor # ejecuta el nodo con múltiples hilos
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,  # perfil QoS predefinido para sensores: BEST_EFFORT, depth=10
)
from sensor_msgs.msg import Image            # mensaje ROS para imágenes de cámara
from std_msgs.msg import Header, String      # Header para timestamps, String para mensajes de debug
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose  # mensajes de detección estándar

# ── ONNX Runtime (importación segura: puede no estar instalado) ───────────────
try:
    import onnxruntime as ort  # biblioteca para correr modelos en formato ONNX (GPU/CPU)
except ImportError as exc:     # si no está instalado, se captura el error para manejarlo en runtime
    ort = None
    _ORT_IMPORT_ERROR = exc    # se guarda el error para mostrarlo cuando se intente usar el nodo
else:
    _ORT_IMPORT_ERROR = None   # si la importación tuvo éxito, no hay error guardado


ARM64_ARCHITECTURES = {'aarch64', 'arm64'}


def _is_arm64_platform() -> bool:
    """Devuelve True si este proceso corre sobre Linux ARM64/aarch64."""
    return platform.machine().strip().lower() in ARM64_ARCHITECTURES


# ── Estructura de datos para una detección ───────────────────────────────────
@dataclass
class DetectionResult:
    """Representa un objeto detectado por YOLO con sus coordenadas y confianza."""
    class_id: int    # índice numérico de la clase (0=person, 2=car, etc.)
    label: str       # nombre de la clase en texto (ej: "person")
    score: float     # confianza del modelo entre 0.0 y 1.0
    x1: float        # esquina superior-izquierda del bounding box en píxeles
    y1: float
    x2: float        # esquina inferior-derecha del bounding box en píxeles
    y2: float

    @property
    def width(self) -> float:
        """Ancho del bounding box en píxeles. max(0) evita valores negativos."""
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        """Alto del bounding box en píxeles."""
        return max(0.0, self.y2 - self.y1)

    @property
    def center_x(self) -> float:
        """Centro horizontal del bounding box (x1 + mitad del ancho)."""
        return self.x1 + self.width * 0.5

    @property
    def center_y(self) -> float:
        """Centro vertical del bounding box."""
        return self.y1 + self.height * 0.5


# ── Función auxiliar: busca el archivo de etiquetas COCO ─────────────────────
def _resolve_default_labels_path() -> str:
    """Busca coco_80.names en el directorio del paquete ROS o junto al código fuente."""
    candidates: List[Path] = []
    try:
        # Intenta encontrar el archivo en el directorio instalado del paquete ROS 2
        share_dir = Path(get_package_share_directory('vision_pipeline'))
        candidates.append(share_dir / 'config' / 'coco_80.names')
    except Exception:
        pass  # si el paquete no está instalado (desarrollo), continúa con el fallback

    # Fallback: busca el archivo dos directorios arriba del script actual
    candidates.append(Path(__file__).resolve().parents[1] / 'config' / 'coco_80.names')

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)  # devuelve la primera ruta válida encontrada
    return ''  # devuelve vacío si no encuentra el archivo (se manejará más adelante)


# ── Función auxiliar: carga las etiquetas desde el archivo .names ─────────────
def _load_labels(path: str) -> List[str]:
    """Lee el archivo de clases COCO y devuelve una lista de strings (una por línea)."""
    if not path:
        return []  # si no hay ruta, devuelve lista vacía (el nodo usará IDs numéricos)

    labels_path = Path(path).expanduser()  # expande ~ a /home/usuario si es necesario
    if not labels_path.exists():
        raise FileNotFoundError(f'class_names_path not found: {labels_path}')

    labels = []
    for raw_line in labels_path.read_text(encoding='utf-8').splitlines():
        line = raw_line.strip()             # elimina espacios y saltos de línea
        if line and not line.startswith('#'):  # ignora líneas vacías y comentarios
            labels.append(line)
    return labels  # ej: ['person', 'bicycle', 'car', ..., 'toothbrush']


# ── Función auxiliar: aplana índices NMS a lista plana de enteros ─────────────
def _flatten_indices(indices: object) -> List[int]:
    """Normaliza la salida de cv2.dnn.NMSBoxes a una lista plana de int.

    OpenCV puede devolver [[0],[2],[5]] o [0,2,5] o un ndarray, dependiendo
    de la versión. Esta función maneja todos los casos.
    """
    if indices is None:
        return []  # NMS no seleccionó ningún cuadro
    if isinstance(indices, np.ndarray):
        return [int(x) for x in indices.flatten().tolist()]  # convierte ndarray a lista
    if isinstance(indices, (list, tuple)):
        flattened: List[int] = []
        for item in indices:
            if isinstance(item, (list, tuple, np.ndarray)):
                flattened.extend(_flatten_indices(item))  # recursivo para listas anidadas
            else:
                flattened.append(int(item))
        return flattened
    return [int(indices)]  # caso escalar: un solo índice


# ── Función auxiliar: valida que una dimensión del modelo sea un int positivo ──
def _maybe_static_dim(value: object) -> Optional[int]:
    """Devuelve la dimensión si es un int positivo fijo. Devuelve None si es dinámica (ej: -1 o 'N')."""
    return value if isinstance(value, int) and value > 0 else None


# ══════════════════════════════════════════════════════════════════════════════
#  NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════
class YoloOnnxDetectorNode(Node):
    """Nodo ROS 2 que corre YOLO en ONNX Runtime en un worker thread separado.

    Flujo:
      /camera/image_raw → _image_callback (hilo ROS)
                        → _worker_loop   (hilo inferencia)
                        → _process_image → modelo ONNX
                        → /detections + /objeto_detectado
    """

    def __init__(self) -> None:
        super().__init__('yolo_onnx_detector')  # nombre del nodo en el grafo ROS 2

        if _is_arm64_platform():
            raise RuntimeError(
                'yolo_onnx_detector is disabled on ARM64/aarch64 platforms. '
                'Run YOLO on an external PC and publish /camera/image_raw + /detections '
                'for the cockpit display.'
            )

        # Verifica que ONNX Runtime esté instalado antes de continuar
        if ort is None:
            raise RuntimeError(
                'onnxruntime is not available. Install python3-onnxruntime or '
                '`pip install onnxruntime` before running this node.'
            ) from _ORT_IMPORT_ERROR

        # ── Declaración de parámetros configurables desde YAML o CLI ─────────
        self.declare_parameter('image_topic', '/camera/image_raw')       # tópico de entrada de imágenes
        self.declare_parameter('detections_topic', '/detections')         # tópico de salida de detecciones
        self.declare_parameter('debug_topic', '/objeto_detectado')        # tópico de debug en texto
        self.declare_parameter('model_path', '')                          # ruta al archivo .onnx
        self.declare_parameter('class_names_path', _resolve_default_labels_path())  # ruta a coco_80.names
        self.declare_parameter('input_width', 640)                        # ancho de entrada del modelo en píxeles
        self.declare_parameter('input_height', 640)                       # alto de entrada del modelo
        self.declare_parameter('conf_threshold', 0.40)                    # confianza mínima para aceptar una detección
        self.declare_parameter('nms_iou_threshold', 0.45)                 # umbral IoU para suprimir duplicados
        self.declare_parameter('max_detections', 20)                      # máximo de objetos por frame
        self.declare_parameter('max_fps', 15.0)                           # límite de FPS para no saturar la CPU
        self.declare_parameter('execution_provider', 'auto')              # auto|cuda|tensorrt|openvino|cpu
        self.declare_parameter('intra_op_threads', 2)                     # hilos dentro de una operación ONNX
        self.declare_parameter('inter_op_threads', 1)                     # hilos entre operaciones ONNX
        self.declare_parameter('warmup_runs', 1)                          # inferencias de calentamiento antes de publicar
        self.declare_parameter('log_interval_sec', 5.0)                   # cada cuántos segundos imprime estadísticas
        self.declare_parameter('publish_empty_debug', True)               # publicar "sin_detecciones" cuando no hay objetos

        # ── Lectura de parámetros con validación ──────────────────────────────
        self._image_topic = str(self.get_parameter('image_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        debug_topic = str(self.get_parameter('debug_topic').value)
        model_path_raw = str(self.get_parameter('model_path').value)
        class_names_path = str(self.get_parameter('class_names_path').value)
        if not class_names_path:
            class_names_path = _resolve_default_labels_path()  # segundo intento si el parámetro está vacío
        self._input_width = max(32, int(self.get_parameter('input_width').value))   # mínimo 32 px
        self._input_height = max(32, int(self.get_parameter('input_height').value))
        self._conf_threshold = float(self.get_parameter('conf_threshold').value)
        self._nms_iou_threshold = float(self.get_parameter('nms_iou_threshold').value)
        self._max_detections = max(1, int(self.get_parameter('max_detections').value))  # al menos 1
        self._max_fps = max(0.0, float(self.get_parameter('max_fps').value))            # 0 = sin límite
        self._execution_provider = str(self.get_parameter('execution_provider').value).lower()
        self._intra_op_threads = max(1, int(self.get_parameter('intra_op_threads').value))
        self._inter_op_threads = max(1, int(self.get_parameter('inter_op_threads').value))
        self._warmup_runs = max(0, int(self.get_parameter('warmup_runs').value))
        self._log_interval_sec = max(0.5, float(self.get_parameter('log_interval_sec').value))
        self._publish_empty_debug = bool(self.get_parameter('publish_empty_debug').value)

        # ── Validación del archivo del modelo ─────────────────────────────────
        self._model_path = Path(model_path_raw).expanduser() if model_path_raw else None
        if self._model_path is None or not self._model_path.exists():
            raise FileNotFoundError(
                'model_path must point to a YOLO ONNX file, for example '
                '/home/user/models/yolov8n.onnx'
            )

        # ── Carga de etiquetas y bridge ROS↔OpenCV ────────────────────────────
        self._labels = _load_labels(class_names_path)  # lista con 80 nombres de clases COCO
        self._bridge = CvBridge()                       # convierte sensor_msgs/Image ↔ numpy array

        # ── Configuración de calidad de servicio (QoS) ────────────────────────
        # QoS para detecciones: RELIABLE asegura entrega, depth=5 mantiene cola pequeña
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # QoS para debug: BEST_EFFORT es suficiente (si se pierde un mensaje de debug no importa)
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Creación de publicadores ──────────────────────────────────────────
        self._detections_pub = self.create_publisher(Detection2DArray, detections_topic, detection_qos)
        self._debug_pub = self.create_publisher(String, debug_topic, debug_qos)

        # QoS para imagen de entrada: depth=1 descarta frames viejos, mantiene solo el último
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Subscripción a la cámara: cada frame recibido llama a _image_callback
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_callback,
            image_qos,
        )

        # ── Sincronización entre hilo ROS y hilo de inferencia ────────────────
        self._frame_lock = threading.Lock()      # mutex para acceder a _latest_msg
        self._frame_event = threading.Event()    # señal: "hay un frame nuevo disponible"
        self._stop_event = threading.Event()     # señal: "el nodo se está cerrando"
        self._latest_msg: Optional[Image] = None # último frame recibido (solo se guarda uno)
        self._latest_seq = 0                     # contador de frames recibidos
        self._processed_seq = -1                 # contador de frames procesados (distinto = hay trabajo)

        # ── Creación e inicio de la sesión ONNX ──────────────────────────────
        self._session, self._providers = self._create_session()  # abre el modelo .onnx
        self._input_name = self._session.get_inputs()[0].name    # nombre del tensor de entrada (ej: 'images')
        self._output_names = [output.name for output in self._session.get_outputs()]  # nombres de tensores de salida
        # Determina si el modelo usa float16 (GPU optimizado) o float32 (CPU)
        self._input_dtype = np.float16 if 'float16' in self._session.get_inputs()[0].type else np.float32
        self._reconcile_input_size_with_model()  # ajusta ancho/alto si el modelo tiene dimensiones fijas
        self._warmup_model()                     # corre inferencias vacías para "calentar" GPU/JIT

        # ── Estadísticas internas ─────────────────────────────────────────────
        self._processed_frames = 0        # total de frames procesados
        self._dropped_frames = 0          # frames que llegaron mientras otro se procesaba
        self._ema_inference_ms = 0.0      # media exponencial del tiempo de inferencia en ms
        self._last_log_monotonic = time.monotonic()  # marca de tiempo del último log impreso

        # ── Hilo de inferencia (separado del loop de ROS 2) ───────────────────
        # Se corre como daemon: se cierra automáticamente cuando el proceso principal termina
        self._worker = threading.Thread(target=self._worker_loop, name='vision-inference', daemon=True)
        self._worker.start()

        self.get_logger().info(
            'yolo_onnx_detector ready '
            f'(image_topic={self._image_topic}, model={self._model_path}, '
            f'input={self._input_width}x{self._input_height}, '
            f'providers={self._providers})'
        )

    # ── Creación de la sesión ONNX Runtime ───────────────────────────────────
    def _create_session(self) -> Tuple[ort.InferenceSession, List[str]]:
        """Crea la sesión ONNX con el backend seleccionado (GPU/CPU) y opciones de optimización."""
        providers = self._select_providers(self._execution_provider)  # elige GPU o CPU

        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL  # máxima optimización del grafo
        options.intra_op_num_threads = self._intra_op_threads   # hilos dentro de operaciones como convoluciones
        options.inter_op_num_threads = self._inter_op_threads   # hilos entre capas del modelo
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL  # ejecuta capas en orden (más predecible)
        options.enable_cpu_mem_arena = True  # reutiliza memoria CPU entre inferencias (más rápido)

        session = ort.InferenceSession(
            str(self._model_path),   # ruta al archivo yolo11n.onnx
            sess_options=options,
            providers=providers,     # lista ordenada de backends a intentar
        )
        return session, session.get_providers()  # devuelve sesión + lista de backends realmente usados

    # ── Selección del backend de inferencia ───────────────────────────────────
    def _select_providers(self, selection: str) -> List[str]:
        """Devuelve la lista de providers ONNX disponibles según la selección del usuario.

        'auto' intenta TensorRT → CUDA → OpenVINO → CPU en ese orden de prioridad.
        Si el provider solicitado no está instalado, ONNX cae al siguiente disponible.
        """
        available = ort.get_available_providers()  # lista de providers instalados en este sistema
        provider_map = {
            'auto': [
                'TensorrtExecutionProvider',    # más rápido: NVIDIA TensorRT
                'CUDAExecutionProvider',         # segundo: CUDA genérico
                'OpenVINOExecutionProvider',     # tercero: Intel OpenVINO
                'CPUExecutionProvider',          # fallback final: solo CPU
            ],
            'tensorrt': ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'],
            'cuda':     ['CUDAExecutionProvider', 'CPUExecutionProvider'],
            'openvino': ['OpenVINOExecutionProvider', 'CPUExecutionProvider'],
            'cpu':      ['CPUExecutionProvider'],
        }

        if selection not in provider_map:
            raise ValueError(
                'execution_provider must be one of: auto, tensorrt, cuda, openvino, cpu'
            )

        # Filtra: solo incluye providers que están realmente disponibles en el sistema
        providers = [provider for provider in provider_map[selection] if provider in available]
        if not providers:
            raise RuntimeError(
                f'No compatible ONNX Runtime providers for selection={selection}. '
                f'Available providers: {available}'
            )
        return providers

    # ── Ajuste del tamaño de entrada al modelo ────────────────────────────────
    def _reconcile_input_size_with_model(self) -> None:
        """Si el modelo tiene dimensiones de entrada fijas, las usa en lugar de los parámetros.

        Algunos modelos ONNX exportados con dimensiones estáticas (ej: 320x320)
        ignorarán cualquier otro tamaño. Esto previene errores silenciosos.
        """
        input_shape = self._session.get_inputs()[0].shape  # shape del tensor: [batch, canales, H, W]
        if len(input_shape) < 4:
            return  # shape inesperado, no se puede inferir dimensiones

        model_height = _maybe_static_dim(input_shape[2])  # dimensión H (puede ser None si es dinámica)
        model_width = _maybe_static_dim(input_shape[3])   # dimensión W
        if model_width is None or model_height is None:
            return  # modelo con dimensiones dinámicas: usa los parámetros tal cual

        if model_width != self._input_width or model_height != self._input_height:
            self.get_logger().warning(
                'input_width/input_height do not match the static ONNX input. '
                f'Using model shape {model_width}x{model_height} instead.'
            )
            self._input_width = model_width    # corrige al tamaño real del modelo
            self._input_height = model_height

    # ── Calentamiento del modelo ──────────────────────────────────────────────
    def _warmup_model(self) -> None:
        """Corre inferencias con imágenes negras para inicializar GPU y compilar JIT.

        Sin warmup, la primera inferencia real tarda 3-10x más por la compilación
        de kernels CUDA y la asignación inicial de memoria en GPU.
        """
        if self._warmup_runs <= 0:
            return

        # Tensor de zeros con la misma forma que una imagen real
        dummy = np.zeros((1, 3, self._input_height, self._input_width), dtype=self._input_dtype)
        for _ in range(self._warmup_runs):
            self._session.run(self._output_names, {self._input_name: dummy})  # descarta el resultado

    # ── Callback del hilo ROS: recibe frames de la cámara ────────────────────
    def _image_callback(self, msg: Image) -> None:
        """Llamado por ROS 2 cada vez que llega un frame. Solo guarda el más reciente.

        No procesa aquí: delega al worker thread para no bloquear el executor de ROS.
        Si el worker todavía está procesando el frame anterior, este lo sobreescribe
        (siempre trabajamos con el frame más fresco, no con cola).
        """
        with self._frame_lock:
            # Si había un frame guardado que aún no fue procesado, se cuenta como descartado
            if self._latest_msg is not None and self._latest_seq != self._processed_seq:
                self._dropped_frames += 1
            self._latest_msg = msg          # guarda el frame más reciente
            self._latest_seq += 1           # incrementa el contador para que el worker sepa que hay algo nuevo
        self._frame_event.set()             # despierta al worker thread si estaba esperando

    # ── Hilo de inferencia: procesa frames en segundo plano ──────────────────
    def _worker_loop(self) -> None:
        """Loop principal del hilo de inferencia. Procesa el frame más reciente disponible.

        Duerme si no hay frames nuevos y respeta el límite de FPS configurado.
        """
        # Convierte FPS máximo a período mínimo entre frames (0 = sin límite)
        min_period = 0.0 if self._max_fps <= 0.0 else 1.0 / self._max_fps

        while not self._stop_event.is_set():
            # Espera hasta 50ms por un frame nuevo (no bloquea indefinidamente)
            self._frame_event.wait(timeout=0.05)
            if self._stop_event.is_set():
                break  # el nodo se está cerrando

            with self._frame_lock:
                # Si no hay frame nuevo o ya fue procesado, limpia la señal y espera
                if self._latest_msg is None or self._latest_seq == self._processed_seq:
                    self._frame_event.clear()
                    continue
                msg = self._latest_msg           # toma el frame actual
                current_seq = self._latest_seq   # guarda el número de secuencia
                self._frame_event.clear()        # limpia la señal de "hay frame nuevo"

            cycle_start = time.perf_counter()
            try:
                self._process_image(msg)  # ← aquí ocurre la inferencia YOLO
            except Exception as exc:
                self.get_logger().error(f'inference failed: {exc}')
            finally:
                with self._frame_lock:
                    self._processed_seq = current_seq  # marca este frame como procesado

            # Control de FPS: si la inferencia fue más rápida que el período mínimo, espera
            elapsed = time.perf_counter() - cycle_start
            if min_period > elapsed:
                self._stop_event.wait(min_period - elapsed)  # duerme el tiempo restante

    # ── Procesamiento de un frame: preproceso → inferencia → postproceso ──────
    def _process_image(self, msg: Image) -> None:
        """Pipeline completo para un frame: decodifica → preprocesa → infiere → publica."""
        # Convierte el mensaje ROS Image a un array BGR de OpenCV (H×W×3)
        frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        original_h, original_w = frame_bgr.shape[:2]  # guarda dimensiones originales para escalar bbox de vuelta

        # Preprocesa: redimensiona y normaliza la imagen para el modelo
        input_tensor, scale, pad = self._preprocess(frame_bgr)

        # ── Inferencia ONNX ───────────────────────────────────────────────────
        inference_start = time.perf_counter()
        outputs = self._session.run(self._output_names, {self._input_name: input_tensor})
        inference_ms = (time.perf_counter() - inference_start) * 1000.0  # tiempo en milisegundos

        # ── Postprocesamiento: convierte tensores del modelo a DetectionResult ─
        detections = self._postprocess(outputs, original_w, original_h, scale, pad)

        if not rclpy.ok():
            return  # el nodo se está cerrando, no publicar

        # ── Publicación de resultados ─────────────────────────────────────────
        self._publish_detections(msg.header, detections)   # → /detections
        self._publish_debug_text(detections)               # → /objeto_detectado

        # ── Actualización de estadísticas ────────────────────────────────────
        self._processed_frames += 1
        if self._ema_inference_ms <= 0.0:
            self._ema_inference_ms = inference_ms  # primer dato: usa directamente
        else:
            # EMA (media exponencial): α=0.1 suaviza picos sin olvidar la tendencia
            self._ema_inference_ms = 0.9 * self._ema_inference_ms + 0.1 * inference_ms

        # Imprime estadísticas cada N segundos para no saturar el log
        now = time.monotonic()
        if now - self._last_log_monotonic >= self._log_interval_sec:
            self._last_log_monotonic = now
            self.get_logger().info(
                'vision stats '
                f'(frames={self._processed_frames}, dropped={self._dropped_frames}, '
                f'inference_ms={self._ema_inference_ms:.1f}, detections={len(detections)})'
            )

    # ── Preprocesamiento: letterbox + normalización ───────────────────────────
    def _preprocess(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """Redimensiona la imagen al tamaño del modelo con letterbox (padding gris).

        Letterbox: escala la imagen manteniendo la proporción y rellena con gris (114)
        los bordes sobrantes. Esto evita distorsión y permite invertir el escalado
        para obtener coordenadas en la imagen original.

        Retorna:
          tensor: array (1, 3, H, W) float32/float16 normalizado [0,1] — listo para ONNX
          scale:  factor de escala aplicado (para invertir el bbox al original)
          pad:    (pad_x, pad_y) píxeles de relleno añadidos a cada lado
        """
        src_h, src_w = image_bgr.shape[:2]

        # Calcula el factor de escala que cabe en el cuadrado del modelo sin distorsionar
        scale = min(self._input_width / src_w, self._input_height / src_h)
        resized_w = max(1, int(round(src_w * scale)))   # nuevo ancho después de escalar
        resized_h = max(1, int(round(src_h * scale)))   # nuevo alto

        # Redimensiona la imagen con interpolación bilineal (mejor para upscaling)
        resized = cv2.resize(image_bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

        # Crea un canvas gris (valor 114, estándar YOLO) del tamaño exacto del modelo
        canvas = np.full((self._input_height, self._input_width, 3), 114, dtype=np.uint8)

        # Calcula el desplazamiento para centrar la imagen en el canvas
        pad_x = (self._input_width - resized_w) // 2
        pad_y = (self._input_height - resized_h) // 2

        # Pega la imagen redimensionada en el centro del canvas gris
        canvas[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w] = resized

        # Convierte BGR (OpenCV) a RGB (YOLO fue entrenado con RGB)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

        # Transpone de (H, W, C) a (C, H, W): formato que espera el modelo (CHW)
        chw = np.transpose(rgb, (2, 0, 1))

        # Normaliza a [0.0, 1.0] dividiendo por 255 y convierte al dtype del modelo
        tensor = np.ascontiguousarray(chw, dtype=np.float32) / 255.0
        tensor = tensor.astype(self._input_dtype, copy=False)  # float16 si el modelo lo requiere

        # Añade dimensión de batch: (C, H, W) → (1, C, H, W)
        tensor = np.expand_dims(tensor, axis=0)
        return tensor, scale, (pad_x, pad_y)

    # ── Postprocesamiento: tensores ONNX → lista de DetectionResult ──────────
    def _postprocess(
        self,
        outputs: Sequence[np.ndarray],
        original_w: int,
        original_h: int,
        scale: float,
        pad: Tuple[int, int],
    ) -> List[DetectionResult]:
        """Interpreta la salida del modelo YOLO y aplica NMS para eliminar duplicados.

        YOLO puede exportarse en dos formatos:
          - End-to-end (E2E): el modelo ya incluye NMS, salida tiene 6 columnas [x1,y1,x2,y2,score,class]
          - Raw: salida tiene 4+80 columnas [cx,cy,w,h, class_scores...], NMS manual necesario
        """
        prediction = np.asarray(outputs[0])  # primer tensor de salida del modelo
        prediction = np.squeeze(prediction)   # elimina dimensiones de tamaño 1 (ej: batch=1)

        if prediction.ndim == 1:
            return []  # tensor 1D no esperado: modelo con problemas o sin detecciones

        if prediction.ndim != 2:
            raise RuntimeError(f'Unsupported YOLO output shape: {outputs[0].shape}')

        # YOLO puede dar (N, 84) o (84, N): si las filas son pocas y columnas muchas, transpone
        if prediction.shape[0] <= 128 and prediction.shape[1] > 128:
            prediction = prediction.T  # normaliza a (N_detecciones, N_valores)

        # Detecta formato E2E: 6 columnas (bbox + score + class_id) o 7 (con rotación)
        if prediction.shape[1] in (6, 7):
            return self._parse_end_to_end_output(prediction, original_w, original_h)

        # Formato raw con NMS manual
        return self._parse_raw_yolo_output(prediction, original_w, original_h, scale, pad)

    # ── Parser para modelos exportados con NMS integrado (E2E) ───────────────
    def _parse_end_to_end_output(
        self,
        prediction: np.ndarray,
        original_w: int,
        original_h: int,
    ) -> List[DetectionResult]:
        """Parsea salida con formato [x1, y1, x2, y2, score, class_id] por fila.

        El modelo ya filtró y aplicó NMS internamente. Solo se filtra por confianza.
        """
        results: List[DetectionResult] = []
        for row in prediction:
            score = float(row[4])                        # columna 4: confianza de la detección
            if score < self._conf_threshold:
                continue  # descarta detecciones débiles

            class_id = int(row[5]) if row.shape[0] >= 6 else 0  # columna 5: clase detectada
            # Clip para asegurar que las coordenadas estén dentro de la imagen
            x1 = float(np.clip(row[0], 0, original_w - 1))
            y1 = float(np.clip(row[1], 0, original_h - 1))
            x2 = float(np.clip(row[2], 0, original_w - 1))
            y2 = float(np.clip(row[3], 0, original_h - 1))
            results.append(
                DetectionResult(
                    class_id=class_id,
                    label=self._class_name(class_id),  # convierte ID → nombre ("person", "car", etc.)
                    score=score,
                    x1=min(x1, x2),  # asegura que x1 < x2 aunque el modelo dé coordenadas invertidas
                    y1=min(y1, y2),
                    x2=max(x1, x2),
                    y2=max(y1, y2),
                )
            )

        results.sort(key=lambda det: det.score, reverse=True)  # ordena de mayor a menor confianza
        return results[:self._max_detections]  # limita al máximo configurado

    # ── Parser para modelos con salida raw (sin NMS integrado) ───────────────
    def _parse_raw_yolo_output(
        self,
        prediction: np.ndarray,
        original_w: int,
        original_h: int,
        scale: float,
        pad: Tuple[int, int],
    ) -> List[DetectionResult]:
        """Parsea salida con formato [cx, cy, w, h, objectness, class_scores...].

        Aplica NMS manualmente con cv2.dnn.NMSBoxes para eliminar cuadros duplicados.
        Invierte el letterbox para obtener coordenadas en la imagen original.
        """
        num_values = prediction.shape[1]  # cantidad de columnas (ej: 84 = 4 bbox + 80 clases)
        if num_values < 5:
            return []  # salida demasiado pequeña, modelo incompatible

        # Determina si hay columna de objectness separada o si las scores de clase son directas
        if self._labels and num_values == len(self._labels) + 4:
            # Formato YOLOv8/v11: [cx, cy, w, h, cls0, cls1, ..., cls79] sin objectness
            objectness = np.ones((prediction.shape[0],), dtype=np.float32)  # objectness implícito = 1
            class_scores = prediction[:, 4:]   # columnas 4..83: una score por clase
        elif self._labels and num_values == len(self._labels) + 5:
            # Formato YOLOv5: [cx, cy, w, h, objectness, cls0, ..., cls79]
            objectness = prediction[:, 4]      # columna 4: probabilidad de que haya objeto
            class_scores = prediction[:, 5:]   # columnas 5..84: scores de clase
        else:
            # Formato desconocido: intenta heurística
            objectness = prediction[:, 4]
            class_scores = prediction[:, 5:] if num_values > 5 else np.ones((prediction.shape[0], 1))

        if class_scores.size == 0:
            return []

        # Para cada ancla/propuesta, encuentra la clase con mayor score
        class_ids = np.argmax(class_scores, axis=1)                              # índice de la clase ganadora por fila
        class_conf = class_scores[np.arange(class_scores.shape[0]), class_ids]  # score de la clase ganadora
        scores = objectness * class_conf  # score final = objectness × confianza de clase

        # Filtra candidatos que superan el umbral de confianza
        keep = scores >= self._conf_threshold
        if not np.any(keep):
            return []  # ningún candidato supera el umbral

        # Aplica la máscara de filtrado
        boxes = prediction[keep, :4].astype(np.float32, copy=False)  # coordenadas [cx, cy, w, h]
        scores = scores[keep].astype(np.float32, copy=False)
        class_ids = class_ids[keep]

        # Si los valores de bbox son ≤ 2.0, están normalizados [0,1]: desnormaliza a píxeles del modelo
        if np.max(boxes[:, :4]) <= 2.0:
            boxes[:, 0] *= float(self._input_width)
            boxes[:, 1] *= float(self._input_height)
            boxes[:, 2] *= float(self._input_width)
            boxes[:, 3] *= float(self._input_height)

        # ── Invierte el letterbox para obtener coordenadas en la imagen original ──
        pad_x, pad_y = pad
        # Convierte de (cx, cy, w, h) en espacio del modelo → (x1, y1, x2, y2) en espacio original
        x1 = (boxes[:, 0] - boxes[:, 2] * 0.5 - pad_x) / scale  # resta el padding y deshace la escala
        y1 = (boxes[:, 1] - boxes[:, 3] * 0.5 - pad_y) / scale
        x2 = (boxes[:, 0] + boxes[:, 2] * 0.5 - pad_x) / scale
        y2 = (boxes[:, 1] + boxes[:, 3] * 0.5 - pad_y) / scale

        # Clamp: asegura que las coordenadas estén dentro de la imagen original
        x1 = np.clip(x1, 0.0, float(max(0, original_w - 1)))
        y1 = np.clip(y1, 0.0, float(max(0, original_h - 1)))
        x2 = np.clip(x2, 0.0, float(max(0, original_w - 1)))
        y2 = np.clip(y2, 0.0, float(max(0, original_h - 1)))

        # Prepara los bboxes en formato [x, y, width, height] que espera cv2.dnn.NMSBoxes
        nms_boxes = []
        for idx in range(scores.shape[0]):
            width = max(0.0, float(x2[idx] - x1[idx]))
            height = max(0.0, float(y2[idx] - y1[idx]))
            nms_boxes.append([float(x1[idx]), float(y1[idx]), width, height])

        # ── Supresión No Máxima (NMS): elimina cuadros solapados duplicados ───
        # Dos cuadros que se solapan más del nms_iou_threshold se fusionan (se queda el de mayor score)
        selected = _flatten_indices(
            cv2.dnn.NMSBoxes(
                nms_boxes,
                scores.tolist(),
                self._conf_threshold,
                self._nms_iou_threshold,
            )
        )
        if not selected:
            return []  # NMS eliminó todo

        # Construye la lista final de detecciones con los índices seleccionados por NMS
        detections: List[DetectionResult] = []
        for idx in selected[:self._max_detections]:
            detections.append(
                DetectionResult(
                    class_id=int(class_ids[idx]),
                    label=self._class_name(int(class_ids[idx])),
                    score=float(scores[idx]),
                    x1=float(x1[idx]),
                    y1=float(y1[idx]),
                    x2=float(x2[idx]),
                    y2=float(y2[idx]),
                )
            )

        detections.sort(key=lambda det: det.score, reverse=True)  # ordena por confianza descendente
        return detections[:self._max_detections]

    # ── Convierte ID numérico a nombre de clase ───────────────────────────────
    def _class_name(self, class_id: int) -> str:
        """Devuelve el nombre de la clase (ej: 'person') o el ID como string si no hay etiquetas."""
        if 0 <= class_id < len(self._labels):
            return self._labels[class_id]  # busca en la lista cargada de coco_80.names
        return str(class_id)               # fallback: devuelve "0", "1", etc.

    # ── Publica las detecciones en ROS 2 ─────────────────────────────────────
    def _publish_detections(self, header: Header, detections: Sequence[DetectionResult]) -> None:
        """Convierte la lista de DetectionResult al mensaje estándar Detection2DArray de ROS."""
        array_msg = Detection2DArray()
        array_msg.header = header  # copia el timestamp y frame_id de la imagen original

        for index, detection in enumerate(detections):
            detection_msg = Detection2D()
            detection_msg.header = header
            detection_msg.id = f'{detection.label}_{index}'  # ID único: "person_0", "car_1", etc.

            # Centro del bounding box en píxeles
            detection_msg.bbox.center.position.x = float(detection.center_x)
            detection_msg.bbox.center.position.y = float(detection.center_y)
            detection_msg.bbox.center.theta = 0.0  # sin rotación (YOLO no predice ángulo)

            # Tamaño del bounding box en píxeles
            detection_msg.bbox.size_x = float(detection.width)
            detection_msg.bbox.size_y = float(detection.height)

            # Hipótesis de clasificación con score
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection.label  # "person", "car", etc.
            hypothesis.hypothesis.score = float(detection.score)
            detection_msg.results.append(hypothesis)

            array_msg.detections.append(detection_msg)

        self._detections_pub.publish(array_msg)  # publica en /detections

    # ── Publica texto de debug con el resumen de detecciones ─────────────────
    def _publish_debug_text(self, detections: Sequence[DetectionResult]) -> None:
        """Publica un resumen legible de las detecciones en /objeto_detectado."""
        debug_msg = String()
        if detections:
            top = detections[0]  # detección con mayor confianza
            # Ejemplo: "top=person score=0.92 total=3 [person:0.92, car:0.85, dog:0.71]"
            summary = ', '.join(f'{det.label}:{det.score:.2f}' for det in detections[:3])
            debug_msg.data = f'top={top.label} score={top.score:.2f} total={len(detections)} [{summary}]'
            self._debug_pub.publish(debug_msg)
            return

        # Si no hay detecciones, publica "sin_detecciones" (si está configurado)
        if self._publish_empty_debug:
            debug_msg.data = 'sin_detecciones'
            self._debug_pub.publish(debug_msg)

    # ── Destrucción limpia del nodo ───────────────────────────────────────────
    def destroy_node(self) -> bool:
        """Detiene el worker thread y libera recursos antes de cerrar el nodo."""
        self._stop_event.set()    # señaliza al worker que debe parar
        self._frame_event.set()   # desbloquea el worker si está esperando un frame
        if hasattr(self, '_worker') and self._worker.is_alive():
            self._worker.join(timeout=1.0)  # espera hasta 1 segundo que el worker termine
        return super().destroy_node()


# ── Punto de entrada del nodo ─────────────────────────────────────────────────
def main(args: Optional[Sequence[str]] = None) -> None:
    """Inicializa ROS 2, crea el nodo y lo corre con executor multi-hilo."""
    rclpy.init(args=args)                                   # inicializa el cliente ROS 2
    node = YoloOnnxDetectorNode()                           # crea el nodo (carga el modelo)
    executor = MultiThreadedExecutor(num_threads=2)          # 2 hilos: uno para callbacks ROS, uno reserva
    executor.add_node(node)
    try:
        executor.spin()                                      # bloquea hasta Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()                                 # limpia el contexto ROS 2


if __name__ == '__main__':
    main()
