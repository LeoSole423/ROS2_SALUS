# vision_pipeline

Estado: paquete activo opcional, separado del mainline de Nav2.

Alcance: adquisición de cámara USB/IP, inferencia YOLO ONNX y visualización web de detecciones.

Fuente de verdad: `setup.py`, `launch/`, `config/` y nodos bajo `vision_pipeline/`.

Pipeline ROS 2 de baja latencia para deteccion de objetos en tiempo real con la topologia:

`v4l2_camera -> /camera/image_raw -> yolo_onnx_detector -> /detections + /objeto_detectado`

## Paquetes ROS usados

- `v4l2_camera` para la camara USB
- `image_transport` para transporte de imagen y compatibilidad con compresion
- `cv_bridge` para convertir `sensor_msgs/msg/Image` a OpenCV
- `vision_msgs` para publicar `Detection2DArray`

## Topicos

- Consume: `/camera/image_raw` (`sensor_msgs/msg/Image`)
- Publica: `/detections` (`vision_msgs/msg/Detection2DArray`)
- Publica: `/objeto_detectado` (`std_msgs/msg/String`)
- Opcional para debug remoto: `/camera/image_raw/compressed` si esta instalado `image_transport_plugins`
- Opcional: `/camera/camera_info` desde `ip_camera_publisher` cuando se habilita calibración/intrinsics

## Estructura

- `vision_pipeline/yolo_onnx_detector.py`: nodo principal en `rclpy`
- `vision_pipeline/ip_camera_publisher.py`: RTSP/MJPEG/snapshot a `sensor_msgs/Image`, con reconexión
- `vision_pipeline/vision_web_server.py`: servidor HTTP de imagen y overlay de detecciones
- `launch/vision_pipeline.launch.py`: lanza `v4l2_camera` y el detector
- `config/v4l2_camera_low_latency.yaml`: camara a 640x480 y 20 FPS
- `config/yolo_detector.yaml`: parametros del detector
- `config/coco_80.names`: etiquetas por defecto para YOLO COCO

## Build

```bash
cd /home/franco/final/ROS2_SALUS
colcon build --packages-select vision_pipeline
source install/setup.bash
```

## Ejecucion

```bash
ros2 launch vision_pipeline vision_pipeline.launch.py model_path:=/ABS/PATH/yolov8n.onnx
```

## Prueba rapida con camara IP

Si la camara actual es Ethernet/RTSP, podés probar primero la IA con un publicador temporal:

```bash
ros2 launch vision_pipeline ip_camera_ai_test.launch.py \
  stream_url:=rtsp://admin:PASS@192.168.1.64:554/Streaming/Channels/101 \
  model_path:=/ABS/PATH/yolov8n.onnx
```

Ese camino mantiene el mismo contrato ROS:

`ip_camera_publisher -> /camera/image_raw -> yolo_onnx_detector -> /detections + /objeto_detectado`

## Recomendaciones de rendimiento

- Mantener `image_size: [640, 480]` o bajar a `416x416`/`320x320` si el hardware es muy justo.
- Mantener `max_fps` entre `10` y `20`.
- Exportar un modelo liviano, por ejemplo `yolov8n` o `yolo11n`.
- Usar `batch=1`.
- Mantener la cola de imagen en tiempo real y descartar frames viejos como hace este nodo.
- En Jetson/NVIDIA, pasar a TensorRT si el objetivo es exprimir latencia.
- En CPU Intel, evaluar OpenVINO.
