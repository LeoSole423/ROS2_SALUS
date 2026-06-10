# sensores

Estado: actual
Alcance: integración ROS 2 con Pixhawk, MAVROS, RS16 y herramientas web auxiliares
Fuente de verdad: `launch/`, `setup.py` y nodos bajo `sensores/`

Paquete ROS 2 para integración con Pixhawk, dashboard web y utilidades auxiliares de cámara. El camino operativo vigente para Pixhawk/GNSS es MAVROS. El driver propio `pixhawk_driver` permanece en el repositorio como codigo legacy/de referencia.

## Documentación relacionada
- Índice general: [docs/INDEX.md](/home/leo/codigo/ROS2_SALUS/docs/INDEX.md)
- Matriz de launches: [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md)
- Arquitectura runtime: [docs/runtime-architecture.md](/home/leo/codigo/ROS2_SALUS/docs/runtime-architecture.md)

## Ejecutables reales
- `mavros_compat_bridge`
- `rtk_bridge`
- `rtk_source_manager`
- `sensores_web`
- `camara`

## Ejecutables legacy / referencia
- `pixhawk_driver`

## Launches reales
- `ros2 launch sensores mavros.launch.py`
- `ros2 launch sensores rs16.launch.py`

## `pixhawk_driver` legacy
Este driver propio fue reemplazado por MAVROS en la navegacion real vigente
(`real_global_v2` y `real_global_v2_wifi`). Se conserva para diagnostico,
comparacion o recuperacion, pero no debe usarse como backend operativo nuevo
sin una decision explicita.

Publica:
- `/imu/data`
- `/gps/fix`
- `/odom`
- `/velocity`
- `/gps/rtk_status`
- `/gps/fix_type`
- `/gps/satellites_visible`
- `/gps/hdop`
- `/gps/rtcm_age_s`
- `/gps/rtcm_received_count`
- `/gps/rtcm_sequence_id`

Parámetros principales:
- `serial_port` (default `/dev/ttyACM0`)
- `baudrate` (default `921600`)
- `odom_frame` (default `odom`)
- `base_link_frame` (default `base_footprint`)
- `imu_frame` (default `imu_link`)
- `gps_frame` (default `gps_link`)
- `publish_rate_hz` (default `200.0`)
- `enable_gps_rtk`
- `enable_rtcm_tcp`
- `rtcm_tcp_host`
- `rtcm_tcp_port`
- `rtcm_topic`
- `yaw_correction_deg`

Ejecución directa:
```bash
ros2 run sensores pixhawk_driver --ros-args -p serial_port:=/dev/ttyACM0 -p baudrate:=921600
```

## `sensores_web`
Sirve `pixhawk_dashboard.html` y expone datos por HTTP y WebSocket.

Parámetros:
- `imu_topic`
- `gps_topic`
- `velocity_topic`
- `odom_topic`
- `http_host`
- `http_port`
- `ws_host`
- `ws_port`
- `html_path`

Ejecutar:
```bash
ros2 run sensores sensores_web
```

Payload adicional cuando hay orientación disponible:
- `imu.yaw_enu_rad`, `imu.yaw_enu_deg`
- `odom.yaw_enu_rad`, `odom.yaw_enu_deg`
- `diagnostics.yaw_delta_deg`
- `topics.{imu,gps,velocity,odom}`

## `camara`
Nodo opcional para control ISAPI de cámara.

Servicios:
- `/camara/camera_pan`
- `/camara/camera_zoom_toggle`
- `/camara/camera_status`

Ejecutar:
```bash
ros2 run sensores camara
```

## Launch de Pixhawk legacy
Solo driver:
```bash
ros2 launch sensores pixhawk.launch.py
```

Driver + dashboard web:
```bash
ros2 launch sensores pixhawk.launch.py launch_web:=true
```

## Launch de MAVROS
Este launch usa `mavros` + `mavros_extras` y es el backend Pixhawk/GNSS vigente.
`real_global_v2` lo incluye directamente con `launch_legacy_compat:=false`.
Cuando se ejecuta solo, por defecto puede levantar un bridge temporal para
reponer el contrato legacy que consumen herramientas antiguas.

Dashboard con MAVROS:
```bash
ros2 launch sensores mavros.launch.py launch_web:=true
```

Parámetros principales:
- `fcu_url` (default `/dev/ttyACM0:921600`)
- `gcs_url`
- `tgt_system`
- `tgt_component`
- `namespace` (default vacío; deja los tópicos nativos en root-level)
- `fcu_protocol` (default `v2.0`)
- `pluginlists_yaml`
- `config_yaml`
- `apm_config_yaml`
- `launch_legacy_compat` (default `true`)

Contrato MAVROS nativo:
- `/imu/data`
- `/global_position/raw/fix`
- `/local_position/velocity_local`
- `/local_position/odom`
- `/state`

Contrato legacy temporal publicado por `mavros_compat_bridge`:
- `/gps/fix`
- `/odom`
- `/velocity`

Bindings por defecto del dashboard:
- IMU: `/imu/data`
- GPS: `/global_position/raw/fix`
- Velocidad: `/local_position/velocity_local`
- Odometría: `/local_position/odom`

El perfil por defecto es `sensor-only` para no exponer interfaces de control desde ROS.

`mavros_compat_bridge` no transforma frames, yaw ni coordenadas. Solo republica mensajes nativos de MAVROS al contrato legacy y emite warnings claros cuando faltan publishers o datos frescos en los tópicos esperados.

## Launch de RS16
Este launch envuelve `rslidar_sdk`, que se considera una dependencia vendorizada del workspace.

Driver:
```bash
ros2 launch sensores rs16.launch.py
```

Driver + RViz:
```bash
ros2 launch sensores rs16.launch.py rviz:=true
```

Config personalizado:
```bash
ros2 launch sensores rs16.launch.py config_path:=/ruta/a/rs16.yaml
```

## Notas
- El nombre correcto del ejecutable Pixhawk es `pixhawk_driver`; `ros2 run sensores sensores` no aplica en este checkout.
- Los defaults de `pixhawk.launch.py` usan `base_footprint` y `gps_link`, pero ese launch es legacy.
- El camino MAVROS no reutiliza `yaw_correction_deg`; cualquier ajuste de heading debe resolverse en localización/configuración y no reinyectando correcciones del driver viejo.
- `mavros.launch.py` requiere que `mavros` y `mavros_extras` estén instalados en el entorno ROS 2.
