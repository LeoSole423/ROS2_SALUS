# Arquitectura Runtime

Estado: actual
Alcance: contratos de tópicos y flujo de control del stack SALUS
Fuente de verdad: README de paquetes, launches y nodos activos del checkout

## Flujo de control principal
```text
Nav2
-> /cmd_vel
-> nav2_collision_monitor
-> /cmd_vel_safe
-> nav_command_server
-> /cmd_vel_final
-> controller_server
-> actuador real o backend sim_gazebo
```

## Control manual web
```text
map_tools/web_zone_server
-> /cmd_vel_teleop (interfaces/msg/CmdVelFinal)
-> nav_command_server
-> /cmd_vel_final
-> controller_server
```

## Sensores y percepción
En el robot real vigente, la telemetria Pixhawk/GNSS entra por MAVROS. El
driver propio `sensores/pixhawk_driver` queda como codigo legacy/de referencia
y no forma parte de `real_global_v2` ni `real_global_v2_wifi`.

```text
Pixhawk 6X + GNSS F9P DroneCAN
-> MAVROS
-> /imu/data
-> /global_position/raw/fix
-> /local_position/velocity_local
-> /local_position/odom
```

```text
RS16
-> /scan_3d
-> pointcloud_to_laserscan
-> /scan
-> scan_noise_filter
-> /scan_clean
```

En Global V2 existe tambien una ruta experimental de filtrado 3D:

```text
RS16
-> /scan_3d
-> lidar_obstacle_filter
-> /obstacles_cloud
-> /scan_filtered
```

Por defecto Global V2 usa la ruta V1.5 conservadora `/scan -> /scan_clean`.
El fallback legacy puro se obtiene con `enable_scan_noise_filter:=False`.
La ruta RANSAC V1 queda como experimental/debug con
`enable_lidar_obstacle_filter:=True` y no se mezcla con `/scan_clean`.

Estado y plan de percepcion LiDAR: [docs/lidar-perception.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-perception.md).

## Localización mainline
- Entradas típicas:
  - `/imu/data`
  - `/global_position/raw/fix` en perfiles reales MAVROS
  - `/gps/fix` en simulación o compatibilidad legacy
  - `/controller/drive_telemetry`
- Salidas:
  - `/odometry/local`
  - `/odometry/gps`
- TF esperada:
  - `map -> odom -> base_footprint`

## Localización V2 local
- Entrada de movimiento:
  - `/controller/drive_telemetry`
- Procesamiento:
  - `ackermann_odometry`
  - EKF local en `odom`
- Salida:
  - `/odometry/local`
- TF:
  - `odom -> base_footprint`

## Localización V2 global
- Base local:
  - `/odometry/local`
- Capa global:
  - `navsat_transform`
  - `/odometry/gps`
  - EKF global
- Salida:
  - `/odometry/global`
- TF:
  - `map -> odom -> base_footprint`

Heading auxiliar por brujula: `compass_heading_gate` puede usar
`/mavros_node/compass_hdg` como referencia debil solo en arranque o reposo
largo, publicando yaw-only en `/imu/compass_heading` y debug en
`/imu/compass_heading/debug`. Esta ayuda esta desactivada por defecto con
`enable_compass_heading:=false`; la fusion EKF queda detras de
`enable_compass_heading_fusion:=false`. No usar `/mavros_node/mag` crudo
directamente en el EKF.
Ver [docs/compass-heading-gate.md](/home/leo/codigo/ROS2_SALUS/docs/compass-heading-gate.md).

## Simulación Gazebo
- Entrada canónica:
  - `tools/launch_sim_global_v2_wifi.sh`
- El wrapper WiFi acepta argumentos extra de launch para cambiar mundo y pose inicial:
  - `world`
  - `world_name`
  - `spawn_x`, `spawn_y`, `spawn_z`
  - `spawn_roll`, `spawn_pitch`, `spawn_yaw`
  - `datum_lat`, `datum_lon`, `datum_yaw_deg`
- Wrapper específico disponible:
  - `tools/launch_sim_global_v2_wifi_sonoma.sh`

Procedimiento para abrir mundos Gazebo/Fuel nuevos: [docs/gazebo-worlds.md](/home/leo/codigo/ROS2_SALUS/docs/gazebo-worlds.md).

## Nodos clave por responsabilidad
- Arbitraje y navegación:
  - `navegacion_gps/nav_command_server`
  - `navegacion_gps/zones_manager`
  - `navegacion_gps/nav_snapshot_server`
  - `navegacion_gps/nav_observability`
- Actuación:
  - `controller_server/controller_server_node`
- Sensores:
  - `mavros/mavros_node`
  - `sensores/rtk_bridge`
  - `sensores/mavros_compat_bridge` (compatibilidad legacy opcional)
  - `sensores/pixhawk_driver` (legacy/referencia)
  - `rslidar_sdk`
- Web:
  - `map_tools/web_zone_server`
  - `sensores/sensores_web`

## Dónde mirar cuando algo falla
- Estado y eventos de navegación:
  - `/nav_command_server/telemetry`
  - `/nav_command_server/events`
- Estado y telemetría del controlador:
  - `/controller/status`
  - `/controller/telemetry`
  - `/controller/drive_telemetry`
- Diagnóstico global:
  - `/diagnostics`
