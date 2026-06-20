# Tópicos y TF — ROS2_SALUS

Estado: documentación read-only del código actual.
Fuente de verdad: launches y nodos bajo `src/`. **(por confirmar)** marca lo que
no pude verificar solo leyendo el código (p. ej. publishers de nodos no leídos
línea por línea).

Hermanos: [ARQUITECTURA](ARQUITECTURA_ROS2_SALUS.md) ·
[NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md) ·
[SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md)

---

## 1. Cadena de control (tópicos)

| Tópico | Tipo | Publica | Consume | Notas |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 `controller_server` (nav2) | `collision_monitor` (`cmd_vel_in_topic`) | salida cruda del planner/controller |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | `collision_monitor` (`cmd_vel_out_topic`) | `nav_command_server` | tras frenado/ralentí por polígonos |
| `/cmd_vel_teleop` | `interfaces/CmdVelFinal` | `map_tools/web_zone_server` | `nav_command_server` | control manual web |
| `/cmd_vel_final` | `interfaces/CmdVelFinal` | `nav_command_server` | `controller_server_node` | comando arbitrado final (twist + `brake_pct`) |
| `/cmd_vel_gazebo` | `geometry_msgs/Twist` | `controller_server_node` (backend sim) | puente Gazebo | solo en sim (`bridge_config_v2.yaml:1`) |

Fuentes: `collision_monitor_v2.yaml:6-7`, `real_global_v2.launch.py:618-659`,
`controller_server_node.py:45,168`.

## 2. Actuación / telemetría del controlador

| Tópico | Tipo | Publica | Notas |
|---|---|---|---|
| `/controller/status` | `std_msgs/String` (JSON) | `controller_server_node` | modo, source, comando aplicado |
| `/controller/telemetry` | `std_msgs/String` (JSON) | `controller_server_node` | telemetría + stats backend |
| `/controller/drive_telemetry` | `interfaces/DriveTelemetry` | `controller_server_node` | velocidad/steer **medidos**, usado por odometría y heading GPS |

Fuente: `controller_server_node.py:169-173`.

## 3. LiDAR / percepción

| Tópico | Tipo | Publica | Consume |
|---|---|---|---|
| `/scan_3d` | `sensor_msgs/PointCloud2` | `rslidar_sdk_node` (frame `lidar_link`) | `pointcloud_to_laserscan`, filtros opcionales |
| `/rslidar_packets` | `rslidar_msg/...` | `rslidar_sdk_node` | (interno SDK) |
| `/rslidar_imu_data` | `sensor_msgs/Imu` | `rslidar_sdk_node` | IMU interna; configurado pero **`imu_port: 0`** → no garantizado como topic activo |
| `/scan` | `sensor_msgs/LaserScan` | `pointcloud_to_laserscan` | costmaps / collision_monitor (o filtros) |
| `/scan_clean` | `sensor_msgs/LaserScan` | `scan_noise_filter` | scan efectivo en real (default) |
| `/scan_filtered` | `sensor_msgs/LaserScan` | `lidar_obstacle_filter` (RANSAC, off) | scan efectivo si rama 3D activa |
| `/obstacles_cloud` | `sensor_msgs/PointCloud2` | `lidar_obstacle_filter` | debug |
| `/scan_3d/no_ground` | `sensor_msgs/PointCloud2` | `scan_ground_filter` (on en real global, off en sim/validación) | entrada alternativa a `pointcloud_to_laserscan` |
| `/scan_wifi_debug` | `sensor_msgs/LaserScan` | `scan_wifi_debug` | scan diezmado para visualización por WiFi |

El **scan efectivo** que consumen Nav2 y collision_monitor se resuelve por launch
(`effective_lidar_scan_topic` en `real_global_v2.launch.py:201-213`) y se inyecta vía
`RewrittenYaml` en los costmaps y el collision_monitor (`nav_global_v2.launch.py:65-96`).
Fuentes: `rs16.yaml:28-32`, `real_global_v2.launch.py:417-562`.

## 4. Sensores (perfil real MAVROS)

| Tópico | Tipo | Origen |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | MAVROS (remap de `mavros_node/data`) |
| `/global_position/raw/fix` | `sensor_msgs/NavSatFix` | MAVROS (`mavros_node/raw/fix`) |
| `/local_position/velocity_local` | `geometry_msgs/TwistStamped` | MAVROS |
| `/local_position/odom` | `nav_msgs/Odometry` | MAVROS |
| `/mavros_node/gps1/raw`, `/gps1/rtk` | MAVROS | diagnóstico RTK (consumido por `rtk_bridge`) |
| `/gps/rtk_status_mavros` | `std_msgs/String` | `rtk_bridge` (status consolidado, `rtk_bridge.py:86`) |

Fuente: `mavros.launch.py:202-323`, `real_global_v2.launch.py:380-407`.

### 4.1 Sensores (perfil legado driver propio — `sensores/pixhawk.launch.py`)

`pixhawk_driver` publica un contrato **distinto** (legado): `/imu/data`, `/gps/fix`,
`/gps/rtk_status`, `/gps/fix_type`, `/gps/satellites_visible`, `/gps/hdop`,
`/gps/rtcm_*`, `/odom`, `/velocity`. Fuente: `pixhawk_driver.py:6-16,323-326`.
Ver discrepancia de convenciones en [DEUDA_TECNICA](DEUDA_TECNICA.md).

## 5. Localización (salidas)

| Tópico | Tipo | Publica | Notas |
|---|---|---|---|
| `/odometry/local` | `nav_msgs/Odometry` | EKF local | base de la capa global |
| `/odometry/gps` | `nav_msgs/Odometry` | `navsat_transform` | medida GPS en `odom`/`map` |
| `/odometry/global` | `nav_msgs/Odometry` | EKF global | pose en `map` |
| `/gps/odometry_map` | `nav_msgs/Odometry` | `map_gps_absolute_measurement` | medida absoluta GPS→map |
| `/gps/course_heading` | `sensor_msgs/Imu` | `gps_course_heading` (`gps_course_heading.py:128`) | yaw-only por rumbo GPS, RTK-gated |

Fuente: `real_global_v2.launch.py:564-719`, `runtime-architecture.md`.

## 6. Navegación / web / misiones

| Tópico/servicio | Tipo | Nodo |
|---|---|---|
| `/nav_command_server/telemetry` | `interfaces/NavTelemetry` | `nav_command_server` (`nav_command_server.py:271`) |
| `/nav_command_server/events` | `interfaces/NavEvent` | `nav_command_server` (`nav_command_server.py:272`) |
| `/nav_command_server/set_goal_ll` | srv `SetNavGoalLL` | `nav_command_server` |
| `/nav_command_server/cancel_goal` | srv `CancelNavGoal` | `nav_command_server` |
| `/nav_command_server/brake` | srv `BrakeNav` | `nav_command_server` |
| `/nav_command_server/set_manual_mode` | srv `SetManualMode` | `nav_command_server` |
| `/route_executor/set_route_ll` | srv `SetRouteMissionLL` | `route_executor` |
| `/zones_manager/set_geojson` | srv `SetZonesGeoJson` | `zones_manager` |
| `/stop_zone`, `/critical_slow_zone`, `/slow_zone` | `geometry_msgs/PolygonStamped` | `polygon_stamped_republisher` (re-publica los `*_raw` del collision_monitor) |

Fuente: `no_go_editor.launch.py`, `nav_global_v2.launch.py:217-252`,
`real_global_v2.launch.py:618-678`.

---

## 7. Árbol TF

```text
map
 └─ odom                 (EKF global / navsat_transform)
     └─ base_footprint   (EKF local desde ackermann_odometry)
         ├─ base_link / chasis      (URDF estático)
         ├─ lidar_link              (URDF estático; RS16)
         ├─ imu_link                (URDF estático)
         └─ gps_link                (URDF estático)
```

- `map -> odom`: capa global (EKF global + navsat / heading GPS).
- `odom -> base_footprint`: EKF local (`robot_localization`) alimentado por
  `ackermann_odometry` (desde `/controller/drive_telemetry`).
- `base_footprint -> {lidar_link, imu_link, gps_link, ...}`: estáticos del URDF
  (`models/cuatri_real_v2.urdf` en perfiles global V2 activos, con LiDAR
  pitcheado 10°).
- `pointcloud_to_laserscan` proyecta a **`base_footprint`** (`target_frame`).

Frames declarados: `pixhawk.launch.py:40-59` (`odom`, `base_footprint`, `imu_link`,
`gps_link`), `rs16.yaml:28` (`lidar_link`), `collision_monitor_v2.yaml:4-5`
(`base_footprint`/`odom`).

Verificación en vivo:
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 run tf2_tools view_frames     # genera frames.pdf
```

> Discrepancia vs prompt: el prompt menciona `robot_localization consume /odom`.
> En el perfil real **mainline** (MAVROS) la odometría de movimiento entra como
> `/controller/drive_telemetry` → `ackermann_odometry` → EKF, no `/odom` directo.
> `/odom` lo publica el driver **legado** `pixhawk_driver`. Ver [DEUDA_TECNICA](DEUDA_TECNICA.md).
