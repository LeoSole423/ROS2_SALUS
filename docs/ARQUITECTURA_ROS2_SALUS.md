# Arquitectura ROS2_SALUS

Estado: documentación técnica del código actual (generada por revisión read-only).
Alcance: mapa del repo + arquitectura runtime.
Fuente de verdad: código bajo `src/`, launches y `tools/`. Donde algo no se pudo
confirmar leyendo el código se marca **(por confirmar)**.

Documentos hermanos:
[LAUNCHES_Y_OPERACION](LAUNCHES_Y_OPERACION.md) ·
[TOPICOS_Y_TF](TOPICOS_Y_TF.md) ·
[NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md) ·
[SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md) ·
[DEUDA_TECNICA](DEUDA_TECNICA.md)

> Nota: el `README.md` y varios docs existentes citan rutas `/home/leo/codigo/ROS2_SALUS`
> (otra máquina). En este checkout la raíz es `/home/franco/final/ROS2_SALUS`.

---

## 1. Plataforma

- Robot: cuatriciclo (ATV) autónomo con dirección **Ackermann**.
- Cómputo: Raspberry Pi 5 (`AGENTS.md`).
- ROS 2 **Humble**, workspace en Docker (`Dockerfile`, `docker-compose*.yml`).
- Stack: **Nav2 + robot_localization** (EKF dual + navsat).
- Sensores: Pixhawk 6X, GNSS F9P (RTK), RoboSense **RS16**, odometría de ruedas.
- RMW por defecto: **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`),
  fijado para evitar timeouts de servicios/acciones con Fast DDS (`README.md`).

---

## 2. Mapa de paquetes

### 2.1 Paquetes propios

| Paquete | Tipo build | Qué hace | Entry points / artefactos |
|---|---|---|---|
| `src/interfaces` | `ament_cmake` (rosidl) | Mensajes y servicios propios del proyecto | ver §2.3 |
| `src/controller_server` | `ament_python` | Convierte `/cmd_vel_final` en comandos al vehículo (UART real o backend Gazebo). | `controller_server_node` |
| `src/navegacion_gps` | `ament_python` | Núcleo de navegación: Nav2 wiring, localización GPS/EKF, arbitraje de comandos, zonas keepout, filtros LiDAR, benchmarks. | ~30 ejecutables (§2.4) |
| `src/map_tools` | `ament_python` | Servidor web de zonas no-go / editor de mapa y puente con la UI. | `web_zone_server` |
| `src/sensores` | `ament_python` | Drivers/puente de Pixhawk-GNSS (MAVROS y driver propio legacy), RTK, cámara, dashboard web. | `pixhawk_driver`, `mavros_compat_bridge`, `rtk_bridge`, `rtk_source_manager`, `sensores_web`, `camara` |

### 2.2 Paquetes vendorizados (no tocar)

| Paquete | Qué es |
|---|---|
| `src/rslidar_msg` | Mensajes RoboSense (`ament_cmake`). |
| `src/rslidar_sdk` | SDK RoboSense: nodo `rslidar_sdk_node` que publica `/scan_3d`. |

> Nota: el vendor `src/autoware_deps/` (POC del filtro de suelo Autoware, v0.51.0)
> **fue eliminado** (no se compilaba ni se usaba). Del filtro de suelo solo quedó el
> algoritmo portado a Python en `navegacion_gps/scan_ground_filter.py`. Historia en
> `docs/investigaciones/autoware-ground-segmentation-integracion.md`.

### 2.3 Otros paquetes presentes (fuera del alcance pedido)

Verificado en auditoría:
- `src/vision_pipeline` **es** un paquete ROS activo (stack de visión YOLO: cámara
  IP + fusión LiDAR + cockpit), pero **no está cableado al mainline de navegación**.
  Se documenta como "visión separada / no mainline".
- `src/lidar_camara` **no es un paquete activo**: solo quedan restos/caché, sin
  `package.xml`/`setup.py`. Tratar como **residuo legado**.

### 2.4 Interfaces propias (`src/interfaces`)

Mensajes (`msg/`): `CmdVelFinal`, `DriveTelemetry`, `NavEvent`, `NavTelemetry`,
`NavSnapshotLayers`, `NoGoPoint`, `NoGoZone`.

Servicios (`srv/`): `SetNavGoalLL`, `CancelNavGoal`, `GetNavState`, `BrakeNav`,
`SetManualMode`, `SetManualCmd`, `SetRouteMissionLL`, `CancelRouteMission`,
`GetRouteMissionState`, `SetZonesGeoJson`, `GetZonesState`, `SetKeepoutZones`,
`GetKeepoutState`, `GetNavSnapshot`, `SetDatum`, `GetDatum`, `CameraPan`,
`CameraStatus`.

`CmdVelFinal` es el contrato central de actuación (twist + `brake_pct`).

---

## 3. Activo vs legado

Clasificación según `AGENTS.md` y `README.md`.

| Estado | Launches / nodos |
|---|---|
| **Activo (mainline)** | `navegacion_gps/sim_global_v2.launch.py`, `real_global_v2.launch.py`, `real_global_v2_wifi.launch.py` (+ sus `rviz_*`). Sensores vía **MAVROS** (`sensores/mavros.launch.py`). LiDAR `sensores/rs16.launch.py`. Web `map_tools/no_go_editor.launch.py`. |
| **Legado / referencia** | `simulacion.launch.py`, `real.launch.py`, `sim_local_v2.launch.py`, `real_local_v2.launch.py`, `nav2_only.launch.py`, `rviz_real.launch.py`. Driver propio `sensores/pixhawk_driver` + `sensores/pixhawk.launch.py` (reemplazado por MAVROS). |
| **POC / experimental** | `validate_scan_ground.launch.py` (validación del filtro de suelo), `lidar_obstacle_filter` (rama RANSAC 3D, default off). `scan_ground_filter` ya es default on en `real_global_v2`; sigue siendo opt-in en sim/validación. |

> Regla operativa de `AGENTS.md`: tratar **toda** navegación distinta de
> `real_global_v2` y `sim_global_v2` como legado salvo indicación explícita.

---

## 4. Arquitectura runtime

### 4.1 Cadena de control (de la decisión al actuador)

```text
Nav2 (controller_server de nav2)
   -> /cmd_vel
nav2_collision_monitor
   -> /cmd_vel_safe                (frena/ralentiza por polígonos + /scan)
navegacion_gps/nav_command_server  (arbitra auto vs manual web)
   -> /cmd_vel_final  (interfaces/msg/CmdVelFinal)
controller_server/controller_server_node
   -> UART al firmware (real)  |  backend sim_gazebo -> /cmd_vel_gazebo (sim)
```

Control manual web (paralelo):

```text
map_tools/web_zone_server  -> /cmd_vel_teleop (CmdVelFinal)
   -> nav_command_server (modo manual) -> /cmd_vel_final -> controller_server
```

Detalle de arbitraje, límites y failsafes: [NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md).

### 4.2 Percepción LiDAR

```text
RS16 (rslidar_sdk_node, frame lidar_link)
   -> /scan_3d (PointCloud2)
pointcloud_to_laserscan (target_frame base_footprint)
   -> /scan (LaserScan)
[opcional] scan_noise_filter -> /scan_clean   (default ON en real)
   -> Nav2 costmaps + collision_monitor
```

Ramas: `scan_ground_filter` es default on en `real_global_v2`
(segmentación de suelo estilo Autoware → `/scan_3d/no_ground`);
`lidar_obstacle_filter` queda como alternativa default off (RANSAC 3D →
`/scan_filtered`).
Detalle completo: [SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md).

### 4.3 Localización (perfil real MAVROS)

```text
Pixhawk 6X + F9P --MAVROS--> /imu/data, /global_position/raw/fix,
                             /local_position/velocity_local, /local_position/odom
controller_server -> /controller/drive_telemetry (velocidad/steer medidos)
navegacion_gps/ackermann_odometry -> EKF local (odom)   -> /odometry/local
navsat_transform + EKF global (map)                      -> /odometry/global, /odometry/gps
TF resultante:  map -> odom -> base_footprint
```

Heading GPS por rumbo: `navegacion_gps/gps_course_heading` (`/gps/course_heading`),
opcional y RTK-gated. Medida absoluta GPS→map: `map_gps_absolute_measurement`.

Árbol de tópicos y TF completo: [TOPICOS_Y_TF](TOPICOS_Y_TF.md).

### 4.4 Capa web / misiones

```text
map_tools/web_zone_server (WebSocket :8766)
  <-> navegacion_gps/zones_manager        (no-go / keepout)
  <-> navegacion_gps/nav_command_server   (metas LL, manual, brake)
  <-> navegacion_gps/route_executor       (misiones multi-waypoint)
  <-> navegacion_gps/nav_snapshot_server  (capas para la UI)
```

---

## 5. Entry points y archivos clave

| Qué | Dónde |
|---|---|
| Ejecutables `navegacion_gps` | `src/navegacion_gps/setup.py` (`console_scripts`) |
| Ejecutable control | `src/controller_server/setup.py` → `controller_server_node` |
| Ejecutables sensores | `src/sensores/setup.py` |
| Ejecutable web zonas | `src/map_tools/setup.py` → `web_zone_server` |
| Interfaces (msg/srv) | `src/interfaces/CMakeLists.txt` |
| Launch real canónico | `src/navegacion_gps/launch/real_global_v2.launch.py` |
| Launch sim canónico | `src/navegacion_gps/launch/sim_global_v2.launch.py` |
| Nav2 + collision_monitor | `src/navegacion_gps/launch/nav_global_v2.launch.py` |
| Localización global | `src/navegacion_gps/launch/localization_global_v2.launch.py` |
| Params Nav2 (real) | `src/navegacion_gps/config/nav2_global_v2_real_rolling_params.yaml` |
| Collision monitor | `src/navegacion_gps/config/collision_monitor_v2.yaml` |

Detalle por launch: [LAUNCHES_Y_OPERACION](LAUNCHES_Y_OPERACION.md).

---

## 6. Operación Docker (resumen)

```bash
# levantar contenedor (perfil salus)
./tools/up-salus.sh
# compilar
./tools/compile-ros.sh                       # todo
./tools/compile-ros.sh navegacion_gps        # un paquete
# entrar
./tools/exec.sh
# bajar
./tools/down-salus.sh
```

El contenedor se llama `ros2_salus`, workspace interno `/ros2_ws`. Ver
[LAUNCHES_Y_OPERACION](LAUNCHES_Y_OPERACION.md) y [DEUDA_TECNICA](DEUDA_TECNICA.md).
