# Launches y operación — ROS2_SALUS

Estado: documentación read-only del código actual.
Fuente de verdad: `src/**/launch/*.launch.py` y `tools/*.sh`.
**(por confirmar)** marca lo no verificado leyendo el código.

Hermanos: [ARQUITECTURA](ARQUITECTURA_ROS2_SALUS.md) ·
[NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md) ·
[SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md)

---

## 1. Operación Docker

| Acción | Comando | Notas |
|---|---|---|
| Levantar | `./tools/up-salus.sh` | `docker compose -p ros2_salus -f docker-compose.yml -f docker-compose.salus.yml up -d --build` |
| Bajar | `./tools/down-salus.sh` | idem `down` |
| Compilar todo | `./tools/compile-ros.sh` | `colcon build --symlink-install` dentro de `ros2_salus` |
| Compilar paquetes | `./tools/compile-ros.sh navegacion_gps sensores …` | `--packages-select` |
| Shell en contenedor | `./tools/exec.sh` | fija `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Shell root | `./tools/root-exec.sh` | |

- Contenedor: `ros2_salus`. Workspace interno: `/ros2_ws`.
- RMW: **CycloneDDS** por defecto (servicios/acciones Nav2 estables).
- Si un launch aborta con `package '...' not found`, recompilar limpio el overlay
  (`rm -rf build install log && colcon build --symlink-install`) — ver
  [DEUDA_TECNICA](DEUDA_TECNICA.md) 5.5.

---

## 2. Launches canónicos

### 2.1 `navegacion_gps/real_global_v2.launch.py`  — robot real (mainline)

`src/navegacion_gps/launch/real_global_v2.launch.py`. Es el orquestador grande
(`use_sim_time:=False`).

Levanta / incluye:

| Componente | Cómo | Notas |
|---|---|---|
| `robot_state_publisher` | OpaqueFunction | URDF `models/cuatri_real_v2.urdf` |
| Sensores Pixhawk/GNSS | include `sensores/mavros.launch.py` | `launch_legacy_compat=false`, `enable_rtk` efectivo, `fcu_url=/dev/ttyACM0:921600` |
| LiDAR | include `sensores/rs16.launch.py` | `config_path=sensores/config/rs16.yaml` |
| `pointcloud_to_laserscan` | Node | params `pointcloud_to_laserscan_real.yaml`, remap `cloud_in:=/scan_3d`, `scan:=/scan` |
| `scan_noise_filter` | Node (IfCondition) | `enable_scan_noise_filter` default **True** → `/scan_clean` |
| `lidar_obstacle_filter` | Node (IfCondition) | `enable_lidar_obstacle_filter` default **False** (RANSAC) |
| `controller_server_node` | Node (nombre `vehicle_controller_server`) | UART `/dev/serial0@115200` |
| `gps_course_heading` | Node (IfCondition) | default **True**, RTK-gated |
| `nav_command_server` | Node | arbitraje + servicios |
| `route_executor` | Node | misiones multi-waypoint |
| `nav_observability` | Node | diagnóstico |
| `scan_wifi_debug` | Node (IfCondition) | default **True** |
| Localización global | include `localization_global_v2.launch.py` | EKF dual + navsat + datum |
| Nav2 + collision_monitor | include `nav_global_v2.launch.py` (TimerAction `nav_start_delay_s=3.0`) | scan efectivo inyectado |
| Web/zonas | include `map_tools/no_go_editor.launch.py` (IfCondition `launch_web_app`) | sin re-lanzar nav_command_server/route_executor |
| RViz | Node (IfCondition `use_rviz` default False) | |

Args destacados (defaults reales):

| Arg | Default | Efecto |
|---|---|---|
| `use_keepout` | `False` | keepout deshabilitado mientras se estabiliza el costmap 300×300 m |
| `enable_rtk` | `True` | cadena RTK observable por default |
| `enable_gps_course_heading` | `True` | heading por rumbo GPS |
| `enable_scan_noise_filter` | `True` | scan efectivo `/scan_clean` |
| `enable_lidar_obstacle_filter` | `False` | rama RANSAC 3D experimental |
| `launch_web_app` | `True` | UI + servidor web (`web_app_port=8766`) |
| `fcu_url` | `/dev/ttyACM0:921600` | puerto Pixhawk |
| `datum_*` | resueltos por `datum_profile_resolver` | origen ENU |

Supuestos de hardware: Pixhawk en `/dev/ttyACM0`, firmware del vehículo en
`/dev/serial0`, RS16 en red (`rs16.yaml`).

### 2.2 `navegacion_gps/sim_global_v2.launch.py` — simulación (mainline)

`use_sim_time:=True`. Equivalente a real pero con Gazebo en vez de hardware.
**Incluye `sim_v2_base.launch.py`** (Gazebo + bridge + spawn + `pointcloud_to_laserscan`),
`world` default `vacio.world`, `model_name=quad_ackermann_viewer_safe`. El controller
usa backend `sim_gazebo` (`/cmd_vel_gazebo`, `/odom_raw`, joint_states),
`use_keepout` default **True**. Entrada recomendada por wrapper:
`tools/launch_sim_global_v2_wifi.sh`.

> Corrección de auditoría: **no** incluye `simulacion.launch.py` (eso es el perfil
> legado), sino `sim_v2_base.launch.py` (`sim_global_v2.launch.py:332`).

Lista exacta de nodos/includes (auditada):
`sim_sensor_normalizer_v2`, include `sim_v2_base`, `scan_noise_filter` (default on),
`lidar_obstacle_filter` (opcional), `vehicle_controller_server`, `nav_command_server`,
`route_executor`, `nav_observability`, `gps_course_heading`, include
`localization_global_v2`, include demorado `nav_global_v2`, include `no_go_editor`.

### 2.3 `navegacion_gps/rviz_real_global_v2.launch.py`

Lanza RViz con `config/rviz_global_v2.rviz` para el perfil real global.
**(por confirmar si además incluye el stack o solo RViz.)** Hay variante WiFi
`rviz_real_global_v2_wifi.launch.py`.

### 2.4 `sensores/rs16.launch.py`

`src/sensores/launch/rs16.launch.py`. Levanta `rslidar_sdk_node` con
`config_path` (default `sensores/config/rs16.yaml`). Args: `use_cyclone_dds`
(fuerza CycloneDDS), `rviz`. Publica `/scan_3d` (frame `lidar_link`).
`/rslidar_packets` está configurado pero **no se publica por default**
(`send_packet_ros: false`, `rs16.yaml`).

### 2.5 `sensores/pixhawk.launch.py` — **LEGADO**

Levanta el driver propio `pixhawk_driver` (contrato `/imu/data`, `/gps/fix`,
`/odom`, `/velocity`, `/gps/rtk_status`, …) + opcional `sensores_web`.
**No** se usa en `real_global_v2` (que usa MAVROS). Útil como referencia o banco.

> El sensor mainline es MAVROS: `ros2 launch sensores mavros.launch.py`
> (`mavros_node` + `mavros_compat_bridge` + `rtk_bridge`/`rtk_source_manager`).

### 2.6 `controller_server/controller_server.launch.py`

`src/controller_server/launch/controller_server.launch.py`. Lanza
`controller_server_node` **con nombre `controller_server`** y UART `/dev/serial0`.
Útil para banco de actuación aislado.

> Cuidado: ese nombre **colisiona** con el `controller_server` de Nav2. En
> `real_global_v2` el nodo propio se renombra `vehicle_controller_server` para
> evitarlo. Ver [DEUDA_TECNICA](DEUDA_TECNICA.md).

### 2.7 `map_tools/no_go_editor.launch.py`

`src/map_tools/launch/no_go_editor.launch.py`. Servidor web + nodos de zonas:
`zones_manager`, `nav_command_server`, `nav_snapshot_server`, `route_executor`,
`web_zone_server` (WebSocket `ws_host:ws_port`, default `0.0.0.0:8766`). Cada nodo
es condicional (`launch_*`). En `real_global_v2` se incluye con
`launch_nav_command_server=false` y `launch_route_executor=false` (ya los levanta
el orquestador). Defaults de tópicos GPS aquí: `/gps/fix`, `/gps/rtk_status`
(el orquestador los sobrescribe a los de MAVROS).

---

## 3. Launches incluidos (segundo nivel)

| Launch | Rol |
|---|---|
| `nav_global_v2.launch.py` | Nav2 (planner, controller, smoother, bt_navigator, behavior, waypoint_follower), `collision_monitor`, republishers de polígonos, dos `lifecycle_manager` (Nav2 + collision_monitor). Inyecta el scan efectivo y selecciona overrides keepout vía `RewrittenYaml`. |
| `localization_global_v2.launch.py` | EKF local + EKF global + navsat_transform + datum + (opcional) heading GPS. |
| `keepout_filters_v2.launch.py` | costmap_filter de keepout (máscara `keepout_mask.yaml`). Solo con `use_keepout:=True`. |
| `localization_v2.launch.py` | localización local (perfiles `*_local_v2`). |

---

## 4. `tools/` por categoría

| Categoría | Scripts |
|---|---|
| Docker up/down/compile/exec | `up-salus.sh`, `down-salus.sh`, `compile-ros.sh`, `exec.sh`, `root-exec.sh`, `docker_ros_env.sh` |
| Launch real | `launch_real_global_v2.sh`, `launch_real_global_v2_rviz.sh`, `launch_real_global_v2_wifi*.sh` |
| Launch sim | `launch_sim_global_v2.sh`, `launch_sim_global_v2_wifi*.sh` (incl. `_slope`, `_sonoma`) |
| Launch infra | `launch_controller.sh`, `launch_no_go_editor.sh` |
| Stop sim | `stop_sim_global_v2.sh`, `stop_sim_local_v2.sh` |
| Healthcheck | `healthcheck-lidar.sh` (hz de `/scan_3d`, `/scan`, `/scan_clean`, `/scan_filtered`, TF) |
| Benchmarks | `run_nav_benchmark.sh`, `compare_nav_benchmarks.sh`, `generate_block_loop_benchmark.sh` |
| Replay / diagnóstico | `record_nav_debug_bag.sh`, `run_localization_replay_compare.sh`, `check_startup_heading.sh`, `send_follow_path_v2.sh`, `closed_loop_step_publisher.py`, `uart_step_sender.py` |
| Legado | `launch_real_nav.sh`, `launch_real_rviz.sh`, `launch_sim_local_v2.sh`, `launch_real_local_v2.sh` |
| VCS | `vcs-pull.sh`, `vcs-push.sh`, `vcs-status.sh` (⚠ `README` desaconseja vcstool en este checkout) |

Healthcheck LiDAR:
```bash
./tools/healthcheck-lidar.sh                 # contenedor ros2_salus
```

---

## 5. Comandos reproducibles

```bash
# Real (mainline)
ros2 launch navegacion_gps real_global_v2.launch.py
ros2 launch navegacion_gps real_global_v2_wifi.launch.py     # recomendado WiFi

# Simulación
ros2 launch navegacion_gps sim_global_v2.launch.py
./tools/launch_sim_global_v2_wifi.sh

# Piezas sueltas
ros2 launch sensores mavros.launch.py
ros2 launch sensores rs16.launch.py
ros2 launch controller_server controller_server.launch.py
ros2 launch map_tools no_go_editor.launch.py

# Validación filtro de suelo (POC, ver SENSORES_Y_LIDAR)
ros2 launch navegacion_gps validate_scan_ground.launch.py enable_scan_ground_filter:=True
```
