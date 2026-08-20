# Referencia integral del codebase ROS2_SALUS

Estado: actual al commit `36adaed` (`main`), auditado el 2026-08-16.

Alcance: inventario de paquetes, composición runtime, contratos públicos, perfiles operativos, herramientas y límites conocidos del checkout actual.

Fuente de verdad: `package.xml`, `CMakeLists.txt`, `setup.py`, código bajo `src/`, launches global V2, YAML instalados y `tools/`. La memoria histórica y otros documentos se usan solo como contexto.

## 1. Resumen del sistema

ROS2_SALUS es el monorepo ROS 2 Humble del vehículo SALUS, un cuatriciclo Ackermann autónomo. Integra:

- Nav2 para planificación, seguimiento, behaviors y collision monitor;
- `robot_localization` para la capa local `odom` y la global `map`;
- Pixhawk/F9P por MAVROS, telemetría de ruedas/dirección y RTK;
- LiDAR RoboSense RS16 con proyección 3D -> 2D y filtros de suelo/ruido;
- actuación por UART a ESP32 o backend equivalente en Gazebo;
- misiones geográficas, patrulla, HOME, acciones por waypoint y retorno por batería;
- WebSocket de operación para Cockpit;
- cámara PTZ y pipeline YOLO opcional.

La ejecución normal ocurre en el contenedor `ros2_salus`, con `/ros2_ws/src`, `tools`, `build`, `install` y `log` montados desde el checkout.

## 2. Paquetes Colcon

El comando `colcon list` detecta nueve paquetes.

| Paquete | Build | Responsabilidad | Entry points principales |
|---|---|---|---|
| `interfaces` | `ament_cmake` + rosidl | Contratos tipados entre navegación, web, sensores y control | mensajes/servicios, sin nodos |
| `controller_server` | `ament_python` | `/cmd_vel_final` -> actuación UART/Gazebo; telemetría y batería | `controller_server_node` |
| `map_tools` | `ament_python` | WebSocket operativo, persistencia y puente ROS/UI | `web_zone_server` |
| `navegacion_gps` | `ament_python` | Bringup Nav2/localización, arbitraje, misiones, LiDAR y observabilidad | 36 `console_scripts` |
| `navegacion_gps_bt` | `ament_cmake` C++ | Plugins BT propios para clearance y trazas | dos shared libraries Nav2 |
| `sensores` | `ament_python` | MAVROS/RTK, cámara PTZ, dashboard; Pixhawk propio legacy | `mavros_compat_bridge`, `rtk_bridge`, `rtk_source_manager`, `sensores_web`, `camara`, `pixhawk_driver` |
| `vision_pipeline` | `ament_python` | Cámara USB/IP, YOLO ONNX, detecciones y visualizador HTTP | `ip_camera_publisher`, `yolo_onnx_detector`, `vision_web_server` |
| `rslidar_msg` | `ament_cmake` | Mensajes RoboSense vendorizados | upstream |
| `rslidar_sdk` | `ament_cmake` | Driver/decoder RS16 vendorizado | `rslidar_sdk_node` |

`src/lidar_camara/` está vacío y no es un paquete. Si aparece en un error de launch, el culpable probable es un overlay `build/install` viejo.

## 3. Contratos de `interfaces`

### Mensajes

- `CmdVelFinal`: `Twist`, porcentaje de freno y source `UNKNOWN/AUTO/MANUAL/SAFETY`.
- `DriveTelemetry`: disponibilidad/freshness, enable/estop/reverse, velocidad y steer medidos, freno y fuente de control.
- `BatteryMissionGuard`: señales filtradas de batería, freshness, tracción, recomendación de HOME, thresholds/persistencia y modelo.
- `NavTelemetry`: actividad de goal/manual, acción Nav2, command freshness, estado de colisión, pose/GPS y último resultado/falla.
- `NavEvent`: evento numerado, severidad, componente, código, texto y key/value details.
- `NavSnapshotLayers`: capas incluidas en una imagen de diagnóstico.
- `NoGoPoint` y `NoGoZone`: geometría geográfica y estado de zonas.

### Servicios

- Navegación simple: `SetNavGoalLL`, `CancelNavGoal`, `GetNavState`, `BrakeNav`, `SetManualMode`, `SetManualCmd`.
- Ruta: `SetRouteMissionLL`, `CancelRouteMission`, `GetRouteMissionState`.
- Patrulla: `SetPatrolMissionLL`, `CancelPatrolMission`, `GetPatrolMissionState`, `RequestReturnHome`.
- Tuning: `SetNavigationProfile` (`urban`/`rural`).
- Batería sim: `SetSimBatteryPreset`, `SetSimBatteryState`.
- Zonas: `SetZonesGeoJson`, `GetZonesState`, `SetKeepoutZones`, `GetKeepoutState`.
- Snapshot: `GetNavSnapshot`.
- Datum legacy: `SetDatum`, `GetDatum`.
- Cámara: `CameraPan`, `CameraStatus`, `CameraPtz`, `CameraPtzState`, `CameraPreset`, `CameraSavePreset`.

Cambiar un `.msg` o `.srv` requiere recompilar `interfaces` antes de sus consumidores. Un import ausente desde `install/interfaces` suele indicar que el overlay no fue regenerado.

## 4. Cadena de control

```text
Nav2 controller_server
  publishes geometry_msgs/Twist /cmd_vel
        |
        v
nav2_collision_monitor
  publishes geometry_msgs/Twist /cmd_vel_safe
        |
        v
navegacion_gps/nav_command_server
  also consumes interfaces/CmdVelFinal /cmd_vel_teleop
  publishes interfaces/CmdVelFinal /cmd_vel_final
        |
        v
controller_server/vehicle_controller_server
  backend uart -> protocolo ESP32
  backend sim_gazebo -> /cmd_vel_gazebo
```

### `nav_command_server`

Responsabilidades verificadas en código:

- convierte goals geográficos con `/fromLL` y fallback configurable;
- usa actions Nav2 `follow_waypoints`, `navigate_through_poses` y `backup`;
- arbitra automático, manual y freno;
- cancela navegación ante takeover manual;
- vigila freshness de `/cmd_vel_safe` y comandos teleop;
- puede aplicar freno sostenido y recovery backup si se habilita;
- publica `/nav_command_server/telemetry` y `/nav_command_server/events`;
- expone servicios bajo `/nav_command_server/*`.

Defaults de interés: timeout manual `0.4 s`, telemetría `5 Hz`, freno crítico habilitado, backup de colisión deshabilitado y fallback aproximado `/fromLL` deshabilitado.

### `controller_server`

El nodo propio se renombra `vehicle_controller_server` en los perfiles globales para no colisionar con el nodo Nav2 homónimo. Declara límites independientes para dirección automática/manual, watchdog automático (`0.7 s` por default), backend UART o sim, y publica:

- `/controller/status` JSON;
- `/controller/telemetry` JSON;
- `/controller/drive_telemetry` tipado;
- `/battery_state`;
- `/battery_mission_guard`.

El backend sim sintetiza velocidad, steer y batería desde odometría/joints y servicios de inyección. El backend real resuelve puerto serial explícito, variable `SALUS_CONTROLLER_SERIAL_PORT`, `by-id`, `ttyUSB*` o `/dev/serial0`.

## 5. Localización y frames

### Capa local

`controller_server` publica `DriveTelemetry`; `ackermann_odometry` usa velocidad/steer medidos y geometría Ackermann para publicar `/wheel/odometry` y `/vehicle/twist`. `localization_v2.launch.py` alimenta el EKF local y produce `/odometry/local` y `odom -> base_footprint`.

### Capa global

Los perfiles global V2 agregan GNSS/RTK e IMU desde MAVROS, conversión geográfica `fromLL`, medida `/gps/odometry_map`, `navsat_transform`, heading de curso y EKF global. La salida incluye `/odometry/gps`, `/odometry/global` y `map -> odom`.

```text
map
 └─ odom
     └─ base_footprint
         ├─ base_link/chasis
         ├─ lidar_link
         ├─ imu_link
         └─ gps_link
```

El datum operativo es fijo por sitio. `datum_setter` y sus servicios se conservan por compatibilidad, pero no son el camino normal de los perfiles globales.

### Heading

- `gps_course_heading`: referencia absoluta mientras el robot avanza con gates de distancia, velocidad, steer y RTK.
- `compass_heading_gate`: ayuda débil de startup/reposo desde `/mavros_node/compass_hdg`; se bloquea cuando el course heading GPS es válido y queda deshabilitada/fuera del EKF salvo flags explícitos.
- `global_odom_stationary_gate`, `global_imu_stationary_gate` y `global_yaw_stationary_hold`: estabilizadores de la capa global.

Diagnóstico: un giro visual del local costmap en RViz con Fixed Frame `map` puede ser la corrección `map -> odom`, no una rotación real de la nube en el frame local.

## 6. LiDAR y collision monitor

```text
RS16 rslidar_sdk_node
  -> /scan_3d (PointCloud2, lidar_link)
  -> [scan_ground_filter] /scan_3d/no_ground
  -> pointcloud_to_laserscan (target base_footprint)
  -> /scan
  -> [scan_noise_filter] /scan_clean
  -> Nav2 costmaps + nav2_collision_monitor
```

Ramas adicionales:

- `lidar_obstacle_filter`: RANSAC/ROI/densidad 3D, publica `/scan_filtered` y `/obstacles_cloud`.
- `scan_wifi_debug`: LaserScan diezmado para visualización remota; nunca sustituye el scan local de navegación.
- `scan_ground_validation`: medición A/B en worlds de rampa.

El scan efectivo se decide en launch y se inyecta tanto en costmaps como collision monitor. Revisar `real_global_v2.launch.py`/`sim_global_v2.launch.py` y el `RewrittenYaml` de `nav_global_v2.launch.py`, no solo el nombre de un YAML.

Para probes usar QoS sensor-data. La build histórica del SDK usa punto XYZI sin ring/timestamp por punto expuesto; no asumir que se puede deskewear sin cambiar el formato y la cadena completa.

## 7. Ruta, patrulla y perfiles de navegación

`route_executor.py` prepara waypoints geográficos, interpola tramos largos, divide la misión en chunks y mantiene un path de debug (`/route_executor/mission_path` y `/route_executor/active_chunk_path`).

### Estado de ruta

Incluye índices de input/expandidos, chunk y loop, progreso geométrico, cross-track/distance, estado bloqueado, retry, acción activa y HOME. Los estados bloqueados son:

- `BLOCKED_WAITING`;
- `BLOCKED_RETRYING`;
- `BLOCKED_NEEDS_OPERATOR`.

El retry puede frenar, esperar, limpiar costmaps y reanclar según pose actual.

### Acciones por waypoint

- `brake_hold`: porcentaje y duración de freno.
- `set_navigation_profile`: conmuta `urban`/`rural`.

Los perfiles cambian dinámicamente inflación local/global y parámetros de pendiente del `scan_ground_filter`. Urban es el perfil restaurado por defecto; rural reduce inflación y permite umbrales de suelo más permisivos sin apagar detección.

### Patrulla estructurada

Una patrulla contiene:

- loop principal;
- HOME;
- conector de retorno;
- conector de salida/depart;
- índice de reentrada al loop.

Fases: `idle`, `depart_home`, `loop_main`, `return_pending`, `return_connector`, `parked_home`. El retorno puede pedirse por operador o por `/battery_mission_guard`.

Servicios:

- `/route_executor/set_route_ll`, `cancel_route`, `get_state`;
- `/route_executor/set_patrol_ll`, `cancel_patrol`, `get_patrol_state`, `request_return_home`;
- `/route_executor/set_navigation_profile`.

## 8. Batería

`BatteryEstimator` mantiene señales con distintas constantes de tiempo:

- voltaje raw;
- loaded fast/slow;
- recovered;
- SOC de operador con curva configurable y filtro de descarga;
- guardia de misión por persistencia bajo carga o en reposo.

Separar siempre presentación y seguridad:

- `/battery_state.percentage`: indicador suave para UI.
- `/battery_mission_guard.return_home_recommended`: decisión de misión.

En simulación los presets `full`, `under_load`, `watching`, `return_home_rest`, `return_home_load`, `stale`, `suspect`, `unavailable` atraviesan el mismo estimador.

## 9. WebSocket y Cockpit

`map_tools/web_zone_server.py` es el bridge de alto nivel, por default `ws://0.0.0.0:8766`. Consume sensores, navegación, batería, controller, diagnostics, rosout, BehaviorTreeLog, cámara y detecciones; crea clients a los servicios anteriores y publica teleop.

Dominios de operaciones:

- conexión/estado/control lock;
- zonas y keepout;
- waypoints, datums y fuentes RTK;
- goal, route, patrol, HOME y navigation profile;
- manual/freno;
- snapshots;
- sesiones `mission.*`;
- rosbag;
- cámara/PTZ/presets.

El contrato de sesión de misión comienza con aceptación real (`GOAL_ACCEPTED`), no solo con request. Cualquier cambio de `op`, payload o lifecycle debe revisarse también en el repo independiente `cockpit/`.

## 10. Sensores, cámara y visión

### MAVROS y RTK

`sensores/mavros.launch.py` es el bringup operativo. `rtk_bridge` consolida GPS RAW/RTK/baseline y publica status; `rtk_source_manager` administra fuentes RTCM y metadatos por topics JSON. `mavros_compat_bridge` solo replica contratos legacy cuando se habilita.

`pixhawk_driver.py` conserva una implementación MAVLink propia a 921600 y tópicos legacy. No usarla como base de una conclusión sobre el perfil real actual sin confirmar el launch.

### PTZ

`sensores/camara.py` controla una cámara por ISAPI, con límites y presets configurables. Lee host/user/pass desde `.env`; no versionar ni mostrar esos valores. Los presets editables se persisten en un JSON local.

### YOLO

`vision_pipeline` puede usar `v4l2_camera` o `ip_camera_publisher`, normaliza a `/camera/image_raw`, ejecuta ONNX Runtime y publica `vision_msgs/Detection2DArray` más `/objeto_detectado`. `vision_web_server` sirve imagen + overlay por HTTP. Este pipeline está activo como paquete pero no es requisito para que Nav2 global arranque.

## 11. Behavior Trees propios

`navegacion_gps_bt` compila:

- `nav2_is_path_clearance_valid_condition_bt_node`: condición que consulta/valida clearance del path.
- `nav2_trace_replan_decorator_bt_node`: decorador que emite trazas/diagnósticos de replanificación.

Los XML relevantes están en `config/navigate_*`. Si se cambia un nombre de plugin, puerto BT, topic o servicio, actualizar C++, XML, package dependencies y tests de contrato juntos.

## 12. Herramientas operativas

### Contenedor/build

```bash
./tools/up-salus.sh
./tools/exec.sh
./tools/compile-ros.sh
./tools/compile-ros.sh interfaces controller_server map_tools navegacion_gps navegacion_gps_bt sensores vision_pipeline
```

### Launch

- `launch_sim_global_v2*.sh` y variantes de world.
- `launch_real_global_v2*.sh`, con WiFi como perfil remoto recomendado.
- `launch_*_rviz*.sh` para PC operador.
- `launch_controller.sh`, `launch_no_go_editor.sh`.

### Diagnóstico

- `record_nav_debug_bag.sh`.
- `record_compass_calibration.sh`.
- `run_localization_replay_compare.sh`.
- `run_nav_benchmark.sh`, `compare_nav_benchmarks.sh`.
- `show_latest_nav_trace.sh`, `regenerate_nav_trace_report.sh`.
- `healthcheck-lidar.sh`.
- `sim_battery.sh`.
- `jetson_power_monitor.py`, `jetson_power_report.py`, `jetson_power_mark.py`.

`vcs-pull.sh`, `vcs-push.sh` y `vcs-status.sh` operan sobre el monorepo raíz; no reconstruyen `src/` como repos múltiples.

## 13. Build y tests

El orden seguro tras cambios de interfaces es:

```bash
./tools/compile-ros.sh interfaces controller_server map_tools navegacion_gps navegacion_gps_bt sensores vision_pipeline
```

Luego, dentro del contenedor, ejecutar tests por paquete para evitar colisiones de nombres como `test_copyright.py`:

```bash
python3 -m pytest -q src/controller_server/test
python3 -m pytest -q src/map_tools/test
python3 -m pytest -q src/navegacion_gps/test
python3 -m pytest -q src/sensores/test
```

Para launches/YAML, además usar `ros2 launch ... --show-args` y tests de contrato global V2. Para cambios físicos, los tests no reemplazan la comprobación del robot.

### Resultado de auditoría 2026-08-16

- La build de los siete paquetes propios terminó correctamente después de regenerar el overlay de interfaces.
- `map_tools`: 41 tests passed.
- `sensores`: 14 tests passed.
- `controller_server`: 50 passed y 1 skipped en `test/` excluyendo el chequeo Flake8; 15 tests internos adicionales passed. El chequeo Flake8 conserva cuatro `E501` preexistentes.
- `navegacion_gps`: 325 passed, 1 skipped y 9 fallas en `test_nav_command_server_loop_helper.py`; el fake node del test no acompaña el atributo actual `_nav_action_results_to_ignore`.
- Monitor de potencia Jetson: 6 tests passed.
- No había nodos SALUS ejecutándose durante la auditoría, por lo que estos resultados prueban build/unidad, no comportamiento físico ni una misión end-to-end.

## 14. Fuentes de verdad y prioridad

En caso de contradicción:

1. código del nodo y launch realmente invocado;
2. YAML/URDF instalado por ese launch;
3. tests de contrato y manifests;
4. esta referencia y README actuales;
5. documentos de investigación o memoria histórica;
6. borradores locales no versionados.

El archivo local `docs/ackermann-steer-rate-limit.md` está fuera de Git y describe símbolos/params que no existen en `main` auditado. No es evidencia de una función implementada.

## 15. Riesgos conocidos

- overlay stale tras cambios rosidl;
- nombres duplicados `controller_server` propio/Nav2;
- convenciones MAVROS vs legacy de GPS/RTK/odom;
- defaults keepout/failsafe que deben verificarse por perfil;
- links absolutos viejos `/home/leo` y `/home/leosole`;
- metadatos `TODO` en algunos `package.xml`/`setup.py`;
- vendors grandes que no deben tocarse para arreglar wiring del proyecto;
- credenciales PTZ/RTK que deben permanecer fuera de logs y docs.

## 16. Documentos relacionados

- [Índice](INDEX.md)
- [Catálogo exhaustivo del código propio](CODE_CATALOG_OWN.md)
- [Catálogo de runtime y herramientas](CODE_CATALOG_RUNTIME_TOOLS.md)
- [Catálogo del vendor RoboSense](CODE_CATALOG_VENDOR.md)
- [Catálogo exhaustivo de Cockpit](../cockpit/CODE_CATALOG.md)
- [Arquitectura](ARQUITECTURA_ROS2_SALUS.md)
- [Launches y operación](LAUNCHES_Y_OPERACION.md)
- [Tópicos y TF](TOPICOS_Y_TF.md)
- [Navegación y control](NAVEGACION_Y_CONTROL.md)
- [Sensores y LiDAR](SENSORES_Y_LIDAR.md)
- [Deuda técnica](DEUDA_TECNICA.md)
- [Integración Cockpit](cockpit-integration.md)
- [Paridad sim/real](sim-real-parity.md)
