# Catálogo exhaustivo del código propio ROS2_SALUS

Estado: auditado archivo por archivo contra `main` local el 2026-08-16.

Alcance: 339 archivos de texto auditados en los siete paquetes propios: Python/C++, interfaces, launches, YAML/XML/GeoJSON, URDF/worlds/RViz, packaging, marcadores Ament, tests y los 33 WSDL/XSD externos empacados por `sensores`. Se excluyeron README/AGENTS, imágenes, modelo ONNX, PGM raw y frontend HTML/CSS/JavaScript. Los 138 archivos RoboSense se describen aparte en `CODE_CATALOG_VENDOR.md`; la selección exacta está en `BACKEND_LINE_MANIFEST.tsv`.

Fuente de verdad: cada archivo listado. Este catálogo explica responsabilidad y relaciones; la referencia arquitectónica sigue en `CODEBASE_REFERENCE.md`.

## 1. `interfaces` — 40 archivos

### Build

- `src/interfaces/CMakeLists.txt`: declara generación rosidl para todos los mensajes/servicios y dependencias.
- `src/interfaces/package.xml`: manifiesto ament/rosidl y dependencias de tipos ROS.
- `src/interfaces/resource/interfaces`: marcador vacío del índice Ament.

### Mensajes

| Archivo | Contrato |
|---|---|
| `src/interfaces/msg/BatteryMissionGuard.msg` | Estado de batería orientado a misión: frescura, tracción, voltajes cargado/recuperado, SOC, thresholds y recomendación de HOME. |
| `src/interfaces/msg/CmdVelFinal.msg` | Twist arbitrado más freno porcentual y fuente `AUTO/MANUAL/SAFETY`. |
| `src/interfaces/msg/DriveTelemetry.msg` | Telemetría del actuador: ready/fresh, enable/estop/reversa, velocidad y dirección medidas, fuente y freno. |
| `src/interfaces/msg/NavEvent.msg` | Evento estructurado con severidad, componente, código, texto, id y detalles key/value. |
| `src/interfaces/msg/NavSnapshotLayers.msg` | Banderas de capas incluidas en una captura de navegación. |
| `src/interfaces/msg/NavTelemetry.msg` | Estado compacto de goal/manual, comandos, collision stop, pose/GPS y último resultado/falla. |
| `src/interfaces/msg/NoGoPoint.msg` | Punto geográfico lat/lon de una zona. |
| `src/interfaces/msg/NoGoZone.msg` | Zona identificada, tipada/habilitada y compuesta por polígono geográfico. |

### Servicios

| Archivo | Contrato |
|---|---|
| `src/interfaces/srv/BrakeNav.srv` | Aplica freno por duración y porcentaje. |
| `src/interfaces/srv/CameraPan.srv` | Orden absoluta de paneo con ángulo aplicado. |
| `src/interfaces/srv/CameraPreset.srv` | Ejecuta preset PTZ y devuelve pose aplicada. |
| `src/interfaces/srv/CameraPtz.srv` | Movimiento PTZ absoluto/relativo por ejes seleccionados. |
| `src/interfaces/srv/CameraPtzState.srv` | Consulta estado PTZ extendido. |
| `src/interfaces/srv/CameraSavePreset.srv` | Persiste preset y opcionalmente zoom. |
| `src/interfaces/srv/CameraStatus.srv` | Consulta último comando, zoom, pose y preset activo. |
| `src/interfaces/srv/CancelNavGoal.srv` | Cancela el goal Nav2 vigente. |
| `src/interfaces/srv/CancelPatrolMission.srv` | Cancela patrulla estructurada. |
| `src/interfaces/srv/CancelRouteMission.srv` | Cancela misión de ruta. |
| `src/interfaces/srv/GetDatum.srv` | Consulta datum, GPS actual, origen de set y RTK. |
| `src/interfaces/srv/GetKeepoutState.srv` | Consulta máscara keepout y zonas tipadas. |
| `src/interfaces/srv/GetNavSnapshot.srv` | Devuelve PNG, metadatos y capas de la captura. |
| `src/interfaces/srv/GetNavState.srv` | Consulta estado goal/manual, comandos y posición geográfica. |
| `src/interfaces/srv/GetPatrolMissionState.srv` | Estado completo de patrulla HOME/loop/return/depart y tramo activo. |
| `src/interfaces/srv/GetRouteMissionState.srv` | Estado detallado de ruta/chunks/progreso/bloqueo/acciones/HOME. |
| `src/interfaces/srv/GetZonesState.srv` | Consulta GeoJSON y estado de máscara. |
| `src/interfaces/srv/RequestReturnHome.srv` | Solicita transición controlada a HOME. |
| `src/interfaces/srv/SetDatum.srv` | Define datum explícito o desde GPS actual y reporta lo aplicado. |
| `src/interfaces/srv/SetKeepoutZones.srv` | Publica conjunto tipado de zonas. |
| `src/interfaces/srv/SetManualCmd.srv` | Comando manual lineal/angular. |
| `src/interfaces/srv/SetManualMode.srv` | Activa/desactiva arbitraje manual. |
| `src/interfaces/srv/SetNavGoalLL.srv` | Goal o lista lat/lon/yaw, loop y supresión de freno de éxito. |
| `src/interfaces/srv/SetNavigationProfile.srv` | Selecciona perfil `urban`/`rural`. |
| `src/interfaces/srv/SetPatrolMissionLL.srv` | Programa loop, HOME, conectores return/depart, acciones y chunking. |
| `src/interfaces/srv/SetRouteMissionLL.srv` | Programa ruta geográfica con acciones/roles y chunking. |
| `src/interfaces/srv/SetSimBatteryPreset.srv` | Aplica preset de batería simulada. |
| `src/interfaces/srv/SetSimBatteryState.srv` | Inyecta voltajes/estado de batería simulada. |
| `src/interfaces/srv/SetZonesGeoJson.srv` | Valida/aplica GeoJSON y devuelve conteos/reload. |

## 2. `controller_server` — 33 archivos

### Runtime y protocolo

| Archivo | Responsabilidad |
|---|---|
| `src/controller_server/controller_server/__init__.py` | Marca el paquete Python. |
| `src/controller_server/controller_server/controller_server_node.py` | Nodo principal: arbitra `CmdVelFinal`, configura backend UART/sim, publica estado/telemetría/batería y servicios de batería simulada. |
| `src/controller_server/controller_server/control_logic.py` | Funciones puras de clamp, Ackermann, límites físicos/operativos y selección auto/manual segura. |
| `src/controller_server/controller_server/battery_estimator.py` | Curva voltaje→SOC, EMAs cargado/recuperado, persistencia/histéresis y estado mission guard. |
| `src/controller_server/controller_server/serial_port_resolver.py` | Resuelve puerto explícito/env/by-id/ttyUSB/serial0 y rechaza ambigüedad. |
| `src/controller_server/controller_server/transport_backends.py` | Factory de backend y adaptador UART. |
| `src/controller_server/controller_server/sim_gazebo_backend.py` | Traduce comando a Gazebo, estima dirección física y sintetiza telemetría/batería con los mismos contratos. |
| `src/controller_server/controller_server/rpy_esp32_comms/__init__.py` | API del subpaquete de comunicación ESP32. |
| `src/controller_server/controller_server/rpy_esp32_comms/__main__.py` | Entrada `python -m` hacia la CLI. |
| `src/controller_server/controller_server/rpy_esp32_comms/cli.py` | CLI interactiva/watch con logging de sesión y formato de telemetría. |
| `src/controller_server/controller_server/rpy_esp32_comms/controller.py` | `CommandState` con clamps y estado de orden a transmitir. |
| `src/controller_server/controller_server/rpy_esp32_comms/protocol.py` | CRC8-Maxim, encode Pi, decode control/batería y parser incremental con resincronización. |
| `src/controller_server/controller_server/rpy_esp32_comms/telemetry.py` | Dataclasses/enums de telemetría y fuente de control. |
| `src/controller_server/controller_server/rpy_esp32_comms/transport.py` | Cliente serial, threads/estadísticas, TX periódico y parsing RX. |
| `src/controller_server/controller_server/controller/artifacts/run_uart_e2e.py` | Runner E2E UART/telnet que captura snapshots y resultados. |
| `src/controller_server/controller_server/controller/artifacts/uart_e2e_results.json` | Resultado versionado de una corrida E2E; evidencia histórica, no test vivo. |

### Launch y packaging

- `src/controller_server/launch/controller_server.launch.py`: expone puerto/baud/frecuencia y lanza `controller_server_node`.
- `src/controller_server/setup.py`: instala launch, dependencia pyserial y entry point del nodo; conserva metadata TODO.
- `src/controller_server/setup.cfg`: rutas de scripts ament.
- `src/controller_server/package.xml`: dependencias ROS y test.
- `src/controller_server/.gitignore`: exclusiones locales del paquete.
- `src/controller_server/resource/controller_server`: marcador vacío del índice Ament.
- `src/controller_server/controller_server/controller/pytest.ini`: configuración de pytest del submódulo UART.
- `src/controller_server/controller_server/controller/requirements.txt`: dependencia Python del submódulo UART.

### Tests

| Archivo | Cobertura |
|---|---|
| `src/controller_server/controller_server/controller/tests/test_controller.py` | Clamps de estado y reset seguro. |
| `src/controller_server/controller_server/controller/tests/test_protocol.py` | 13 casos de frames, CRC, sentinels, stream mixto, reversa y hazard. |
| `src/controller_server/test/test_battery_estimator.py` | Curva SOC, sag cargado, persistencias, histéresis y prioridades de estado. |
| `src/controller_server/test/test_control_logic.py` | 20 casos de escala, reversa, deadband, mínimos, Ackermann, límites y timeout. |
| `src/controller_server/test/test_serial_port_resolver.py` | Prioridad y errores del discovery serial. |
| `src/controller_server/test/test_sim_gazebo_backend.py` | Actuación, dirección, flags, factory, telemetría y presets de batería sim. |
| `src/controller_server/test/test_copyright.py` | Linter ament de copyright. |
| `src/controller_server/test/test_flake8.py` | Linter Flake8; actualmente reporta cuatro `E501`. |
| `src/controller_server/test/test_pep257.py` | Linter de docstrings. |

## 3. `map_tools` — 14 archivos

| Archivo | Responsabilidad |
|---|---|
| `src/map_tools/map_tools/__init__.py` | Marca el paquete. |
| `src/map_tools/map_tools/datum_file_utils.py` | Normaliza ids/nombres/yaw, parsea y persiste `datums.yaml` de forma segura. |
| `src/map_tools/map_tools/waypoints_file_utils.py` | Normaliza waypoints/acciones/patrulla y lee/escribe el YAML canónico. |
| `src/map_tools/map_tools/web_zone_server.py` | Borde ROS/WebSocket: estado, locks, zonas, datums/RTK, goals/rutas/patrulla, HOME/perfiles, snapshots, rosbag, sesiones y cámara. |
| `src/map_tools/launch/no_go_editor.launch.py` | Lanza opcionalmente zones manager, command server, snapshots, route executor y servidor WS. |
| `src/map_tools/.gitignore` | Exclusiones locales del paquete. |
| `src/map_tools/resource/map_tools` | Marcador vacío del índice Ament. |
| `src/map_tools/setup.py` | Instala config/launch/web y expone `web_zone_server`. |
| `src/map_tools/setup.cfg` | Configuración ament de scripts. |
| `src/map_tools/package.xml` | Dependencias ROS/Python del backend. |
| `src/map_tools/test/test_datum_file_utils.py` | Validación y round-trip de datums. |
| `src/map_tools/test/test_waypoints_file_utils.py` | 12 casos de formatos, auto-yaw, acciones, HOME y patrulla. |
| `src/map_tools/test/test_no_go_editor_launch.py` | Contrato de override del tópico odom. |
| `src/map_tools/test/test_web_zone_server.py` | 25 contratos de perfiles, patrulla, diagnósticos, rosbag, RTK, yaws, sesiones y batería. |

### Operaciones WebSocket implementadas por `web_zone_server.py`

Entrantes: `get_state`, `get_rosbag_status`, `set_control_lock`, `control_heartbeat`, `set_sensor_info_view`, `set_zones_geojson`, `load_zones_file`, `save_waypoints_file`, `load_waypoints_file`, `get_datums`, `save_datum`, `delete_datum`, `select_datum`, `select_rtk_source`, `upsert_rtk_source`, `capture_current_gps_datum`, `set_goal_ll`, `set_navigation_profile`, `set_route_ll`, `set_patrol_ll`, `cancel_goal`, `cancel_route`, `cancel_patrol`, `request_return_home`, `brake`, `set_manual_mode`, `set_manual_cmd`, `get_nav_snapshot`, `mission.list_sessions`, `mission.get_session`, `mission.download_session`, `mission.get_status`, `start_rosbag`, `stop_rosbag`, `camera_pan`, `camera_zoom_toggle`, `get_camera_status`, `camera_ptz_move`, `camera_ptz_preset`, `camera_ptz_set_preset` y `get_camera_ptz_state`.

Salientes principales: `ack`, `state`, `nav_telemetry`, `nav_event`, `nav_alerts`, `robot_pose`, `gps_status`, `sensor_info`, `drive_telemetry`, `rosbag_status`, `nav_snapshot`, `camera_frame`, `camera_detections`, `camera_status`, `camera_ptz_state`, `datums` y familia `mission.*`.

## 4. `navegacion_gps`: módulos Python

### Localización, heading y conversión geográfica

| Archivo | Responsabilidad |
|---|---|
| `src/navegacion_gps/navegacion_gps/__init__.py` | Marca el paquete. |
| `src/navegacion_gps/navegacion_gps/ackermann_odometry.py` | Integra telemetría de ruedas/dirección en `/wheel/odometry` y twist con covarianzas/TF configurables. |
| `src/navegacion_gps/navegacion_gps/gps_course_heading_core.py` | Estimador puro de rumbo por desplazamiento GNSS y convención ENU. |
| `src/navegacion_gps/navegacion_gps/gps_course_heading.py` | Nodo que gatea heading GNSS por distancia, velocidad, steer, yaw-rate, edad y calidad RTK. |
| `src/navegacion_gps/navegacion_gps/heading_math.py` | Normalización, distancia angular, yaw de quaternion y media circular. |
| `src/navegacion_gps/navegacion_gps/compass_heading_gate.py` | Convierte compass_hdg NED a yaw ENU y solo publica una orientación aceptable en startup/estacionario. |
| `src/navegacion_gps/navegacion_gps/compass_calibration_recorder.py` | Sincroniza brújula/GPS/IMU/telemetría, filtra muestras y produce reporte JSON/Markdown de bias/confianza. |
| `src/navegacion_gps/navegacion_gps/global_imu_stationary_gate.py` | Cero de velocidad angular global cuando el vehículo está quieto con telemetría fresca. |
| `src/navegacion_gps/navegacion_gps/global_odom_stationary_gate.py` | Cero de twist de odometría global en reposo. |
| `src/navegacion_gps/navegacion_gps/global_yaw_stationary_hold.py` | Publica medición yaw-only en reposo para estabilizar EKF global. |
| `src/navegacion_gps/navegacion_gps/map_gps_absolute_measurement.py` | Convierte fixes vía `/fromLL` en odometría absoluta `map`. |
| `src/navegacion_gps/navegacion_gps/datum_profile_resolver.py` | Resuelve archivo y datum seleccionado desde configuración. |
| `src/navegacion_gps/navegacion_gps/datum_setter.py` | Setter dinámico legacy de datum con preferencia RTK y fallback de servicio. |
| `src/navegacion_gps/navegacion_gps/gps_profiles.py` | Perfiles simulados `ideal`, `f9p_rtk`, `m8n` y custom con ruido/throttle/hold. |
| `src/navegacion_gps/navegacion_gps/replay_localization_compare.py` | Carga bags derivados, alinea series y compara pose/yaw/debug entre baseline y replay. |
| `src/navegacion_gps/navegacion_gps/startup_heading_diagnosis.py` | Captura TF/heading inicial y construye diagnóstico humano/estructurado. |

### Control, misión y seguridad

| Archivo | Responsabilidad |
|---|---|
| `src/navegacion_gps/navegacion_gps/cmd_vel_ackermann_bridge_v2.py` | Traduce desired command a Twist de Gazebo conservando curvatura y límites Ackermann. |
| `src/navegacion_gps/navegacion_gps/goal_pose_to_follow_path_v2.py` | Convierte goal pose local en path Ackermann y envía FollowPath con monitor de separación. |
| `src/navegacion_gps/navegacion_gps/nav_command_server.py` | Autoridad de goal/manual/freno: LL→map, acciones Nav2, arbitraje cmd_vel, collision recovery, loop, eventos y telemetría. |
| `src/navegacion_gps/navegacion_gps/route_executor.py` | Máquina de misión para expansión/chunks, progreso, retries bloqueados, acciones, perfiles, patrulla HOME y batería. |
| `src/navegacion_gps/navegacion_gps/path_clearance_validator.py` | Servicio BT que muestrea costmap/laterales, cachea resultado y emite trace; falla abierto sin costmap válido. |
| `src/navegacion_gps/navegacion_gps/polygon_stamped_republisher.py` | Re-publica polígonos de collision monitor periódicamente para visualización/consumidores. |
| `src/navegacion_gps/navegacion_gps/nav_observability.py` | Consolida frescura/estado de GPS, odom, scan, cadena cmd, collision, command server y controller en `/diagnostics`. |
| `src/navegacion_gps/navegacion_gps/nav_snapshot_server.py` | Renderiza PNG con costmaps, keepout, footprint, zonas, scan, plan, polígonos e inset global. |

### LiDAR

| Archivo | Responsabilidad |
|---|---|
| `src/navegacion_gps/navegacion_gps/scan_ground_filter.py` | Segmentación radial estilo Autoware con perfiles/pendientes y salida `/scan_3d/no_ground`. |
| `src/navegacion_gps/navegacion_gps/lidar_obstacle_filter.py` | Compensa roll/pitch, ajusta suelo, filtra densidad/persistencia y genera cloud/scan de obstáculos. |
| `src/navegacion_gps/navegacion_gps/scan_noise_filter.py` | Quita speckles aislados del LaserScan conservando metadatos. |
| `src/navegacion_gps/navegacion_gps/scan_wifi_debug.py` | Reduce beams/rango/sector y tasa para debugging remoto. |
| `src/navegacion_gps/navegacion_gps/scan_ground_validation.py` | Mide falsos positivos de costmap y episodios slow/stop durante validación. |

### Zonas, simulación y benchmarks

| Archivo | Responsabilidad |
|---|---|
| `src/navegacion_gps/navegacion_gps/zones_geojson_utils.py` | Normaliza Polygon/MultiPolygon, valida coordenadas y rasteriza anillos/huecos. |
| `src/navegacion_gps/navegacion_gps/keepout_mask_utils.py` | Rasteriza núcleos y gradientes exponenciales de keepout. |
| `src/navegacion_gps/navegacion_gps/zones_manager.py` | GeoJSON→map usando fromLL, genera PGM/YAML, recarga map server y limpia global costmap. |
| `src/navegacion_gps/navegacion_gps/gazebo_utils.py` | Normalizador legacy de frames/sensores y cmd_vel, con realismo GPS. |
| `src/navegacion_gps/navegacion_gps/sim_sensor_normalizer_v2.py` | Normaliza IMU/GPS/LiDAR/odom simulados y aplica perfil GNSS V2. |
| `src/navegacion_gps/navegacion_gps/sim_drive_telemetry.py` | Deriva DriveTelemetry desde odom y joints en simulación. |
| `src/navegacion_gps/navegacion_gps/sim_compass_hdg.py` | Sintetiza compass_hdg desde IMU con ruido/bias/offset de spawn. |
| `src/navegacion_gps/navegacion_gps/sim_battery_publisher.py` | Publicador de batería sim legacy con servicio low; el backend moderno vive en controller_server. |
| `src/navegacion_gps/navegacion_gps/loop_waypoint_benchmark_core.py` | Geometría pura y YAML del loop de manzana. |
| `src/navegacion_gps/navegacion_gps/loop_waypoint_benchmark.py` | Nodo que genera/ejecuta el loop geográfico y registra resultado. |
| `src/navegacion_gps/navegacion_gps/nav_benchmarking.py` | Catálogo/selección de escenarios y métricas escalares/angulares comparables. |
| `src/navegacion_gps/navegacion_gps/nav_benchmark_runner.py` | Orquesta escenarios ROS y escribe sesiones benchmark. |
| `src/navegacion_gps/navegacion_gps/nav_benchmark_report.py` | CLI de reporte simple o comparación baseline/candidato. |
| `src/navegacion_gps/navegacion_gps/sim_global_straight_benchmark.py` | Benchmark end-to-end recto de global localization/goal y métricas de deriva. |
| `src/navegacion_gps/navegacion_gps/sim_localization_benchmark.py` | Orquestador de launches/probes/cleanup para benchmarks de localización. |
| `src/navegacion_gps/navegacion_gps/nav_trace_recorder.py` | Graba JSONL/eventos/path/BT y detecta geometría O, intersecciones y ráfagas de replan. |
| `src/navegacion_gps/navegacion_gps/nav_trace_report.py` | Entrada CLI para regenerar resumen de trace. |

## 5. `navegacion_gps`: launches

| Archivo | Composición |
|---|---|
| `src/navegacion_gps/launch/keepout_filters_v2.launch.py` | Map server de máscara, filter-info server y lifecycle keepout. |
| `src/navegacion_gps/launch/localization_v2.launch.py` | Ackermann odometry + EKF local `odom`. |
| `src/navegacion_gps/launch/localization_global_v2.launch.py` | Gates estacionarios, navsat_transform, EKF `map`, medición GPS absoluta y datum legacy opcional. |
| `src/navegacion_gps/launch/nav2_only.launch.py` | Nav2 convencional configurable, collision monitor, keepout, RViz y RSP opcionales. |
| `src/navegacion_gps/launch/nav_local_v2.launch.py` | Planner/controller/smoother/BT/behaviors/waypoints/collision para frame local. |
| `src/navegacion_gps/launch/nav_global_v2.launch.py` | Stack Nav2 global con clearance BT, collision polygons y lifecycle. |
| `src/navegacion_gps/launch/sim_v2_base.launch.py` | Gazebo/ros_gz, spawn, bridges, RSP y pipeline LiDAR base común. |
| `src/navegacion_gps/launch/sim_local_v2.launch.py` | Base sim + normalización + controller sim + localización/Nav2 local + command server. |
| `src/navegacion_gps/launch/sim_global_v2.launch.py` | Base sim + sensores/heading + controller + localización/Nav2 global + misión/backend/trace. |
| `src/navegacion_gps/launch/sim_global_v2_wifi.launch.py` | Wrapper global remoto con params WiFi y scan reducido. |
| `src/navegacion_gps/launch/real_local_v2.launch.py` | RSP, LiDAR, controller UART, sensores, localización/Nav2 local y command server. |
| `src/navegacion_gps/launch/real_global_v2.launch.py` | Stack real vigente: MAVROS/RTK/cámara, LiDAR, controller, dual EKF, Nav2, route executor, WS y observabilidad. |
| `src/navegacion_gps/launch/real_global_v2_wifi.launch.py` | Wrapper del real global con params/scan/viewer adecuados a WiFi. |
| `src/navegacion_gps/launch/replay_localization_global_v2.launch.py` | Reutiliza localización global sobre bag con sim time para comparación. |
| `src/navegacion_gps/launch/rviz_real.launch.py` | RSP + RViz legacy real. |
| `src/navegacion_gps/launch/rviz_real_local_v2.launch.py` | RSP + RViz local real. |
| `src/navegacion_gps/launch/rviz_real_global_v2.launch.py` | RViz global real; puede omitir RSP cuando ya corre en robot. |
| `src/navegacion_gps/launch/rviz_real_global_v2_wifi.launch.py` | Wrapper de viewer global real remoto. |
| `src/navegacion_gps/launch/rviz_sim_global_v2.launch.py` | Viewer global de simulación. |
| `src/navegacion_gps/launch/rviz_sim_global_v2_wifi.launch.py` | Viewer global sim remoto. |
| `src/navegacion_gps/launch/validate_scan_ground.launch.py` | Escenario sim de rampa, pipeline seleccionable, KPI node y TF estático opcional. |
| `src/navegacion_gps/launch/validate_scan_ground_real.launch.py` | Solo el medidor KPI sobre tópicos del robot real. |
| `src/navegacion_gps/launch/real.launch.py` | Bringup histórico general con dual EKF/navsat, editor y opciones legacy. No es el perfil global V2 recomendado. |
| `src/navegacion_gps/launch/simulacion.launch.py` | Bringup histórico monolítico de sim, Gazebo, dual EKF, keepout, editor, collision y viewer. |

## 6. `navegacion_gps`: configuración

### Bridges, DDS y datos persistibles

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/.mapviz_config` | Vista Mapviz en `odom`, tile OSM, GPS y publicación de clicks en `map`. |
| `src/navegacion_gps/config/bridge_config.yaml` | Mapeo ros_gz histórico de clock, joints, odom, IMU, GPS, cmd y LiDAR. |
| `src/navegacion_gps/config/bridge_config_v2.yaml` | Mapeo V2 materializado por mundo/modelo para sim actual. |
| `src/navegacion_gps/config/cyclonedds_lan.xml` | Perfil DDS para LAN cableada. |
| `src/navegacion_gps/config/cyclonedds_wifi.xml` | Perfil DDS para operación WiFi. |
| `src/navegacion_gps/config/datums.yaml` | Documento versionado de datums y selección. |
| `src/navegacion_gps/config/default-waypoints.yaml` | Waypoints/patrulla iniciales del backend. |
| `src/navegacion_gps/config/no_go_zones.geojson` | Fuente geográfica canónica de zonas. |
| `src/navegacion_gps/config/no_go_zones.yaml` | Formato legacy de zonas por frame/polígonos. |
| `src/navegacion_gps/config/keepout_mask.pgm` | Imagen raster actual de máscara; puede ser regenerada por zones manager. |
| `src/navegacion_gps/config/keepout_mask.yaml` | Metadata map_server de la PGM. |

### Localización

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/localization_v2.yaml` | EKF local V2: fuentes, frames y covarianzas. |
| `src/navegacion_gps/config/localization_global_v2.yaml` | EKF map + navsat del perfil global vigente. |
| `src/navegacion_gps/config/dual_ekf_navsat_params.yaml` | Dual EKF/navsat legacy completo. |
| `src/navegacion_gps/config/dual_ekf_navsat_params.sim_decouple_global_linear_twist_only.yaml` | Experimento replay que desacopla twist lineal global. |
| `src/navegacion_gps/config/dual_ekf_navsat_params.sim_decouple_global_twist_only.yaml` | Experimento replay que desacopla twist global. |
| `src/navegacion_gps/config/dual_ekf_navsat_params.sim_decouple_global_yaw.yaml` | Experimento replay que desacopla yaw global. |
| `src/navegacion_gps/config/dual_ekf_navsat_params.sim_navsat_imu_heading.yaml` | Variante navsat que usa heading IMU simulado. |

### Nav2 y collision monitor

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/nav2_global_v2_params.yaml` | Baseline global V2. |
| `src/navegacion_gps/config/nav2_global_v2_real_rolling_params.yaml` | Perfil real rolling global/local con clearance y sensores. |
| `src/navegacion_gps/config/nav2_global_v2_real_rolling_wifi_params.yaml` | Variante real WiFi con rangos/tuning remoto. |
| `src/navegacion_gps/config/nav2_global_v2_sim_rolling_params.yaml` | Variante sim rolling. |
| `src/navegacion_gps/config/nav2_global_v2_sim_rolling_wifi_params.yaml` | Variante sim WiFi; mantiene paridad de tuning relevante. |
| `src/navegacion_gps/config/nav2_local_v2_params.yaml` | Perfil local V2. |
| `src/navegacion_gps/config/nav2_local_v2_keepout_overrides.yaml` | Override keepout local actualmente vacío. |
| `src/navegacion_gps/config/nav2_local_v2_no_keepout_overrides.yaml` | Remueve plugin/filtro keepout de ambos costmaps. |
| `src/navegacion_gps/config/nav2_no_map_params.yaml` | Perfil Nav2 histórico sin mapa persistente. |
| `src/navegacion_gps/config/collision_monitor.yaml` | Collision monitor legacy. |
| `src/navegacion_gps/config/collision_monitor_lidar_only.yaml` | Perfil restringido al LiDAR. |
| `src/navegacion_gps/config/collision_monitor_v2.yaml` | Zonas V2 stop/critical-slow/slow y approach. |

### Behavior Trees

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/navigate_through_poses_trace.xml` | BT de tracing que anota causas de replan en sim/diagnóstico. |
| `src/navegacion_gps/config/navigate_through_poses_w_replanning_and_recovery_no_spin.xml` | Through-poses con replanning/recovery sin spin. |
| `src/navegacion_gps/config/navigate_to_pose_w_replanning_and_recovery_no_spin.xml` | Navigate-to-pose con clearance/recovery sin spin. |
| `src/navegacion_gps/config/navigate_to_pose_w_replanning_and_recovery_no_spin_no_backup.xml` | Variante sin spin ni backup. |

### LiDAR y benchmark

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/pointcloud_to_laserscan.yaml` | Proyección genérica cloud→scan. |
| `src/navegacion_gps/config/pointcloud_to_laserscan_real.yaml` | Alturas/rangos del robot real. |
| `src/navegacion_gps/config/pointcloud_to_laserscan_real_cuatri_real_v2.yaml` | Corte para el URDF realista V2. |
| `src/navegacion_gps/config/pointcloud_to_laserscan_tilted_lidar_sim.yaml` | Proyección de LiDAR inclinado en sim. |
| `src/navegacion_gps/config/scan_ground_filter.param.yaml` | Parámetros del segmentador de suelo. |
| `src/navegacion_gps/config/nav_benchmark_scenarios.yaml` | Perfiles `smoke/heading_core/regression_core/full` y escenarios de maniobra. |

### RViz

| Archivo | Uso |
|---|---|
| `src/navegacion_gps/config/rviz_global_v2.rviz` | Viewer global completo. |
| `src/navegacion_gps/config/rviz_global_v2_wifi.rviz` | Viewer global remoto con scan debug. |
| `src/navegacion_gps/config/rviz_global_v2_wifi_2d.rviz` | Viewer WiFi liviano 2D. |
| `src/navegacion_gps/config/rviz_global_v2_wifi_scan_ground.rviz` | Viewer WiFi con cloud sin suelo. |
| `src/navegacion_gps/config/rviz_local_v2.rviz` | Viewer local V2. |
| `src/navegacion_gps/config/rviz_nav2_full.rviz` | Configuración Nav2 histórica/genérica completa. |

## 7. Modelos, mundos, packaging y script del paquete

### URDF

- `src/navegacion_gps/models/cuatri.urdf`: modelo Ackermann base sim.
- `src/navegacion_gps/models/cuatri_real.urdf`: geometría real histórica.
- `src/navegacion_gps/models/cuatri_real_v2.urdf`: modelo real vigente con frames/sensores y RS16 inclinado.
- `src/navegacion_gps/models/cuatri_ultrasound.urdf`: variante experimental con ultrasonido.
- `src/navegacion_gps/models/modelo.urdf`: modelo de referencia antiguo.
- `src/navegacion_gps/models/my_robot.urdf`: modelo de referencia/experimento.

### Worlds

- `src/navegacion_gps/worlds/default.sdf`: copia SDF completa del escenario `pasillos_obstaculos`, con plugins, datum geográfico, física y geometría materializada.
- `src/navegacion_gps/worlds/vacio.world`: mundo vacío base.
- `src/navegacion_gps/worlds/pasillos_obstaculos.world`: pasillos y obstáculos.
- `src/navegacion_gps/worlds/slope_lidar.world`: pendiente para LiDAR/ground filter.
- `src/navegacion_gps/worlds/sonoma_salus.world`: escenario Sonoma específico SALUS.
- `src/navegacion_gps/worlds/tugbot_depot.world`: depósito de referencia.

### Packaging/operación

- `src/navegacion_gps/.gitignore`: exclusiones locales del paquete.
- `src/navegacion_gps/resource/navegacion_gps`: marcador vacío del índice Ament.
- `src/navegacion_gps/package.xml`: manifiesto ROS y dependencias.
- `src/navegacion_gps/setup.py`: instala launches/config/modelos/worlds y registra 37 console scripts.
- `src/navegacion_gps/setup.cfg`: configuración ament/pytest.
- `src/navegacion_gps/scripts/run_scan_ground_validation.sh`: wrapper package-local de la validación del filtro.

## 8. `navegacion_gps`: tests

| Archivo | Cobertura principal |
|---|---|
| `src/navegacion_gps/test/test_ackermann_odometry.py` | Modelo yaw-rate, signo de steer, integración y normalización. |
| `src/navegacion_gps/test/test_cmd_vel_ackermann_bridge_v2.py` | Traducción de steering y clamp de reversa. |
| `src/navegacion_gps/test/test_compass_calibration_recorder.py` | Parsing, filtros de movimiento/edad y recomendación de bias. |
| `src/navegacion_gps/test/test_compass_heading_gate.py` | Convención ENU, gates de movimiento/edad/jump/GPS y ventana inicial. |
| `src/navegacion_gps/test/test_gazebo_utils.py` | Cmd final/freno, curvatura/deadband y perfiles GPS sim. |
| `src/navegacion_gps/test/test_global_imu_stationary_gate.py` | Cero/passthrough por movimiento, stale y QoS. |
| `src/navegacion_gps/test/test_global_odom_stationary_gate.py` | Gate de twist por reposo/frescura. |
| `src/navegacion_gps/test/test_global_yaw_stationary_hold.py` | Publicación/supresión yaw-only. |
| `src/navegacion_gps/test/test_goal_pose_to_follow_path_v2.py` | Geometría Ackermann, orientación y distancia al path. |
| `src/navegacion_gps/test/test_gps_course_heading.py` | Normalización/allow-list de estados RTK. |
| `src/navegacion_gps/test/test_gps_course_heading_core.py` | Cardinales, gates y hold del estimador. |
| `src/navegacion_gps/test/test_gps_profiles.py` | Metadata, resolución legacy y ruido/throttle/hold por perfil. |
| `src/navegacion_gps/test/test_keepout_mask_utils.py` | Raster core y gradiente. |
| `src/navegacion_gps/test/test_launch_contracts.py` | Paridad real/sim, contratos globales y export del plugin BT. |
| `src/navegacion_gps/test/test_lidar_obstacle_filter.py` | Suelo/plano/pendiente, obstáculos, tilt gate y persistencia. |
| `src/navegacion_gps/test/test_loop_waypoint_benchmark.py` | Geometría/yaws/YAML del loop. |
| `src/navegacion_gps/test/test_map_gps_absolute_measurement.py` | FromLL, fallback, covarianza y fixes inválidos. |
| `src/navegacion_gps/test/test_nav_benchmarking.py` | Coordenadas relativas, yaw, métricas y selección de catálogo. |
| `src/navegacion_gps/test/test_nav_command_server_arbitration.py` | Auto/manual, critical slow, backup recovery, watchdog y brake hold. |
| `src/navegacion_gps/test/test_nav_command_server_fromll_fallback.py` | Aproximación ENU y criterio de fallback degenerado. |
| `src/navegacion_gps/test/test_nav_command_server_loop_helper.py` | Segmentación/rotación loop y callbacks de resultado; 9 casos fallan por fixture sin `_nav_action_results_to_ignore`. |
| `src/navegacion_gps/test/test_nav_observability.py` | Edad, parsing y diagnósticos de controller/nav/collision. |
| `src/navegacion_gps/test/test_nav_trace_recorder.py` | Firma O, cambios de path, ráfagas y resumen autocontenido. |
| `src/navegacion_gps/test/test_path_clearance_validator.py` | Costos letales/inflados, offsets, fail-open, cache, eventos y params dinámicos. |
| `src/navegacion_gps/test/test_real_global_v2_launch.py` | Wrapper real, datum, rolling costmap, URDF/compass y perfil WiFi. |
| `src/navegacion_gps/test/test_replay_localization_compare.py` | Reuso de launch, wrap yaw y reporte comparativo. |
| `src/navegacion_gps/test/test_route_executor.py` | 68 casos de expansión/chunks/acciones/patrulla/progreso/bloqueo/retry/batería/HOME. |
| `src/navegacion_gps/test/test_scan_ground_filter.py` | Rays, pendientes, obstáculos, perfiles y update atómico. |
| `src/navegacion_gps/test/test_scan_ground_validation.py` | FP, updates de costmap y episodios stop/slow. |
| `src/navegacion_gps/test/test_scan_noise_filter.py` | Speckles, NaN/Inf, rangos y metadata. |
| `src/navegacion_gps/test/test_scan_wifi_debug.py` | Downsample, recorte y clipping. |
| `src/navegacion_gps/test/test_sim_compass_hdg.py` | Convención compass, bias y offset inicial. |
| `src/navegacion_gps/test/test_sim_global_v2_launch.py` | 18 contratos de composición/tuning/paridad/trace/costmaps. |
| `src/navegacion_gps/test/test_sim_local_v2_launch.py` | Cadena realista y ausencia de bridges legacy. |
| `src/navegacion_gps/test/test_sim_sensor_normalizer_v2.py` | Perfiles ideal/M8N/F9P y hold estacionario. |
| `src/navegacion_gps/test/test_sim_wheel_joints_are_continuous.py` | Tipo continuous de joints de rueda sim. |
| `src/navegacion_gps/test/test_startup_heading_diagnosis.py` | Matemática de yaw y quaternion. |
| `src/navegacion_gps/test/test_zones_geojson_utils.py` | Auto-close, coordenadas, multipolygon/huecos y buffer. |
| `src/navegacion_gps/test/test_zones_manager_keepout_degrade.py` | Máscara binaria/degradada, radio y escala PGM. |
| `src/navegacion_gps/test/test_copyright.py` | Linter ament de copyright. |
| `src/navegacion_gps/test/test_flake8.py` | Linter Flake8 genérico. |
| `src/navegacion_gps/test/test_pep257.py` | Linter de docstrings genérico. |

## 9. `navegacion_gps_bt` — 6 archivos

| Archivo | Responsabilidad |
|---|---|
| `src/navegacion_gps_bt/include/navegacion_gps_bt/is_path_clearance_valid_condition.hpp` | Declaración del ConditionNode BT que consulta clearance de path. |
| `src/navegacion_gps_bt/src/is_path_clearance_valid_condition.cpp` | Cliente de servicio, timeout/fail-open y registro BehaviorTree.CPP. |
| `src/navegacion_gps_bt/include/navegacion_gps_bt/trace_replan_decorator.hpp` | Declaración del decorator que publica causa/etapa de replan. |
| `src/navegacion_gps_bt/src/trace_replan_decorator.cpp` | Implementación/registro del trace decorator. |
| `src/navegacion_gps_bt/CMakeLists.txt` | Compila ambas shared libraries, exporta plugins/dependencias e instala headers. |
| `src/navegacion_gps_bt/package.xml` | Manifiesto ament_cmake de plugins BT. |

## 10. `sensores` — 24 propios + 33 contratos externos

### Nodos

| Archivo | Responsabilidad |
|---|---|
| `src/sensores/sensores/__init__.py` | Marca el paquete. |
| `src/sensores/sensores/camara.py` | Cliente ISAPI/PTZ con límites, estado y presets base/overrides; la persistencia reescribe el JSON directamente y las credenciales llegan desde `.env`. |
| `src/sensores/sensores/mavros_compat_bridge.py` | Replica tópicos MAVROS nativos a aliases legacy y diagnostica staleness. |
| `src/sensores/sensores/rtk_bridge_core.py` | Resolución pura de calidad RTK desde GPSRAW/RTK/NavSat/RTCM. |
| `src/sensores/sensores/rtk_bridge.py` | Recibe RTCM TCP/tópico, lo envía a MAVROS y publica estado/edad/conteos. |
| `src/sensores/sensores/rtk_source_manager.py` | Catálogo de caster sources, selección/upsert y worker TCP reconectable. |
| `src/sensores/sensores/web_server.py` | Dashboard HTTP/JSON de Pixhawk, GNSS/RTK, tópicos y gestión de sources. |
| `src/sensores/sensores/pixhawk_driver.py` | Driver MAVLink directo histórico: conversiones NED/ENU y FRD/FLU, IMU/GPS/odom y RTCM. No es el backend vigente del global V2. |

### Launch/config/build

| Archivo | Responsabilidad |
|---|---|
| `src/sensores/launch/camara.launch.py` | Lanza el nodo PTZ. |
| `src/sensores/launch/mavros.launch.py` | MAVROS vigente + compat bridge/web/RTK opcionales. |
| `src/sensores/launch/pixhawk.launch.py` | Driver MAVLink directo legacy + dashboard opcional. |
| `src/sensores/launch/rs16.launch.py` | Nodo RoboSense y RViz opcional. |
| `src/sensores/config/mavros_apm_overrides.yaml` | Frame ids, rates y overrides de plugins APM. |
| `src/sensores/config/mavros_sensor_only_pluginlists.yaml` | Allow/deny de plugins MAVROS para modo sensor-only. |
| `src/sensores/config/rs16.yaml` | Config RoboSense usada por el launch del sensor. |
| `src/sensores/config/rtk_sources.yaml` | Fuentes RTK configurables; puede contener campos sensibles y no debe copiarse a logs. |
| `src/sensores/.env.example` | Nombres de variables de entorno admitidas, sin copiar credenciales operativas. |
| `src/sensores/.gitignore` | Exclusiones locales del paquete. |
| `src/sensores/resource/sensores` | Marcador vacío del índice Ament. |
| `src/sensores/setup.py` | Instala WSDL, dashboard, launch/config y seis entry points. |
| `src/sensores/setup.cfg` | Configuración ament. |
| `src/sensores/package.xml` | Dependencias ROS/MAVROS/web. |

### Tests

- `src/sensores/test/test_camara_presets.py`: siete casos de parse/overlay/persistencia y rollback de presets.
- `src/sensores/test/test_rtk_bridge_core.py`: siete casos de precedencia/estados RTK y RTCM stale.

### Contratos WSDL/XSD importados — 33 archivos

Se empaquetan recursivamente en `setup.py`; describen protocolos ONVIF/OASIS/W3C y no decisiones de negocio SALUS.

| Archivo | Contrato importado |
|---|---|
| `src/sensores/wsdl/accesscontrol.wsdl` | Servicio ONVIF de acceso, credenciales, puertas y políticas. |
| `src/sensores/wsdl/actionengine.wsdl` | Motor ONVIF de acciones, reglas y configuraciones. |
| `src/sensores/wsdl/addressing` | Schema histórico de WS-Addressing. |
| `src/sensores/wsdl/advancedsecurity.wsdl` | Operaciones ONVIF de seguridad avanzada, claves y certificados. |
| `src/sensores/wsdl/analytics.wsdl` | Configuración y reglas de analítica de video. |
| `src/sensores/wsdl/analyticsdevice.wsdl` | Control de engines/modules de dispositivo analítico. |
| `src/sensores/wsdl/b-2.xsd` | Tipos OASIS WS-Notification base. |
| `src/sensores/wsdl/bf-2.xsd` | Tipos OASIS WS BaseFaults. |
| `src/sensores/wsdl/bw-2.wsdl` | Bindings WSDL de WS-BaseNotification. |
| `src/sensores/wsdl/deviceio.wsdl` | Entradas/salidas, relés, audio, video y puertos seriales del dispositivo. |
| `src/sensores/wsdl/devicemgmt.wsdl` | Gestión ONVIF del dispositivo, red, usuarios, certificados y sistema. |
| `src/sensores/wsdl/display.wsdl` | Layouts y salidas del servicio de display. |
| `src/sensores/wsdl/doorcontrol.wsdl` | Estado, control y configuración de puertas. |
| `src/sensores/wsdl/envelope` | Schema del envelope SOAP 1.1. |
| `src/sensores/wsdl/events.wsdl` | Pull points, subscriptions y propiedades de eventos ONVIF. |
| `src/sensores/wsdl/imaging.wsdl` | Ajustes de imagen, foco y opciones por video source. |
| `src/sensores/wsdl/include` | Schema W3C XOP Include para contenido binario. |
| `src/sensores/wsdl/media.wsdl` | Profiles, fuentes/encoders y URIs de streaming/media. |
| `src/sensores/wsdl/onvif.xsd` | Schema central de tipos ONVIF compartidos. |
| `src/sensores/wsdl/ptz.wsdl` | Configuración, estado, movimientos y presets PTZ. |
| `src/sensores/wsdl/r-2.xsd` | Tipos OASIS WS-ResourceProperties. |
| `src/sensores/wsdl/receiver.wsdl` | Configuración y modo de receivers de media. |
| `src/sensores/wsdl/recording.wsdl` | Jobs, tracks y configuración de grabaciones. |
| `src/sensores/wsdl/remotediscovery.wsdl` | Operaciones de descubrimiento remoto ONVIF. |
| `src/sensores/wsdl/replay.wsdl` | URI y configuración de replay de grabaciones. |
| `src/sensores/wsdl/rw-2.wsdl` | Bindings WSDL de WS-ResourceProperties. |
| `src/sensores/wsdl/search.wsdl` | Búsquedas de grabaciones, eventos y metadata. |
| `src/sensores/wsdl/t-1.xsd` | Tipos OASIS WS-Topics. |
| `src/sensores/wsdl/types.xsd` | Tipos comunes del stack de Web Services incluido. |
| `src/sensores/wsdl/ws-addr.xsd` | Schema W3C WS-Addressing 1.0. |
| `src/sensores/wsdl/ws-discovery.xsd` | Schema de WS-Discovery. |
| `src/sensores/wsdl/xml.xsd` | Declaraciones estándar del namespace XML. |
| `src/sensores/wsdl/xmlmime` | Schema W3C de tipos MIME en XML. |

## 11. `vision_pipeline` — 14 archivos

| Archivo | Responsabilidad |
|---|---|
| `src/vision_pipeline/vision_pipeline/__init__.py` | Marca el paquete realtime vision. |
| `src/vision_pipeline/vision_pipeline/ip_camera_publisher.py` | Captura RTSP/MJPEG/snapshot, reconecta, redimensiona y publica Image/CameraInfo opcional. |
| `src/vision_pipeline/vision_pipeline/yolo_onnx_detector.py` | Preprocesa, ejecuta ONNX Runtime, hace NMS y publica Detection2DArray/debug con provider configurable. |
| `src/vision_pipeline/vision_pipeline/vision_web_server.py` | Sirve dashboard/stream JPEG con overlay de detecciones. |
| `src/vision_pipeline/launch/ip_camera_ai_test.launch.py` | Cámara IP + detector para prueba rápida. |
| `src/vision_pipeline/launch/vision_pc_gpu.launch.py` | Cámara IP + YOLO con provider/threads para PC GPU. |
| `src/vision_pipeline/launch/vision_pipeline.launch.py` | Cámara V4L2 USB + detector. |
| `src/vision_pipeline/config/coco_80.names` | Etiquetas COCO por índice. |
| `src/vision_pipeline/config/v4l2_camera_low_latency.yaml` | Parámetros de captura USB de baja latencia. |
| `src/vision_pipeline/config/yolo_detector.yaml` | Tamaño, thresholds, FPS y provider del detector. |
| `src/vision_pipeline/resource/vision_pipeline` | Marcador vacío del índice Ament. |
| `src/vision_pipeline/setup.py` | Instala config/launch/web y tres entry points. |
| `src/vision_pipeline/setup.cfg` | Configuración ament. |
| `src/vision_pipeline/package.xml` | Dependencias ROS/vision/OpenCV. |

No hay tests propios versionados en este paquete. Su validación actual es build/import y prueba de pipeline con cámara/modelo disponibles.

## 12. Cobertura y exclusiones explícitas

- Los 339 archivos de texto de paquetes definidos en el alcance están enumerados por ruta en este catálogo o, para el grupo homogéneo WSDL/XSD, en `BACKEND_LINE_MANIFEST.tsv` y la sección `sensores` de `BACKEND_LINE_AUDIT.md`.
- Los README/AGENTS/docs no se duplican aquí; son explicación, no implementación.
- HTML/CSS/JavaScript de frontend, imágenes, el modelo ONNX, el PGM raw, bags, logs y build/install no se presentan como texto backend leído. Los tres binarios asociados versionados están en `BACKEND_BINARY_INVENTORY.tsv`.
- WSDL/XSD de ONVIF se reconocen como contratos externos, no se reinterpretan como lógica propia.
- RoboSense se separa por ser vendor; ver `CODE_CATALOG_VENDOR.md`.
- Una entrada en este catálogo no significa que el archivo sea vigente: se marcan explícitamente paths legacy, experimentales o históricos.
