# Catálogo exhaustivo: runtime, contenedor y herramientas

Estado: auditado contra `main` local el 2026-08-16.

Alcance: todos los archivos versionados ejecutables o de configuración ubicados en la raíz, `.github/`, `docs/upstream-sources.yaml` y `tools/`. Cada ruta de este alcance aparece una vez en este catálogo. Los documentos Markdown, imágenes y mapas mentales `.mb` no son código y quedan fuera de este inventario.

Fuente de verdad: el archivo mencionado; esta descripción sirve como índice y no reemplaza inspeccionar argumentos y variables de entorno antes de operar hardware.

## Runtime raíz

| Archivo | Responsabilidad |
|---|---|
| `.gitignore` | Excluye outputs Colcon, bags, caches, frontends/repos anidados y scratch local del repositorio principal. |
| `.bashrc` | Inicializa ROS 2 Humble, autocompletado Colcon, CycloneDDS, dominio 0 y el overlay si existe. |
| `.github/workflows/ci.yml` | Construye la imagen, instala rosdep y ejecuta build/test en GitHub Actions. Actualmente selecciona `interfaces controller_server map_tools navegacion_gps sensores`; omite `navegacion_gps_bt` y `vision_pipeline`. |
| `Dockerfile` | Imagen Humble Perception con Nav2, robot_localization, MAVROS, visión y dependencias; Gazebo es condicional por arquitectura y Mapviz solo se instala en amd64. |
| `docker-compose.yml` | Stack de desarrollo completo: contenedor privilegiado `ros2_salus`, mounts del workspace y servicio Netdata. |
| `docker-compose.salus.yml` | Variante anclada al checkout que levanta solo el servicio ROS y conserva los mismos mounts/runtime. |
| `entrypoint.sh` | Sourcea Humble y el overlay instalado antes de ejecutar el comando del contenedor. |
| `mapviz_gps.mvc` | Configuración de Mapviz copiada al home del usuario ROS en la imagen. |
| `docs/upstream-sources.yaml` | Manifiesto de procedencia de componentes upstream/vendor para trazabilidad. |

## Entrada al contenedor y ciclo de build

| Archivo | Responsabilidad |
|---|---|
| `tools/docker_ros_env.sh` | Biblioteca shell compartida: resuelve X11 y arma `docker exec` con entorno ROS/RMW reproducible. |
| `tools/exec.sh` | Ejecuta un comando o abre shell interactiva en `ros2_salus` usando el helper de entorno. |
| `tools/root-exec.sh` | Abre shell root dentro del contenedor; usar solo cuando los permisos lo requieren. |
| `tools/up-salus.sh` | Levanta `docker-compose.salus.yml` con build y rutas relativas al checkout. |
| `tools/down-salus.sh` | Detiene el compose SALUS. |
| `tools/compile-ros.sh` | Ejecuta `colcon build --symlink-install`; acepta paquetes o compila todo y muestra el source del overlay. |
| `tools/create_pkg.sh` | Wrapper validado para `ros2 pkg create` dentro de `/ros2_ws/src`. |
| `tools/vcs-status.sh` | Muestra estado Git del monorepo raíz; no gestiona repos múltiples bajo `src`. |
| `tools/vcs-pull.sh` | Hace pull del repo raíz según su implementación; es una acción remota y requiere autorización explícita. |
| `tools/vcs-push.sh` | Hace push del repo raíz; nunca ejecutarlo implícitamente. |

## Launchers de robot real

| Archivo | Responsabilidad |
|---|---|
| `tools/launch_controller.sh` | Lanza solo `controller_server.launch.py`. |
| `tools/launch_no_go_editor.sh` | Lanza el backend/editor WebSocket de zonas, navegación y rutas. |
| `tools/launch_real_nav.sh` | Wrapper del stack legacy `real.launch.py`. |
| `tools/launch_real_rviz.sh` | Viewer RViz legacy con `cuatri_real.urdf`. |
| `tools/launch_real_local_v2.sh` | Arranca navegación local V2 real. |
| `tools/launch_real_local_v2_rviz.sh` | Abre el RViz local V2 en la PC operadora. |
| `tools/launch_real_global_v2.sh` | Arranca el perfil global V2 base con CycloneDDS; queda para LAN/compatibilidad. |
| `tools/launch_real_global_v2_rviz.sh` | Viewer local del perfil global real base. |
| `tools/launch_real_global_v2_scan_ground.sh` | Perfil global real con segmentación de suelo habilitada y CycloneDDS LAN. |
| `tools/launch_real_global_v2_wifi.sh` | Perfil operativo recomendado por WiFi; usa el wrapper WiFi o cae al launch base con opciones remotas. |
| `tools/launch_real_global_v2_wifi_cuatri_real_v2.sh` | Añade URDF realista V2 y parámetros de proyección LiDAR específicos. |
| `tools/launch_real_global_v2_wifi_rviz.sh` | RViz remoto completo del perfil WiFi. |
| `tools/launch_real_global_v2_wifi_rviz_2d.sh` | RViz remoto liviano 2D para reducir ancho de banda. |
| `tools/launch_real_global_v2_wifi_rviz_scan_ground.sh` | RViz remoto que además visualiza `/scan_3d/no_ground`. |

## Launchers de simulación

| Archivo | Responsabilidad |
|---|---|
| `tools/launch_sim_local_v2.sh` | Arranca el perfil local V2 de simulación. |
| `tools/stop_sim_local_v2.sh` | Detiene por patrones los procesos del stack local V2; revisar el patrón antes de usar. |
| `tools/launch_sim_global_v2.sh` | Arranca simulación global V2 con GPS F9P simulado, backend web y RViz. |
| `tools/launch_sim_global_v2_wifi.sh` | Variante global WiFi/headless más viewer remoto. |
| `tools/launch_sim_global_v2_wifi_cuatri_real_v2.sh` | Viewer/sim WiFi con URDF V2 y proyección de LiDAR inclinado. |
| `tools/launch_sim_global_v2_wifi_slope.sh` | Ejecuta el mundo `slope_lidar.world` para pendientes y abre el viewer WiFi. |
| `tools/launch_sim_global_v2_wifi_sonoma.sh` | Delega al launcher WiFi usando `sonoma_salus.world`. |
| `tools/launch_scan_ground_ramp_debug.sh` | Orquesta escenario de rampa, genera un RViz temporal y levanta validación/filtro para depurar suelo. |
| `tools/stop_sim_global_v2.sh` | Detiene por patrones Gazebo, localización, Nav2, filtros, backend y viewers globales. |

## Navegación, diagnóstico y bags

| Archivo | Responsabilidad |
|---|---|
| `tools/check_startup_heading.sh` | Ejecuta `startup_heading_diagnosis` durante una ventana configurable. |
| `tools/record_compass_calibration.sh` | Ejecuta el recorder de calibración y guarda reporte/datos de brújula vs heading GPS. |
| `tools/record_nav_debug_bag.sh` | Graba el conjunto de tópicos de diagnóstico de navegación en un bag con nombre controlado. |
| `tools/run_localization_replay_compare.sh` | Copia un bag, lanza la localización replay, regraba resultados y genera comparación. Tiene cleanup de procesos. |
| `tools/run_nav_benchmark.sh` | Ejecuta escenarios del catálogo mediante `nav_benchmark_runner`. |
| `tools/compare_nav_benchmarks.sh` | Compara baseline y candidato con `nav_benchmark_report`. |
| `tools/generate_block_loop_benchmark.sh` | Genera waypoints de loop de manzana mediante el nodo benchmark. |
| `tools/regenerate_nav_trace_report.sh` | Regenera el reporte de una sesión de trace existente. |
| `tools/show_latest_nav_trace.sh` | Localiza y muestra el trace/reporte más reciente. |
| `tools/send_follow_path_v2.sh` | Envía manualmente una acción `/follow_path` de tres poses en `odom`. |
| `tools/healthcheck-lidar.sh` | Comprueba frecuencia de scans, cloud de obstáculos y TF `odom -> lidar_link`. |
| `tools/sim_battery.sh` | CLI para presets/estado de la batería simulada usando servicios ROS. |

## Pruebas determinísticas de control y UART

| Archivo | Responsabilidad |
|---|---|
| `tools/closed_loop_step_publisher.py` | Publica una secuencia determinística continua de pasos `/cmd_vel_ws` y registra eventos. |
| `tools/uart_step_sender.py` | Envía frames Pi→ESP32 con CRC y decodifica telemetría de estado para pruebas UART. |

## Monitor de potencia Jetson

| Archivo | Responsabilidad |
|---|---|
| `tools/jetson_power_lib.py` | Núcleo: descubre rail INA3221, lee tegrastats/JSONL segmentado, reconstruye boots y clasifica brownout/reset/gap. |
| `tools/jetson_power_monitor.py` | Proceso de muestreo host-side con perfiles, thresholds, segmentos y eventos. |
| `tools/jetson_power_mark.py` | Agrega marcas de operador al log de eventos actual. |
| `tools/jetson_power_report.py` | Resume boots y diagnósticos del monitor. |
| `tools/run_jetson_power_monitor.sh` | Wrapper que ejecuta el monitor con Python del host. |
| `tools/install_jetson_power_monitor_service.sh` | Instala, habilita y verifica la unidad systemd; modifica el host. |
| `tools/systemd/jetson-power-monitor.service` | Unidad systemd del monitor. |
| `tools/test_jetson_power_lib.py` | Seis tests: discovery de VDD_IN, JSONL truncado, orden de segmentos y tres clasificaciones de boot. |

## Observaciones de seguridad y mantenimiento

- Los launchers reales pueden comandar hardware. Antes de ejecutarlos comprobar checkout, contenedor, puertos seriales, dominio ROS y estado físico.
- Los scripts `stop_*` usan coincidencias de proceso amplias; no son equivalentes a lifecycle shutdown ordenado.
- `vcs-pull.sh`, `vcs-push.sh`, instalación systemd y compose cambian estado externo; no se ejecutan como parte de una auditoría read-only.
- El CI no cubre todavía todos los paquetes propios y no demuestra comportamiento físico.
