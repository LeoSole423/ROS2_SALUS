# Auditoría exhaustiva línea por línea del backend

Estado: completo para el alcance acordado, auditado línea por línea el 2026-08-16. Se cerraron raíz/runtime, paquetes ROS propios, contratos externos empacados y código vendorizado RoboSense. El frontend permanece excluido por pedido explícito del usuario.

Alcance: todos los archivos versionados seleccionados del backend de `ROS2_SALUS`, más dos CMake generados presentes dentro del source vendor: runtime del contenedor, herramientas, paquetes ROS propios, interfaces, launches, configuración, modelos, tests, contratos ONVIF y código vendorizado RoboSense. Se excluyen deliberadamente Cockpit y los archivos HTML/CSS/JavaScript de frontend por pedido del usuario.

Fuente de verdad: contenido del checkout local `/home/franco/final/ROS2_SALUS` identificado en `BACKEND_LINE_MANIFEST.tsv`, con ruta, cantidad de líneas, bytes y SHA-256 por archivo. Si cambia un hash, la conclusión correspondiente debe revalidarse contra el archivo nuevo.

## 1. Método y estado de cobertura

La revisión no consiste en inferir responsabilidades desde nombres de archivo. Cada archivo entra en la cobertura solo después de recorrer todo su contenido, incluidas líneas vacías, comentarios, constantes, ramas de error, packaging y tests. Los catálogos asociados documentan individualmente cada ruta; este documento conserva relaciones, invariantes y hallazgos transversales.

Alcance final: 541 archivos de texto y 125.954 líneas POSIX —saltos de línea contados con `wc -l`—. El primer inventario de 508 omitía marcadores Ament sin extensión y formatos auxiliares como `.ini`, `.txt`, `.sdf`, `.launch`, `.example` y dotfiles; después se amplió a 536. La reconciliación final añadió cinco artefactos vendor que también contienen lógica o configuración de build: el socket Windows y cuatro proyectos/solución Visual Studio. La cifra incluye contratos WSDL/XSD empacados con `sensores`, dos CMake generados presentes en el source del driver y el vendor RoboSense. Los binarios asociados —por ejemplo ONNX, PNG y el PGM raw de keepout— quedan fuera de la lectura por líneas y se registran separadamente en `BACKEND_BINARY_INVENTORY.tsv`. Un archivo de configuración RTK puede contener credenciales operativas: se revisó sin copiar valores sensibles a documentación, logs ni respuestas.

| Tanda | Archivos | Estado |
|---|---:|---|
| Raíz y runtime de contenedor | 9 | leído completo |
| `src/interfaces` | 40 | leído completo |
| `tools` y unidad systemd | 55 | leído completo |
| `src/controller_server` | 33 | leído completo |
| `src/map_tools` —sin `web/index.html`— | 14 | leído completo |
| Raíz/packaging de `src/navegacion_gps` | 5 | leído completo |
| `src/navegacion_gps/config` —45 textos; PGM separado— | 45 | leído completo |
| `src/navegacion_gps/launch` | 24 | leído completo |
| `src/navegacion_gps/models` | 6 | leído completo |
| `src/navegacion_gps/worlds` | 6 | leído completo |
| `src/navegacion_gps/scripts` | 1 | leído completo |
| `src/navegacion_gps/navegacion_gps` | 46 | leído completo |
| `src/navegacion_gps/test` | 42 | leído completo |
| `src/navegacion_gps_bt` —sin README— | 6 | leído completo |
| `src/sensores` —24 propios + 33 contratos; sin README/HTML/PNG— | 57 | leído completo |
| `src/vision_pipeline` —sin README/HTML/ONNX— | 14 | leído completo |
| Vendor `rslidar_msg` + `rslidar_sdk` | 138 | leído completo |

La suma cerrada es 541/541 archivos de texto del alcance. El manifiesto es el cierre verificable de esa cobertura; documentación narrativa, imágenes, binarios y frontend no se cuentan como código leído por líneas.

## 2. Raíz y contenedor

### Construcción

- `Dockerfile` parte de ROS 2 Humble Perception, instala Nav2, `robot_localization`, MAVROS, visión y utilidades, y condiciona Gazebo a arquitecturas compatibles. Mapviz se instala solo en amd64. La imagen copia `mapviz_gps.mvc`, el entrypoint y el `.bashrc`; el código de trabajo llega por bind mount de Compose, no por `COPY` del repositorio.
- `entrypoint.sh` carga `/opt/ros/$ROS_DISTRO/setup.bash` y después `/ros2_ws/install/setup.bash` si existe. Por eso un overlay viejo puede ocultar cambios del source aunque el checkout sea correcto.
- `.bashrc` repite el source de Humble/overlay y fija CycloneDDS y dominio ROS 0 para shells interactivos. Un proceso no interactivo depende del entrypoint o del wrapper que lo invoque.

### Compose y superficie del host

- Ambos Compose usan red del host, modo privilegiado y mounts de `/dev`, X11, source, build/install/log y herramientas. Son perfiles de desarrollo/hardware, no aislamiento fuerte.
- `docker-compose.yml` agrega Netdata y monta información de host, además de `docker.sock` en lectura. `docker-compose.salus.yml` conserva solo el contenedor ROS.
- `mapviz_gps.mvc` usa `odom` como fixed frame, mientras la herramienta de click declara `map`; esa diferencia debe recordarse al interpretar un goal o una corrección `map -> odom`. Los campos de API key están vacíos en el archivo versionado.
- `docs/upstream-sources.yaml` registra procedencia vendorizada; no demuestra por sí solo que el código local siga idéntico al upstream.

### CI

`.github/workflows/ci.yml` construye la imagen, resuelve dependencias, compila y prueba `interfaces`, `controller_server`, `map_tools`, `navegacion_gps` y `sensores`. No selecciona `navegacion_gps_bt` ni `vision_pipeline`; por tanto el verde del workflow no cubre esos dos paquetes ni comportamiento físico.

## 3. Contratos de `interfaces`

`CMakeLists.txt` enumera explícitamente 8 mensajes y 29 servicios para `rosidl_generate_interfaces`; `package.xml` exporta `rosidl_interface_packages`. Agregar un archivo al directorio sin sumarlo al CMake no genera el tipo.

Invariantes observados en los contratos:

- `CmdVelFinal` transporta `Twist`, freno porcentual y fuente enumerada. El freno no forma parte de `geometry_msgs/Twist`, de modo que no puede preservarse si un consumidor reduce el contrato a `/cmd_vel`.
- `DriveTelemetry` distingue disponibilidad, frescura, drive enable, E-stop y reversa; los consumidores de seguridad no deberían derivar esas condiciones solo de velocidad cero.
- `BatteryMissionGuard` separa voltaje cargado y recuperado, expone persistencias/umbrales y una recomendación de HOME. Es un resultado de estimación para misión, no una lectura cruda única.
- `NavTelemetry` y `NavEvent` separan snapshot de estado y secuencia de eventos. Los details de eventos son dos arrays paralelos key/value: su asociación depende de conservar igual longitud y orden.
- Las rutas y patrullas aceptan arrays relacionados —waypoints, acciones y roles—. La validación de tamaños pertenece a los servidores porque ROSIDL no expresa esa restricción.
- Los servicios de cámara cubren pan legacy, PTZ, estado, presets y persistencia. Cambiar uno exige recompilar primero `interfaces` y luego todos sus consumidores.

La descripción ruta por ruta está en `CODE_CATALOG_OWN.md`, sección `interfaces`.

## 4. Herramientas operativas

### Entorno y build

- `docker_ros_env.sh` centraliza X11, `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` y el source del overlay para `docker exec`.
- `exec.sh` delega comandos a `bash -lc`; su entrada se considera de operador confiable, no una API segura para texto arbitrario.
- `compile-ros.sh` elimina `build/<paquete>` e `install/<paquete>` antes de una build selectiva y usa `--symlink-install`. Archivos root-owned en esos árboles pueden impedir la limpieza y producir un falso diagnóstico de compilación.
- `create_pkg.sh` valida la presencia del nombre y del tipo, pero arma una orden de shell dentro del contenedor. No debe recibir nombres u opciones provenientes de usuarios no confiables.
- `vcs-pull.sh` exige actualización fast-forward; `vcs-push.sh` publica el checkout actual. Ninguno forma parte de una validación local automática.

### Launchers y parada

- Los launchers reales fijan configuración DDS, cargan overlay y terminan en un `ros2 launch`. Variantes WiFi cambian configuración CycloneDDS, parámetros remotos, URDF y/o viewer; el nombre del wrapper no reemplaza revisar el launch efectivo.
- Hay dos inconsistencias de naming que pueden confundir operación: `launch_real_global_v2.sh` usa configuración WiFi pese a describirse como base, y `launch_real_global_v2_wifi_rviz_2d.sh` usa el XML LAN.
- Los launchers de simulación primero invocan scripts de parada, levantan procesos en background y esperan tiempos fijos. Un `sleep 5` solo da tiempo de arranque; no prueba readiness de Nav2, Gazebo, TF o sensores.
- `launch_scan_ground_ramp_debug.sh` genera una configuración RViz temporal con Python embebido y fuerza el cleanup previo. Es una herramienta de escenario, no un launch composable.
- `stop_sim_local_v2.sh` y `stop_sim_global_v2.sh` buscan procesos por patrones tanto dentro del contenedor como en host, escalan a `SIGKILL` y abarcan varias familias de nodos. Deben considerarse acciones destructivas de procesos, no shutdown lifecycle ordenado.

### Bags, replay y diagnóstico

- `record_nav_debug_bag.sh` tiene perfiles `core` y `full_nav2`; siempre incluye TF, rosout, GPS, odometrías, scans, cadena de comandos, controller y observabilidad. Usa `docker exec -it`, por lo que una ejecución sin TTY puede requerir ajuste.
- `record_compass_calibration.sh` sí detecta si hay TTY y escapa argumentos adicionales antes de construir el comando interno.
- `run_localization_replay_compare.sh` inspecciona `metadata.yaml`, elige replay con GPS crudo o fallback de odometría/heading, crea overrides QoS, inicia launch y grabación por tiempos fijos, reproduce el bag y genera `compare.json`. Borra previamente rutas destino explícitas dentro del contenedor y reemplaza la salida host al copiar el resultado.
- `regenerate_nav_trace_report.sh` restringe la traza a una ruta dentro del checkout antes de traducirla a `/ros2_ws`; evita usar una ruta host arbitraria como ruta de contenedor.
- `healthcheck-lidar.sh` comprueba frecuencia y TF, pero una respuesta puntual no valida calibración, ausencia de ruido ni cobertura del campo visual.

### Pruebas determinísticas de actuación

- `closed_loop_step_publisher.py` publica cinco fases a frecuencia monotónica, registra transiciones CSV y emite cinco frames de stop al final. El mensaje conserva `angular.z = 0`, por lo que solo prueba la cadena longitudinal.
- `uart_step_sender.py` construye frames `0xAA`, versión/flags, steer, accel, brake y CRC; resincroniza RX buscando frames `0x55` de cuatro bytes. El flag `--require-telemetry` solo exige alguna telemetría no reservada durante fases con aceleración, no que cada fase tenga respuesta.

### Monitor eléctrico Jetson

- `jetson_power_lib.py` descubre canales INA3221 por etiquetas sysfs, requiere `VDD_IN`, parsea `tegrastats`, rota JSONL por tiempo y usa `fdatasync` periódico. Tolera una cola JSONL con NUL/JSON inválido conservando registros anteriores y marcando corrupción.
- Los perfiles `aggressive` y `moderate` cambian frecuencia, rotación, sync y ventana de caída. La clasificación distingue `clean_shutdown`, `internal_rail_drop_suspected`, `abrupt_reset_internal_rail_stable` y `monitor_gap_unknown`.
- Una línea interna estable antes del reset no absuelve la alimentación: el propio reporte conserva que el camino upstream de 19 V/regulador no está observado.
- `jetson_power_monitor.py` analiza la sesión anterior al iniciar, registra eventos de cierre abrupto/corrupción, muestrea rail rápido y contexto lento, y solo escribe `monitor_stopped_cleanly` ante SIGINT/SIGTERM o `max_samples` alcanzado.
- `tools/systemd/jetson-power-monitor.service` contiene un `WorkingDirectory` y un `ExecStart` históricos bajo `/home/admin/Desktop/SALUS/ROS2_SALUS`; no apuntan al checkout actual `/home/franco/final/ROS2_SALUS`. El instalador además modifica systemd y elimina entradas de crontab coincidentes.
- Los tests del helper cubren discovery, cola corrupta, rotación y tres diagnósticos, pero no prueban el loop completo de muestreo ni hardware real.

## 5. `controller_server`: control, transporte y batería

### Conversión de comandos y seguridad

- `control_logic.py` convierte `CmdVelFinal` a un `DesiredCommand` Ackermann: limita avance/reversa, aplica deadband y velocidad mínima efectiva, deriva curvatura `angular_z / linear_x`, convierte a ángulo con la distancia entre ejes y limita dirección primero por perfil operativo y finalmente por el límite físico. La fuente manual puede usar un límite operativo distinto; una fuente desconocida conserva el límite automático.
- Con velocidad lineal nula pero giro pedido usa una velocidad virtual para poder alinear la dirección sin habilitar tracción. Los flags `min_speed_enforced`, `speed_limited`, `steer_saturated` y `used_steering_speed_fallback` conservan qué transformaciones se aplicaron.
- Cualquier `brake_pct > 0` convierte el comando en E-stop, anula velocidad y dirección. En el tick del nodo, un E-stop eleva además el freno como mínimo a `estop_brake_pct` —100 por defecto—; por eso una solicitud parcial puede terminar como frenado total en el backend.
- `select_effective_command()` solo decide entre comando automático fresco y stop por timeout. La arbitrariedad manual/automática global ocurre aguas arriba; no está implementada dentro de este paquete.
- Dos argumentos de `command_from_cmd_vel()` están presentes pero no afectan hoy el resultado: `max_abs_angular_z` se convierte a `float` y se descarta, y `reverse_brake_pct` no se consulta. Los tests pasan ambos parámetros, pero no prueban ningún efecto de ellos.
- La ruta normal publica logs `INFO` por cada `CmdVelFinal` recibido. A frecuencia alta puede producir volumen considerable de `rosout`.

### Nodo y selección de backend

- `controller_server_node.py` suscribe `/cmd_vel_final`, ejecuta el watchdog, aplica el comando al transporte y publica `DriveTelemetry`, `BatteryTelemetry` y `BatteryMissionGuard`. También ofrece servicios para habilitar drive, E-stop, reset, estado y presets de batería simulada.
- El constructor resuelve `serial_port` antes de seleccionar `transport_backend`. Con `serial_port=auto`, un host sin candidato serial puede fallar al construir el nodo incluso si el backend solicitado es `sim_gazebo`; las pruebas del factory eluden esto pasando `/dev/null` directamente.
- El launch configura por defecto el backend UART y `invert_steer_from_cmd_vel=true`, mientras que el valor declarado en el nodo es `false`. Para conocer el comportamiento real hay que revisar el launch/override efectivo, no solo el default Python.
- El `setup.py` conserva descripción, licencia y mantenedor de plantilla `TODO`; no son metadatos confiables de propiedad o licencia.

### UART y protocolo ESP32

- `rpy_esp32_comms/protocol.py` implementa protocolo v2 con CRC-8/MAXIM: comando Pi de 7 bytes con cabecera `0xAA`, telemetría de control de 8 bytes con `0x55` y batería de 8 bytes con `0x56`. Velocidad y dirección usan centiunidades y sentinels para valores no disponibles.
- El parser incremental resincroniza por cabecera, valida versión y CRC y contabiliza bytes descartados, frames válidos, errores de CRC y frames desconocidos. La telemetría decodificada distingue ready, E-stop, frescura Pi, fuente de control, overspeed y reversa.
- `transport.py` mantiene hilos TX/RX, comienza en estado seguro, transmite a frecuencia configurada y durante el cierre fija estado seguro y envía tres frames cero adicionales. El estado seguro de cierre no aplica freno: drive deshabilitado, velocidad/dirección cero y `brake_pct=0`.
- Los errores de escritura se absorben dentro del loop y solo incrementan estadísticas; quien llama `apply_command()` no recibe una excepción de transmisión. La salud debe observarse mediante estadísticas/telemetría, no por éxito del setter.
- La resolución serial es fail-closed y prioriza: ruta explícita, variable `SALUS_CONTROLLER_SERIAL_PORT`, único CP2102 `by-id`, único UART genérico `by-id`, único `ttyUSB` y finalmente `/dev/serial0`. Candidatos múltiples producen error con diagnóstico.

### Backend Gazebo

- `sim_gazebo_backend.py` traduce el mismo `DesiredCommand` a `Twist` para Gazebo y sintetiza el contrato de telemetría del ESP32 a partir de odometría y joints. Tiene inversiones separadas para signo de actuación y signo medido.
- Para dirección medida reconstruye el ángulo central Ackermann desde las ruedas. Prefiere joints cuando concuerdan con la estimación de odometría y usa odometría cuando la diferencia supera el umbral configurado.
- La rapidez simulada usa `hypot(linear.x, linear.y)`, por lo que la medición publicada no conserva signo de marcha; el pedido de reversa solo permanece en el estado de comando.
- La batería simulada ofrece presets y overrides cargado/recuperado. Una muestra marcada fresca renueva `rx_monotonic_s` y reporta edad cero en cada lectura; una muestra stale conserva su instante anterior.

### Estimación y guard de batería

- `battery_estimator.py` filtra voltaje con constantes rápidas/lentas, separa condición con tracción de voltaje recuperado y traduce voltaje a SOC mediante curva monótona por tramos. Durante tracción congela el recuperado y evita que el SOC operativo suba por una recuperación espuria.
- La recomendación HOME se enclava por persistencia de voltaje cargado bajo o recuperado bajo y solo se limpia cuando ambas señales superan sus umbrales más histéresis. El estado del sensor (`UNAVAILABLE`, `STALE`, `LINK_STALE`, `SUSPECT`) tiene prioridad visual sobre el guard energético.
- El payload diagnóstico llamado `mission_guard_state` se llena con `battery_state_text`, que también puede contener estados de validez del sensor, mientras el mensaje `BatteryMissionGuard.mission_guard_state` conserva `OK` o `LOW_ENERGY_GO_HOME`. Los dos campos homónimos no tienen exactamente el mismo dominio semántico.

### Pruebas y artefacto histórico

- Los tests cubren CRC, sentinels, re-sincronización, clamps, deadband, dirección automática/manual, watchdog, curva SOC, persistencias/histéresis, selección serial y traducción/síntesis Gazebo. Son pruebas unitarias y dobles de nodo; no validan UART, ESP32, motor, freno ni steering físicos actuales.
- `controller/artifacts/run_uart_e2e.py` es una herramienta que ordena enable, velocidad, dirección, freno, E-stop y reset sobre hardware real. Contiene rutas, host y puerto históricos fijos y no debe ejecutarse como un test unitario ordinario.
- `uart_e2e_results.json` es evidencia histórica, no estado actual: registra frames sin errores CRC en esa captura, ciclos tardíos y un salto Hall inválido. No autoriza afirmar salud presente del robot.

## 6. `map_tools`: gateway ROS/WebSocket y archivos operativos

### Composición y superficie de red

- `no_go_editor.launch.py` puede iniciar `zones_manager`, `nav_command_server`, `nav_snapshot_server`, `route_executor` y `web_zone_server`, y propaga explícitamente tópicos, servicios, datum, timeouts y opciones de lock. El gateway escucha por defecto en `0.0.0.0:8766`.
- `web_zone_server.py` concentra 5.338 líneas y une WebSocket con zonas, navegación, rutas/patrulla, teleoperación, perfiles, RTK, datum, snapshots, cámara/PTZ, batería, diagnósticos, rosbag y sesiones de misión. No hay autenticación, autorización por cliente ni TLS en este servidor. El control lock existe, pero `enable_control_lock` es `false` tanto en el nodo como en el launch.
- Cuando el lock está habilitado bloquea creación de goals/rutas/patrullas, cambio de perfil, HOME, activación manual y comandos manuales. No bloquea, entre otras operaciones, freno, cancelaciones, edición de zonas/datum/RTK, cámara ni control de rosbag; esta es la frontera exacta del código, no una política general de acceso.
- Cada mensaje recibido crea una tarea async independiente. Los locks de envío serializan respuestas por socket, pero no ordenan mutaciones incompatibles; dos solicitudes concurrentes pueden llegar a los servicios en otro orden que el de finalización de sus ACK.

### Estado ROS y teleoperación

- El nodo mantiene snapshots bajo lock y combina `NavTelemetry`, `DriveTelemetry`, JSON legacy del controller, GPS/IMU/odometría, diagnósticos y estados de ruta/patrulla. Una vez recibida batería del controller ignora `sensor_msgs/BatteryState` para evitar que una fuente genérica reemplace ese snapshot.
- Las llamadas ROS esperan disponibilidad hasta dos segundos y luego sondean el future cada 10 ms hasta el timeout. Desde WebSocket se desplazan normalmente a `asyncio.to_thread`; el bootstrap también se ejecuta fuera del loop.
- `set_manual_cmd` exige que el estado manual cacheado esté habilitado, valida finitud, limita solo el freno a 0..100 y publica `CmdVelFinal`. No limita aquí velocidad lineal ni angular y no asigna explícitamente el campo `source`; los límites físicos efectivos quedan en consumidores aguas abajo.
- El heartbeat vuelve a bloquear tras timeout solo cuando el lock está habilitado. Con la configuración por defecto deshabilitada, `set_control_lock` y `control_heartbeat` responden exitosamente pero no aplican bloqueo.

### Waypoints, rutas, datum y zonas

- `waypoints_file_utils.py` acepta claves lat/lon alternativas, yaw opcional, una única marca HOME, acciones y un perfil de patrulla. Valida números finitos, pero no rango geográfico de latitud/longitud. Los índices de listas se filtran a 0..N-1; los índices escalares solo se corrigen si exceden N, por lo que negativos distintos de `-1` pueden persistir.
- El gateway calcula yaw automático siguiendo la tangente de la ruta en convención ENU —0° este, 90° norte—, conserva y normaliza yaw manual, y usa pose/heading actual como fallback para un punto aislado.
- Solo normaliza dos acciones de waypoint en la entrada WebSocket: `brake_hold` —duración >0 y <=600 s, freno limitado— y `set_navigation_profile` con `urban/rural`. HOME no puede llevar acciones. El helper de archivo conserva listas de acciones sin validar internamente su esquema; la entrada efectiva determina dónde ocurre la validación.
- `datum_file_utils.py` sí valida rango lat/lon, nombre, números finitos, yaw, IDs únicos y YAML seguro. Seleccionar un datum solo modifica el archivo: el payload declara `apply_mode=next_restart` y calcula `pending_restart` contra el datum runtime.
- Las escrituras YAML crean directorios y reemplazan directamente el archivo, sin temporal+rename atómico. Un fallo durante escritura podría dejar contenido parcial.
- GeoJSON exige `FeatureCollection`; la conversión de estado admite Polygon/MultiPolygon, usa el anillo exterior y omite agujeros. La validación geométrica definitiva al guardar pertenece a `zones_manager`.

### RTK, cámara y datos sensibles

- La selección/gestión RTK viaja en tópicos `String` con JSON. `upsert_rtk_source` acepta usuario y contraseña y los publica en texto dentro del grafo ROS. El gateway no registra esos valores por sí mismo, pero cualquier observador autorizado del tópico podría verlos; nunca deben copiarse a documentación o logs.
- El bridge HTTP de sensores es opcional, hace GET con timeout corto y exige objeto JSON. Sin bridge construye fallbacks parciales desde GPS/IMU; las vistas `topics`, `lidar` y `camera` del inspector están explícitamente no implementadas.
- Cámara convierte cada `Image` con `cv_bridge`, limita FPS y ancho, codifica JPEG/PNG y difunde base64. Las detecciones asocian dimensiones por timestamp con tolerancia de 250 ms y normalizan bounding boxes. No hay cola/backpressure específica por cliente más allá del lock de envío.

### Rosbag y sesiones de misión

- Los perfiles rosbag son listas fijas; el nombre se valida y cada ruta/tópico se cita antes de formar `bash -lc`. El proceso usa su propio grupo, espera 0,4 s para detectar fallo temprano y al parar escala `SIGINT`, `SIGTERM` y finalmente `SIGKILL`.
- Las sesiones JSONL se guardan en `/tmp/mission_sessions`, empiezan solo con `GOAL_ACCEPTED` y terminan ante resultado/cancelación/rechazo. Deduplican snapshots de telemetría y diagnósticos por claves, pero cargan una sesión completa en memoria para descargarla y no imponen límite de tamaño.
- La limpieza se ejecuta antes de crear una sesión y borra mientras existen más de siete. Por el orden actual, en régimen estable pueden quedar ocho archivos después de crear el nuevo.
- La escritura de JSONL/status no usa archivo temporal ni un lock que cubra toda la operación de E/S; callbacks concurrentes comparten el mismo destino. Las rutas de descarga sí restringen el nombre a basename con sufijo `.jsonl`.

### Pruebas y packaging

- Los tests cubren YAML, perfiles/acciones, yaws automáticos, payloads GPS/batería/ruta bloqueada, filtro de diagnósticos y comienzo/fin de sesión. No cubren autenticación —no existe—, orden de operaciones WebSocket concurrentes, ciclo real de rosbag, carga/backpressure de cámara ni corrupción por escrituras simultáneas.
- `setup.py` instala también `web/*`, aunque el HTML se excluye de esta auditoría por pedido expreso. El paquete declara MIT, pero mantiene mantenedor `TODO`.

## 7. `navegacion_gps`: packaging y configuración

### Packaging

- El marcador `resource/navegacion_gps` está vacío, como corresponde al índice Ament. `setup.py` instala launches, configuración, mundos, modelos y scripts, y declara 39 `console_scripts`; por tanto el mapa de ejecutables no se deduce solo de los nombres de módulos Python.
- Hay divergencia de metadatos: `package.xml` conserva versión `0.0.0` y campos `TODO`, mientras `setup.py` declara `0.0.1` y otro mantenedor. Ninguno debe usarse como evidencia suficiente de release o autoría.
- La entrada `datum_setter` está rotulada como legacy en el packaging. Que siga instalada no la vuelve parte del perfil `*_global_v2` vigente.

### Localización y contratos de frames

- `localization_v2.yaml` define la capa local continua en `odom`: fusiona pose/velocidad de ruedas con velocidad angular de IMU y publica `odom -> base_footprint`.
- `localization_global_v2.yaml` define la capa global en `map`: de `/odometry/local_global` toma `vx`, la restricción no holonómica `vy` y `vyaw`, de GPS toma solo `x/y`, y de `/imu/data` solo `yaw_rate`. No copia `x/y/yaw` absolutos del EKF local. Los launches pueden inyectar además heading GPS y un pseudo-yaw de reposo; esa activación se verificará al revisar los launches.
- `dual_ekf_navsat_params.yaml` es el contrato anterior: el EKF local consume `/odom`, el global también toma yaw/velocidades locales, `navsat_transform` publica UTM, usa yaw de odometría, declinación `-0.122` y un datum fijo. No es semánticamente intercambiable con `localization_global_v2.yaml`.
- Las tres variantes `sim_decouple_*` y `sim_navsat_imu_heading` son overrides experimentales acotados de la fusión; no constituyen launches autónomos.
- `datums.yaml` conserva tres capturas RTK casi coincidentes y selecciona la última. Eso documenta alternativas del archivo, no prueba qué datum tiene cargado un proceso vivo.

### Nav2, cinemática y costmaps

- Los perfiles base usan Smac Hybrid A* con modelo `DUBIN`, Pure Pursuit regulado y reversa nominal deshabilitada. `nav2_global_v2_params.yaml` trabaja en `map`; `nav2_local_v2_params.yaml` mantiene navegación global/local en `odom`; `nav2_no_map_params.yaml` vuelve a `map` con costmaps rolling y una huella/inflación más conservadoras.
- Los cuatro perfiles `*_rolling*` agregan `path_clearance_validator`, lista explícita de plugins BT, waypoint follower y costmap global móvil de 300×300 m a 0,25 m. Planner y smoother usan radio mínimo de 4 m; los perfiles WiFi llevan velocidad nominal a 1,6 m/s, lookahead 2,3–4,5 m y tiempo de colisión 1,5 s.
- Los perfiles WiFi reducen ancho de banda: publicación global a 0,5 Hz y voxel debug deshabilitado; amplían el costmap local a 30×30 m y agregan padding de huella de 0,05 m. Aun así fuerzan mapas locales completos para evitar costmaps latcheados con timestamp viejo en RViz.
- `nav2_global_v2_sim_rolling_params.yaml` incorpora `DenoiseLayer` con grupo mínimo de dos celdas y persistencia local instantánea; su par real no WiFi no incorpora esa capa y extiende marcado LiDAR a 15 m apoyándose en el filtro de suelo. La paridad declarada en comentarios no significa identidad de parámetros.
- Todos los YAML rolling, incluso los llamados `real`, contienen `use_sim_time: true`. El comportamiento real depende de que el launch lo sobrescriba; esa dependencia queda pendiente de cruzar con los 24 launches.
- Los costmaps separan marcado y clearing en varias variantes para que retornos infinitos limpien sin marcar. Los keepout filters usan `/costmap_filter_info`, costo letal 254 y tolerancia TF de un segundo.

### Collision monitor, LiDAR y bridge Gazebo

- `collision_monitor.yaml` es el perfil sim con LiDAR y cinco ultrasonidos. `collision_monitor_lidar_only.yaml` retiene solo LiDAR. `collision_monitor_v2.yaml` define la variante real con dos zonas de stop y dos escalones de reducción —0,4375 y 0,75—; los polígonos publicados alimentan los displays RViz de stop/slowdown.
- Las cuatro configuraciones `pointcloud_to_laserscan*` convierten nube 3D a `/scan`; las reales recortan altura 0,50–1,50 m, mientras simulación y LiDAR inclinado usan ventanas distintas. `scan_ground_filter.param.yaml` filtra en `base_footprint` con pendientes generales/locales de 10°/13° y parámetros geométricos Ackermann.
- `bridge_config.yaml` está ligado al mundo/modelo legacy `pasillos_obstaculos`; `bridge_config_v2.yaml` usa la topología global v2, pero conserva un tópico Gazebo de clock fijado al mundo `vacio`. Cambiar de world exige verificar esas rutas de bridge.
- `cyclonedds_wifi.xml` autodetecta interfaz, limita fragmentación a MTU 1400 y usa multicast solo para SPDP. `cyclonedds_lan.xml` fija `eth0`/`eno1` y peers `192.168.1.208` y `192.168.1.195`; son topología histórica versionada, no observación de red actual.

### Árboles BT, benchmarks y zonas

- Los árboles de NavigateToPose y NavigateThroughPoses evitan `Spin` y usan `IsPathClearanceValid`. El árbol con trazas envuelve tres causas de replan —goal actualizado, clearance inválido y path inválido— con `TraceReplan`; sus recoveries limpian costmaps y esperan, sin maniobra de giro in-place.
- `nav_benchmark_scenarios.yaml` define perfiles `smoke`, `heading_core`, `regression_core` y `full`, con ocho escenarios desde reposo de 12 s hasta recta de 25 m. Son especificaciones de ensayo; no contienen resultados ni demuestran que una combinación actual haya pasado.
- `no_go_zones.yaml` contiene dos polígonos diferentes con el mismo `id: zone_2`. Cualquier consumidor que indexe por ID puede sobreescribir o volver ambigua una zona. El GeoJSON asociado usa coordenadas geográficas y el YAML usa el esquema interno; no deben mezclarse sin pasar por la conversión/validación del gestor.
- `keepout_mask.yaml` describe una máscara de 3000×3000 a 0,1 m/píxel y origen `[-150, -150, 0]`. `keepout_mask.pgm` es PGM raw binario de 9.000.017 bytes: se excluye correctamente de las 536 entradas de texto y se registrará por tamaño/hash en el manifiesto binario.

### RViz y Mapviz

- `rviz_local_v2.rviz` usa fixed frame `odom` y muestra odometría local, LiDAR 3D/2D, plan y debug de tracking. `rviz_global_v2.rviz` usa `map` y superpone odometría local/global, proyección GPS, costmaps, misión y chunk activo.
- Las variantes WiFi reducen frame rate y privilegian `/scan_wifi_debug`; la variante `wifi_2d` omite el display de global costmap, y `wifi_scan_ground` agrega deliberadamente `/scan_3d/no_ground`, rotulado en el archivo como pesado para WiFi.
- `rviz_nav2_full.rviz` es una vista Nav2 amplia con `/map`, máscara keepout, cost cloud, footprints y panel `Navigation 2`. Incluye tópicos y QoS distintos de las vistas global-v2, por lo que no debe asumirse como el viewer operativo de esos perfiles.
- `.mapviz_config`, igual que el archivo Mapviz de raíz, fija/centra en `odom` pero publica clicks en `map`; interpretar un click exige tener disponible y fresca la transformación `map -> odom`.

### Launches y composición efectiva

- Los 24 launches se recorrieron completos. La familia vigente es `sim_global_v2*`/`real_global_v2*`; `sim_local_v2`/`real_local_v2`, `simulacion.launch.py`, `real.launch.py` y `nav2_only.launch.py` quedan como perfiles locales o históricos. Que sigan instalados no los vuelve equivalentes al global v2.
- Varios launches resuelven primero `src/navegacion_gps/config/<archivo>` cuando el share instalado permite deducir un workspace. Esto facilita `--symlink-install`, pero también significa que un proceso puede consumir el source del checkout y no la copia del overlay. La ruta efectiva debe verificarse al diagnosticar parámetros.
- `localization_v2.launch.py` compone odometría Ackermann y EKF local. `localization_global_v2.launch.py` la incluye y agrega `navsat_transform`, EKF global, gates estacionarios y fuentes opcionales de yaw GPS/brújula y GPS absoluto. Por defecto activa los gates de odometría, IMU y yaw en reposo; deja apagados brújula/course heading/GPS absoluto a nivel del launch base, para que el wrapper operativo los decida.
- `nav_local_v2.launch.py` y `nav_global_v2.launch.py` instancian explícitamente planner, controller, smoother, BT navigator, behaviors, waypoint follower, collision monitor y sus lifecycle managers. Global agrega `path_clearance_validator` y republica zonas stop, critical-slow y slow; local solo republica stop. Keepout se sirve en `odom` para local y en `map` para global.
- La elección del override Nav2 con/sin keepout en ambos launches compara literalmente contra `"True"`. Wrappers propios usan esa capitalización, pero un override manual en minúsculas puede elegir el archivo sin keepout aunque `IfCondition` sí lo considere verdadero.
- `real_global_v2.launch.py` es el ensamblado real principal: MAVROS, RTK, RS16, cámara opcional, filtro LiDAR, controller UART, heading GPS/brújula opcional, `nav_command_server`, `route_executor`, observabilidad, localización global, Nav2 global y gateway web. Sus defaults efectivos son RTK, GPS absoluto, course heading, filtro de suelo, cámara, web y debug WiFi activos; keepout, RViz, brújula y `lidar_obstacle_filter` inactivos. El course heading exige por defecto un estado RTK reciente y fuerza a levantar la cadena RTK si hiciera falta.
- El pipeline LiDAR real impide activar simultáneamente `scan_ground_filter` y `lidar_obstacle_filter`, rechaza salidas que colisionen con `/scan`, `/scan_3d` o `/scan_3d/no_ground`, y elige para Nav2 entre `/scan`, el scan limpio o el scan filtrado. El URDF real v2 modela el pitch de 10° del RS16; reemplazarlo por el URDF plano cambia la geometría usada para quitar suelo.
- `sim_global_v2.launch.py` mantiene la misma arquitectura de control/misión sobre Gazebo. Sus defaults activan GPS absoluto, course heading, perfil GPS `f9p_rtk`, keepout, gateway web y grabación de trazas; dejan apagados filtro de suelo, `lidar_obstacle_filter` y brújula simulada. Cuando hay trazas selecciona un BT ThroughPoses instrumentado y guarda bajo `/ros2_ws/artifacts/nav_traces`.
- Los wrappers `*_global_v2_wifi.launch.py` no duplican nodos: fijan los YAML WiFi y reenvían argumentos al launch global. Ambos activan un `/scan_wifi_debug` frontal, reducido a 2 Hz y stride 4. El real WiFi mantiene keepout apagado; el sim WiFi lo enciende. Los viewers WiFi son wrappers RViz separados y no prueban que la pila de navegación esté levantada.
- `sim_v2_base.launch.py` concentra Gazebo, bridge de world, spawn, joint states y las alternativas de conversión LiDAR. Reescribe a un YAML temporal los tópicos dependientes del nombre real del world. `sim_local_v2`/`real_local_v2` superponen control, localización y Nav2 en `odom`; los dos filtros de suelo alternativos también son mutuamente excluyentes allí.
- `replay_localization_global_v2.launch.py` adapta el launch global a bags: `use_sim_time=true`, GPS absoluto y course heading activos, requisito RTK apagado y signo de steering invertido. Es una composición de replay, no el perfil de hardware.
- Los launches `validate_scan_ground*` solo preparan escenarios/KPI. El de simulación puede publicar `map -> odom` identidad para evitar que el BT entre en recovery por falta de TF; el real únicamente adjunta el nodo de validación a una pila ya levantada.
- `real.launch.py` y `simulacion.launch.py` conservan el ensamblado anterior directo de EKF/Nav2/zonas/snapshot. El real permite alternar MAVROS con `pixhawk_driver`; el sim conserva `realism_mode`, perfiles GPS y overlays experimentales de localización. Son referencia/legacy frente a global v2.

### Modelos URDF

- Los seis URDF se recorrieron completos. `cuatri.urdf`, `cuatri_real.urdf`, `cuatri_real_v2.urdf` y `cuatri_ultrasound.urdf` forman la familia Ackermann grande: distancia entre ejes 0,94 m, trocha 0,75 m, radio de rueda 0,24 m y dirección limitada a aproximadamente ±30°. El plugin de Gazebo consume `/cmd_vel_steer`, publica odometría y deja desactivada la TF de odometría para no competir con la localización ROS.
- `cuatri_real_v2.urdf` es la variante que representa el RS16 inclinado 10° en `lidar_link`. Frente a `cuatri_real.urdf`, también centra lateralmente la IMU y desplaza la antena GPS. Aunque los nombres digan `real`, ambos archivos incluyen plugins y sensores Gazebo; describen geometría y simulación, no prueban la disposición física actual.
- `cuatri_ultrasound.urdf` agrega cinco sensores modelados como rayos de corto alcance: trasero central, trasero izquierdo/derecho y delantero izquierdo/derecho. Publican bajo `/ultrasound/*`, a 10 Hz y con rango 0,05–2 m. No existe un ultrasónico delantero central en ese modelo, lo que coincide con los cinco tópicos de la configuración de collision monitor de simulación.
- `modelo.urdf` es otro Ackermann, sensiblemente menor —wheelbase 0,60 m, separación 0,56 m y radio 0,15 m—, con cámara además de LiDAR/IMU/GPS. Su plugin sí publica `odom -> base_footprint` y TF de ruedas. No debe sustituir a los `cuatri*` dentro de una pila que ya deja esa TF a `robot_localization`, porque introduciría dos autoridades potenciales.
- `my_robot.urdf` es un prototipo independiente de 1,6 m de chasis y ruedas de radio nominal 0,30/0,32 m. Aunque define articulaciones delanteras orientables, el único plugin motriz es diferencial sobre las ruedas traseras, escucha `/cmd_vel`, publica `/odom` y su TF, y no conecta la dirección delantera al controlador. No implementa el contrato Ackermann de la familia operativa.
- Hay diferencias deliberadas de sensores: los `cuatri*` simulan un LiDAR frontal de 360×8 rayos y 20 m; `modelo.urdf` usa un bloque equivalente; `my_robot.urdf` aumenta a 720×16 y 50 m. Las cámaras solo aparecen en `modelo.urdf` y `my_robot.urdf`. Por eso cambiar de URDF altera tópicos, carga, ruido y autoridad TF, no solo la apariencia visual.
- En los `cuatri*` la carrocería se desplaza respecto de `base_link`, pero `base_footprint` y `base_link` se unen con origen coincidente; `modelo.urdf` repite un origen coincidente y `my_robot.urdf` usa 0,08 m en Z. Es un contrato geométrico que debe cruzarse con TF observada antes de atribuir alturas o contacto con suelo al hardware.

### Mundos y validación A/B del filtro de suelo

- Los seis mundos y `scripts/run_scan_ground_validation.sh` se recorrieron completos. `vacio.world` es el escenario plano de 2 km por lado y `slope_lidar.world` conserva el mismo origen geográfico de Córdoba, pero agrega una rampa ancha de 10° y dos obstáculos elevados. Este último es el fixture específico del ensayo de falsos positivos del filtro de suelo.
- `pasillos_obstaculos.world` define tres carriles de seis metros entre cuatro muros, con uno, tres y tres obstáculos estáticos por carril. `default.sdf` usa el mismo nombre interno `pasillos_obstaculos`, pero no es una copia: sus muros y varios obstáculos tienen poses diferentes —incluido un bloque central rotado y elevado casi a 2 m— y usa SDF 1.9/ODE en lugar de SDF 1.6/DART. Debe tratarse como una captura/variante separada, no como alias nominal.
- `tugbot_depot.world` incluye por URL el mundo remoto Tugbot Depot, pero declara internamente `world name="pasillos_obstaculos"`. `sonoma_salus.world` incluye por URL Sonoma Raceway y cambia el origen NavSat a California. Ambos necesitan acceso al recurso remoto o caché de Gazebo y el nombre efectivo del world debe obtenerse del SDF, no inferirse del archivo.
- Los orígenes geográficos tampoco son uniformes: `vacio`/`slope_lidar` usan aproximadamente -31,4858/-64,2411; `pasillos`, `default` y `tugbot_depot` usan capturas cercanas a -31,4218/-64,1025; Sonoma usa 38,1606/-122,4540. Una misma pose local produce fixes GPS distintos según el mundo.
- El script A/B ejecuta baseline y filtro durante 60 s por defecto, concede otros 60 s al arranque y compara seis KPI JSON. La propia nota aclara que sin mandar una meta solo mide falsos positivos estáticos; el script no conduce al robot por sí mismo.
- Cada launch está envuelto en `timeout ... || true`, de modo que timeout o fallo no detienen el ensayo. Además, el script reutiliza el directorio de salida sin borrar `baseline.json`/`filtered.json`: un archivo de una corrida anterior puede superar el `-f` y entrar en la comparación si la corrida nueva no lo reemplaza. La presencia de ambos JSON no demuestra por sí sola que los dos casos actuales se ejecutaron.
- El comparador solo captura errores de apertura; un JSON truncado o inválido propaga la excepción. La métrica porcentual se omite cuando el baseline vale cero, por lo que ese caso necesita lectura de valores absolutos y no puede interpretarse como variación del 0 %.

### Módulos Python de navegación

Los 46 archivos y sus 22.621 líneas se recorrieron completos. La responsabilidad individual de cada ruta está en `CODE_CATALOG_OWN.md`, sección `navegacion_gps: módulos Python`; aquí se documentan los contratos cruzados y las ramas que no son evidentes desde el nombre del archivo.

#### Odometría, GPS y heading

- `ackermann_odometry.py` integra con método de punto medio la rapidez medida y el ángulo de dirección de `DriveTelemetry`, descarta intervalos mayores al máximo configurado y publica `/wheel/odometry`; no publica TF. La rapidez llega como magnitud y el signo de reversa se reconstruye con `reverse_requested`.
- Los gates estacionarios de IMU, odometría y yaw exigen telemetría fresca antes de imponer velocidad cero o una medición yaw-only. En cambio, `gps_course_heading.py` conserva indefinidamente el último estado de frescura/steering recibido: el nodo no aplica una edad propia a `DriveTelemetry`. También acepta fixes por lat/lon finitos sin comprobar `NavSatStatus`.
- El estimador de course heading requiere baseline, velocidad mínima, steering pequeño, yaw-rate bajo y —según perfil— RTK reciente. `compass_heading_gate.py` bloquea la brújula cuando existe heading GPS válido y solo la admite durante startup o reposo prolongado; su aceptación actual sí exige muestras recientes de compass, IMU y drive.
- `compass_calibration_recorder.py` empareja brújula y heading GPS en una ventana de un segundo, pero velocidad, dirección y yaw-rate no tienen timestamp ni control de edad propio. Una telemetría de movimiento vieja puede pasar el filtro de una muestra nueva. La duración usa reloj ROS y el JSON se reemplaza directamente, sin escritura atómica.
- `map_gps_absolute_measurement.py` mantiene una llamada `/fromLL` en vuelo y solo el fix pendiente más reciente. Si el servicio falla usa una aproximación geográfica local; valida finitud/rangos numéricos pero no el status del fix. `gps_profiles.py` implementa `ideal`, `f9p_rtk`, `m8n` y custom con ruido, sesgo y rate limiting; seed cero significa aleatoriedad no determinística.
- `datum_profile_resolver.py` prefiere el `config/` del source cuando puede deducir el workspace y cae a un datum Córdoba fijo ante excepciones amplias. `datum_setter.py` es legacy: conserva el último RTK textual/NavSat sin caducidad, acepta un fix finito aunque sea `NO_FIX` y puede disparar datum sobre ese estado histórico. Los perfiles global-v2 vigentes no lo levantan por defecto.

#### Comandos, seguridad y máquina de misión

- `nav_command_server.py` es la autoridad entre `/cmd_vel_safe` y `/cmd_vel_final`, manual, freno, conversión LL→map, acciones Nav2, loops segmentados y recovery BackUp. El manual tiene watchdog monotónico; collision STOP y critical-slow se convierten a comandos con freno explícito. El recovery de retroceso está deshabilitado por defecto.
- La cancelación bloqueante considera éxito cualquier respuesta no nula de `cancel_goal_async()` y no inspecciona `goals_canceling`. El servicio `cancel_goal` responde `ok=true` antes de conocer el resultado asíncrono, incluso si no había handle activo.
- Los callbacks de resultado Nav2 no conservan la identidad del goal handle que los creó. Tras cancelar un goal y aceptar otro, un resultado tardío del anterior puede limpiar `_current_goal_handle`, cambiar modo/resultado y aplicar freno sobre el nuevo. El contador global usado durante collision recovery también depende del orden de llegada de resultados.
- La conversión geográfica acepta coordenadas finitas pero no comprueba rangos lat/lon. El fallback aproximado puede continuar cuando `/fromLL` no está disponible, aunque `_resolve_fromll_client()` ya haya emitido un evento `FROMLL_FAILED`; queda una alerta de error aun si luego el goal aproximado se acepta.
- `route_executor.py` expande segmentos, conserva checkpoints sintéticos/reales, despacha chunks, ejecuta `brake_hold`/cambio de perfil, reancla retries bloqueados, y modela patrulla `HOME`, conectores de salida/retorno y retorno por batería. Los servicios de estado exponen ruta expandida, acciones, roles, índices fuente, progreso, cross-track y fases HOME.
- Los cambios `urban/rural` actualizan tres nodos mediante el servicio no atómico `SetParameters`. Si un batch devuelve resultados mixtos, el rollback no revierte el mismo costmap que pudo quedar parcialmente actualizado; en algunas ramas solo revierte ground filter y/o el otro costmap. Además, varias fallas posteriores al dispatch limpian la misión sin restaurar siempre el perfil urbano.
- La ruta común valida finitud pero no rango geográfico. La validación de patrulla ni siquiera recorre lat/lon/yaw para exigir finitud antes de construir `RouteWaypoint`; solo HOME tiene esa comprobación explícita.
- Tras recibir una vez `BatteryMissionGuard`, `_battery_guard_seen` desactiva para siempre el fallback desde `BatteryState`. Si después el guard queda stale, unavailable o deja de publicar, esas muestras se ignoran y el SOC genérico ya no vuelve a conducir la protección.

#### Costmaps, LiDAR y zonas

- `path_clearance_validator.py` muestrea el path y offsets laterales hasta una distancia máxima, distingue costo letal de costo alto sostenido y cachea por firma/path+costmap. Falla abierto —responde válido— si falta/venció el costmap o no hay TF; las muestras fuera de los límites del costmap también se omiten.
- `scan_ground_filter.py` porta el clasificador radial non-grid de Autoware, transforma primero a `base_footprint` y permite cambiar tres umbrales de perfil en runtime. `scan_noise_filter.py` elimina rangos inválidos y speckles sin soporte; `scan_wifi_debug.py` recorta y diezma para observación remota.
- `lidar_obstacle_filter.py` combina compensación IMU, RANSAC de suelo, densidad y persistencia temporal antes de producir cloud/scan. Declara `voxel_size_z`, pero tanto densidad como persistencia agrupan solo X/Y. Cuando la IMU vence, devuelve roll/pitch cero y el tilt gate puede abrir sin una actitud fresca.
- `zones_manager.py` normaliza GeoJSON, convierte cada vértice con `/fromLL`, rasteriza anillos/huecos, agrega halo y recarga el map server. Admite éxito parcial de conversión si al menos un polígono queda; polígonos fallidos o totalmente fuera de máscara solo generan warning. Las escrituras PGM/YAML/GeoJSON no forman una transacción atómica.
- Con `clear_global_after_reload=false`, la recarga puede devolver éxito pero `_mask_ready` queda falso porque se calcula como `map_reloaded and global_cleared`. Si el mapa ya fue recargado y luego falla guardar GeoJSON, la función retorna antes de actualizar `_geojson_doc`: runtime, disco y estado consultable pueden divergir.

#### Observabilidad, trazas, simulación y benchmarks

- `nav_observability.py` publica seis diagnósticos de frescura/estado; en simulación marca controller físico como no esperado. `nav_snapshot_server.py` renderiza local/global/keepout/scan/plan a PNG: `snapshot_timeout_ms` solo dispara un warning después del trabajo, no cancela la generación. Si falta TF del robot usa el centro geométrico del costmap, por lo que la rama de error `missing TF` no representa el fallback real.
- `nav_trace_recorder.py` abre una sesión al evento de misión, copia params/BT con hashes, graba timeline/planes/chunks y detecta saltos, O-paths, intersecciones y bursts. Sus artefactos incluyen lat/lon de misión y se escriben directamente; son evidencia de una corrida, no estado actual.
- `gazebo_utils.py` normaliza sensores legacy y opcionalmente traduce `/cmd_vel_final`. En el modo realista llama `self.sim_max_steering_angle_rad`, atributo que nunca se declara ni asigna, por lo que ese callback puede fallar con `AttributeError`.
- `sim_global_straight_benchmark.py` busca el último evento terminal de toda la vida del nodo sin fijar un índice de inicio; un resultado anterior al goal medido puede cerrar la corrida nueva inmediatamente. Si vence el timeout tampoco solicita cancelación. `sim_localization_benchmark.py` orquesta el launch legacy `simulacion.launch.py`, no `sim_global_v2`.
- Los runners/reports de benchmark calculan deriva, estabilidad angular, saltos, lateralidad, heading y eventos, pero no prueban por sí mismos hardware ni runtime actual. `replay_localization_compare.py` alinea por timestamp más cercano dentro de tolerancia; un reporte válido depende de que ambos bags pertenezcan realmente al par grabación/replay que se pretende comparar.

### Tests de `navegacion_gps`

Los 42 archivos y sus 7.659 líneas se recorrieron completos. Son tests unitarios, dobles construidos con `object.__new__`, validaciones numéricas y contratos de texto; no sustituyen `launch_testing`, una ejecución dentro del overlay vigente ni una comprobación del grafo ROS/hardware.

- Odometría, gates estacionarios, brújula, course heading, GPS absoluto, perfiles GPS y conversión LL tienen casos nominales, umbrales, hold y muestras vencidas. Los tests de `gps_course_heading.py` validan estados RTK permitidos pero no que `DriveTelemetry` caduque; los de calibración de brújula vencen el par compass/GPS, no el estado de movimiento; y los de compass gate no reproducen una interrupción de telemetría durante el período estacionario. Por eso el hallazgo 25 no queda refutado.
- Los filtros de scan/LiDAR prueban recorte, speckles, suelo, RANSAC, densidad y persistencia. No ejercitan que `voxel_size_z` cambie el agrupamiento ni el callback del nodo con IMU vencida, por lo que el hallazgo 28 permanece.
- `test_path_clearance_validator.py` prueba explícitamente que el servicio responde válido sin costmap y con costmap stale, además de costos letales/sostenidos, offsets laterales, caché y evento `CLEARANCE_INVALID`. No prueba una TF ausente ni el efecto de muestras fuera del mapa; el fail-open del hallazgo 27 sí está confirmado por el propio test.
- Los tests de `nav_command_server` cubren manual/auto, watchdog, freno, collision monitor, backup, segmentos de loop, abortos, cancelación manual y fallback aproximado de `/fromLL`. Los dobles aceptan que modo manual quede habilitado aunque falle la cancelación y llaman al callback de resultado sin asociarlo a un handle concreto; no simulan dos goals superpuestos ni un resultado tardío, de modo que los hallazgos 30 y 31 siguen sin cobertura de regresión específica.
- `test_route_executor.py`, con 1.664 líneas, cubre expansión, chunks, checkpoints sintéticos, acciones, HOME/conectores, perfiles, bloqueos/retries, batería y fases de patrulla. Confirma expresamente que una muestra `BatteryMissionGuard` stale deshabilita el trigger porcentual legacy. El rollback testeado restaura el filtro de suelo si rechaza el costmap global, pero no verifica restauración transaccional de cada batch/nodo ya modificado ni todas las salidas de error; permanecen los hallazgos 33–35.
- Zonas prueban normalización GeoJSON, cierre de anillos, rangos, multipolígonos, huecos, buffer y degradación pura de máscara/YAML. No se instancia el gestor para simular fallas entre escritura, recarga y limpieza, por lo que no cubren divergencia de estado ni `mask_ready` de los hallazgos 36–37.
- Los tests de launch leen archivos como texto y buscan fragmentos literales de argumentos, parámetros, XML BT, RViz y wrappers. Son útiles como contrato de configuración versionada, pero no expanden `LaunchDescription`, no arrancan lifecycle nodes y no validan TF/tópicos efectivos.
- `test_gazebo_utils.py` crea un fake que asigna manualmente `sim_max_steering_angle_rad`; por esa razón el test del puente realista no detecta que el nodo real no declara el atributo. Los tests de benchmarking/snapshot/observabilidad tampoco ejercitan el timeout no cancelable del snapshot ni la reutilización de un evento terminal viejo.

## 8. `navegacion_gps_bt`: plugins BehaviorTree

Los seis archivos y sus 353 líneas se recorrieron completos. El paquete compila e instala dos bibliotecas compartidas y sus manifiestos de plugin; no contiene tests propios.

- `IsPathClearanceValid` consulta el servicio de clearance desde un `ConditionNode`. Conserva la misma política fail-open del servidor: un servicio ausente, no listo, vencido o con error no bloquea por sí solo el árbol.
- `TraceReplan` envuelve un hijo, publica eventos estructurados en `/navigation_trace/events` con QoS 50 y conserva causa, goal, path, duración, cancelación y secuencias globales de evento/replan. Los contadores son atómicos a nivel de proceso, no una identidad persistente entre reinicios.
- El header del decorador usa `diagnostic_msgs::msg::KeyValue` apoyándose en declaraciones/includes transitivos. Compila solo mientras sus dependencias sigan exponiendo ese tipo indirectamente; conviene incluir el header exacto.
- `package.xml` conserva un mantenedor de plantilla y no hay prueba automática del registro/carga de las dos bibliotecas en BehaviorTree.CPP.

## 9. `sensores`: MAVROS, RTK, cámara y contratos ONVIF

Se recorrieron los 24 archivos propios —4.674 líneas— y los 33 contratos WSDL/XSD importados —31.305 líneas—. `setup.py` instala recursivamente todos esos contratos, el dashboard y seis entry points. El HTML del dashboard y la imagen PNG se excluyeron por el alcance acordado; el YAML RTK se leyó sin reproducir credenciales.

### MAVROS y compatibilidad

- `mavros.launch.py` usa MAVROS como backend vigente y levanta por defecto el bridge de compatibilidad; source manager, bridge RTK TCP y servidor web son opcionales. Si se habilita el source manager, el bridge TCP directo se desactiva y consume el tópico ROS del manager.
- `mavros_compat_bridge.py` replica GPS, odometría y velocidad a contratos legacy y diagnostica frescura. Monitorea IMU pero no la republica porque el launch MAVROS remapea la salida IMU al nombre canónico.
- `rtk_bridge_core.py` prioriza `GPSRAW` fresco, luego diagnósticos RTK y finalmente `NavSatFix`; haber recibido RTCM recientemente no equivale a tener solución RTK.

### RTCM y selección de caster

- `rtk_bridge.py` acepta RTCM por TCP o tópico, busca preámbulo y longitud y reinyecta a MAVROS, pero no valida CRC-24Q. El fallback de `NavSatFix` no conserva un timestamp local de recepción: tras envejecer las fuentes con reloj propio puede clasificar usando un fix histórico. El worker TCP y callbacks ROS comparten estado sin una sincronización explícita.
- `rtk_source_manager.py` carga y guarda credenciales en YAML de texto plano, escribe el archivo directamente y publica catálogo/selección mediante JSON en tópicos ROS sin autenticación. Una entrada inválida puede incluir el diccionario recibido en la excepción; eso puede filtrar campos sensibles a logs.
- La selección/upsert también acepta mutaciones desde tópicos ROS. El ACK del dashboard indica solicitud aceptada antes de confirmar conexión al caster. El socket del worker se asigna después del handshake, lo que impide interrumpir rápidamente ese tramo; acepta cualquier primera línea que contenga `200`, puede entregar bytes de headers al parser y tampoco valida CRC RTCM.

### Cámara y servidores web

- `camara.py` implementa Hikvision ISAPI con Digest Auth, no usa los WSDL ONVIF empacados. Si el probe inicial falla, `_ready` queda falso y no existe reconexión periódica. La persistencia de presets reescribe el JSON directamente, sin reemplazo atómico.
- `web_server.py` escucha en `0.0.0.0`, permite CORS `*` y no implementa autenticación. Expone por HTTP/WebSocket telemetría y gestión RTK; el JSON puede contener credenciales del catálogo y refleja `requested_source`. No hay límite explícito de body y cada WebSocket se atiende secuencialmente.
- `pixhawk_driver.py` es legacy frente a MAVROS. Tras desconexión puede volver a publicar valores de sensor antiguos con stamp actual; el handler `GPS_RTK` no hace nada, la selección RTCM toma el primer tipo soportado sin CRC y la cola es no acotada. Varias covarianzas están fijas o en cero.
- Los dos archivos de test cubren presets/rollback de cámara y precedencia/estados del resolver RTK; no ejercitan sockets, reconexión, seguridad web, CRC ni el driver MAVLink completo.

### Contratos WSDL/XSD

- Los 33 archivos son XML bien formado según `xml.etree.ElementTree`; esto corrige la sospecha visual inicial de cierres faltantes. Son copias ONVIF/OASIS históricas y no lógica SALUS, aunque forman parte del paquete instalado.
- Varios `wsdl:service` fijan endpoints privados `192.168.0.51:8888`. No reflejan la cámara/host configurados en runtime y `camara.py` no los consulta.
- `recording.wsdl` declara `RecordingBinding`, pero su puerto de servicio referencia `DeviceBinding`. `media.wsdl` conserva varios `soapAction` aparentemente mal formados, con el separador ausente antes de nombres como `GetVideoSources`, `GetProfile` y opciones de video.
- `deviceio.wsdl` conserva el nombre `ForcePersistance` y una acción plural para `GetSerialPortConfiguration`; `advancedsecurity.wsdl` define bindings pero no un endpoint de servicio. Son defectos/inconsistencias del artefacto importado, no evidencia de un servicio ONVIF activo.

## 10. `vision_pipeline`: captura, inferencia y streaming

Los 14 archivos de texto backend y sus 2.323 líneas se recorrieron completos. Se excluyeron el README, el HTML del dashboard y el binario ONNX; el modelo se registrará por hash/tamaño. El paquete no contiene tests.

- Los launches de cámara IP construyen una URL RTSP que puede incorporar usuario/contraseña y la pasan como parámetro ROS. `ip_camera_publisher.py` registra la URL completa al conectar/reconectar, por lo que esas credenciales pueden llegar a logs y herramientas de introspección.
- El publicador usa threads separados para captura y publicación, reconecta OpenCV y puede caer a snapshots. `CameraInfo` se estima desde resolución/FOV, no desde calibración. No publica un health explícito y, después de tres errores, los fallos de snapshot quedan silenciosos.
- `yolo_onnx_detector.py` se deshabilita en ARM64, asume entrada NCHW y usa la primera salida del modelo. La NMS es independiente de clase; cajas solapadas de clases distintas pueden suprimirse. La rama end-to-end tiene supuestos de columnas/coordenadas que pueden no coincidir con todos los exports YOLO y no valida rangos de thresholds al inicio.
- El detector detiene su worker con un `join` acotado; no demuestra que la inferencia nativa haya terminado. Sin tests ni fixture del modelo, preprocessing, letterbox, salida end-to-end y providers quedan sin regresión automática.
- `vision_web_server.py` escucha en todas las interfaces, permite CORS `*` y no autentica. Marca health de imagen de forma acumulativa tras el primer JPEG, superpone la última detección sin sincronización temporal con el frame, codifica JPEG dentro del callback y no limita clientes de stream.

## 11. Vendor RoboSense: `rslidar_msg` y `rslidar_sdk`

Se recorrieron completos los 8 archivos de texto seleccionados de `rslidar_msg` y los 130 del SDK/driver. El detalle ruta por ruta está en `CODE_CATALOG_VENDOR.md`. El alcance incluye build ROS 1/2, mensajes, manager/sources/destinations, decoders de todos los modelos, inputs UDP/PCAP/raw, Linux/Windows, demos, herramientas, tests, RViz y configuración. No incluye manuales upstream ni imágenes, que no son lógica ejecutable. SALUS usa RS16; los demás modelos se documentan porque se compilan dentro del vendor multiproducto.

### Integración ROS, build y tipo de punto

- El `package.xml` superior declara versión `1.5.16`, mientras `CHANGELOG.md`, el CMake y el header de versión del driver embebido llegan a `1.5.18`. La identidad del paquete ROS no coincide con la librería realmente compilada.
- El CMake superior usa por defecto `POINT_TYPE=XYZI`. El decoder RS16 calcula `ring` y timestamp por punto, pero esa información no llega a `sensor_msgs/PointCloud2` con el tipo por defecto. La transformación y la validación CRC están desactivadas por defecto; el parseo DIFOP y el aumento del receive buffer están activados.
- Los archivos generados `rs_driverConfig.cmake` y `rs_driverConfigVersion.cmake` viven dentro del árbol source ignorado. El primero conserva rutas absolutas de `/ros2_ws` y `/usr/local/rslidar_sdk`; no es un artefacto portable entre checkouts.
- `humble_start.py` intenta ejecutar `sudo apt-get update/install` durante el launch y altera la configuración CycloneDDS. No es un bringup hermético. SALUS normalmente integra el SDK mediante `sensores/launch/rs16.launch.py` y `sensores/config/rs16.yaml`, no mediante el launch/config upstream cuyo modelo inicial es RSM1.
- `NodeManager` lee `send_point_cloud_proto` y `send_packet_proto`, pero no crea destinos para esos flags. Las opciones aparecen aceptadas en configuración sin producir la salida indicada.
- La conversión ROS de timestamp redondea nanosegundos sin normalizar el caso `1.000.000.000`; un valor cerca del segundo siguiente puede producir un stamp inválido en vez de propagar el carry.

### Lifecycle, threads, colas y errores

- `NodeManager::~NodeManager()` detiene las sources y luego las destruye; `SourceDriver::~SourceDriver()` vuelve a ejecutar `stop()`. Este último hace `join()` incondicional del thread de point cloud, sin comprobar `joinable()`: la doble parada puede lanzar o terminar el proceso.
- Si se habilita IMU, `SourceDriver` crea un thread adicional pero `stop()` no lo une. Destruir el objeto con ese thread joinable puede terminar el proceso. La variante ROS 2 de `SourcePacketRos` también conserva un thread de spin sin destructor que lo cancele ni lo una explícitamente.
- Diversos flags de parada/estado se leen y escriben entre threads como `bool` ordinarios. Eso introduce data races en driver, inputs y demos. Los demos usan además un `popWait()` que puede quedar bloqueado al intentar una salida ordenada.
- La copia de `Packet` conserva solo el buffer y pierde timestamp, secuencia, DIFOP/frame-begin y frame id. Un `Error` construido por defecto deja su categoría sin inicializar. Varias rutas invocan callbacks sin guard y ciertos parámetros inválidos llaman a `exit(-1)` desde la librería.
- Cuando se agota el pool de packets, la recuperación vacía toda la cola pendiente. No se limita a descartar una muestra y puede provocar un salto grande en la secuencia procesada.
- Los getters de point cloud e IMU esperan en loop hasta que el callback entregue un objeto, sin backoff propio. Su progreso depende por completo de que el consumidor reponga buffers.

### Decoders y modelo RS16

- RS16 espera datagramas de 1.248 bytes, 16 láseres, 12 bloques y 32 canales por bloque. Calcula azimuth interpolado, distancia, coordenadas, ring y tiempo por punto; con el build SALUS `XYZI`, solo XYZ/intensidad quedan en la cloud publicada.
- Los iteradores guardan offsets/durations en arrays fijos de hasta 12 bloques y dependen de `assert`; un layout nuevo o inválido en una build sin asserts no queda protegido por una comprobación runtime equivalente.
- RS32 reporta el código de error de MSOP id cuando encuentra un block id inválido, en vez del código específico de block. RS80/RS128 silencian deliberadamente ese reporte. RSP48/RSP80/RSP128 caen a retorno dual ante un modo desconocido.
- En RSP80 los arrays temporales parten en cero y solo se recalculan cuando cambia `lidar_model` respecto del valor inicial; un header de modelo cero puede dejar timings por canal en cero. En RSM3 se calcula temperatura pero no se marca `is_get_temperature_`, por lo que `getTemperature()` permanece falso.
- RSMX declara coordenadas con signo, pero decodifica X en `uint16_t`; un X negativo se convierte en un valor positivo grande. AIRY reconoce el enum de 192 canales al inspeccionar el packet y luego lo rechaza en decode; para 96 canales, la inicialización depende de disponer de install info/modo y después no se adapta a un cambio tardío. Su `isNewFrame()` compara un solo byte de un block id de dos bytes.
- La inicialización lazy de parámetros constantes usa un `bool` estático no atómico. Dos primeras construcciones concurrentes pueden competir. Los helpers CRC/entero reinterpretan punteros de bytes como `uint32_t*`, con riesgo de alineación y aliasing en plataformas estrictas; CRC no está activo por defecto.

### UDP, PCAP, raw, Linux y Windows

- Los inputs raw restan offset/tail sin validar tamaño y copian a buffers fijos sin comprobar capacidad. Un paquete menor que esos offsets puede producir underflow; uno mayor puede desbordar el destino.
- Los parsers PCAP normal y jumbo confían en `header->len` en vez de `caplen` y no validan que existan headers Ethernet/IP/UDP completos antes de leer/copiar. Una captura truncada o malformada puede causar acceso fuera de límites; jumbo acumula en un buffer fijo de 65.536 bytes sin una protección integral de longitud.
- Los resultados de `pcap_compile` se ignoran, los programas de filtro no se liberan y el replay usa pausa fija aproximada en lugar del argumento de delay disponible.
- En Linux epoll, el evento IMU guarda por error el fd DIFOP, nunca asigna `fds_[2]` al fd IMU y el array se inicializa parcialmente como `{-1, 0, 0}`. El destructor puede cerrar stdin mientras el socket IMU queda abierto. También hay caminos donde `epfd_` queda sin inicializar y no se comprueban retornos de `epoll_ctl`.
- En select Unix, un fallo al consultar/fijar receive buffer retorna sin cerrar el socket. En Windows, `WSAStartup` se comprueba con `< 0` aunque los errores son códigos no cero, no se llama `WSACleanup`, `SOCKET` se almacena en `int` y se repiten el underflow de offsets y la inicialización parcial de arrays de fds.

### Demos, herramientas, build Windows y tests

- `rs_driver_pcdsaver` y `rs_driver_viewer` arrancan su thread consumidor antes de comprobar `driver.init()`. Si init falla, queda un `std::thread` joinable al salir; si sí arranca, el `popWait()` puede impedir que `join()` termine. En el saver, el loop infinito deja cleanup normal inalcanzable.
- Los proyectos Visual Studio fijan toolset v142 y rutas locales a WpdPack, PCL, OpenNI y una lista rígida de librerías VTK/PCL. Las configuraciones Win32 no son equivalentes a x64; `demo_pcap` Release x64 tampoco define `ENABLE_PCAP_PARSE` como Debug x64.
- `create_debian.sh` elimina directorios de empaquetado y ejecuta instalaciones con `sudo`; es una herramienta mutante, no parte de una auditoría segura. `dds_mod.sh` pone el shebang después de 32 líneas de comentarios, por lo que la ejecución directa puede fallar con format error, y ejecuta apt sin `-y`.
- El CMake de tests referencia `../test/res/angle.csv`, pero ese recurso no existe en este checkout. La suite cubre decoders RS16/RS32/RSBP y utilidades, no lifecycle/IMU, límites de input ni la mayoría de modelos. El test del iterador dual repite `iter.get(2)` donde el comentario anuncia el cuarto bloque, reduciendo la cobertura efectiva.

## 12. Cierre de cobertura

Quedaron leídos todos los archivos de texto del backend acordado. La selección exacta, sus líneas, bytes y hashes está en `BACKEND_LINE_MANIFEST.tsv`; los artefactos binarios asociados están en `BACKEND_BINARY_INVENTORY.tsv`. Los frontend HTML/CSS/JavaScript, Cockpit, manuales Markdown upstream e imágenes de documentación permanecen deliberadamente fuera del alcance y no se presentan como código leído.

## 13. Riesgos confirmados

1. CI incompleto para dos paquetes propios.
2. Overlay instalado potencialmente viejo frente al source.
3. Scripts de parada con coincidencias amplias y `SIGKILL`.
4. Readiness de simulación basada en esperas fijas.
5. Unidad systemd del monitor con rutas de otro checkout/usuario.
6. Variantes DDS cuyo nombre de wrapper no coincide en dos casos con el XML efectivo.
7. Compose privilegiado y con acceso amplio al host.
8. Simulación acoplada prematuramente a la resolución de un puerto serial.
9. Parámetros de límite angular y freno de reversa aceptados pero actualmente inertes.
10. Solicitudes de freno parcial elevadas a E-stop y, por defecto, a 100 % de freno.
11. Dos campos llamados `mission_guard_state` con dominios semánticos distintos.
12. WebSocket operativo en todas las interfaces sin autenticación/TLS propios y con lock deshabilitado por defecto.
13. Mutaciones WebSocket procesadas concurrentemente sin garantía de orden entre solicitudes.
14. Credenciales RTK transportadas como JSON en un tópico ROS de texto.
15. Escrituras YAML/JSON de estado sin reemplazo atómico y sesiones sin límite de descarga.
16. Dos zonas no-go diferentes comparten `id: zone_2`.
17. Los YAML Nav2 reales conservan `use_sim_time: true` y dependen de override del launch.
18. Configuraciones DDS/bridge contienen interfaces, peers, mundos y modelos históricos fijos.
19. Las vistas Mapviz mezclan frame de visualización `odom` con click publicado en `map`.
20. Varios launches pueden preferir configuración del source a la copia instalada del overlay.
21. La selección de override keepout compara literalmente `True` y puede divergir de `IfCondition` con otras capitalizaciones verdaderas.
22. Los URDF históricos `modelo.urdf` y `my_robot.urdf` publican TF de odometría propia y usan contratos de control distintos de global v2; cargarlos en la pila vigente puede crear doble autoridad TF o dejar la dirección Ackermann sin controlar.
23. El script A/B del filtro tolera fallos de launch y no limpia JSON previos, por lo que puede presentar resultados obsoletos como comparación actual.
24. Dos mundos dependen de descargas Fuel y `tugbot_depot.world` conserva un nombre interno de otro escenario; rutas de bridge derivadas solo del nombre de archivo pueden quedar apuntando a un world inexistente.
25. Course heading y calibración de brújula conservan estado de movimiento sin una caducidad local completa; una muestra nueva puede evaluarse contra steering/velocidad/yaw-rate viejos.
26. El setter de datum legacy conserva RTK/fix sin caducidad suficiente y no exige `NavSatStatus` válido.
27. `path_clearance_validator` falla abierto ante costmap/TF ausente o vencido y omite las muestras fuera del mapa.
28. `lidar_obstacle_filter` ignora el parámetro Z de voxel y ante IMU vencida puede abrir el tilt gate usando actitud cero.
29. El bridge realista legacy de `gazebo_utils` referencia un atributo de ángulo máximo inexistente.
30. Resultados Nav2 tardíos pueden modificar o frenar un goal posterior porque `nav_command_server` no liga cada callback al handle activo.
31. Los contratos de cancelación reportan aceptación antes de confirmar que Nav2 canceló realmente el objetivo.
32. Ruta y goal solo comprueban finitud, no rangos geográficos; la entrada de patrulla omite además validar finitud de sus arrays de waypoints.
33. El cambio de perfil de navegación puede dejar una combinación parcial de parámetros si un servicio `SetParameters` acepta unos campos y rechaza otros.
34. Una sola muestra de `BatteryMissionGuard` deshabilita permanentemente el fallback desde `BatteryState`, aunque luego el guard se vuelva stale o desaparezca.
35. Varias salidas de error de misión no restauran el perfil urbano, por lo que un perfil rural puede persistir después de abortar.
36. `zones_manager` admite polígonos omitidos con warning, no escribe sus tres artefactos de forma transaccional y puede divergir entre map server, disco y estado interno.
37. Con limpieza global deshabilitada, `zones_manager` puede devolver éxito mientras reporta internamente `mask_ready=false`.
38. El timeout de snapshot es solo diagnóstico y la ausencia de TF se sustituye silenciosamente por el centro del costmap.
39. El benchmark recto puede consumir un evento terminal anterior y no cancela el goal cuando vence su timeout.
40. El parser RTCM no valida CRC-24Q, el fallback puede usar un `NavSatFix` histórico y el bridge comparte estado entre threads/callbacks sin sincronización explícita.
41. El gestor RTK persiste credenciales en texto plano con escritura no atómica, admite administración por tópicos ROS sin autenticación y puede incorporar datos sensibles de una entrada inválida en excepciones/logs.
42. El dashboard de sensores está expuesto en todas las interfaces con CORS abierto y sin autenticación, puede transportar credenciales RTK y confirma solicitudes antes del resultado efectivo del caster.
43. Un fallo del probe inicial deja al nodo de cámara sin mecanismo periódico de recuperación y los presets se persisten sin reemplazo atómico.
44. El driver Pixhawk legacy puede presentar datos viejos con timestamp nuevo tras desconexión y deja sin implementar `GPS_RTK`.
45. Las URLs de cámara IP con credenciales viajan como parámetros ROS y se escriben completas en logs de conexión/reconexión.
46. La NMS de YOLO es agnóstica de clase y la rama end-to-end depende de convenciones de export no verificadas; el paquete de visión carece de tests.
47. El servidor web de visión es abierto, su health queda pegado tras el primer frame y combina detecciones/frames sin sincronización temporal.
48. Los WSDL empacados fijan endpoints privados históricos que no representan la cámara configurable del runtime.
49. Los contratos importados conservan referencias y acciones inconsistentes —incluido el puerto de `recording.wsdl` y varios `soapAction` de `media.wsdl`— aunque los 33 XML son sintácticamente bien formados.
50. `navegacion_gps_bt` no tiene tests propios y uno de sus headers depende de includes transitivos para un tipo diagnóstico.
51. El paquete ROS RoboSense declara `1.5.16`, pero el driver compilado y su changelog están en `1.5.18`.
52. El tipo de punto vendor por defecto es XYZI: RS16 calcula ring/timestamp, pero la cloud ROS publicada por el build actual no los conserva.
53. Los CMake config generados dentro del source contienen rutas absolutas de otro entorno de instalación.
54. El launch Humble upstream instala paquetes con `sudo apt` y modifica DDS durante el arranque.
55. `SourceDriver` puede recibir doble `stop()`, unir un thread no joinable y no une el thread IMU; `SourcePacketRos` ROS 2 tampoco cierra explícitamente su thread de spin.
56. Flags ordinarios compartidos entre threads y esperas bloqueantes sin señal de wakeup hacen que varios cierres tengan data races o puedan quedar colgados.
57. La copia de `Packet` pierde metadatos, `Error` deja una categoría sin inicializar y varios callbacks/parámetros inválidos fallan sin una frontera recuperable.
58. Los inputs raw/select no validan integralmente offset, tail y capacidad antes de restar/copiar.
59. Los parsers PCAP usan longitud original en vez de longitud capturada y carecen de validación suficiente de headers/buffers; una captura truncada puede acceder fuera de límites.
60. El backend epoll registra incorrectamente el socket IMU, filtra ese fd y puede cerrar stdin por la inicialización parcial del array de sockets.
61. El backend Windows comprueba mal `WSAStartup`, almacena `SOCKET` en `int`, no hace `WSACleanup` y repite fallas de límites/cierre.
62. Herramientas viewer/PCD y demos pueden terminar o bloquearse por iniciar consumidores antes de `driver.init()`, no unir IMU o esperar indefinidamente una cola.
63. RSM3 nunca confirma temperatura, RSMX pierde el signo de X, AIRY acepta/rechaza inconsistentemente 192 canales y RSP80 puede conservar timings cero para modelo inicial cero.
64. Los proyectos Visual Studio dependen de rutas/versiones locales rígidas y tienen macros/dependencias diferentes entre Win32, Debug y Release.
65. La suite vendor referencia un `angle.csv` ausente, cubre solo tres familias de decoder y contiene una repetición que reduce el test del cuarto bloque dual.

Estos puntos son hallazgos documentales; no se modificó lógica de runtime ni se ejecutaron launchers, hardware, push o instalación systemd durante la lectura.
