# navegacion_gps

Estado: actual
Alcance: resumen operativo del paquete y clasificación de perfiles
Fuente de verdad: `launch/`, `config/`, `setup.py` y tests del paquete

`navegacion_gps` integra navegación, localización, zonas de exclusión, observabilidad y el backend ROS que arbitra control automático y manual.

## Documentación del paquete
- Resumen de perfiles y launches: [docs/launch-matrix.md](/home/leo/codigo/ROS2_SALUS/docs/launch-matrix.md)
- Politica de paridad sim/real: [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md)
- Arquitectura runtime: [docs/runtime-architecture.md](/home/leo/codigo/ROS2_SALUS/docs/runtime-architecture.md)
- Base local V2 usada por Global V2:
  - [LOCAL_NAV_V2.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/LOCAL_NAV_V2.md)
- Launches locales standalone de referencia:
  - [SIM_LOCAL_V2_FIDELITY.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/SIM_LOCAL_V2_FIDELITY.md)
  - [REAL_LOCAL_V2_CHECKLIST.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/REAL_LOCAL_V2_CHECKLIST.md)
- Navegación global V2:
  - [REAL_GLOBAL_V2_CHECKLIST.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/REAL_GLOBAL_V2_CHECKLIST.md)
- Benchmarks y observabilidad:
  - [docs/nav-benchmarks.md](/home/leo/codigo/ROS2_SALUS/docs/nav-benchmarks.md)
- Diseño y transición:
  - [IMPLEMENTATION_PLAN_GLOBAL_NAV_V2.md](/home/leo/codigo/ROS2_SALUS/src/navegacion_gps/IMPLEMENTATION_PLAN_GLOBAL_NAV_V2.md)
  - clasificación recomendada: histórico / transición

## Perfiles operativos
### Navegacion vigente
- `ros2 launch navegacion_gps sim_global_v2.launch.py`
- `ros2 launch navegacion_gps real_global_v2.launch.py`
- `ros2 launch navegacion_gps real_global_v2_wifi.launch.py`
- `ros2 launch navegacion_gps rviz_real_global_v2.launch.py`

`real_global_v2_wifi` es el perfil recomendado para operacion remota del robot por WiFi; `real_global_v2` queda como perfil base no-WiFi compatible.
`real_global_v2` y `sim_global_v2` son las navegaciones base vigentes del paquete.

La V2 global agrega la capa `map -> odom` sobre la base local Ackermann, con `navsat_transform`, EKF global, datum configurable y goals LL en `map`.
En los perfiles globales actuales, la corrección absoluta del EKF de `map` entra como `/gps/odometry_map` en frame `map` obtenido vía `fromLL`, y el heading global puede asistir al filtro mediante `/gps/course_heading`.
La base local V2 sigue vigente como bloque interno: `ackermann_odometry`, `localization_v2` y `/odometry/local` sostienen la capa global.
El datum de V2 global es fijo por sitio operativo. Los mecanismos para setear datum automaticamente quedan como LEGACY y no forman parte de los bringups vigentes.

### Navegacion LEGACY / referencia
- `ros2 launch navegacion_gps simulacion.launch.py`
- `ros2 launch navegacion_gps real.launch.py`
- `ros2 launch navegacion_gps rviz_real.launch.py`
- `ros2 launch navegacion_gps sim_local_v2.launch.py`
- `ros2 launch navegacion_gps real_local_v2.launch.py`
- `ros2 launch navegacion_gps rviz_real_local_v2.launch.py`

Estos perfiles viejos pueden servir para consultar implementaciones puntuales, pero no deben usarse como navegacion vigente.

## Nodos propios más relevantes
- `zones_manager`
- `nav_command_server`
- `nav_snapshot_server`
- `nav_observability`
- `ackermann_odometry`
- `gazebo_utils`
- `sim_sensor_normalizer_v2`
- `gps_course_heading`

## Componentes LEGACY
- Perfiles mainline viejos:
  - `simulacion.launch.py`, `real.launch.py`, `rviz_real.launch.py`
- Launches locales standalone:
  - `sim_local_v2.launch.py`, `real_local_v2.launch.py`, `rviz_real_local_v2.launch.py`
  - Son referencia para validar la base local V2, pero no perfiles operativos finales.
- `datum_setter`
  - Nodo historico para setear datum automaticamente o por servicio.
  - No usar en operacion normal de `real_global_v2` / `sim_global_v2`; el datum vigente debe salir de la configuracion fija del sitio.
- `interfaces/srv/SetDatum.srv` y `interfaces/srv/GetDatum.srv`
  - Contratos conservados por compatibilidad con tooling viejo.

## Contratos públicos más usados
- Control:
  - `/cmd_vel_safe`
  - `/cmd_vel_teleop`
  - `/cmd_vel_final`
- Localización:
  - `/odometry/local`
  - `/odometry/gps`
  - `/gps/odometry_map`
  - `/odometry/global`
- Observabilidad:
  - `/nav_command_server/telemetry`
  - `/nav_command_server/events`
  - `/diagnostics`
  - `/scan_wifi_debug`

## Helpers del workspace
- `./tools/launch_sim_global_v2.sh`
- `./tools/launch_real_global_v2.sh`
- `./tools/launch_real_global_v2_wifi.sh`
- `./tools/record_nav_debug_bag.sh`
- `./tools/run_nav_benchmark.sh`
- `./tools/compare_nav_benchmarks.sh`

## Nota de uso
- Si buscás una guía operativa corta, usá este README y la matriz de launches.
- Si necesitás wiring fino o tuning de V2, usá los documentos específicos de V2.
- Si encontrás decisiones escritas en futuro o en tono de propuesta, tratarlas como documentación histórica, no como fuente de verdad del checkout actual.
- En `real_global_v2`, el nodo `scan_wifi_debug` publica un `LaserScan` de debug para Wi‑Fi en `/scan_wifi_debug` sin reemplazar `/scan`; el objetivo es visualización remota liviana, no alimentar Nav2.
