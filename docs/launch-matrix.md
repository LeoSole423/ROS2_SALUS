# Matriz de Launches

Estado: actual
Alcance: clasificación operativa de launches y scripts helper
Fuente de verdad: `src/**/launch/*.launch.py` y `tools/*.sh`

## Navegacion vigente
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| Sim global Ackermann | `ros2 launch navegacion_gps sim_global_v2.launch.py` | `./tools/launch_sim_global_v2.sh` | contenedor | vigente |
| Real global Ackermann | `ros2 launch navegacion_gps real_global_v2.launch.py` | `./tools/launch_real_global_v2.sh` | robot/contenedor | vigente, perfil base no-WiFi |
| Real global Ackermann WiFi | `ros2 launch navegacion_gps real_global_v2_wifi.launch.py` | `./tools/launch_real_global_v2_wifi.sh` | robot/contenedor | recomendado para operacion remota WiFi |
| RViz real global V2 | `ros2 launch navegacion_gps rviz_real_global_v2.launch.py` | `./tools/launch_real_global_v2_rviz.sh` | PC local | vigente, con perfil CycloneDDS Wi-Fi seguro |
| Replay offline localizacion global | `ros2 launch navegacion_gps replay_localization_global_v2.launch.py` | `./tools/run_localization_replay_compare.sh <bag_dir>` | contenedor | soporte |

`real_global_v2` y `sim_global_v2` son las unicas navegaciones operativas vigentes. La arquitectura local V2 sigue siendo base tecnica vigente dentro de esos perfiles globales, pero sus launches standalone no se usan para operacion normal.

## Infraestructura vigente
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| MAVROS Pixhawk/GNSS | `ros2 launch sensores mavros.launch.py` | n/a | robot/contenedor | vigente, usado por `real_global_v2` |
| RS16 | `ros2 launch sensores rs16.launch.py` | n/a | robot/contenedor | actual |
| Web editor / backend | `ros2 launch map_tools no_go_editor.launch.py` | `./tools/launch_no_go_editor.sh` | contenedor | actual |
| Controlador | `ros2 launch controller_server controller_server.launch.py` | `./tools/launch_controller.sh` | robot/contenedor | actual |

## Navegacion LEGACY / referencia
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| Simulacion mainline vieja | `ros2 launch navegacion_gps simulacion.launch.py` | n/a | contenedor | LEGACY / no usar como navegacion vigente |
| Navegacion real mainline vieja | `ros2 launch navegacion_gps real.launch.py` | `./tools/launch_real_nav.sh` | robot/contenedor | LEGACY / no usar como navegacion vigente |
| RViz real mainline viejo | `ros2 launch navegacion_gps rviz_real.launch.py` | `./tools/launch_real_rviz.sh` | PC local | LEGACY |
| Pixhawk propio | `ros2 launch sensores pixhawk.launch.py` | n/a | robot/contenedor | LEGACY / referencia, reemplazado por MAVROS |
| Sim local Ackermann | `ros2 launch navegacion_gps sim_local_v2.launch.py` | `./tools/launch_sim_local_v2.sh` | contenedor | referencia / no operativo |
| Real local Ackermann | `ros2 launch navegacion_gps real_local_v2.launch.py` | `./tools/launch_real_local_v2.sh` | robot/contenedor | referencia / no operativo |
| RViz real local V2 | `ros2 launch navegacion_gps rviz_real_local_v2.launch.py` | `./tools/launch_real_local_v2_rviz.sh` | PC local | referencia |

Nota operativa:
el perfil CycloneDDS Wi‑Fi busca mejorar la unión RViz<->robot en redes débiles, pero no garantiza visualización de LiDAR remoto por Wi‑Fi. Para `/scan` y `/scan_3d`, Ethernet sigue siendo la referencia operativa.
En `real_global_v2` queda disponible `/scan_wifi_debug` como `LaserScan` reducido para observación remota liviana por Wi‑Fi, manteniendo `/scan` local para navegación.
En Global V2 la ruta LiDAR conservadora por default es `/scan -> /scan_clean`;
`enable_scan_noise_filter:=False` vuelve al `/scan` legacy puro y
`enable_lidar_obstacle_filter:=True` habilita la ruta RANSAC experimental.

## Build y regeneracion
| Tarea | Comando | Nota |
| --- | --- | --- |
| Recompilar cambios de navegación/control | `./tools/compile-ros.sh controller_server navegacion_gps` | recompila dentro del contenedor |
| Abrir shell del contenedor | `./tools/exec.sh` | usar si hace falta correr `colcon` o `ros2` a mano |
| Lanzar `real_global_v2` base | `./tools/launch_real_global_v2.sh` | wrapper corto no-WiFi sobre `ros2 launch navegacion_gps real_global_v2.launch.py`; mantiene compatibilidad con pruebas locales |
| Lanzar `real_global_v2` WiFi | `./tools/launch_real_global_v2_wifi.sh` | perfil operativo recomendado para robot remoto por WiFi; usa `real_global_v2_wifi.launch.py` y params Nav2 con menor trafico |
| Lanzar `real_global_v2` WiFi | `./tools/launch_real_global_v2_wifi.sh` | perfil operativo recomendado; usa `cuatri_real_v2.urdf` por default |
| Lanzar sim WiFi global V2 | `./tools/launch_sim_global_v2_wifi.sh` | espejo de real global V2; usa `cuatri_real_v2.urdf` por default |
| Probar brujula gateada en sim V2 | `./tools/launch_sim_global_v2_wifi.sh enable_sim_compass:=true enable_compass_heading:=true enable_compass_initial_guess:=true` | publica `/sim/compass_hdg`, `/imu/compass_heading` y `/imu/compass_heading/debug`; no fusiona al EKF salvo `enable_compass_heading_fusion:=true` |
| Probar brujula gateada en robot real | `./tools/launch_real_global_v2_wifi.sh enable_compass_initial_guess:=true` | usa `/mavros_node/compass_hdg` solo como guess inicial; no vuelve a publicar tras `startup_window_s` |
| Medir bias brujula vs GPS RTK | `./tools/record_compass_calibration.sh east_run_01 60` | herramienta pasiva; no mueve el robot, guarda JSON en `/ros2_ws/artifacts/compass_calibration/` |
| Lanzar `real_global_v2` con datum explícito | `./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch navegacion_gps real_global_v2.launch.py datum_lat:=<lat> datum_lon:=<lon> datum_yaw_deg:=<yaw_deg>"` | usar cuando el sitio operativo no coincide con el default |

Nota GUI Docker:
los helpers que abren RViz/Gazebo deben pasar por `tools/exec.sh` o `tools/docker_ros_env.sh`. Esa capa refresca `DISPLAY`, valida el socket X11 actual y autoriza el usuario local con `xhost`, evitando que RViz quede atado al `DISPLAY` con el que se creo el contenedor.

## Politica de datum
- La navegacion global vigente usa datum fijo por sitio operativo. `datum_lat`, `datum_lon` y `datum_yaw_deg` deben venir del launch/configuracion del sitio y no de la posicion instantanea del robot.
- `datum_setter` y los servicios `SetDatum` / `GetDatum` quedan clasificados como LEGACY. Eran soporte para setear datum automaticamente o por servicio; no se usan en `real_global_v2`, `sim_global_v2` ni replay global vigentes.
- No reactivar auto-set de datum en operacion normal: moveria el origen de `map` y puede desalinear goals LL, zonas no-go y keepout persistentes.

## Nota sim vs real
- La regla de mantenimiento completa esta en [docs/sim-real-parity.md](/home/leo/codigo/ROS2_SALUS/docs/sim-real-parity.md).
- Los cambios en `sim_global_v2` que dependan de `joint_states` o `odom_raw` pertenecen a simulación y no pasan solos a `real_global_v2`.
- `real_global_v2` ahora comparte con `sim_global_v2` el anclaje global en `map`: GPS geográfico -> `fromLL` -> `/gps/odometry_map`, además de soporte para `/gps/course_heading`.
- `real_global_v2` usa un params file propio de Nav2 para dejar el `global_costmap` en rolling window sin tocar la localización global ni arrastrar overrides específicos de simulación.
- La base local V2 no es legacy: `ackermann_odometry`, `localization_v2` y `/odometry/local` son parte activa de la navegacion global vigente. Lo no operativo son los launches locales standalone.
- En `real_global_v2`, la fuente de steering que debe mantenerse estable es `/controller/drive_telemetry.steer_deg_measured`.
- Si el robot mide el ángulo en la barra central de dirección, ese dato es el que debe alimentar la odometría Ackermann real.
- El gating de `/gps/course_heading` debe mantenerse alineado entre sim y real; solo cambian las fuentes de GPS/RTK.
- La brujula gateada esta documentada en [docs/compass-heading-gate.md](/home/leo/codigo/ROS2_SALUS/docs/compass-heading-gate.md). Queda apagada por defecto y no debe competir con `/gps/course_heading` en movimiento.

## Operación y diagnóstico
| Herramienta | Comando | Estado |
| --- | --- | --- |
| Rosbag debug navegación | `./tools/record_nav_debug_bag.sh` | vigente, ahora graba GPS crudo/procesado + RTK para replay offline |
| Calibración brújula vs GPS RTK | `./tools/record_compass_calibration.sh <label> [duration_s]` | vigente, pasivo, emite JSON para agentes y humanos |
| Replay + compare de bag localización | `./tools/run_localization_replay_compare.sh <bag_dir>` | soporte |
| Generador loop tipo cuadra | `./tools/generate_block_loop_benchmark.sh` | vigente |
| Healthcheck LiDAR | `./tools/healthcheck-lidar.sh` | vigente |
| Envío de path V2 | `./tools/send_follow_path_v2.sh` | soporte |
| Stop sim local V2 | `./tools/stop_sim_local_v2.sh` | soporte |
| Stop sim global V2 | `./tools/stop_sim_global_v2.sh` | soporte |
| Heading startup | `./tools/check_startup_heading.sh` | soporte |

## Criterio de uso
- Para navegacion real remota por WiFi, usar `real_global_v2_wifi`.
- Para navegacion real base/local, usar `real_global_v2`.
- Para simulacion de la navegacion vigente, usar `sim_global_v2`.
- Usar `real.launch.py` o `simulacion.launch.py` solo como material legacy.
- Usar `real_local_v2` o `sim_local_v2` solo para validar/consultar la base local V2, no como perfil operativo final.
- Si un script helper y un launch discrepan, el launch es la fuente de verdad y el script debe considerarse conveniencia operativa.
