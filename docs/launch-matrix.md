# Matriz de Launches

Estado: actual
Alcance: clasificación operativa de launches y scripts helper
Fuente de verdad: `src/**/launch/*.launch.py` y `tools/*.sh`

## Navegacion vigente
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| Sim global Ackermann | `ros2 launch navegacion_gps sim_global_v2.launch.py` | `./tools/launch_sim_global_v2.sh` | contenedor | vigente |
| Real global Ackermann | `ros2 launch navegacion_gps real_global_v2.launch.py` | `./tools/launch_real_global_v2.sh` | robot/contenedor | vigente, con perfil CycloneDDS Wi-Fi seguro y `global_costmap` rolling |
| RViz real global V2 | `ros2 launch navegacion_gps rviz_real_global_v2.launch.py` | `./tools/launch_real_global_v2_rviz.sh` | PC local | vigente, con perfil CycloneDDS Wi-Fi seguro |
| Replay offline localizacion global | `ros2 launch navegacion_gps replay_localization_global_v2.launch.py` | `./tools/run_localization_replay_compare.sh <bag_dir>` | contenedor | soporte |

`real_global_v2` y `sim_global_v2` son las unicas navegaciones operativas vigentes. La arquitectura local V2 sigue siendo base tecnica vigente dentro de esos perfiles globales, pero sus launches standalone no se usan para operacion normal.

## Infraestructura vigente
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| Pixhawk propio | `ros2 launch sensores pixhawk.launch.py` | n/a | robot/contenedor | actual |
| MAVROS | `ros2 launch sensores mavros.launch.py` | n/a | robot/contenedor | actual |
| RS16 | `ros2 launch sensores rs16.launch.py` | n/a | robot/contenedor | actual |
| Web editor / backend | `ros2 launch map_tools no_go_editor.launch.py` | `./tools/launch_no_go_editor.sh` | contenedor | actual |
| Controlador | `ros2 launch controller_server controller_server.launch.py` | `./tools/launch_controller.sh` | robot/contenedor | actual |

## Navegacion LEGACY / referencia
| Perfil | Launch | Helper | Destino | Estado |
| --- | --- | --- | --- | --- |
| Simulacion mainline vieja | `ros2 launch navegacion_gps simulacion.launch.py` | n/a | contenedor | LEGACY / no usar como navegacion vigente |
| Navegacion real mainline vieja | `ros2 launch navegacion_gps real.launch.py` | `./tools/launch_real_nav.sh` | robot/contenedor | LEGACY / no usar como navegacion vigente |
| RViz real mainline viejo | `ros2 launch navegacion_gps rviz_real.launch.py` | `./tools/launch_real_rviz.sh` | PC local | LEGACY |
| Sim local Ackermann | `ros2 launch navegacion_gps sim_local_v2.launch.py` | `./tools/launch_sim_local_v2.sh` | contenedor | referencia / no operativo |
| Real local Ackermann | `ros2 launch navegacion_gps real_local_v2.launch.py` | `./tools/launch_real_local_v2.sh` | robot/contenedor | referencia / no operativo |
| RViz real local V2 | `ros2 launch navegacion_gps rviz_real_local_v2.launch.py` | `./tools/launch_real_local_v2_rviz.sh` | PC local | referencia |

Nota operativa:
el perfil CycloneDDS Wi‑Fi busca mejorar la unión RViz<->robot en redes débiles, pero no garantiza visualización de LiDAR remoto por Wi‑Fi. Para `/scan` y `/scan_3d`, Ethernet sigue siendo la referencia operativa.
En `real_global_v2` queda disponible `/scan_wifi_debug` como `LaserScan` reducido para observación remota liviana por Wi‑Fi, manteniendo `/scan` local para navegación.

## Build y regeneracion
| Tarea | Comando | Nota |
| --- | --- | --- |
| Recompilar cambios de navegación/control | `./tools/compile-ros.sh controller_server navegacion_gps` | recompila dentro del contenedor |
| Abrir shell del contenedor | `./tools/exec.sh` | usar si hace falta correr `colcon` o `ros2` a mano |
| Lanzar `real_global_v2` | `./tools/launch_real_global_v2.sh` | wrapper corto sobre `ros2 launch navegacion_gps real_global_v2.launch.py`; hoy arranca con `use_keepout:=False` y `global_costmap` rolling por default |
| Lanzar `real_global_v2` con datum explícito | `./tools/exec.sh "source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; ros2 launch navegacion_gps real_global_v2.launch.py datum_lat:=<lat> datum_lon:=<lon> datum_yaw_deg:=<yaw_deg>"` | usar cuando el sitio operativo no coincide con el default |

## Politica de datum
- La navegacion global vigente usa datum fijo por sitio operativo. `datum_lat`, `datum_lon` y `datum_yaw_deg` deben venir del launch/configuracion del sitio y no de la posicion instantanea del robot.
- `datum_setter` y los servicios `SetDatum` / `GetDatum` quedan clasificados como LEGACY. Eran soporte para setear datum automaticamente o por servicio; no se usan en `real_global_v2`, `sim_global_v2` ni replay global vigentes.
- No reactivar auto-set de datum en operacion normal: moveria el origen de `map` y puede desalinear goals LL, zonas no-go y keepout persistentes.

## Nota sim vs real
- Los cambios en `sim_global_v2` que dependan de `joint_states` o `odom_raw` pertenecen a simulación y no pasan solos a `real_global_v2`.
- `real_global_v2` ahora comparte con `sim_global_v2` el anclaje global en `map`: GPS geográfico -> `fromLL` -> `/gps/odometry_map`, además de soporte para `/gps/course_heading`.
- `real_global_v2` usa un params file propio de Nav2 para dejar el `global_costmap` en rolling window sin tocar la localización global ni arrastrar overrides específicos de simulación.
- La base local V2 no es legacy: `ackermann_odometry`, `localization_v2` y `/odometry/local` son parte activa de la navegacion global vigente. Lo no operativo son los launches locales standalone.
- En `real_global_v2`, la fuente de steering que debe mantenerse estable es `/controller/drive_telemetry.steer_deg_measured`.
- Si el robot mide el ángulo en la barra central de dirección, ese dato es el que debe alimentar la odometría Ackermann real.
- En `sim_global_v2`, el gating del heading GPS ahora tiene knobs propios:
- `gps_course_heading_invalid_hold_s`
- `gps_course_heading_hold_yaw_variance_multiplier`

## Operación y diagnóstico
| Herramienta | Comando | Estado |
| --- | --- | --- |
| Rosbag debug navegación | `./tools/record_nav_debug_bag.sh` | vigente, ahora graba GPS crudo/procesado + RTK para replay offline |
| Replay + compare de bag localización | `./tools/run_localization_replay_compare.sh <bag_dir>` | soporte |
| Generador loop tipo cuadra | `./tools/generate_block_loop_benchmark.sh` | vigente |
| Healthcheck LiDAR | `./tools/healthcheck-lidar.sh` | vigente |
| Envío de path V2 | `./tools/send_follow_path_v2.sh` | soporte |
| Stop sim local V2 | `./tools/stop_sim_local_v2.sh` | soporte |
| Stop sim global V2 | `./tools/stop_sim_global_v2.sh` | soporte |
| Heading startup | `./tools/check_startup_heading.sh` | soporte |

## Criterio de uso
- Para navegacion real del robot, usar `real_global_v2`.
- Para simulacion de la navegacion vigente, usar `sim_global_v2`.
- Usar `real.launch.py` o `simulacion.launch.py` solo como material legacy.
- Usar `real_local_v2` o `sim_local_v2` solo para validar/consultar la base local V2, no como perfil operativo final.
- Si un script helper y un launch discrepan, el launch es la fuente de verdad y el script debe considerarse conveniencia operativa.
