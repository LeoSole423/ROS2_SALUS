# Politica de Paridad Sim/Real

Estado: actual
Alcance: regla de mantenimiento para cambios en `sim_global_v2`, `real_global_v2` y `real_global_v2_wifi`
Fuente de verdad: launches, YAML Nav2, tests de `navegacion_gps` y validaciones estaticas del checkout actual

La simulacion global V2 y la navegacion real global V2 deben mantenerse sincronizadas en comportamiento de navegacion. La simulacion no es un perfil de tuning separado: es el banco de prueba del comportamiento que se espera en el robot real.

## Regla principal
- `sim_global_v2`, `real_global_v2` y `real_global_v2_wifi` deben compartir la misma logica de Nav2, localizacion global, heading GPS, keepout, route execution y arbitraje de comandos.
- Cualquier cambio en planner, controller, smoother, behavior server, waypoint follower, costmaps, filters, collision monitor, `nav_command_server`, `route_executor`, localizacion global, heading GPS o parametros Ackermann debe revisarse contra sim y real.
- No crear defaults divergentes entre sim y real salvo que la diferencia sea necesaria por hardware, sensores, `use_sim_time`, topicos fisicos, disponibilidad RTK o reduccion de trafico WiFi.
- Las diferencias intencionales deben quedar documentadas cerca del cambio: comentario en launch/YAML, test de contrato o este documento.

## Diferencias permitidas
- `use_sim_time`: sim usa tiempo simulado; real usa tiempo del sistema/ROS real.
- Topicos de sensores:
  - Sim puede usar `/gps/fix`, `/scan`, `/odom_raw` o `joint_states` simulados.
  - Real puede usar topicos fisicos como `/global_position/raw/fix`, `/scan_3d`, `/gps/rtk_status_mavros` y telemetria del controlador.
- Hardware y drivers:
  - Sim puede arrancar Gazebo/normalizadores/backends simulados.
  - Real puede arrancar MAVROS, RS16, Pixhawk y bridges de hardware.
- WiFi:
  - Los perfiles WiFi pueden bajar trafico o visualizacion remota.
  - No deben cambiar la logica principal de navegacion.
  - Diferencias aceptadas: `publish_frequency`, `publish_voxel_map`, `always_send_full_costmap` cuando sea para costmaps remotos.

## Keepout
- `use_keepout=True` debe activar keepout de la misma forma en sim y real.
- `use_keepout=False` debe desactivar keepout de la misma forma en sim y real.
- Los costmaps de sim y real deben declarar `filters: ["keepout_filter"]`; los overlays de keepout/no-keepout son los que habilitan o deshabilitan el filtro.

## Heading GPS
- Los thresholds y gates de `gps_course_heading` deben coincidir entre sim y real.
- La diferencia esperada es la fuente de datos:
  - Sim: GPS y RTK simulados.
  - Real: GPS y RTK del stack fisico/MAVROS.
- Si se cambia un threshold de heading GPS en real, aplicar el mismo cambio a sim salvo que exista una razon tecnica explicita.

## Validacion esperada
Antes de cerrar un cambio de navegacion global V2:
- Comparar sim vs real en YAML/launch y listar las diferencias que quedan.
- Confirmar que las diferencias WiFi son solo de trafico/visualizacion.
- Ejecutar, si el entorno lo permite:
  - `src/navegacion_gps/test/test_launch_contracts.py`
  - `src/navegacion_gps/test/test_real_global_v2_launch.py`
  - `src/navegacion_gps/test/test_sim_global_v2_launch.py`
- Compilar al menos `navegacion_gps` cuando se cambien launches, YAML instalados o codigo del paquete.

Si no se pueden ejecutar tests o build por entorno ROS/Docker, dejar asentado exactamente que fallo y que verificacion estatica se hizo.
