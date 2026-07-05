# map_tools

Estado: actual
Alcance: backend web, edición de zonas y utilidades de waypoints
Fuente de verdad: `launch/no_go_editor.launch.py` y `map_tools/web_zone_server.py`

`map_tools` agrupa el backend web de operación y edición de zonas, más helpers de persistencia de waypoints.

## Ejecutable real
- `web_zone_server`

## Launch principal
```bash
ros2 launch map_tools no_go_editor.launch.py
```

Helper del workspace:
```bash
./tools/launch_no_go_editor.sh
```

## Qué levanta
`no_go_editor.launch.py` puede levantar:
- `navegacion_gps/zones_manager`
- `navegacion_gps/nav_command_server`
- `navegacion_gps/nav_snapshot_server`
- `map_tools/web_zone_server`

Los tres nodos de `navegacion_gps` se pueden desactivar por argumento si ya están corriendo en otro bringup.

## Responsabilidades de `web_zone_server`
- servir la interfaz web en WebSocket
- publicar control manual en `/cmd_vel_teleop`
- hablar con servicios de navegación y zonas
- exponer snapshots de navegación
- exponer eventos recientes y alertas activas
- arrancar/parar rosbag de debug
- persistir waypoints YAML

Los perfiles de rosbag del backend web incluyen la cadena GPS necesaria para replay offline de localización global:
- `/global_position/raw/fix`
- `/gps/fix`
- `/gps/rtk_status_mavros`
- `/gps/odometry_map`
- `/gps/course_heading`
- `/gps/course_heading/debug`

## Parámetros y contratos importantes
- Parámetros:
  - `ws_host`
  - `ws_port`
  - `map_frame`
  - `gps_topic`
  - `odom_topic`
  - `waypoints_file`
- Servicios consumidos:
  - `/zones_manager/set_geojson`
  - `/zones_manager/get_state`
  - `/zones_manager/reload_from_disk`
  - `/nav_command_server/set_goal_ll`
  - `/nav_command_server/cancel_goal`
  - `/nav_command_server/brake`
  - `/nav_command_server/set_manual_mode`
  - `/nav_command_server/get_state`
  - `/nav_snapshot_server/get_nav_snapshot`
- Tópicos consumidos:
  - `/gps/fix`
  - `/odometry/local`
  - `/nav_command_server/telemetry`
  - `/nav_command_server/events`
  - `/battery_state`
  - `/controller/telemetry`
  - `/diagnostics`
- Tópico publicado:
  - `/cmd_vel_teleop`

## Batería en WebSocket
- `battery_pct` sigue siendo el valor principal para UI.
- El backend también propaga desde `/controller/telemetry`:
  - `battery_state`
  - `battery_mission_state`
  - `battery_return_home_recommended`
  - `battery_loaded_voltage_v`
  - `battery_recovered_voltage_v`
  - `battery_voltage_v`

## Tests
```bash
python3 -m pytest -q src/map_tools/test
```
