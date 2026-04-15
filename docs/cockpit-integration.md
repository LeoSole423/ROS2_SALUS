# Integración Con Cockpit

Estado: actual
Alcance: contrato operativo entre `cockpit` y `ROS2_SALUS`
Fuente de verdad: `web_zone_server.py`, launches `real_global_v2` / `sim_global_v2` y `sensores_web`

## Resumen
- `cockpit` se conecta al WebSocket de SALUS en `ws://<host>:8766`.
- `web_zone_server` sigue siendo el backend principal de navegación.
- En `real_global_v2`, `web_zone_server` ahora puede puentear datos de `sensores_web` vía `http://127.0.0.1:8000/data`.
- El contrato nuevo cubre `set_control_lock`, `control_heartbeat` y `set_sensor_info_view`, además de los ops que SALUS ya exponía.

## Qué queda cubierto
- Navegación:
  - `set_goal_ll`
  - `cancel_goal`
  - `set_manual_mode`
  - `set_manual_cmd`
  - `get_nav_snapshot`
- Estado:
  - `state`
  - `nav_telemetry`
  - `nav_event`
  - `nav_alerts`
  - `robot_pose`
  - `gps_status`
- Cámara:
  - `camera_pan`
  - `camera_zoom_toggle`
  - `get_camera_status`
- Cockpit bridge:
  - `set_control_lock`
  - `control_heartbeat`
  - `set_sensor_info_view`

## Sensor Info
- `general`:
  - datum fijo del perfil global v2
  - RTK source state si `sensores_web` lo publica
  - `gps_meta` consolidado
- `pixhawk_gps`:
  - snapshot crudo de `sensores_web`
  - yaw delta, RTK status, GPS, IMU, velocity y odom cuando están disponibles
- `topics`, `lidar`, `camera`:
  - hoy responden como no implementados desde SALUS

## Launches
- `real_global_v2.launch.py`
  - arranca `sensores_web` junto al stack MAVROS cuando `launch_web_app=true`
  - pasa a `web_zone_server` el datum fijo del perfil
  - habilita el bridge HTTP hacia `sensores_web`
- `sim_global_v2.launch.py`
  - expone datum fijo
  - no habilita bridge de sensores por default

## Uso recomendado
1. Levantar SALUS con:
```bash
ros2 launch navegacion_gps real_global_v2.launch.py
```
2. En `cockpit`, usar host del robot y puerto `8766`.
3. Si `cockpit` corre fuera del robot, asegurar acceso a `ws://<robot>:8766`.

## Limitaciones actuales
- El tab `topics` de `cockpit` todavía no tiene stream ROS detallado desde SALUS.
- `rtk_sources` y `rtk_source_state` dependen de que la cadena RTK publique esos datos en `sensores_web`.
- El battery percentage no existe hoy como señal canónica en este bridge; `cockpit` lo verá como `0`.
