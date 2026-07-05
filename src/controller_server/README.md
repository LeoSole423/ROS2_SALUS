# controller_server

Estado: actual
Alcance: puente ROS entre navegación y actuador real/simulado
Fuente de verdad: `controller_server_node.py`, `launch/controller_server.launch.py`

Paquete ROS 2 para traducir `/cmd_vel_final` al backend de actuación del vehículo. El mismo nodo soporta UART real y backend `sim_gazebo`.

## Ejecutable real
- `controller_server_node`

## Entrada y salida
### Suscripción
- `/cmd_vel_final` (`interfaces/msg/CmdVelFinal`)

`controller_server` no consume `/cmd_vel_safe` directamente. Ese tópico existe aguas arriba y es arbitrado por `navegacion_gps/nav_command_server`.

### Publicaciones
- `/controller/status` (`std_msgs/msg/String`, payload JSON)
- `/controller/telemetry` (`std_msgs/msg/String`, payload JSON)
- `/controller/drive_telemetry` (`interfaces/msg/DriveTelemetry`)
- `/battery_state` (`sensor_msgs/msg/BatteryState`)
- `/battery_mission_guard` (`interfaces/msg/BatteryMissionGuard`)

## Backends
- `transport_backend:=uart`
  - uso real sobre `serial_port:=auto` por default
  - resuelve `SALUS_CONTROLLER_SERIAL_PORT`, USB-TTL por `by-id`, `ttyUSB*` o `/dev/serial0`
- `transport_backend:=sim_gazebo`
  - usado por `sim_local_v2` y `sim_global_v2`
  - publica `/cmd_vel_gazebo` y sintetiza `DriveTelemetry` desde estado de simulación

## Parámetros principales
- `serial_port`
- `serial_baud`
- `serial_tx_hz`
- `transport_backend`
- `max_speed_mps`
- `max_reverse_mps`
- `control_hz`
- `telemetry_pub_hz`
- `auto_timeout_s`
- `max_abs_angular_z`
- `operational_steering_limit_rad`
- `manual_operational_steering_limit_rad`
- `vx_deadband_mps`
- `vx_min_effective_mps`
- `reverse_brake_pct`
- `invert_steer_from_cmd_vel`
- `auto_drive_enabled`
- `estop_brake_pct`
- `battery_state_topic`
- `battery_guard_topic`
- `battery_full_voltage` (default operativo en reposo: `60.0 V`; el pico de carga alto puede ser mayor)
- `battery_empty_voltage`
- `battery_low_voltage`
- `battery_critical_voltage`
- `battery_telemetry_stale_timeout_s`
- `battery_soc_curve_points`
- `battery_loaded_fast_tau_s`
- `battery_loaded_slow_tau_s`
- `battery_recovered_tau_s`
- `battery_soc_discharge_tau_s`
- `battery_guard_loaded_low_voltage`
- `battery_guard_recovered_low_voltage`
- `battery_guard_loaded_low_persist_s`
- `battery_guard_recovered_low_persist_s`

## Batería real
- La ESP32 publica por UART una medición calibrada de voltaje (`battery_cv`) y la edad de muestra.
- `controller_server` separa batería en dos lógicas:
  - **SOC de operador**: `%` suave publicado en `/battery_state`
  - **guardia de misión**: recomendación de `return_home` publicada en `/battery_mission_guard`
- `/battery_state` publica `voltage=filtered_voltage_v` y `percentage=SOC de operador`.
- `/battery_mission_guard` publica si la misión debería volver a HOME usando persistencia temporal y voltajes de carga/recuperación.
- `/controller/telemetry` conserva el voltaje crudo recibido por UART y agrega:
  - `raw_voltage_v`
  - `filtered_voltage_v`
  - `loaded_voltage_fast_v`
  - `loaded_voltage_slow_v`
  - `recovered_voltage_v`
  - `soc_voltage_v`
  - `operator_soc_pct`
  - `traction_active`
  - `mission_guard_state`
  - `return_home_recommended`
  - `loaded_low_persist_s`
  - `recovered_low_persist_s`
  - `operator_soc_model`
  - `mission_guard_model`
- El protocolo UART de batería no cambia: la mejora de suavizado/SOC ocurre del lado ROS2.

## Launch
```bash
ros2 launch controller_server controller_server.launch.py
```

Override explícito de puerto:
```bash
ros2 launch controller_server controller_server.launch.py \
  controller_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Helper del workspace:
```bash
./tools/launch_controller.sh
```

## Comando manual de prueba
```bash
ros2 topic pub --once /cmd_vel_final interfaces/msg/CmdVelFinal \
"{twist: {linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}, brake_pct: 0}"
```

## Validación
```bash
python3 -m pytest -q src/controller_server/test/test_control_logic.py
./tools/compile-ros.sh controller_server
./tools/exec.sh "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch controller_server controller_server.launch.py --show-args"
```

## Documentación específica UART
- [controller/controller/README.md](/home/leo/codigo/ROS2_SALUS/src/controller_server/controller_server/controller/README.md)
- [controller/controller/COMUNICACIONES_UART_V2.md](/home/leo/codigo/ROS2_SALUS/src/controller_server/controller_server/controller/COMUNICACIONES_UART_V2.md)

Esos documentos describen el cliente/protocolo UART usado por el nodo y su operación en Raspberry. Se mantienen como documentación específica de ese entorno.
