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

## Backends
- `transport_backend:=uart`
  - uso real sobre `/dev/serial0`
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
- `vx_deadband_mps`
- `vx_min_effective_mps`
- `reverse_brake_pct`
- `invert_steer_from_cmd_vel`
- `auto_drive_enabled`
- `estop_brake_pct`

## Launch
```bash
ros2 launch controller_server controller_server.launch.py
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
