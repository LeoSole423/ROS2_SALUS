# AGENTS.md

Role
- You are editing the `navegacion_gps` package in a ROS 2 Humble workspace.
- Prefer small, safe changes and keep launch arguments, YAML params, topics, and frames aligned.

Quick map
- `launch/`: current navigation entry points are `sim_global_v2.launch.py`, `sim_global_v2_wifi.launch.py`, `real_global_v2.launch.py`, `real_global_v2_wifi.launch.py`, and their RViz wrappers.
- `simulacion.launch.py`, `real.launch.py`, `rviz_real.launch.py`, `sim_local_v2.launch.py`, and `real_local_v2.launch.py` are legacy/reference navigation profiles.
- `navegacion_gps/`: active runtime includes `ackermann_odometry`, `zones_manager`, `nav_command_server`, `route_executor`, `nav_snapshot_server`, heading/localization helpers, LiDAR filters and observability nodes.
- `config/`: Nav2, collision monitor, robot_localization, keepout mask assets, RViz.
- `models/`, `worlds/`: simulation assets.

Runtime truth
- `/cmd_vel_safe` is the filtered Nav2 output.
- `/cmd_vel_teleop` uses `interfaces/msg/CmdVelFinal`.
- `nav_command_server` publishes `/cmd_vel_final`.
- `controller_server` consumes `/cmd_vel_final`.
- Local localization uses `/controller/drive_telemetry` -> `ackermann_odometry` plus IMU and publishes `/odometry/local`.
- Global localization uses MAVROS GNSS/RTK, `/gps/odometry_map`, navsat and gated heading and publishes `/odometry/gps`, `/odometry/global`, and `map -> odom`.
- `route_executor` owns chunked missions, waypoint actions, blocked retries, `urban/rural` profiles, structured patrol and HOME return.
- `/battery_mission_guard`, not only the displayed BatteryState percentage, is the return-home decision input.

Launch guidance
- `sim_global_v2.launch.py` is the current simulation navigation entry point.
- `real_global_v2.launch.py` is the current real-robot navigation bringup.
- `rviz_real_global_v2.launch.py` is the current real-robot visualization entry point.
- WiFi wrappers mirror the base global V2 behavior and may only diverge for traffic/visualization and explicitly documented tuning.
- Do not use legacy navigation profiles for operational guidance unless the user explicitly asks for legacy/reference analysis.
- Do not reintroduce references to removed launches such as `navegacion.launch.py`, `dual_ekf_navsat.launch.py`, or `mapviz.launch.py`.

Editing guidance
- When changing a topic or frame, update:
  - launch arguments,
  - YAML config,
  - README if user-facing behavior changes.
- Keep `zones_manager`, `nav_command_server`, and `nav_snapshot_server` service names stable unless explicitly requested.
- Keep `route_executor` route/patrol/profile services and WebSocket consumers aligned when changing mission contracts.
- Avoid broad refactors in `models/` or `worlds/` unless needed for the task.

Validation
- Preferred validation path is inside the Docker workspace:
  - `./tools/compile-ros.sh interfaces navegacion_gps navegacion_gps_bt map_tools controller_server`
  - `ros2 launch navegacion_gps real_global_v2.launch.py --show-args`
  - `ros2 launch navegacion_gps sim_global_v2.launch.py --show-args`
- Run pytest per package directory; collecting several packages together can collide on repeated module names such as `test_copyright.py`.
