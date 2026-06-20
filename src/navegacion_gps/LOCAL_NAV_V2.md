# Navegacion Local V2 Ackermann

Estado: base tecnica vigente de Global V2. La navegacion operativa final del robot es `real_global_v2`; la simulacion operativa final es `sim_global_v2`.

## Resumen
La `v2` es una base de navegacion local para el robot Ackermann que corre completamente en `odom`.
La capa local actual ya no usa `goal_pose_to_follow_path_v2` ni `velocity_smoother`: hoy la navegacion se apoya en un stack Nav2 nativo sobre `odom`, con localizacion Ackermann + EKF local + planner + smoother + BT + costmaps rolling + collision monitor.

Documentos complementarios:

- [SIM_LOCAL_V2_FIDELITY.md](SIM_LOCAL_V2_FIDELITY.md): explica por que `sim_local_v2` ahora sigue la misma cadena de mando que `real_local_v2`, que se cambio respecto del esquema legacy y como validar la simulacion.
  Tambien documenta la convencion de signos de steering y donde se adapta entre ROS, controlador, Gazebo y telemetria.

Objetivos de esta fase:

- levantar una simulacion local reproducible;
- validar localizacion y seguimiento de paths cortos;
- tunear la base local antes de reintroducir una capa global.

Queda explicitamente fuera de esta fase:

- `map -> odom`;
- `navsat_transform`;
- goals geograficos LL;
- fusion GPS en la capa local;
- navegacion global basada en `map`.

## Arquitectura general
### Flujo en simulacion
Launch principal:

```bash
ros2 launch navegacion_gps sim_local_v2.launch.py
```

Composicion efectiva:

1. `sim_v2_base.launch.py`
2. `sim_sensor_normalizer_v2`
3. `vehicle_controller_server` (`transport_backend=sim_gazebo`)
4. `nav_command_server`
5. `localization_v2.launch.py`
6. `keepout_filter_mask_server`
7. `keepout_costmap_filter_info_server`
8. `lifecycle_manager_keepout_filters`
9. `nav_local_v2.launch.py`
10. `rviz_local_v2.rviz`

Notas importantes:

- `sim_local_v2` ya no usa `sim_drive_telemetry` en el launch principal.
- `sim_local_v2` ya no usa `cmd_vel_ackermann_bridge_v2` en el launch principal.
- La fuente de `/controller/drive_telemetry` en simulacion ahora es `vehicle_controller_server` con `transport_backend=sim_gazebo`.

### Flujo en robot real
Launch principal:

```bash
ros2 launch navegacion_gps real_local_v2.launch.py
```

Composicion efectiva:

1. `mavros.launch.py`
2. `rs16.launch.py`
3. `vehicle_controller_server`
4. `nav_command_server`
5. `pointcloud_to_laserscan`
6. `localization_v2.launch.py`
7. `nav_local_v2.launch.py`
8. `rviz_local_v2.rviz`

## Nodos principales de la v2
| Nodo | Rol |
| --- | --- |
| `sim_sensor_normalizer_v2` | Normaliza frames y covarianzas de IMU, GPS, nube y odometria bridged desde Gazebo. |
| `vehicle_controller_server` | En real habla UART; en `sim_local_v2` usa `transport_backend=sim_gazebo`, publica `/cmd_vel_gazebo` y expone `/controller/drive_telemetry`. |
| `nav_command_server` | Arbitra `/cmd_vel_safe` y teleop, y publica `/cmd_vel_final` tanto en real como en `sim_local_v2`. |
| `ackermann_odometry` | Integra odometria planar Ackermann y publica `/wheel/odometry` y `/vehicle/twist`. |
| `ekf_filter_node_local_v2` | Fusiona wheel odom + IMU y publica `/odometry/local` y TF `odom -> base_footprint`. |
| `planner_server` | Calcula el plan global-local en `odom` usando el costmap rolling. |
| `smoother_server` | Suaviza el plan generado antes de entregarlo al controlador. |
| `controller_server` | Sigue el path con `RegulatedPurePursuitController` y publica `/cmd_vel`. |
| `bt_navigator` | Orquesta `ComputePath`, `SmoothPath`, `FollowPath` y recoveries del BT. |
| `behavior_server` | Ejecuta behaviors de recuperacion (`BackUp`, `DriveOnHeading`, `Wait`). |
| `waypoint_follower` | Expone la navegacion por waypoints para el perfil local-v2. |
| `collision_monitor` | Aplica stop preventivo y publica `/cmd_vel_safe`. |
| `keepout_filter_mask_server` | Publica la mascara keepout estatica usada por los costmaps de la `v2`. |
| `keepout_costmap_filter_info_server` | Publica metadata del filtro keepout en `/costmap_filter_info`. |
| `polygon_stamped_republisher` | Republlica `/stop_zone_raw` hacia `/stop_zone` para RViz y consumers legacy. |

## Sensores y señales usadas por la navegacion local
### En robot real
- `/controller/drive_telemetry`
- `/imu/data`
- `/scan_3d` -> `/scan`

### En simulacion
- `/odom_raw`
- `/joint_states`
- `/imu/data_raw` -> `/imu/data`
- `/scan_3d_raw` -> `/scan_3d` -> `/scan`
- `/gps/fix_raw` -> `/gps/fix` solo para observabilidad

### Decisiones explicitas de la v2
- La capa local no usa `/local_position/odom`.
- La capa local no usa `/odom` legacy como entrada principal.
- GPS queda fuera de `odom -> base_footprint`.
- Todo el stack Nav2 local opera en `odom`.

## TF y frames
Arbol esperado:

```text
odom
└── base_footprint
    └── base_link
        ├── imu_link
        ├── gps_link
        ├── lidar_link
        ├── front_left_steer_link
        ├── front_right_steer_link
        ├── front_left_wheel_link
        ├── front_right_wheel_link
        ├── rear_left_wheel_link
        └── rear_right_wheel_link
```

### Que publica cada parte
- `robot_localization` publica `odom -> base_footprint`.
- `robot_state_publisher` publica la cadena fija del modelo a partir del URDF.
- Gazebo publica `/odom_raw`, pero la referencia operativa de navegacion es `/odometry/local`.
- `sim_sensor_normalizer_v2` corrige `frame_id` y covarianzas; no inventa un TF nuevo.

### Por que `odom` queda fijo
En la `v2`, `world_frame = odom` en `robot_localization`. Eso significa:

- `odom` es el frame local fijo donde arranca el robot;
- el que se mueve es `base_footprint`;
- no hay `map -> odom` en esta fase;
- por eso en RViz es normal que `odom` quede quieto y el robot se mueva respecto de el.

### Tabla de frames
| Frame | Productor | Uso |
| --- | --- | --- |
| `odom` | `ekf_filter_node_local_v2` | referencia local fija de navegacion |
| `base_footprint` | `ekf_filter_node_local_v2` + URDF | base plana del robot para costmaps y control |
| `base_link` | `robot_state_publisher` | base mecanica del modelo |
| `imu_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | IMU para EKF |
| `gps_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | GPS solo observabilidad en `v2` |
| `lidar_link` | `robot_state_publisher` / `sim_sensor_normalizer_v2` | percepcion local |

## Localizacion local
### `ackermann_odometry`
Entrada:

- `interfaces/msg/DriveTelemetry`

Salidas:

- `/wheel/odometry` (`nav_msgs/Odometry`)
- `/vehicle/twist` (`geometry_msgs/TwistWithCovarianceStamped`)

Modelo usado:

1. toma `speed_mps_measured`;
2. aplica signo con `reverse_requested`;
3. opcionalmente invierte el signo de `steer_deg_measured` con `invert_measured_steer_sign`;
4. convierte `steer_deg_measured` a radianes;
5. satura el steering al limite configurado;
6. calcula:

```text
yaw_rate = v * tan(delta) / wheelbase
```

7. integra pose planar con un paso tipo midpoint:

```text
x, y, yaw <- integrate_planar(x, y, yaw, v, yaw_rate, dt)
```

La `v2` local usa esta odometria como base del movimiento longitudinal y del yaw estimado por el modelo Ackermann.

### EKF local (`localization_v2.yaml`)
Configuracion base:

- `two_d_mode: true`
- `world_frame: odom`
- `odom_frame: odom`
- `base_link_frame: base_footprint`

Entradas fusionadas:

- `/wheel/odometry`
- `/imu/data`

De `/wheel/odometry` se usan:

- pose planar `x`, `y`, `yaw`
- `linear.x`
- `linear.y`
- `angular.z`

De `/imu/data` se usa:

- `angular.z`

La IMU no se usa para pose absoluta; aporta principalmente yaw rate. La pose local sigue siendo independiente de GPS.

### Covarianzas y tuning expuesto por launch
`localization_v2.launch.py` expone:

- `pose_covariance_xy`
- `pose_covariance_yaw`
- `twist_covariance_vx`
- `twist_covariance_vy`
- `twist_covariance_yaw_rate`
- `wheelbase_m`
- `invert_measured_steer_sign`

Defaults actuales:

- `pose_covariance_xy = 0.05`
- `pose_covariance_yaw = 0.1`
- `twist_covariance_vx = 0.05`
- `twist_covariance_vy = 0.01`
- `twist_covariance_yaw_rate = 0.1`
- `wheelbase_m = 0.94`
- `invert_measured_steer_sign = True` tanto en `sim_local_v2` como en `real_local_v2`

## Navegacion local y pipeline de comandos
La navegacion local actual usa Nav2 nativo sobre `odom`. El goal operativo ya no pasa por `goal_pose_to_follow_path_v2`.

Cadena nominal de control en simulacion:

```text
RViz Nav2 goal / bt_navigator
-> planner_server
-> smoother_server
-> controller_server
-> /cmd_vel
-> collision_monitor
-> /cmd_vel_safe
-> nav_command_server
-> /cmd_vel_final
-> vehicle_controller_server (sim_gazebo)
-> /cmd_vel_gazebo
-> /cmd_vel_steer (Gazebo)
```

Cadena nominal de control en robot real:

```text
RViz Nav2 goal / bt_navigator
-> planner_server
-> smoother_server
-> controller_server
-> /cmd_vel
-> collision_monitor
-> /cmd_vel_safe
-> nav_command_server
-> /cmd_vel_final
-> vehicle_controller_server
-> actuador UART
```

### Topics relevantes
- `/plan`: plan visible de Nav2
- `/cmd_vel`: salida del `controller_server`
- `/cmd_vel_safe`: salida del `collision_monitor`
- `/cmd_vel_final`: comando arbitrado que consumen el controlador real y el controlador simulado
- `/cmd_vel_gazebo`: salida del `vehicle_controller_server` en `sim_local_v2`
- `/odometry/local`: odometria filtrada de referencia para control y costmaps

### `vehicle_controller_server` en `sim_local_v2`
En `sim_local_v2`, `vehicle_controller_server` comparte el mismo `controller_server_node` del robot real, pero cambia el backend interno a `sim_gazebo`.

En esta fase:

- consume `/cmd_vel_final`;
- publica `/cmd_vel_gazebo` hacia Gazebo;
- publica `/controller/status`, `/controller/telemetry` y `/controller/drive_telemetry`;
- sintetiza `DriveTelemetry` desde `/odom_raw` y `/joint_states` para que `ackermann_odometry` consuma el mismo contrato ROS que en real.

`bridge_config_v2.yaml` sigue bridgeando `/cmd_vel_gazebo` hacia Gazebo como `/cmd_vel_steer`.

### `collision_monitor`
Usa:

- entrada `/cmd_vel`
- salida `/cmd_vel_safe`
- frame base `base_footprint`
- frame de odometria `odom`
- fuente de observacion `/scan`

Poligonos actuales:

- `footprint`
- `stop_zone`

`collision_monitor` publica la zona de stop en `/stop_zone_raw`. Luego `polygon_stamped_republisher` la republia en `/stop_zone` para visualizacion y consumers que esperan `PolygonStamped` estable.

### `nav_command_server` en `real_local_v2` y `sim_local_v2`
En `real_local_v2` y `sim_local_v2`, `nav_command_server` forma parte del pipeline de mando.

En esta fase se usa como puente/arbitraje de comandos:

- consume `/cmd_vel_safe`
- publica `/cmd_vel_final`
- mantiene el contrato esperado por `vehicle_controller_server`
- en ambos perfiles se lanza con `forward_cmd_vel_safe_without_goal=true`, para que el comando local de Nav2 llegue al controlador aunque el goal no haya sido iniciado por los servicios legacy del propio `nav_command_server`

La navegacion local `v2` no depende de goals LL para mover el robot, pero el nodo conserva sus servicios legacy para compatibilidad operativa.

En pruebas reales sobre `salus` se observo una transicion corta en telemetria cuando entra un comando automatico:

- `/cmd_vel_final` aparece primero en ROS;
- `vehicle_controller_server` confirma recepcion del comando;
- `DriveTelemetry.drive_enabled` puede pasar a `true` antes de que `control_source` cambie de `NONE` a `PI`;
- la velocidad medida tarda algunos ciclos mas en despegar de `0.0`.

Para validar la cadena real no alcanza con mirar una sola muestra inmediatamente despues del comando; conviene observar una ventana corta de varios mensajes.

Tambien se valido la convencion de giro en el robot real con ruedas levantadas:

- un comando con `linear.x > 0` y `angular.z > 0` hace que el robot intente girar a la izquierda visto desde atras;
- `vehicle_controller_server` debe mantener `invert_steer_from_cmd_vel=True` para respetar el signo fisico correcto del actuador;
- la telemetria `steer_deg_measured` llega con convencion opuesta a la que necesita la odometria Ackermann;
- por eso `real_local_v2` y `sim_local_v2` activan `invert_measured_steer_sign=True` en `ackermann_odometry`;
- con esa combinacion quedan coherentes entre si:
  - giro fisico real del robot,
  - `steer_deg_measured`,
  - `/wheel/odometry`,
  - `/odometry/local`.

## Keepout y stop zone
### Keepout estatico de la `v2`
`nav_local_v2` levanta un keepout estatico en `odom` con estos nodos:

- `keepout_filter_mask_server`
- `keepout_costmap_filter_info_server`
- `lifecycle_manager_keepout_filters`

La mascara proviene por defecto de:

- `config/keepout_mask.yaml`

Y se publica en:

- `/keepout_filter_mask`
- `/costmap_filter_info`

Los costmaps local y global de `nav2_local_v2_params.yaml` usan `keepout_filter`, por eso tanto `sim_local_v2` como `real_local_v2` necesitan los publishers del filtro aunque no exista `map -> odom`.

Para diagnostico existe el flag de launch:

- `use_keepout:=True` por defecto
- `use_keepout:=False` desactiva el bringup del keepout y hace que `nav_local_v2` cargue overrides de params sin `keepout_filter`

### Stop zone
Ademas del keepout de costmap, `collision_monitor` mantiene una `stop_zone` reactiva para frenado inmediato basada en `/scan`.

Topics asociados:

- `/stop_zone_raw`
- `/stop_zone`

## Parametros y configuraciones usadas
Archivos principales:

- `config/localization_v2.yaml`
- `config/nav2_local_v2_params.yaml`
- `config/collision_monitor_v2.yaml`
- `config/rviz_local_v2.rviz`
- `config/keepout_mask.yaml`

### Parametros clave
| Parametro | Default actual | Donde se usa |
| --- | --- | --- |
| `wheelbase_m` | `0.94` | `ackermann_odometry`, modelo Ackermann |
| `operational_steering_limit_rad` | `0.3142` (18°) | `vehicle_controller_server`, límite automático |
| `steering_limit_rad` | `0.5236` (30°) | `vehicle_controller_server`, tope físico |
| `vx_deadband_mps` | `0.01` | `vehicle_controller_server` |
| `vx_min_effective_mps` | `0.5` | `vehicle_controller_server` |
| `xy_goal_tolerance` | `1.2` | `PositionGoalChecker` |
| `desired_linear_vel` | `1.2` | `RegulatedPurePursuitController` |
| `lookahead_dist` | `1.6` | `RegulatedPurePursuitController` |
| `minimum_turning_radius` | `2.2` | `SmacPlannerHybrid` |

En perfiles globales V2 actuales, el radio de planificación es `4.0m`: sale del
limite operativo de patrulla `1.6m/s / 0.4rad/s = 4.0m`. No confundir con el
radio por direccion a 18° (~2.89m) ni con el radio físico a 30° (~1.63m).

Los perfiles globales V2 sin spin evaluan replanning a `0.666 Hz`, pero solo
llaman a Smac si cambia la meta, si `IsPathValid` invalida el path por costmap o
si `IsPathClearanceValid` detecta inflacion alta en los proximos `12m` del path.
`RemovePassedGoals` usa `radius=2.5` para dar margen a la plataforma Ackermann
grande y evitar que queden waypoints ya superados dentro de la lista al replanificar.
En los perfiles `_wifi`, el RPP conserva `desired_linear_vel=1.6`, pero limita
la mirada a `1.4..3.0 m` con `lookahead_time=1.5` para que el robot corrija antes
sin pedir curvas por debajo del radio de planificacion.

### Defaults de sim
- `use_sim_time = True`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `invert_steer_from_cmd_vel = True`
- `invert_measured_steer_sign = True`
- `nav_start_delay_s = 4.0`
- mundo por defecto: `vacio.world`

### Defaults de real
- `use_sim_time = False`
- `wheelbase_m = 0.94`
- `vx_min_effective_mps = 0.5`
- `invert_steer_from_cmd_vel = True` en `vehicle_controller_server`
- `invert_measured_steer_sign = True` en `ackermann_odometry`

### Parametros a recalibrar con el robot
- `wheelbase_m`
- footprint y `stop_zone`
- covarianzas de wheel odom
- `vx_min_effective_mps`
- `lookahead_dist`
- `desired_linear_vel`
- `minimum_turning_radius`
- tolerancia de goal

## Operacion y debugging
### Como lanzar
Dentro del contenedor:

```bash
ros2 launch navegacion_gps sim_local_v2.launch.py
ros2 launch navegacion_gps real_local_v2.launch.py
```

Desde el host, usando scripts del workspace:

```bash
./tools/launch_sim_local_v2.sh
./tools/launch_real_local_v2.sh
```

Checklist de validacion en robot real:

- [REAL_LOCAL_V2_CHECKLIST.md](REAL_LOCAL_V2_CHECKLIST.md)

### Que mirar primero
- `/odometry/local`
- `/wheel/odometry`
- `/plan`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/cmd_vel_final`
- `/cmd_vel_gazebo`
- `/keepout_filter_mask`
- `/costmap_filter_info`
- `/stop_zone_raw`
- `/stop_zone`

### RViz
El plan visible relevante en `rviz_local_v2.rviz` es `/plan`.
El flujo historico basado en `/goal_pose_path` ya no describe la navegacion local actual.
Para la operacion del stack actual, el objetivo debe interpretarse como un goal de Nav2 y no como el path generado por el helper antiguo.
Si se esta validando contra el robot real, reiniciar limpio `real_local_v2` antes de sacar conclusiones: una sesion vieja puede dejar nodos del stack anterior vivos y mezclar la observacion.

### Fallas tipicas
**No se mueve**

- revisar `/cmd_vel`, `/cmd_vel_safe`, `/cmd_vel_final` y `/cmd_vel_gazebo`;
- revisar `collision_monitor`;
- revisar `nav_command_server` y `vehicle_controller_server`;
- revisar `vx_min_effective_mps`.

**No recibe keepout**

- revisar `/keepout_filter_mask` y `/costmap_filter_info`;
- revisar `keepout_filter_mask_server` y `keepout_costmap_filter_info_server`;
- revisar que el `frame_id` del filtro sea `odom`.

**Oscila o abre demasiado la curva**

- revisar `lookahead_dist`;
- revisar `desired_linear_vel`;
- revisar `minimum_turning_radius`;
- revisar el plan en `/plan` y no solo la trayectoria ejecutada.

**No llega al goal como se espera**

- revisar `xy_goal_tolerance`;
- revisar `use_final_approach_orientation` del planner;
- revisar el BT y el `smoother_server`.

### Checklist rapido de validacion
Sim:

1. levantar `sim_local_v2`;
2. confirmar TF `odom -> base_footprint`;
3. confirmar `/wheel/odometry` y `/odometry/local`;
4. confirmar `/keepout_filter_mask` y `/costmap_filter_info`;
5. enviar un goal y mirar `/plan`;
6. verificar `/cmd_vel`, `/cmd_vel_safe` y `/cmd_vel_gazebo`.

Real:

1. levantar `real_local_v2`;
2. verificar `/controller/drive_telemetry`;
3. verificar `/wheel/odometry`, `/odometry/local` y `/cmd_vel_final`;
4. probar con ruedas levantadas;
5. validar avance, giro y freno por `/cmd_vel_final`;
6. recien despues probar goals Nav2;
7. ajustar covarianzas y wheelbase si hace falta.

## Contratos publicos de la v2
Contratos ROS relevantes de esta fase:

- `/controller/drive_telemetry`
- `/wheel/odometry`
- `/vehicle/twist`
- `/odometry/local`
- `/plan`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/cmd_vel_final`
- `/cmd_vel_gazebo`
- `/keepout_filter_mask`
- `/costmap_filter_info`
- `/stop_zone_raw`
- `/stop_zone`

## Limitaciones actuales
- La capa sigue siendo local-only; no hay `map -> odom`.
- La calidad del seguimiento sigue dependiendo del tuning de planner, smoother, controller y planta.
- La planta simulada y el robot real todavia requieren ajuste fino de footprint, covarianzas y limites.
- `sim_local_v2` y `real_local_v2` comparten hoy la misma cadena ROS de mando local, pero no modelan aun ruido de sensores, slip ni latencias fisicas del actuador real.
