# Navegación y control — ROS2_SALUS

Estado: documentación read-only del código actual.
Fuente de verdad: `src/navegacion_gps/**` y `src/controller_server/**`.
**(por confirmar)** marca lo no verificado leyendo el código línea por línea.

Hermanos: [ARQUITECTURA](ARQUITECTURA_ROS2_SALUS.md) ·
[TOPICOS_Y_TF](TOPICOS_Y_TF.md) · [LAUNCHES_Y_OPERACION](LAUNCHES_Y_OPERACION.md)

---

## 1. Navegación (`src/navegacion_gps`)

### 1.1 Cadena de arbitraje de comandos

```text
Nav2 -> /cmd_vel -> collision_monitor -> /cmd_vel_safe
                                            \
web (/cmd_vel_teleop) ----------------------> nav_command_server -> /cmd_vel_final -> controller
```

`nav_command_server` (`navegacion_gps/nav_command_server.py`) arbitra entre el
comando autónomo (`/cmd_vel_safe`) y el manual web (`/cmd_vel_teleop`), aplica
frenos y publica `/cmd_vel_final` (`interfaces/CmdVelFinal`). Parámetros clave en
`real_global_v2.launch.py:618-658`:

| Parámetro | Valor | Rol |
|---|---|---|
| `cmd_vel_safe_topic` | `/cmd_vel_safe` | entrada auto |
| `teleop_cmd_topic` | `/cmd_vel_teleop` | **entrada manual real** suscrita (`nav_command_server.py:314`) |
| `manual_cmd_topic` | `/cmd_vel_safe` | parámetro presente; el manual efectivo entra por `teleop_cmd_topic` |
| `cmd_vel_final_topic` | `/cmd_vel_final` | salida |
| `forward_cmd_vel_safe_without_goal` | `True` | reenvía aun sin meta activa |
| `brake_*` | count 5, interval 0.1 s | freno repetido para robustez |
| `manual_cmd_timeout_s` | 0.4 | watchdog del manual |
| `fromll_service` (+fallback + approx) | `/fromLL` … | conversión LL→map para metas |

Servicios expuestos: `set_goal_ll`, `cancel_goal`, `brake`, `set_manual_mode`,
`get_state`. Telemetría: `/nav_command_server/telemetry`, `/events`.

`route_executor` (`route_executor.py`) ejecuta misiones multi-waypoint sobre
`nav_command_server` (acciones programables por waypoint, reintentos con
re-anclaje al bloquearse, `blocked_retry_*`).

### 1.2 Localización

Dos capas EKF (`robot_localization`) + `navsat_transform`
(`localization_global_v2.launch.py`):

- **Local** (`odom`): `ackermann_odometry` desde `/controller/drive_telemetry` +
  `/imu/data` → `/odometry/local`, TF `odom -> base_footprint`.
- **Global** (`map`): EKF global + `navsat_transform` (`/global_position/raw/fix`)
  + medida absoluta `map_gps_absolute_measurement` (`/gps/odometry_map`) +
  heading por rumbo `gps_course_heading` → `/odometry/global`, TF `map -> odom`.

Heading GPS (`gps_course_heading`, `real_global_v2.launch.py:564-616`):
RTK-gated (`require_rtk=True`, estados `RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK`),
mínimos de distancia/velocidad para muestrear rumbo, hold breve al perder validez.

Configs EKF: `config/dual_ekf_navsat_params*.yaml`,
`localization_global_v2.yaml`, `localization_v2.yaml`.

### 1.3 Nav2

`nav_global_v2.launch.py` levanta planner, controller, smoother, bt_navigator,
behavior, waypoint_follower, `collision_monitor` y dos `lifecycle_manager`
(autostart). Detalles:

- BTs sin spin: `navigate_to_pose_w_replanning_and_recovery_no_spin.xml` y
  `navigate_through_poses_..._no_spin.xml`.
- Los BTs globales evaluan replanning a `0.333 Hz` (~3.0 s), pero solo llaman a
  Smac cuando cambia la meta (`GlobalUpdatedGoal`), el path actual queda invalido
  por colision (`IsPathValid` falla) o pierde margen lateral en costo inflado
  (`IsPathClearanceValid` falla). Si el robot viene siguiendo bien la curva y el
  path mantiene margen, se conserva el path vigente para evitar que un replan
  desde una pose lateralmente corrida genere un giro Dubins en O.
- En `NavigateThroughPoses`, las limpiezas de costmap del BT usan
  `server_timeout=5000` para evitar aborts prematuros en Raspberry Pi 5 cuando el
  clear de costmaps se demora bajo carga.
- `path_clearance_validator` revisa `/global_costmap/costmap_raw` sobre los
  proximos `12m` del path con umbral de costo `100`, offsets laterales
  `0.0, +/-0.45m` y falla abierto si no tiene costmap fresco. El timeout de
  frescura es `4.0s` para ser compatible con costmaps WiFi a `0.5 Hz`, y el nodo
  cachea validaciones repetidas durante `0.75s`; el BT le da `server_timeout=1000`
  para evitar falsos timeouts de clearance bajo carga.
  Esto fuerza replan cuando el path pasa por la zona gris/inflada alrededor de
  autos u obstaculos.
- En `NavigateThroughPoses`, primero se conserva el path vigente si sigue valido
  y con clearance. `RemovePassedGoals` corre recien dentro de la rama que necesita
  recalcular y usa `radius=2.5`, para dar margen a waypoints ya superados por el
  robot. Pasar un punto intermedio no modifica por si solo la lista de metas ni
  dispara Smac; si un obstaculo invalida el path, los puntos ya superados se
  podan antes del replan.
- En reintentos por bloqueo (`blocked_retry_*`), el re-anclaje usa el tramo activo
  de la ruta como barrera de progreso. En rutas `loop=True`, no puede volver a un
  indice anterior ya consumido antes de cruzar el cierre del loop; solo puede
  envolver a 0 cuando el chunk activo ya atraviesa ese cierre.
- Los aborts clasificados como `COSTMAP_CLEAR_TIMEOUT` se tratan como bloqueo
  retryable: la misión espera `blocked_retry_wait_s` antes de reintentar y solo
  queda para operador si agota `blocked_retry_max_attempts`.
- Waypoints con acción (`brake_hold`) cortan el chunk activo, ejecutan freno por
  `duration_s` y luego continúan. En loop, esos índices no se auto-saltean aunque
  el robot ya esté dentro de tolerancia, para repetir la acción en cada pasada.
- Para tramos largos, `route_executor` sigue generando puntos intermedios por
  `leg_spacing_m`, pero los marca internamente como sintéticos. Esos puntos ayudan
  a formar la geometría del path, pero no cierran chunks: cada chunk se extiende
  hasta el siguiente waypoint principal aunque supere `chunk_span_m` o
  `chunk_max_waypoints`. Por eso el éxito y despacho de un nuevo goal ocurre en
  los puntos manuales, no al pasar por un punto sintético. Si hay un reintento y
  el robot ya progresó sobre el segmento siguiente, el próximo chunk también
  puede saltar un punto sintético atrasado. Los puntos manuales y los que tienen
  acciones siguen siendo puntos clave y no se saltean por esta regla.
- En `sim_global_v2`, `nav_trace_recorder` abre una sola sesión por misión de
  `route_executor`. El BT instrumentado emite la causa exacta de cada ejecución
  de Smac (`goal_updated`, `path_invalid` o `clearance_invalid`) y guarda paths,
  progreso y anomalías en `artifacts/nav_traces/`. Los perfiles reales conservan
  el BT productivo y no levantan este recorder.
- El **scan efectivo** se inyecta vía `RewrittenYaml` en
  `local_costmap.voxel_layer.scan_*.topic`, `global_costmap.obstacle_layer.scan.topic`
  y `collision_monitor.scan.topic` (`nav_global_v2.launch.py:65-96`).
- Params: `nav2_global_v2_real_rolling_params.yaml` (real),
  `nav2_global_v2_sim_rolling_params.yaml` (sim), variantes `_wifi`.
- En perfiles `_wifi` se mantiene `desired_linear_vel=1.6`, pero el lookahead
  escalado queda acotado a `1.8..3.6 m` con `lookahead_time=1.8`. La intención es
  reducir la corrección tardía sin cambiar la velocidad nominal de patrulla.
- Overrides keepout: `nav2_local_v2_keepout_overrides.yaml` vs
  `nav2_local_v2_no_keepout_overrides.yaml` según `use_keepout`.

### 1.4 Keepout / zonas no-go

- **Keepout costmap filter**: `keepout_filters_v2.launch.py` + máscara
  `config/keepout_mask.yaml`/`.pgm`. Solo con `use_keepout:=True` (default
  **False** en real global, **True** en sim).
- **Zonas no-go dinámicas** (web): `zones_manager` (servicios `set_geojson`,
  `get_state`, `reload_from_disk`), persistidas en `config/no_go_zones.geojson`.
- **Polígonos del collision_monitor**: `stop_zone`, `critical_slow_zone`,
  `slow_zone` se publican `*_raw` y `polygon_stamped_republisher` los re-emite
  estampados (`nav_global_v2.launch.py:217-252`).

### 1.5 Integración web / map_tools

`map_tools/web_zone_server` (WebSocket :8766) es el puente entre la UI y los
servicios ROS (zonas, metas LL, manual, brake, snapshot, cámara, datum). Levantado
por `no_go_editor.launch.py`. La UI vive en `src/map_tools/web/index.html`.

### 1.6 Diferencias sim vs real

| Aspecto | Sim (`sim_global_v2`) | Real (`real_global_v2`) |
|---|---|---|
| `use_sim_time` | True | False |
| Sensores | Gazebo + `sim_sensor_normalizer_v2` | MAVROS (Pixhawk/GNSS) |
| Actuación | backend `sim_gazebo` (`/cmd_vel_gazebo`) | UART `serial_port:=auto` (`USB-TTL by-id` o `/dev/serial0`) |
| `use_keepout` | True | False |
| Params Nav2 | `*_sim_rolling*` | `*_real_rolling*` |
| `pointcloud_to_laserscan` | `pointcloud_to_laserscan.yaml` (sim) | `pointcloud_to_laserscan_real.yaml` |
| GPS topic | `/gps/fix` (compat) **(por confirmar)** | `/global_position/raw/fix` |

Política de paridad documentada en `docs/sim-real-parity.md`.

---

## 2. Control (`src/controller_server`)

### 2.1 Pipeline

`controller_server_node.py` consume `/cmd_vel_final` (`CmdVelFinal`) y, vía
`control_logic.py`, lo transforma en un `DesiredCommand` (velocidad, steer %,
brake %) que entrega a un **transport backend** (`transport_backends.py`):

- **`uart`** (real): envía al firmware del vehículo por serie
  (`serial_port:=auto`, `serial_baud:=115200`, `serial_tx_hz=50`). El resolvedor
  prioriza `SALUS_CONTROLLER_SERIAL_PORT`, un USB-TTL estable por `by-id`, luego
  `ttyUSB*`, y por último `/dev/serial0`. Código de referencia del firmware en el robot:
  `~/codigo/RASPY_SALUS` (`AGENTS.md`).
- **`sim_gazebo`** (sim): traduce a `/cmd_vel_gazebo` + lee `/odom_raw` y
  `/joint_states` para telemetría (`sim_gazebo_backend.py`).

Publica `/controller/status`, `/controller/telemetry` (JSON) y
`/controller/drive_telemetry` (`DriveTelemetry`, consumido por odometría/heading).

### 2.2 Conversión cmd_vel → Ackermann (`command_from_cmd_vel`)

- Curvatura deseada desde `linear_x`/`angular_z`; ángulo de dirección desde
  `wheelbase_m`; recorte operativo con `operational_steering_limit_rad` y
  recorte físico final con `steering_limit_rad`.
- Reporta **saturación** de steering (`steer_saturated`) con warning
  (`controller_server_node.py:202-213`).
- `vx_deadband_mps`: zona muerta de velocidad.
- `vx_min_effective_mps`: piso de velocidad efectiva (evita comandos que no mueven).
- `invert_steer_from_cmd_vel`: inversión de signo de dirección (default True real).

La fuente de verdad de giro automático es el límite operativo de dirección:

```text
radio = wheelbase_m / tan(operational_steering_limit_rad)
0.94m / tan(18deg) = 2.89m
1.6m/s / 0.4rad/s = 4.0m -> Nav2 global V2 usa 4.0m
```

`steering_limit_rad=30deg` queda como límite físico/hard cap. El radio de
planificación global se tunea con margen sobre el límite operativo: `4.0m` deja
reserva entre la curva planificada (~13° a wheelbase 0.94m) y el límite
automático de dirección (18°) para que el controlador pueda corregir seguimiento
sin que el planner tenga que inventar giros cerrados.

Los obstaculos no dependen de replanning por reloj: cuando el global costmap
marca un obstaculo sobre el path, `IsPathValid` invalida la trayectoria y el BT
recalcula. Si el path no colisiona pero atraviesa inflacion alta, el validador de
clearance tambien fuerza replan. El `collision_monitor` sigue siendo la capa
reactiva para frenar o reducir velocidad aunque el path global todavia no haya
cambiado.

### 2.3 Límites (defaults del nodo)

| Parámetro | Default nodo | Real (`real_global_v2`) | Significado |
|---|---|---|---|
| `max_speed_mps` | 4.0 | (default) | velocidad máx adelante |
| `max_reverse_mps` | 1.30 | 1.30 | velocidad máx atrás |
| `max_abs_angular_z` | 0.4 | 0.4 | legacy/compat; no define el radio operativo |
| `steering_limit_rad` | 0.5236 (30°) | 0.5236 | tope físico de dirección |
| `operational_steering_limit_rad` | 0.3142 (18°) | 0.3142 | tope automático de dirección |
| `wheelbase_m` | 0.94 | 0.94 | batalla Ackermann |
| `vx_deadband_mps` | 0.10 | 0.01 | zona muerta |
| `vx_min_effective_mps` | 0.75 | 0.5 | piso efectivo |
| `reverse_brake_pct` | 20 | — | freno asistido en reversa |
| `estop_brake_pct` | 100 | — | freno en e-stop |
| `control_hz` | 30 | — | lazo de control |

Fuente: `controller_server_node.py:26-56`.

### 2.4 Watchdogs y failsafes

| Mecanismo | Cómo | Efecto |
|---|---|---|
| **Watchdog de comando auto** | `select_effective_command` con `auto_timeout_s=0.7` | si `/cmd_vel_final` está “viejo” (>0.7 s), source pasa a `auto_timeout` y se aplica `safe_command()` |
| **`safe_command()`** | `control_logic.py:38-44` | velocidad 0, `estop=False`, `brake_pct=0` → **detención por coast (sin freno activo)**. ⚠ Ver nota abajo |
| **E-stop** | `brake_pct>0` en `CmdVelFinal` ⇒ `estop=True`; en `_control_tick` fuerza `drive_enabled=False`, `speed=0`, `brake_pct≥estop_brake_pct(100)` | freno activo total |
| **Telemetría stale** | `telemetry_stale_timeout_s=0.5` | marca `DriveTelemetry.fresh=False` |
| **Manual watchdog** | `nav_command_server` `manual_cmd_timeout_s=0.4`, `manual_watchdog_hz=10` | corta manual viejo |
| **collision_monitor** | polígonos `stop`/`slowdown` + `source_timeout=1.0` | si el scan se cae, deja al monitor sin fuente |

> ⚠ **Nota de seguridad (confirmado en repo; físico depende del firmware):** al
> expirar el watchdog de comando auto, `safe_command()` ordena velocidad 0
> (`drive_enabled=False`, `estop=False`, `brake_pct=0`), es decir **detención por
> inercia, no freno activo**. El freno fuerte solo ocurre vía e-stop (`brake_pct>0`).
> El protocolo UART transmite esos flags (`rpy_esp32_comms/protocol.py:38`) y la doc
> del firmware describe ese estado seguro como `drive=off, estop=off, brake=0`
> (`controller/COMUNICACIONES_UART_V2.md:75`). El comportamiento físico final lo
> define el firmware/vehículo. Ver [DEUDA_TECNICA](DEUDA_TECNICA.md).

### 2.5 collision_monitor (`collision_monitor_v2.yaml`)

| Polígono | Acción | Geometría (m) | Efecto |
|---|---|---|---|
| `footprint` | `stop` | ~1.05 × ±0.38 | parada dura si hay punto dentro |
| `stop_zone` | `approach` | ~2.05 × ±0.68 | frena ante obstáculo frontal al avanzar, pero permite retroceder |
| `critical_slow_zone` | `slowdown` | ~3.50 × ±0.98 | `slowdown_ratio=0.4375` (~0.7 m/s) |
| `slow_zone` | `slowdown` | ~5.35 × ±1.18 | `slowdown_ratio=0.75` (~1.2 m/s) |

`cmd_vel_in=/cmd_vel`, `cmd_vel_out=/cmd_vel_safe`, `base_frame=base_footprint`,
`source_timeout=1.0`, fuente única `scan` (tópico inyectado por launch).
