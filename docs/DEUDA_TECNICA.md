# Deuda técnica y riesgos — ROS2_SALUS

Estado: revisión read-only del código actual. **No** se corrigió nada; cada punto
es una observación. **(por confirmar)** marca lo que requiere validar con el robot
o con quien conoce el firmware.

Hermanos: [ARQUITECTURA](ARQUITECTURA_ROS2_SALUS.md) ·
[NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md) ·
[SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md)

Severidad: 🔴 alta · 🟠 media · 🟡 baja.

---

## 1. Seguridad operacional

| # | 🔺 | Hallazgo | Evidencia | Sugerencia |
|---|----|----------|-----------|------------|
| 1.1 | 🔴 | **Pérdida de comando = coast, no freno (confirmado en repo).** Al expirar el watchdog `auto_timeout_s=0.7`, `safe_command()` ordena velocidad 0 con `drive_enabled=False`, `estop=False`, `brake_pct=0` (detención por inercia). El freno fuerte solo ocurre en e-stop (`brake_pct>0`). | `control_logic.py:38-44,155`, `controller_server_node.py:227,232-247`, protocolo `rpy_esp32_comms/protocol.py:38`, firmware `controller/COMUNICACIONES_UART_V2.md:75` | Confirmar con el vehículo si “speed 0 sin freno” es seguro ante caída de Nav2; evaluar freno activo en timeout. (Comportamiento físico = firmware.) |
| 1.2 | 🟠 | **Keepout deshabilitado por default en real** (`use_keepout=False`). El robot real no aplica la máscara no-go salvo override explícito. | `real_global_v2.launch.py:254` | Documentar/operar con `use_keepout:=True` cuando la máscara esté validada. |
| 1.3 | 🟠 | **collision_monitor con fuente única `scan` y `source_timeout=1.0`.** Si el LiDAR/scan se cae, el monitor queda sin observación; combinado con 1.1 el fallo no frena activamente. | `collision_monitor_v2.yaml:12,57-61` | Definir comportamiento fail-safe ante pérdida de scan. |
| 1.4 | 🟡 | `min_height=0.50` en `pointcloud_to_laserscan_real.yaml` tapa el piso pero **pierde obstáculos bajos** (falsos negativos conocidos). | `pointcloud_to_laserscan_real.yaml:11` | El `scan_ground_filter` (POC) lo resuelve; falta validarlo y cablearlo en real. |

---

## 2. Colisiones de nombres / tópicos / frames

| # | 🔺 | Hallazgo | Evidencia |
|---|----|----------|-----------|
| 2.1 | 🟠 | **Doble `controller_server`.** El nodo propio se llama `controller_server` en `controller_server.launch.py` (choca con el `controller_server` de Nav2). En `real_global_v2` se renombra `vehicle_controller_server` para evitarlo, pero el launch standalone no. | `controller_server.launch.py:11`, `real_global_v2.launch.py:517` vs `nav_global_v2.launch.py:147-158` |
| 2.2 | 🟠 | **Dos convenciones GPS.** Mainline MAVROS usa `/global_position/raw/fix`; el driver legado `pixhawk_driver` y los defaults de `no_go_editor` usan `/gps/fix`. | `mavros.launch.py:210`, `pixhawk_driver.py:324`, `no_go_editor.launch.py:58` |
| 2.3 | 🟠 | **Status RTK con dos nombres**: `/gps/rtk_status` (default de `rtk_bridge`/`no_go_editor`) vs `/gps/rtk_status_mavros` (lo que setea `real_global_v2`). | `mavros.launch.py:108-111`, `real_global_v2.launch.py:380-383` |
| 2.4 | 🟡 | **Dos convenciones de odometría.** Mainline: `/controller/drive_telemetry`→`ackermann_odometry`→EKF. Legado: `pixhawk_driver` publica `/odom`. El prompt asume `/odom`. | `runtime-architecture.md`, `pixhawk_driver.py:325` |

> Riesgo: un launch o script que asuma la convención equivocada queda sin datos
> (subscriber sin publisher) de forma silenciosa.

---

## 3. Launches y configs legacy / duplicados

| # | 🔺 | Hallazgo |
|---|----|----------|
| 3.1 | 🟡 | Muchos launches legacy conviven con los mainline: `simulacion.launch.py`, `real.launch.py`, `sim_local_v2`, `real_local_v2`, `nav2_only`, `rviz_real*`. `AGENTS.md` pide tratarlos como referencia, pero siguen ejecutables. |
| 3.2 | 🟡 | **Sprawl de configs Nav2**: `nav2_global_v2_{real,sim}_rolling[_wifi]_params.yaml`, `nav2_local_v2_*`, `nav2_no_map_params.yaml`. Y de collision_monitor: `collision_monitor.yaml` (legacy), `_v2.yaml`, `_lidar_only.yaml`. Riesgo de editar el archivo equivocado. |
| 3.3 | 🟡 | **Variantes EKF**: v2 global usa `localization_global_v2.yaml` + `localization_v2.yaml`; los 5 `dual_ekf_navsat_params*.yaml` quedan como **legado/sprawl** (auditado). |
| 3.4 | 🟡 | `tools/vcs-*.sh` existen pero `README` desaconseja vcstool en este checkout (monorepo). Mensaje contradictorio. |

---

## 4. Dependencias implícitas

| # | 🔺 | Hallazgo |
|---|----|----------|
| 4.1 | 🟠 | **MAVROS** es dependencia de ejecución del perfil real (apt `ros-humble-mavros*`); declarada en `sensores/package.xml` pero su instalación correcta + dialecto APM es supuesto. |
| 4.2 | 🟠 | **`pointcloud_to_laserscan`** (paquete Nav2/ROS) requerido por todos los perfiles; no es paquete propio. |
| 4.3 | 🟡 | El POC `scan_ground_filter` real necesitaría sumar deps de sistema al `Dockerfile` si se compilara Autoware (no es el caso del port Python, que solo usa numpy). |
| 4.4 | 🟡 | Firmware del vehículo (UART) es dependencia externa no versionada aquí: `~/codigo/RASPY_SALUS` en el robot. |

---

## 5. Higiene del repositorio

| # | 🔺 | Hallazgo |
|---|----|----------|
| 5.2 | 🟡 | Raíz del repo con artefactos no versionados/transitorios: `aw_build.log`, `aw_deps.log`, `borrador/`, `cockpit*/`, `*.mb`, `franco-borrador_force-push_diff.patch`, `.codex`. Conviene `.gitignore`/limpieza. |
| 5.3 | 🟡 | `README.md` y varios `docs/*` citan rutas `/home/leo/codigo/ROS2_SALUS` (otra máquina); los links internos quedan rotos en este checkout. |
| 5.4 | ✅ | `src/autoware_deps/` (~624 MB, vendor POC), los configs `autoware_*.param.yaml`, los logs `aw_*.log` y la imagen Docker `:aw` **fueron eliminados** (solo eran referencia). Resuelto. |
| 5.5 | 🟠 | **Overlay/`install` stale aborta launches.** Lanzar aborta con `package 'lidar_camara' not found` (overlay viejo referencia un paquete que ya no existe como tal). | Recompilar limpio: `rm -rf build install log && colcon build --symlink-install`. |

---

## 6. Cosas que parecen incompletas / a confirmar

- `scan_ground_filter`: validado en sim pero **no cableado en real**; falta FN 0/2
  en el `/scan` 2D final. (Ver [SENSORES_Y_LIDAR](SENSORES_Y_LIDAR.md) §4.)
- `sim_global_v2.launch.py` se leyó parcialmente; confirmar lista exacta de nodos.
- Tipos de mensaje de varios tópicos de nav/heading marcados **(por confirmar)** en
  [TOPICOS_Y_TF](TOPICOS_Y_TF.md).
- `src/lidar_camara` y `src/vision_pipeline` (stack visión YOLO): estado activo/legado
  no confirmado en esta revisión.

---

## 7. Resumen ejecutivo

### ✅ Qué está sólido
- Arquitectura de control clara y desacoplada:
  `Nav2 → collision_monitor → nav_command_server → controller_server`, con contrato
  propio `CmdVelFinal` y telemetría de actuación.
- Paridad sim/real bien pensada (mismo wiring, params por perfil, backend de
  control intercambiable uart/sim_gazebo).
- Localización en dos capas EKF + navsat + heading GPS RTK-gated, robusta y
  documentada.
- Inyección del scan efectivo y de overrides keepout por `RewrittenYaml` (un solo
  punto de verdad para qué scan consume todo el stack).
- `scan_ground_filter` portado, testeado y validado en sim (FP −95 %).

### ⚠️ Qué está frágil
- Failsafe ante pérdida de comando/scan es **coast, no freno** (1.1, 1.3) — el
  punto más importante a confirmar.
- Convenciones dobles de tópicos GPS/RTK/odometría (2.2–2.4): fácil romper en
  silencio.
- Colisión de nombre `controller_server` (2.1).
- Higiene: overlay stale que aborta launches (5.5), rutas de docs rotas (5.3),
  sprawl de configs (3.2).

### 🧪 Qué conviene probar en sim
- Re-correr el A/B del `scan_ground_filter` confirmando **FN 0/2 en `/scan` 2D**
  con `scan_ground_max_height`.
- Verificar comportamiento de `collision_monitor` + watchdog ante caída del scan
  (matar el LiDAR y observar `/cmd_vel_final`).
- Validar `use_keepout:=True` end-to-end antes de habilitarlo en real.

### 🤖 Qué conviene probar en robot
- Confirmar el contrato de freno real del firmware ante `safe_command()` (1.1).
- Cadena RTK real (`rtk_bridge` + `/gps/rtk_status_mavros`) y heading GPS.
- Latencia `/scan_3d → /scan(_clean) → costmap` en la Raspberry Pi 5.

### 🚫 Qué no tocar todavía
- `src/rslidar_*` (vendor).
- Launches de producción (`real_global_v2*`, `sim_global_v2*`) sin un plan de
  validación: cambiar convenciones de tópicos o nombres ahí puede romper el stack.
- El wiring del `scan_ground_filter` en real hasta cerrar la validación 2D.
