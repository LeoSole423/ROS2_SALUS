# Sensores y LiDAR — ROS2_SALUS

Estado: documentación read-only del código actual.
Fuente de verdad: `src/sensores/**`, `src/navegacion_gps/**`, `src/rslidar_*`.
**(por confirmar)** marca lo no verificado leyendo el código línea por línea.

Hermanos: [ARQUITECTURA](ARQUITECTURA_ROS2_SALUS.md) ·
[TOPICOS_Y_TF](TOPICOS_Y_TF.md) · [NAVEGACION_Y_CONTROL](NAVEGACION_Y_CONTROL.md)

---

## 1. Pixhawk / GNSS

### 1.1 Mainline: MAVROS (`sensores/mavros.launch.py`)

`mavros_node` se conecta a la Pixhawk (`fcu_url=/dev/ttyACM0:921600`) y **remapea**
a contrato canónico:

| MAVROS nativo | Remap |
|---|---|
| `mavros_node/data` | `/imu/data` |
| `mavros_node/raw/fix` | `/global_position/raw/fix` |
| `mavros_node/velocity_local` | `/local_position/velocity_local` |
| `mavros_node/odom` | `/local_position/odom` |

Nodos acompañantes (condicionales):
- `mavros_compat_bridge` — re-publica al contrato legacy (`launch_legacy_compat`).
- `rtk_bridge` — feed RTCM al plugin `gps_rtk` y publica status RTK consolidado
  (`rtk_status_topic`, default `/gps/rtk_status`; en real global se usa
  `/gps/rtk_status_mavros`).
- `rtk_source_manager` — gestor de bases NTRIP para la UI (`enable_rtk_source_manager`).
- `sensores_web` — dashboard (`launch_web`).

Config MAVROS: `sensores/config/mavros_sensor_only_pluginlists.yaml`,
`mavros_apm_overrides.yaml`, `rtk_sources.yaml`.

### 1.2 Legado: `pixhawk_driver` (`sensores/pixhawk.launch.py`)

Driver propio que lee MAVLink directo y publica `/imu/data`, `/gps/fix`,
`/gps/rtk_status`, `/gps/fix_type`, `/gps/satellites_visible`, `/gps/hdop`,
`/gps/rtcm_*`, `/odom`, `/velocity`. Frames: `odom`, `base_footprint`, `imu_link`,
`gps_link`. **No** forma parte de `real_global_v2`. Útil como referencia/banco.
(`pixhawk_driver.py:6-16`).

> Conviven dos convenciones de tópicos GPS (`/gps/fix` legacy vs
> `/global_position/raw/fix` MAVROS) y de status RTK (`/gps/rtk_status` vs
> `/gps/rtk_status_mavros`). Ver [DEUDA_TECNICA](DEUDA_TECNICA.md).

### 1.3 Cámara / web

`sensores/camara` (servicios `CameraPan`, `CameraStatus`) y `sensores/web_server`
(`sensores_web`, dashboard `pixhawk_dashboard.html`). **(por confirmar detalle.)**

### 1.4 Healthchecks

- `tools/healthcheck-lidar.sh` (ver §3.5).
- `sensores_web` / `pixhawk_dashboard.html`: dashboard de IMU/GPS/RTK.
- No se identificó un healthcheck dedicado de Pixhawk/MAVROS más allá del
  dashboard y `/diagnostics`. **(por confirmar.)**

---

## 2. RS16 (LiDAR 3D)

- Driver: `rslidar_sdk_node` (`sensores/rs16.launch.py`, config
  `sensores/config/rs16.yaml`).
- Tipo `RS16`, `msop_port: 6699`, frame `lidar_link`, publica `/scan_3d`
  (PointCloud2). `/rslidar_packets` está configurado pero **no se publica por
  default** (`send_packet_ros: false`). `/rslidar_imu_data` está configurado con
  **`imu_port: 0`**, por lo que no es un topic activo garantizado (no lo usa el EKF).
- Montaje físico: ~0.65 m sobre `base_link`, x≈+0.92 m. URDF plano
  `cuatri_real.urdf`; variante con pitch +10° `cuatri_real_v2.urdf`
  (peor caso “robot cabeceado”). Ver `docs/lidar_puntos_fantasma_datos_proyecto.md`.

---

## 3. Pipeline LiDAR → obstáculos

### 3.1 Flujo base (default real)

```text
RS16 -> /scan_3d (PointCloud2, lidar_link)
     -> pointcloud_to_laserscan (target_frame base_footprint) -> /scan
     -> scan_noise_filter (default ON) -> /scan_clean
     -> Nav2 costmaps + collision_monitor
```

El **scan efectivo** lo elige el launch (`effective_lidar_scan_topic`) y se inyecta
en costmaps y collision_monitor por `RewrittenYaml`. Orden de preferencia
(`real_global_v2.launch.py:201-213`):
1. `enable_lidar_obstacle_filter=True` → `/scan_filtered`
2. else `enable_scan_noise_filter=True` → `/scan_clean` (default)
3. else `/scan`

### 3.2 `pointcloud_to_laserscan`

Aplana la nube 3D a 2D. Configs por perfil:
- `pointcloud_to_laserscan_real.yaml` (real): `target_frame=base_footprint`,
  `min_height=0.50`, `max_height=1.50`, `range 0.4–20`.
- `pointcloud_to_laserscan.yaml` (sim), `_tilted_lidar_sim.yaml`,
  `_real_cuatri_real_v2.yaml` (variantes según montaje/mundo).

El `min_height=0.50` alto es la mitigación actual de **puntos fantasma del piso**:
recorta retornos rasantes a costa de perder obstáculos bajos (falsos negativos).

### 3.3 `scan_noise_filter` (default ON en real)

`navegacion_gps/scan_noise_filter`. Filtro de speckle + recorte de rango sobre el
`/scan` 2D → `/scan_clean`. Params (real): `filter_range 0.4–20`,
`speckle_filter_window=2`, `speckle_max_range_m=12`, `speckle_max_deviation_m=0.30`.

### 3.4 `lidar_obstacle_filter` (rama RANSAC 3D, default OFF)

`navegacion_gps/lidar_obstacle_filter`. Compensación IMU + RANSAC de suelo + voxel
+ gate de inclinación + persistencia temporal, sobre `/scan_3d` → `/scan_filtered`
(+ `/obstacles_cloud` debug). Se activa con `enable_lidar_obstacle_filter:=True`.
ROI/umbrales configurables por launch (`lidar_filter_*`). Estado: experimental,
validado en sim en la rama de fantasmas. Ver
`docs/lidar_puntos_fantasma_datos_proyecto.md` y `docs/plan_correccion_puntos_fantasma.md`.

### 3.5 Healthcheck LiDAR

```bash
./tools/healthcheck-lidar.sh    # hz de /scan_3d, /scan, /scan_clean,
                                # /scan_filtered, /obstacles_cloud + TF
```

---

## 4. `scan_ground_filter` (segmentación de suelo estilo Autoware) — POC

### 4.1 Qué problema resuelve

El RS16 montado con pitch ve el piso como obstáculo (**puntos fantasma**), que
generan frenados falsos. La mitigación actual (`min_height=0.50`) tapa el piso pero
pierde obstáculos bajos. El `scan_ground_filter` elimina el suelo en **3D** antes de
aplanar a 2D, permitiendo bajar `min_height` (recuperar obstáculos bajos) y subir
`max_height` (recuperar obstáculos sobre terreno inclinado) sin reintroducir piso.

### 4.2 Estado: cableado detrás de flag, default OFF

| Aspecto | Detalle |
|---|---|
| Nodo | `navegacion_gps/scan_ground_filter` (`ScanGroundSegmenter` puro + nodo) |
| Origen | Port del algoritmo *non-grid* de Autoware `scan_ground_filter` (v0.51.0), **sin** compilar Autoware. Verificado vs `node.cpp` (fidelidad). |
| Interfaz | `/scan_3d` → `/scan_3d/no_ground` (transforma a `base_footprint` antes de clasificar) |
| Config | `config/scan_ground_filter.param.yaml` (defaults RS16: global 10°, local 13°, radial 1°) |
| Flag | `enable_scan_ground_filter` (**default False**) en `sim_v2_base.launch.py` (vía `OpaqueFunction _build_lidar_pipeline`), forwardeado por `sim_local_v2` y `validate_scan_ground.launch.py` |
| Ventana 2D | con el flag on: `scan_ground_min_height` (default 0.10) y `scan_ground_max_height` (default 2.50) sobreescriben el `pointcloud_to_laserscan` |
| Tests | `test/test_scan_ground_filter.py` (núcleo) + `test/test_scan_ground_validation.py` |
| Rendimiento | ~25 ms / 28.8k pts (lazo en floats Python + `lexsort`) |

### 4.3 Validación (rampa) y qué falta antes de producción

Escenario A/B: `launch/validate_scan_ground.launch.py` sobre `worlds/slope_lidar.world`
+ nodo medidor `scan_ground_validation` (FP en `/local_costmap/costmap` y eventos de
freno `/cmd_vel` vs `/cmd_vel_safe`). Runner `scripts/run_scan_ground_validation.sh`.

Resultado A/B (Codex, sim headless): **FP −95.3 %** (16 654 → 791),
FN 0/2 a nivel `/scan_3d/no_ground`.

**Falta antes de producción:**
1. Re-correr A/B confirmando **FN 0/2 en el `/scan` 2D final** con
   `scan_ground_max_height` (el `slope_obstacle_right` quedaba sobre `max_height`).
2. **No** está cableado en `real_global_v2.launch.py` (solo en la cadena sim).
   Si valida, replicar el wiring ahí + sumar al `Dockerfile`, tuneando min/max a
   la geometría real.
3. El vendor `src/autoware_deps/` (~624 MB) y la imagen Docker `:aw` **fueron
   eliminados** (solo eran referencia; el algoritmo ya vive portado en Python).

Doc completo: `docs/autoware_ground_segmentation_integracion.md`
(§8 wiring, §9 validación).

---

## 5. Consumidores del scan

| Consumidor | Tópico | Frame |
|---|---|---|
| `local_costmap` voxel_layer | scan efectivo | `base_footprint`/`odom` |
| `global_costmap` obstacle_layer | scan efectivo | `map` |
| `collision_monitor` | scan efectivo (`scan.topic`) | `base_footprint` |

`collision_monitor` y los costmaps **comparten** el mismo scan efectivo, inyectado
en `nav_global_v2.launch.py:65-96`.
