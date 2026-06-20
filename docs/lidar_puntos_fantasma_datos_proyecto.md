# Puntos fantasma en LiDAR inclinable — datos reales del proyecto SALUS

Companion del informe *"Eliminación y filtrado de puntos fantasmas en un LiDAR con
soporte inclinable en ROS2"* (ChatGPT, PDF en Descargas). Este documento completa
las incógnitas que el informe deja abiertas y mapea cada recomendación contra lo
que ya existe en este repo, para seguir investigando con datos concretos.

## 1. Respuestas a las "preguntas abiertas" del PDF

### ¿El LiDAR es 2D o 3D?

**3D.** RoboSense **RS16** (mecánico, 16 haces) en el robot real:

- Config: `src/sensores/config/rs16.yaml` — `lidar_type: RS16`, `min_distance: 0.4`,
  `max_distance: 20`, frame `lidar_link`, publica PointCloud2 en **`/scan_3d`**.
- Launch: `src/sensores/launch/rs16.launch.py` (envuelve `rslidar_sdk` vendorizado).
- Ojo: `src/rslidar_sdk/config/config.yaml` dice `RSM1`, pero es el default del SDK;
  el launch real usa `rs16.yaml`.
- En simulación global V2: sensor `gpu_lidar` Gazebo, 360 muestras, FOV ±90°,
  ruido σ=0.003, usando `models/cuatri_real_v2.urdf` por default.

Conclusión del PDF aplicable: al ser 3D, la ruta correcta es
**gravity alignment + ground removal + persistencia**, no gating de scans 2D.

### ¿Altura exacta de montaje?

Del URDF real (`src/navegacion_gps/models/`):

| URDF | `lidar_joint` (base_link → lidar_link) | Inclinación |
|---|---|---|
| `cuatri_real.urdf` | `xyz="0.92 0.0 0.65"` | `rpy="0 0 0"` (plano) |
| `cuatri_real_v2.urdf` | `xyz="0.92 0.0 0.65"` | `rpy="0 0.1745 0"` → **pitch +10° (hacia abajo)** |

- Altura: **0.65 m** sobre `base_link` (= `base_footprint`, offset 0).
- Adelanto: 0.92 m respecto del eje base.
- El "soporte inclinable" del informe es el montaje v2 con pitch fijo de 10°.
  A 0.65 m con 10° de pitch, el haz central toca el piso a ~3.7 m delante del
  sensor (~4.6 m de base_link) — dentro del `obstacle_max_range: 10.0` del costmap.

### ¿El frenado falso ocurre en costmap, collision_monitor o ambos?

Hay **tres consumidores de scan que pueden frenar**:

| Consumidor | Tópico que consume | ¿Filtrado? |
|---|---|---|
| Nav2 costmaps (voxel/obstacle layer) | `effective_lidar_scan_topic` (ver §2) | Sí (cuando los filtros están activos) |
| `collision_monitor` v2 (`config/collision_monitor_v2.yaml`) | `effective_lidar_scan_topic` (RewrittenYaml en `nav_global_v2.launch.py:91`) | Sí |
| `lidar_brake_guard` (commit `a57e0c5`) | **`/scan` crudo** | **No** — pero no existe en esta rama (solo en `camara_lidear`, `pagina-terminada-yolo-sin-rtsp`, `pagina_bonita`; verificar si corre en el deployment 2antenas) |

- `collision_monitor` v2: polígonos `footprint` (stop), `stop_zone` (approach,
  2.05 m, TTC 2.0 s), `critical_slow_zone` (3.50 m, slowdown
  0.4375) y `slow_zone`; `/cmd_vel` → `/cmd_vel_safe`. El `collision_monitor.yaml`
  viejo (stop_zone 2.19 m, `/scan` crudo, `use_sim_time: true`) es legacy.
- `lidar_brake_guard`: publica `/fusion/brake_active=True` si hay algo a
  < `brake_distance_m: 1.5` en ±30° frontal de `/scan`; dispara `obstacle_recovery`.

Con pitch de 10°, el haz central toca el suelo a ~3.7 m — **dentro de la
`critical_slow_zone` (3.50 m)**: un retorno de suelo mal proyectado puede
provocar slowdown directo además de marcar el costmap. Cabeceos del chasis o
rampas acercan aún más ese punto de contacto.

**Estado actualizado**: esa sospecha ya quedó resuelta en los perfiles globales
V2 activos. `sim_global_v2*`, `real_global_v2*` y sus RViz usan por default
`cuatri_real_v2.urdf`, por lo que el TF publicado refleja el LiDAR inclinado.
Este documento se conserva como trazabilidad del diagnóstico original; ante duda,
la fuente operativa es `docs/launch-matrix.md`.

## 2. Pipeline actual del proyecto (lo que el PDF llama "pipeline sugerido")

```
RS16 → /scan_3d (PointCloud2, frame lidar_link)
  ├── pointcloud_to_laserscan → /scan (target_frame base_footprint)
  │     ├── [default] scan_noise_filter → /scan_clean
  │     ├── [opcional] (no se usa si lidar_obstacle_filter está activo)
  │     ├── collision_monitor  ← consume /scan CRUDO
  │     └── lidar_brake_guard  ← consume /scan CRUDO
  └── [opcional] lidar_obstacle_filter → /obstacles_cloud + /scan_filtered

effective_lidar_scan_topic = /scan_filtered si enable_lidar_obstacle_filter
                            sino /scan_clean si enable_scan_noise_filter (default True)
                            sino /scan
→ va a Nav2 (nav_global_v2.launch.py) y scan_wifi_debug
```

Selección en `launch/real_global_v2.launch.py:201-232`. Defaults:
`enable_lidar_obstacle_filter=False`, `enable_scan_noise_filter=True`.
**Es decir: en producción hoy corre solo el speckle filter 2D; la rama 3D con
compensación IMU + RANSAC existe pero está apagada.**

### pointcloud_to_laserscan (config real: `pointcloud_to_laserscan_real.yaml`)

| Parámetro | Valor real | Nota |
|---|---|---|
| `target_frame` | `base_footprint` | |
| `min_height` / `max_height` | **0.50 / 1.50** | El comentario del yaml documenta el problema exacto: el chasis cabecea sin que aparezca en el TF plano, y se subió la cota para recortar retornos rasantes. Costo: obstáculos < 0.5 m invisibles (falsos negativos). |
| `range_min` / `range_max` | 0.4 / 20.0 | |
| FOV | ±90° (`±1.5708`) | |
| `transform_tolerance` | 0.1 | |

La variante sim inclinada (`pointcloud_to_laserscan_tilted_lidar_sim.yaml`) usa
min_height −0.10 / max 1.60 — sirve de baseline para comparar.

### scan_noise_filter (rama default, `navegacion_gps/scan_noise_filter.py`)

Speckle 2D: `speckle_filter_window: 2`, `speckle_max_deviation_m: 0.30`,
aplicado hasta `speckle_max_range_m: 12.0`, rango 0.4–20 m. Equivale a la
"primera barrera" outlier del PDF, pero sobre LaserScan, no sobre la nube.

### lidar_obstacle_filter (rama 3D, apagada por default — `navegacion_gps/lidar_obstacle_filter.py`)

Implementa casi todo lo que el PDF recomienda para sensores 3D:

| Etapa PDF | Implementación | Parámetros actuales |
|---|---|---|
| Compensación IMU (gravity alignment) | roll/pitch desde `/imu/data` (cuaternión), `use_imu_compensation: True`, `imu_max_age_s: 0.5` | rota la nube con R(−roll,−pitch) |
| CropBox / ROI | ROI en frame del robot | x ∈ [−0.4, 12], y ∈ [±2.5], z ∈ [−1, 2] |
| Ground removal | RANSAC de plano | `ransac_iterations: 64`, `ground_distance_threshold: 0.18`, `min_ground_points: 24`, `ground_candidate_percentile: 95` |
| Banda de obstáculo | altura relativa | `min_obstacle_height: 0.22`, `max_obstacle_height: 1.40` |
| Clustering / densidad | grilla voxel (no euclidean) | voxel 0.25×0.25×0.20 m, `min_voxel_points: 3` |
| Salida | `/obstacles_cloud` (PC2) + `/scan_filtered` (LaserScan 2D) | rango 0.4–12 m |

Lo que **no** tiene respecto del pipeline del PDF: gate de evento de inclinación
(rechazo de frame si |roll|/|pitch| > umbral — solo compensa, nunca rechaza),
persistencia temporal multi-frame, y distortion correction intra-scan.

### IMU y robot_localization

- Fuente: Pixhawk vía `pixhawk_driver.py` → `/imu/data` (orientación de
  ATTITUDE_QUATERNION, accel/gyro de SCALED_IMU2). La IMU está en el chasis
  (`imu_link`), **no solidaria al LiDAR** (primera recomendación mecánica del PDF).
- `two_d_mode: true` en **todos** los EKF (`localization_v2.yaml`,
  `dual_ekf_navsat_params.yaml`, `localization_global_v2.yaml`) — exactamente el
  caso que advierte el PDF: el EKF anula roll/pitch. Por eso `lidar_obstacle_filter`
  lee el IMU crudo y no la salida del EKF. Correcto según el informe: la rama de
  navegación sigue 2D, la rama de percepción preserva roll/pitch.

## 3. Nav2: valores del proyecto vs sugerencias del PDF

Local costmap (`nav2_global_v2_real_rolling_params.yaml`, VoxelLayer):

| Parámetro | Proyecto | PDF sugiere | Comentario |
|---|---|---|---|
| `plugins` | `[voxel_layer, inflation_layer]` | + **DenoiseLayer** antes de inflation | **Gap: no hay DenoiseLayer** |
| `observation_persistence` | **0.6 s** | **0.0–0.05 s** | Gap: un frame malo inclinado sobrevive 0.6 s |
| `mark_threshold` | **0** | **2–3** | Gap: un solo voxel ocupado marca obstáculo |
| `z_resolution` / `z_voxels` | 0.05 / 16 | — | banda 0–0.8 m |
| `min/max_obstacle_height` | 0.0 / 1.60 | — | min 0.0 deja pasar retornos a ras |
| `obstacle_max_range` | 10.0 (marca) / 20.0 (raytrace) | — | ya tuneado: comentario del yaml explica que se bajó para no marcar retornos rasantes lejanos |
| footprint | `[[1.05,0.38],[1.05,-0.38],[-0.12,-0.38],[-0.12,0.38]]` | — | |

Global costmap: ObstacleLayer, `observation_persistence: 1.0`, `obstacle_max_range: 8.0`.

## 4. Umbrales del PDF mapeados a valores del proyecto

| Clase (tabla del PDF) | Umbral sugerido PDF | Valor actual en SALUS |
|---|---|---|
| Evento de inclinación (gate) | 5–7° abs, >3° salto | **No implementado** (nominal sería 10° por el soporte v2) |
| Suelo: pendiente global/local | 8° / 10° | N/A (se usa RANSAC, no slope-based) |
| Suelo: altura relativa | 0.2 m | `ground_distance_threshold: 0.18`, `min_obstacle_height: 0.22` ✓ |
| RANSAC distancia al plano | 0.01–0.03 indoor / 0.03–0.08 outdoor | **0.18** — mucho más laxo; candidato a bajar a 0.05–0.10 outdoor |
| Cluster válido | 5–15 puntos | `min_voxel_points: 3` por voxel (más laxo) |
| Persistencia temporal | rechazar si <2 de 3 frames | **No implementado** |
| `minimal_group_size` (Denoise) | 2 | **No hay DenoiseLayer** |
| `mark_threshold` (voxel) | 2–3 | **0** |
| `observation_persistence` | 0.0–0.05 | **0.6** |

## 5. Líneas de investigación priorizadas (gaps concretos)

1. **Verificar la extrínseca LiDAR–base_link** (URDF default plano vs montaje
   físico inclinado 10°) y corregir el TF. Ver plan detallado en
   `docs/plan_correccion_puntos_fantasma.md`.
2. **Activar la rama 3D**: `enable_lidar_obstacle_filter:=True` en
   `real_global_v2.launch.py` y validar con bag. La infraestructura ya está; está
   apagada por default. Con eso `min_height: 0.50` del p2l podría volver a bajar
   (hoy oculta obstáculos < 0.5 m).
3. **Agregar gate de inclinación** en `lidar_obstacle_filter`: rechazar/degradar el
   frame si |roll| o |pitch| del IMU supera nominal+5–7° o salta >3° entre frames
   (hoy solo compensa, nunca rechaza).
4. **Costmap**: probar `mark_threshold: 2`, `observation_persistence: 0.0–0.05` y
   agregar `nav2_costmap_2d::DenoiseLayer` (`minimal_group_size: 2`) antes de
   inflation.
5. **Persistencia temporal** en `lidar_obstacle_filter` (exigir N de M frames antes
   de publicar un voxel como obstáculo).
6. **Distortion correction**: el RS16 es mecánico a ~10 Hz y el robot se mueve; no
   hay corrección intra-scan. Evaluar si vale la pena antes que 1–5.
7. **RANSAC threshold**: 0.18 m es laxo para outdoor según PDF (0.03–0.08);
   barrer 0.05/0.10/0.18 con bags.
8. **Validación con rosbag2**: escenarios del PDF (plano, inclinación 0/3/5/8°,
   aceleración/frenado, rampas). Tooling existente:
   `tools/run_localization_replay_compare.sh`, grabación de misiones jsonl.
   KPI principal: frenados sin obstáculo real por hora / por 100 m.

## 6. Notas operativas

- El robot real (100.111.4.7) ejecuta el contenedor montado sobre
  `/home/franco/final/2antenas` **(desactualizado: este trabajo se valida y
  despliega sobre ROS2_SALUS, no sobre 2antenas — verificar/recrear los montajes
  del contenedor del robot antes de la sesión de campo; ver Etapa B del plan)**.
- Historia git relevante: `feature/lidar-3d-ground-filter` (merge `c574cab`),
  "Filter ground hits in global v2 lidar" (`91259c8`), "Merge lidar ghost clearing
  fixes" (`4cc63da`), "Tune real lidar height filters" (`c8aa574`), speckle filter
  (`cdefde2`), `lidar_brake_guard` (`a57e0c5`).
- Checklists de campo: `src/navegacion_gps/REAL_GLOBAL_V2_CHECKLIST.md` y
  `REAL_LOCAL_V2_CHECKLIST.md` (collision_monitor y stop_zone activos).
