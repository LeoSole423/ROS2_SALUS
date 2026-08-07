# Investigación: integración de Autoware `ground_segmentation` en SALUS (camino A)

Estado: **POC en simulación** — no toca el deployment de producción.
Fecha de inicio: 2026-06-13. Responsable: Franco.

## 1. Objetivo y decisión

Atacar el problema de **puntos fantasma del piso** (ver
`docs/investigaciones/lidar-puntos-fantasma-datos-proyecto.md`) reemplazando la rama de
ground-removal artesanal de `lidar_obstacle_filter.py` por el filtro
**`scan_ground_filter`** de Autoware (`autoware_ground_segmentation`), que es
código probado en producción.

Decisión tomada (2026-06-13):

- **Camino A**: integrar *solo* el componente de ground segmentation de Autoware
  junto a Nav2. NO se reemplaza Nav2 ni el resto del stack.
- **Probar primero en simulación** sobre bag/Gazebo, sin tocar el `Dockerfile`
  de producción ni el robot real.
- **Vendorizar** los paquetes de Autoware en `src/autoware_deps/` con un archivo
  `.repos` pineado a versiones del release Autoware v0.51.0 (reproducible).

## 2. Arquitectura objetivo

Hoy:

```
RS16 → /scan_3d (PointCloud2) → pointcloud_to_laserscan → /scan → Nav2
```

Con el filtro (detrás de arg `enable_autoware_ground_seg`, default False):

```
RS16 → /scan_3d (PointCloud2)
     → scan_ground_filter (Autoware) → /scan_3d/no_ground (PointCloud2)
     → pointcloud_to_laserscan (cloud_in := /scan_3d/no_ground) → /scan → Nav2
```

Interfaz limpia: entrada y salida son `sensor_msgs/PointCloud2` estándar. El
suelo se elimina en 3D **antes** de aplanar a 2D, por lo que el `min_height` del
`pointcloud_to_laserscan` (hoy en 0.50 m para tapar el piso) puede volver a bajar
a ~0.10 m y recuperar obstáculos bajos (falsos negativos actuales).

## 3. Dependencias (árbol de `autoware_ground_segmentation`)

Del `package.xml` (autoware.universe @ 0.51.0):

| Depend | Repo / origen | Versión |
|---|---|---|
| `autoware_pointcloud_preprocessor` | autoware_universe | 0.51.0 |
| `autoware_utils` | autoware_utils | 1.8.0 |
| `autoware_vehicle_info_utils` | autoware_universe (common/) | 0.51.0 |
| `managed_transform_buffer` | managed_transform_buffer | 0.2.0 |
| `autoware_cmake` (buildtool) | autoware_cmake | 1.2.0 |
| `autoware_msgs` (perception/vehicle) | autoware_msgs | 1.13.0 |
| `pcl_ros`, `tf2_*`, `yaml-cpp`, `libopencv-dev` | apt Humble | — |

> Posibles deps transitivas a resolver en build: `autoware_internal_msgs`,
> `autoware_point_types`, `cuda_blackboard` (pointcloud_preprocessor GPU path).
> Se resuelven iterativamente con `rosdep` + vcs durante el build.

## 4. Pasos

1. [ ] Crear `src/autoware_deps/autoware_ground_seg.repos` y `vcs import`.
2. [ ] Build aislado en contenedor: `colcon build --packages-up-to autoware_ground_segmentation`.
3. [ ] Resolver deps faltantes (rosdep / vcs adicionales).
4. [ ] Wiring en launch de **sim** detrás de `enable_autoware_ground_seg`.
5. [ ] `config/autoware_ground_segmentation.param.yaml` ajustado a RS16.
6. [ ] Validar en sim vs baseline (KPI: frenados sin obstáculo real).
7. [ ] Si mejora: replicar wiring en `real_global_v2.launch.py` + Dockerfile.

## 5. Log de progreso

- 2026-06-13: decisión y plan registrados. Inicio del overlay vendorizado.
- 2026-06-13: clonados (shallow, host git) `autoware_cmake` 1.2.0, `autoware_utils`
  1.8.0, `autoware_msgs` 1.13.0, `autoware_internal_msgs` 1.12.1,
  `managed_transform_buffer` 0.2.0, `autoware_universe` 0.51.0 (414 MB) en
  `src/autoware_deps/`. El `vcs` del host está roto (`pkg_resources`), se usó git.
- 2026-06-13: el contenedor `ros2_salus` en ejecución monta `/home/franco/final/2antenas`,
  NO ROS2_SALUS (confirmado con `docker inspect`). Para build/sim hay que recrear el
  contenedor con `docker-compose.salus.yml` (mounts a ROS2_SALUS) — ver memoria
  `project_deployment_2antenas`.

### Hallazgo de acoplamiento (scope creep)

`autoware_ground_segmentation` → `autoware_pointcloud_preprocessor`, que en v0.51.0
arrastra dependencias ajenas a segmentar suelo:

- `autoware_point_types`, `autoware_lanelet2_utils` → repo `autoware_core` (trae lanelet2)
- `autoware_lanelet2_extension` → repo aparte + apt `lanelet2`
- `autoware_agnocast_wrapper` → repo `agnocast` (capa de transporte, innecesaria aquí)
- `point_cloud_msg_wrapper` → repo aparte (chico)
- apt: `libcgal-dev`, `sophus`, `lanelet2`

Ya disponibles en lo clonado: `autoware_pcl_extensions`, `autoware_sensing_msgs`,
`autoware_vehicle_msgs`, `autoware_internal_debug_msgs`.

**Decisión pendiente** sobre cómo seguir (ver §6).

## 6. Caminos para resolver el acoplamiento

1. **Vendor completo v0.51.0**: agregar `agnocast`, `autoware_core`,
   `autoware_lanelet2_extension`, `point_cloud_msg_wrapper` + apt. Corre Autoware
   real y último, pero ~3-4 GB y build lento; riesgo de más cascada.
2. **Pinear a una versión más vieja y liviana** de autoware.universe donde
   `pointcloud_preprocessor` no dependía de lanelet2/agnocast. Menos peso, sigue
   siendo código Autoware real (nombres de paquete pre-refactor).
3. **Portar solo el algoritmo `scan_ground_filter`** (ring-based, ~1 archivo) a un
   nodo propio en `navegacion_gps`, sin compilar Autoware. Cero dependency-hell,
   pero es "inspirado en Autoware", no el binario probado.

**Decisión (2026-06-13): Opción 1 — vendor completo v0.51.0.**

## 7. Vendor completo — resolución

`.repos` final fijado a `build_depends_stable.repos` de universe 0.51.0 (no al
`autoware.repos` maestro, que tenía versiones distintas: `autoware_utils` 1.7.1
vs 1.8.0, `autoware_msgs` 1.12.0 vs 1.13.0). `agnocast` es de **tier4**
(`backport-jazzy-support-v2.1.2`), no autowarefoundation. Los paquetes que
"faltaban" (`autoware_point_types`, `autoware_lanelet2_utils`,
`autoware_agnocast_wrapper`) viven en **`autoware_core/common`**.

Repos vendorizados en `src/autoware_deps/` (~640 MB, shallow): `autoware_cmake`
1.2.0, `autoware_utils` 1.7.1, `autoware_msgs` 1.12.0, `autoware_internal_msgs`
1.12.1, `autoware_lanelet2_extension` 1.0.0, `autoware_core` 1.8.0,
`managed_transform_buffer` 0.2.0, `agnocast` (tier4), `autoware_universe` 0.51.0.

**Subconjunto real a compilar: 36 paquetes** (`colcon ... --packages-up-to
autoware_ground_segmentation`) — sin tensorrt/cuda. Incluye toda la familia
`autoware_utils_*`, los `*_msgs`, `lanelet2_extension/utils`,
`pointcloud_preprocessor`, `pcl_extensions`, `vehicle_info_utils`,
`agnocast_wrapper`, `managed_transform_buffer`.

Deps de sistema (apt, vía rosdep): `ros-humble-point-cloud-msg-wrapper`,
`ros-humble-lanelet2-*`, `libcgal-dev`, `ros-humble-sophus`, `libpugixml-dev`,
`librange-v3-dev`, `ros-humble-autoware-adapi-v1-msgs`, etc. → para producción
hay que sumarlas al `Dockerfile`.

### Build (POC)

Contenedor aparte `salus_aw_build` (imagen `ros2-humble-perception-ws-salus`,
mounts a ROS2_SALUS, NO molesta al `ros2_salus` de 2antenas):

```
colcon build --packages-up-to autoware_ground_segmentation --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 8. Pivot a Opción 3 (port del algoritmo) — 2026-06-13

El build del vendor completo (Opción 1) se intentó con blindaje de memoria para
no tildar la PC (14 GB RAM): contenedor con `--memory=9g --memory-swap=11g`,
`--parallel-workers 2` y `MAKEFLAGS="-j2"`. **Eso resolvió el tilde** (32/36
paquetes compilaron sin que la RAM del host bajara de ~9.6 GB libres), pero el
build no cierra: `autoware_lanelet2_extension` linkea mal (undefined refs a
`lanelet::routing`, `GeographicLib::MGRS`). Y todo lanelet2 entra **solo** porque
`autoware_pointcloud_preprocessor` empaqueta un `vector_map_filter` que no usamos
— la cascada de §6 confirmada.

**Decisión: Opción 3.** Se portó el algoritmo *non-grid* (ring-based) de
`ScanGroundFilterComponent::classifyPointCloud` (universe 0.51.0, Apache-2.0) a un
nodo Python propio. Cero compilación de Autoware, cero lanelet2.

- Nodo: `navegacion_gps/scan_ground_filter.py`
  (clases `ScanGroundSegmenter` puro + `ScanGroundFilterNode`).
- Entry point: `scan_ground_filter = navegacion_gps.scan_ground_filter:main`.
- Config: `config/scan_ground_filter.param.yaml` (mismos defaults RS16 que el
  param de Autoware; `wheel_base=0.90` para el punto de suelo virtual).
- Interfaz: `/scan_3d` (PointCloud2) → `/scan_3d/no_ground` (PointCloud2). La nube
  se transforma a `target_frame` (`base_footprint`) con TF antes de clasificar,
  porque el RS16 va montado con pitch (el algoritmo asume z = altura).
- Tests: `test/test_scan_ground_filter.py` (10 casos del núcleo, verde) + smoke
  test del nodo completo (PointCloud2 → no_ground) OK.
- Verificación independiente (Codex): fidelidad confirmada línea por línea vs
  `node.cpp` (convertPointcloud + classifyPointCloud), sin divergencias de
  comportamiento; 10/10 tests.
- Rendimiento: el lazo de clasificación corre sobre floats Python puros (numpy
  solo para binning/orden vía `lexsort`). Benchmark RS16 sintético (28.8k pts):
  **~25 ms media / ~29 ms p95** (antes ~110/131 ms con numpy escalar). Margen
  cómodo bajo el presupuesto de 100 ms a 10 Hz. Cuidado: distancia entre puntos
  es 3D (incluye dy), igual que `calc_distance3d` de Autoware.

Wiring (hecho): en `launch/sim_v2_base.launch.py`, detrás de
`enable_scan_ground_filter` (default `False`). Con el flag en `True` se intercala
el nodo entre `/scan_3d` y `pointcloud_to_laserscan` (que pasa a consumir
`/scan_3d/no_ground`) y se baja el `min_height` del proyector vía arg
`scan_ground_min_height` (default `0.10`). Implementado con la `OpaqueFunction`
`_build_lidar_pipeline`. Verificado por introspección: off → 1 nodo,
`cloud_in:=/scan_3d`; on → 2 nodos, `cloud_in:=/scan_3d/no_ground`.

```
ros2 launch navegacion_gps sim_v2_base.launch.py enable_scan_ground_filter:=True
```

## 9. Escenario de validación (rampa) — armado

Harness A/B para medir el efecto del filtro sobre `slope_lidar.world` (rampa 10°
con obstáculos encima) con el URDF plano, replicando la metodología de Etapa A
(`docs/investigaciones/plan-correccion-puntos-fantasma.md`).

- Forwarding: `sim_local_v2.launch.py` ahora reenvía `world`, `custom_urdf`,
  `enable_scan_ground_filter` y `scan_ground_min_height` a `sim_v2_base`
  (aditivo; defaults = comportamiento actual).
- Launch: `launch/validate_scan_ground.launch.py` levanta el stack local completo
  (Nav2 + collision_monitor) sobre la rampa + URDF plano y agrega el nodo medidor.
  Fuerza Gazebo server-only (`gz_args:="-s -r "`) para poder correr en CI/terminal
  headless.
- Nodo medidor: `navegacion_gps/scan_ground_validation.py` (`scan_ground_validation`).
  KPIs durante `duration_s` → JSON:
  - FP: celdas ocupadas (≥100) en el costmap local, acumuladas + máx/frame. El
    nodo toma el full grid de `/local_costmap/costmap` y aplica
    `/local_costmap/costmap_updates`, porque `always_send_full_costmap:false`
    publica updates parciales entre grids completos.
  - Eventos de freno falsos: `/cmd_vel` vs `/cmd_vel_safe` (flanco de frenado con
    avance comandado), separando slowdown de stop.
  - Lógica de conteo aislada en `ValidationAccumulator` + tests
    `test/test_scan_ground_validation.py` (7 verde).
  - El launch de validación fuerza `use_keepout:=False` para que el conteo de
    celdas letales mida rampa/LiDAR y no la máscara de zonas prohibidas.
- Runner: `scripts/run_scan_ground_validation.sh [DURATION] [OUTDIR]` corre
  baseline (off) y filtered (on) y compara.

```
ros2 launch navegacion_gps validate_scan_ground.launch.py \
  enable_scan_ground_filter:=True label:=filtered output_path:=/tmp/filtered.json
```

Nota: los FP aparecen aun con el robot quieto (el piso de la rampa entra como
obstáculo) — ahí está el efecto dominante del filtro. Los eventos de freno solo se
cuentan si se comanda avance: mandar una meta con `nav_command_server` durante la
ventana para medirlos.

### Resultado A/B (Codex, sim headless sobre la rampa)

| métrica | baseline (off) | filtered (on) | cambio |
|---|---:|---:|---:|
| fp_accumulated | 16 654 | 791 | **−95.3 %** |
| fp_max_frame | 176 | 9 | −94.9 % |
| fp_mean_per_frame | 169.94 | 8.07 | −95.3 % |
| FN a nivel `/scan_3d/no_ground` | n/a | **0/2** | el filtro no se come obstáculos |

El filtro baja FP drásticamente sin perder obstáculos reales en la nube filtrada.
Fixes de Codex para poder correrlo: reconstrucción del costmap desde
`/local_costmap/costmap_updates` (por `always_send_full_costmap:false`, +dep
`map_msgs`), publicación robusta de PointCloud2 (padding/dtype de Gazebo),
filtrado de infinitos pre-TF, Gazebo headless + keepout off en el launch de
validación. 17 tests verde.

### Recorte 2D `max_height` (resuelto)

El A/B mostró FN 1/2 en el `/scan` 2D final: el `slope_obstacle_right` (caja 0.8 m
apoyada en x=10.2, arriba de la pendiente 10°) queda en world-z ≈ [1.45, 2.25],
por encima del `max_height: 1.60` del `pointcloud_to_laserscan` → se recortaba en
la proyección 3D→2D, **no** por el filtro. Como el suelo ya se quita en 3D, subir
`max_height` es seguro (no reintroduce piso fantasma). Se agregó el arg
`scan_ground_max_height` (default **2.50**, espejo de `scan_ground_min_height`),
que solo aplica con el filtro on. Reentra en la ventana a ambos obstáculos.

Pendiente: re-correr el A/B confirmando FN 0/2 en el `/scan` 2D con
`scan_ground_max_height`; si pasa, replicar el wiring en
`real_global_v2.launch.py` + Dockerfile (tunear min/max a la geometría real).

> **Limpieza (2026-06-13):** el vendor `src/autoware_deps/` (~624 MB), los configs
> `config/autoware_*.param.yaml`, los logs `aw_*.log` y la imagen Docker `:aw`
> fueron **eliminados**: solo eran referencia para portar el algoritmo, que ya vive
> en `navegacion_gps/scan_ground_filter.py`. Este documento queda como historia.
</content>
