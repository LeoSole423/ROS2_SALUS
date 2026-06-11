# Plan de corrección — puntos fantasma / frenados falsos del LiDAR

Basado en los dos informes (PDF *"Eliminación y filtrado de puntos fantasmas..."*
y *"Resumen Ejecutivo"*) cruzados con el estado real del repo
(ver `docs/lidar_puntos_fantasma_datos_proyecto.md`).

**Contexto confirmado:**
- El RS16 está montado **plano** (la extrínseca del URDF default es correcta;
  no hay bug de TF).
- El problema es **cabeceo dinámico**: al acelerar/frenar o pasar irregularidades,
  el chasis (y el LiDAR con él) pitchea algunos grados, el haz barre el piso y
  esos retornos entran como obstáculos. Es exactamente el escenario "soporte
  inclinable" de los informes, versión dinámica.
- **No hay robot disponible ahora** → el plan se divide en una Etapa A (todo
  sin hardware: simulación + tests + replays) y una Etapa B (una sola sesión de
  campo al final).

KPI: frenados/slowdowns sin obstáculo real (por corrida en sim, por 100 m en
campo), falsos negativos con obstáculo real, latencia nube→scan filtrado.

**Infraestructura que ya existe para esto** (commit `aba3e49`, rama
`feature/lidar-3d-ground-filter`):
- `worlds/slope_lidar.world`: rampa de 10° a x=8 con obstáculos reales encima
  (`slope_obstacle_left/right`) — reproduce la geometría del cabeceo y permite
  medir FP (suelo) y FN (obstáculos sobre la rampa) a la vez.
- `models/cuatri_real_v2.urdf`: LiDAR con pitch fijo de 10° — "robot cabeceado
  congelado", el peor caso estático.
- `lidar_obstacle_filter.py` (rama 3D completa, apagada por default) +
  `test_lidar_obstacle_filter.py` como patrón de tests.
- `pointcloud_to_laserscan_tilted_lidar_sim.yaml` como config de comparación.

---

## Etapa A — Sin hardware (ahora)

### A1. Reproducir el fantasma en Gazebo y fijar el baseline

Dos escenarios complementarios:

```bash
# Escenario 1: cabeceo geométrico real (rampa de 10°)
ros2 launch navegacion_gps sim_global_v2.launch.py \
  world:=$(ros2 pkg prefix navegacion_gps)/share/navegacion_gps/worlds/slope_lidar.world

# Escenario 2: peor caso estático (LiDAR pitcheado 10° en mundo plano)
ros2 launch navegacion_gps sim_global_v2.launch.py \
  custom_urdf:=$(ros2 pkg prefix navegacion_gps)/share/navegacion_gps/models/cuatri_real_v2.urdf
```

Grabar bags de sim (reemplazan a los bags reales para todo el tuning de la etapa):

```bash
ros2 bag record /scan_3d /scan /scan_clean /imu/data /odometry/local \
  /cmd_vel /cmd_vel_safe /collision_monitor_state /tf /tf_static
```

> **Estado: MEDIDO** (jun 2026). El host no tiene ROS; se compilo en Docker
> (`ros2-humble-perception-ws-salus`) con `build/install/log` bajo
> `/tmp/salus_lidar_validation`. Gazebo se ejecuto headless inyectando `-s` en
> `world:=...` porque la GUI falla sin OpenGL. Bags:
> `/tmp/salus_lidar_validation/bags/a1_slope_baseline` y
> `/tmp/salus_lidar_validation/bags/a1_tilted_baseline`.

Medir baseline: marcas fantasma en el costmap local y eventos de
`/collision_monitor_state` sin obstáculo dentro del polígono. En Humble el
`state_topic` quedo sin publisher activo (solo subscribers); se midio el efecto
real comparando `/cmd_vel` contra `/cmd_vel_safe`.

**Limitación a documentar**: el modelo de Gazebo es rígido (sin suspensión), así
que el pitch transitorio por frenada no se reproduce físicamente; la rampa y el
URDF inclinado cubren la misma geometría en régimen. El componente *transitorio*
(salto de pitch entre frames) se cubre con tests sintéticos en A4.

Resultados A1:

| Escenario | FP costmap local acumulado | FP max/frame | FN obstaculos reales | Eventos falsos collision_monitor |
|---|---:|---:|---:|---:|
| `slope_lidar.world`, URDF plano | 174,946 celdas ocupadas | 8,419 | 0/2 perdidos (ambos detectados) | 5 slowdown + 4 stop |
| `vacio.world`, `cuatri_real_v2.urdf` | 0 | 0 | N/A | 0 |

Hallazgo: el escenario de URDF v2 no reproduce fantasmas en este launch porque
el TF publicado coincide con el LiDAR inclinado; los retornos de suelo quedan por
debajo de la banda de obstaculo. La rampa de 10 grados si reproduce el problema.

**Salida**: bags de ambos escenarios + baseline medido.

### A2. Activar la rama 3D y tunear el RANSAC (en sim)

```bash
ros2 launch navegacion_gps sim_global_v2.launch.py \
  world:=.../slope_lidar.world enable_lidar_obstacle_filter:=True
```

Con eso Nav2 y collision_monitor v2 consumen `/scan_filtered` (la selección de
tópico ya está en `real_global_v2.launch.py:201` y su análogo sim).

- Barrer `ground_distance_threshold`: **0.05 / 0.08 / 0.12 / 0.18** (actual).
  Criterio: el suelo de la rampa no marca, los `slope_obstacle_*` sí.
- Si pasan motas: `min_voxel_points: 3 → 5`.
- Medir latencia del nodo (Python+NumPy a 10 Hz): si supera ~50 ms por frame,
  recortar ROI o decimar la nube antes del RANSAC.

> **Estado: MEDIDO** (jun 2026). Barrido offline sobre el bag A1 para aislar
> RANSAC+persistencia, y corrida live end-to-end con
> `enable_lidar_obstacle_filter:=True` y `ground_distance_threshold:=0.05`.
> Bag live: `/tmp/salus_lidar_validation/bags/a2_slope_filter_005`.

Barrido `slope_lidar.world` con gate deshabilitado solo para medir RANSAC:

| `ground_distance_threshold` | Beams fantasma acumulados | Max/frame | Deteccion `left` | Deteccion `right` | Latencia p95 offline |
|---:|---:|---:|---:|---:|---:|
| 0.05 | 77 | 39 | 159/175 frames | 165/196 frames | 4.6 ms |
| 0.08 | 1,380 | 77 | 143/175 | 159/196 | 4.7 ms |
| 0.12 | 1,057 | 47 | 138/175 | 163/196 | 5.6 ms |
| 0.18 | 601 | 48 | 139/175 | 151/196 | 6.1 ms |

Con gate default activo (7 grados), la rampa de 10 grados bloquea la mayoria de
los frames: 308/373 en replay offline y 323/374 en la corrida live. En el bag
live A2, aun asi los dos obstaculos reales se mantuvieron visibles en el costmap
(`left` 38/39 frames, `right` 41/42 frames), los FP bajaron a 194 celdas
acumuladas (max 10/frame), y no hubo slow/stop falsos. Latencia real grabada
`/scan_3d` -> `/scan_filtered`: media 6.6 ms, p95 9.6 ms, max 14.0 ms.

En `vacio.world` con `cuatri_real_v2.urdf`, todos los thresholds dieron 0 beams
fantasma y 0 bloqueos del gate; no hay obstaculos reales en ese mundo para FN.

**Recomendacion A2**: usar `ground_distance_threshold: 0.05` en sim para el
siguiente ciclo. Fue el mejor balance del barrido: minimo suelo residual y mayor
conteo de deteccion de ambos obstaculos. Queda como issue de campo revisar el
`tilt_gate_max_offset_deg` si SALUS debe aceptar rampas sostenidas de ~10 grados;
el valor de 7 grados esta bien para transitorios de cabeceo, pero bloquea rampas
largas en sim.

### A3. Endurecer Nav2 (en sim, un cambio por corrida)

Sobre `nav2_global_v2_sim_rolling_params.yaml`:

| Parámetro | Hoy | Cambio | Fuente |
|---|---|---|---|
| `plugins` local costmap | `[voxel_layer, inflation_layer]` | insertar `denoise_layer` (`nav2_costmap_2d::DenoiseLayer`, `minimal_group_size: 2`) antes de inflation | PDF 1 |
| `mark_threshold` (voxel) | `0` | `2` | PDF 1 (2–3) |
| `observation_persistence` (local) | `0.6` | `0.0` (si parpadea, `0.05`) | PDF 1 |
| `min_obstacle_height` (scan_marking) | `0.0` | `0.10` | corta retornos a ras residuales |

Cuidado con la interacción `mark_threshold` ↔ obstáculos delgados (postes): si
aparecen FN, volver a 1 antes de tocar lo demás.

**Criterio de salida**: replay de los bags A1 — un frame malo no deja huella
de más de un ciclo en el costmap, sin FN nuevos.

> **Estado: IMPLEMENTADO/MEDIDO** (jun 2026). Se hizo replay del bag A1 de rampa
> contra `nav_global_v2.launch.py` con YAMLs temporales, un cambio por corrida.
> Bags de salida bajo `/tmp/salus_lidar_validation/replay_bags/`.

Resultados A3 (`/scan_clean` baseline, local costmap):

| Configuracion | FP acumulado | FP max/frame | FN | Decision |
|---|---:|---:|---:|---|
| Baseline | 168,294 | 8,067 | 0/2 perdidos | referencia |
| + `denoise_layer`, `minimal_group_size: 2` | 142,471 | 7,356 | 0/2 perdidos | aplicar |
| `mark_threshold: 2` | 0 | 0 | 2/2 perdidos | descartar |
| `observation_persistence: 0.0` | 138,133 | 4,941 | 0/2 perdidos | aplicar |
| `scan_marking.min_obstacle_height: 0.10` | 0 | 0 | 2/2 perdidos | descartar |
| `denoise_layer` + `observation_persistence: 0.0` | 123,794 | 4,691 | 0/2 perdidos | aplicado |

Los eventos de `/cmd_vel_safe` no cambian con A3 porque `collision_monitor`
consume el scan efectivo directamente, no el costmap. A3 endurece Nav2/costmap;
la reduccion de frenados falsos viene de A2 cuando el scan efectivo pasa a
`/scan_filtered`.

### A4. Gate de inclinación + persistencia temporal (desarrollo con tests, cero hardware)

> **Estado: IMPLEMENTADO** (jun 2026). `TiltGate` y `VoxelPersistenceFilter` en
> `lidar_obstacle_filter.py`, con 10 tests nuevos en
> `test_lidar_obstacle_filter.py` (17/17 pasan en contenedor
> `ros2-humble-perception-ws-salus`). Parámetros nuevos del nodo:
> `tilt_gate_enabled` (True), `tilt_gate_nominal_roll_deg`/`_pitch_deg` (0.0),
> `tilt_gate_max_offset_deg` (7.0), `tilt_gate_max_jump_deg` (3.0),
> `persistence_enabled` (True), `persistence_min_hits` (2),
> `persistence_window` (3). Estado del gate publicado en
> `/lidar_obstacle_filter/tilt_gate_blocked` (Bool). Cuando el gate bloquea,
> el nodo no publica nada (el costmap retiene la última observación válida).
> Semántica del jump: cada transición brusca cuesta exactamente un frame
> rechazado; una actitud nueva estable dentro del envelope se vuelve a aceptar.

Los dos gaps que ningún nodo cubre hoy, en `lidar_obstacle_filter.py`:

1. **Gate de evento de inclinación** (PDF 1): si `|roll|` o
   `|pitch − nominal|` > **7°**, o salto > **3°** entre frames consecutivos del
   IMU → no publicar ese frame (mantener el último válido o scan vacío +
   estado degradado). Nominal parametrizado (0° para el montaje plano).
   Esto es lo que ataja el transitorio de frenada que la sim no reproduce.
2. **Persistencia N-de-M** (PDF 1): voxel ocupado debe repetirse en ≥2 de los
   últimos 3 frames antes de salir en `/scan_filtered`. A 10 Hz agrega ~200 ms
   de latencia de detección (~32 cm a 1.6 m/s) — aceptable contra la stop_zone
   de ~2 m, pero queda documentado.

Validación enteramente sintética, siguiendo el patrón de
`test_lidar_obstacle_filter.py`:
- secuencia de IMU con salto de pitch 0°→5°→0° en 3 frames → el gate descarta
  el frame del medio;
- nube de suelo rotada 5° + obstáculo persistente → el suelo (intermitente
  tras el gate) no pasa la persistencia, el obstáculo sí.

**Criterio de salida**: tests verdes + replay de bags A1 sin regresión.

### A5. Cambiar defaults y dejar todo listo para el campo

- `enable_lidar_obstacle_filter` default `True` en launches v2 (sim y real).
- Actualizar `REAL_GLOBAL_V2_CHECKLIST.md` con los pasos de verificación nuevos
  (tópico efectivo, estado del gate).
- Preparar el script/lista de grabación de bags de campo (escenarios de B1).

---

## Etapa B — Con el robot (una sesión de campo)

**B1. Grabar bags reales** (~15 min): piso plano recto, frenado/aceleración
fuerte (el escenario que la sim no cubre), rampa o junta si hay, persona a
2–5 m, obstáculo bajo (~0.3 m). Mismo `ros2 bag record` de A1.

Antes de grabar, confirmar qué corre el robot (monta
`/home/franco/final/2antenas`, no este repo):
- launch y args efectivos (`enable_lidar_obstacle_filter`?),
- si corre `lidar_brake_guard` (`ros2 node list | grep brake`) — ese nodo
  consume `/scan` crudo y solo existe en ramas `camara_lidear`/`pagina_*`;
  si corre, apuntarlo a `/scan_filtered` (el parámetro `scan_topic` ya existe).

**B2. Replay offline y ajuste fino**: correr los bags reales contra el pipeline
de la Etapa A; ajustar el umbral del gate (el cabeceo real por frenada puede ser
mayor o menor que los 7° supuestos) y el RANSAC con datos verdaderos.

**B3. Sincronizar a 2antenas y validar en campo** repitiendo los escenarios de
B1, comparando KPI contra el baseline. Cerrar actualizando
`docs/lidar_puntos_fantasma_datos_proyecto.md` con los valores finales.

---

## Orden de trabajo sugerido (Etapa A)

A4 (gate + persistencia) puede arrancar ya y en paralelo con A1: es desarrollo
con tests sintéticos puro. El camino crítico es
A1 (baseline) → A2 (RANSAC) → A3 (Nav2) → A5, todo medible con bags de sim.
