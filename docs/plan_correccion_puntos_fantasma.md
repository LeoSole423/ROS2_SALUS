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

Medir baseline: marcas fantasma en el costmap local y eventos de
`/collision_monitor_state` sin obstáculo dentro del polígono. Anotar números.

**Limitación a documentar**: el modelo de Gazebo es rígido (sin suspensión), así
que el pitch transitorio por frenada no se reproduce físicamente; la rampa y el
URDF inclinado cubren la misma geometría en régimen. El componente *transitorio*
(salto de pitch entre frames) se cubre con tests sintéticos en A4.

**Salida**: bags de ambos escenarios + número baseline de FP/FN.

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

**Criterio de salida**: en `slope_lidar.world`, cero marcas de suelo y los dos
obstáculos de la rampa detectados, en ambos URDF.

### A3. Endurecer Nav2 (en sim, un cambio por corrida)

Sobre `nav2_global_v2_real_rolling_params.yaml` y equivalentes sim/wifi/local:

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
