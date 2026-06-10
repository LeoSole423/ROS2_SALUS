# Plan V2 De Percepcion LiDAR 3D

Estado: futuro / no implementar de momento
Alcance: arquitectura definitiva futura para una percepcion LiDAR 3D robusta con RS16 en navegacion Global V2
Fuente de verdad: investigacion tecnica de pipelines 3D tipo Autoware/Nav2/PCL, `docs/lidar-perception.md`, pruebas reales con RS16 y codigo actual del checkout

## Estado Actual De Este Plan
Este documento NO es el plan de trabajo inmediato.

Queda como direccion tecnica futura si las mejoras conservadoras sobre el pipeline legacy no alcanzan.

El plan inmediato actual es:

```text
docs/lidar-noise-reduction-plan.md
```

Motivo:

- antes del filtro RANSAC, el LiDAR funcionaba razonablemente bien;
- el problema inmediato parece ser ruido puntual y obstaculos fantasma;
- un rediseño 3D completo requiere mas tiempo, bags y validacion;
- conviene probar primero una solucion V1.5 mas pequeña y reversible.

## Objetivo
Rehacer la percepcion LiDAR para que el robot use el RS16 como sensor 3D real, no como un `LaserScan` 2D derivado de forma fragil.

El problema a resolver es concreto:

- eliminar suelo en pendientes transitables;
- eliminar puntos y clusters fantasmas;
- conservar obstaculos reales;
- evitar que ruido puntual genere rodeos gigantes en Nav2;
- mantener una ruta de rollback segura a `/scan`.

## Decision Tecnica
No seguir evolucionando el filtro V1 como solucion final.

El filtro V1:

```text
/scan_3d
-> ROI fija
-> RANSAC de plano unico
-> altura sobre plano
-> voxel density
-> /scan_filtered
```

queda como experimento. Sirvio para aprender, pero no es la arquitectura final para robot real.

La arquitectura V2 debe parecerse mas a pipelines serios de LiDAR 3D, como los usados conceptualmente en Autoware:

```text
/scan_3d
-> transformacion a base_footprint
-> crop / ROI
-> self-filter del robot
-> voxel downsample
-> outlier filter
-> segmentacion de suelo por rayos/anillos
-> nube non-ground
-> clustering/densidad de obstaculos
-> costmap local temporal 3D
-> salida compacta para Nav2/collision monitor
```

## Que Tomamos De Autoware
Autoware separa percepcion en bloques. Esa es la leccion principal.

No se debe mezclar todo en un unico nodo opaco que termina publicando un `LaserScan` sin explicar que paso. La V2 debe separar y publicar estados intermedios.

Practicas que vamos a adoptar:

- preprocesamiento de nube antes de segmentar;
- transformaciones y frames validados antes de clasificar;
- filtros de outliers antes de generar obstaculos;
- segmentacion de suelo especifica, no un plano global unico;
- debug con nubes separadas: ground, non-ground, obstacles;
- criterio temporal para no reaccionar a ruido de un solo frame;
- menor peso de obstaculos dinamicos en global costmap.

Practicas que no vamos a adoptar de entrada:

- migrar el stack completo a Autoware;
- traer modulos grandes sin medir CPU/RAM en Raspberry Pi 5;
- cambiar el modelo de control/Nav2 por uno automotriz completo;
- reemplazar Nav2 mientras el problema sea percepcion.

## Contrato Final Esperado
La V2 debe exponer estas entradas:

```text
/scan_3d              sensor_msgs/PointCloud2
/tf                  tf2
/tf_static           tf2
/imu/data            sensor_msgs/Imu, opcional para diagnostico
/controller/drive_telemetry  opcional para deskew/diagnostico futuro
```

Debe publicar estas salidas:

```text
/lidar/roi_cloud
/lidar/self_filtered_cloud
/lidar/ground_cloud
/lidar/non_ground_cloud
/lidar/obstacles_cloud
/lidar/diagnostics
/scan_filtered_v2
```

`/scan_filtered_v2` se mantiene como salida de compatibilidad para Nav2/collision monitor. La nube principal de debug y decision debe ser 3D.

El launch debe mantener rollback:

```bash
enable_lidar_obstacle_filter:=False
```

y agregar un selector explicito:

```bash
lidar_filter_version:=legacy|v1|v2
```

En robot real, mientras V2 no este validado:

```bash
lidar_filter_version:=legacy
```

## Etapa 0: Congelar V1 Y Proteger Robot Real
Objetivo: evitar que el filtro fallido vuelva a ser tratado como solucion final.

Tareas:

1. Cambiar el default real a fallback legacy si todavia no esta hecho:

```bash
enable_lidar_obstacle_filter:=False
```

2. Mantener V1 disponible solo bajo pedido explicito:

```bash
enable_lidar_obstacle_filter:=True
lidar_filter_version:=v1
```

3. Agregar texto en launch/docs:

```text
lidar_obstacle_filter V1 es experimental y no esta validado para patrullaje real.
```

4. Confirmar que Nav2 puede volver a consumir `/scan` sin tocar costmaps manualmente.

Entregables:

- launch real con rollback claro;
- healthcheck que indique si Nav2 esta usando `/scan`, `/scan_filtered` o `/scan_filtered_v2`;
- documentacion de operacion.

Criterio de salida:

- el robot real puede arrancar con `/scan` legacy;
- se puede activar V1 solo con launch arg explicito;
- no hay cambios escondidos en costmaps para compensar V1.

## Etapa 1: Dataset Real Obligatorio
Objetivo: dejar de tunear con intuicion.

Antes de implementar V2, grabar bags reales. Cada bag debe incluir:

```text
/scan_3d
/scan
/scan_filtered
/obstacles_cloud
/imu/data
/tf
/tf_static
/odom
/odometry/local
/odometry/global
/local_costmap/costmap
/global_costmap/costmap
/cmd_vel
/cmd_vel_safe
/cmd_vel_final
/controller/drive_telemetry
```

Escenarios minimos:

1. `plano_limpio_detenido`: robot quieto en piso plano sin obstaculos.
2. `plano_limpio_lento`: robot moviendose lento en piso plano.
3. `pendiente_transitable`: robot mirando hacia arriba, abajo y cruzado en pendiente.
4. `pasto_tierra`: superficie irregular pero transitable.
5. `laterales_libres`: sin obstaculos cerca, revisar fantasmas a izquierda/derecha.
6. `pared_auto_cono`: obstaculos reales a 3 m, 6 m y 10 m.
7. `borde_cuneta`: borde lateral que no debe confundirse con pared completa.

Cada bag debe tener al menos:

- 30 s detenido;
- 60 s en movimiento lento;
- una captura RViz o nota indicando que se veia en el entorno.

Entregables:

- carpeta de bags nombrada por fecha y escenario;
- tabla con escenario, ubicacion, superficie, velocidad y observaciones;
- al menos un caso donde aparezcan obstaculos fantasma reales.

Criterio de salida:

- minimo 7 bags completos;
- al menos 1 bag reproduce el fallo de obstaculos fantasma;
- al menos 1 bag reproduce pendiente transitable.

## Etapa 2: Validacion Geometrica Y Temporal
Objetivo: corregir errores de frames antes de tocar algoritmos.

Tareas:

1. Medir fisicamente `lidar_link -> base_footprint`:

```text
x, y, z, roll, pitch, yaw
```

2. Confirmar altura real del RS16 respecto al piso.
3. Confirmar pitch/roll de montaje del RS16 con el robot nivelado.
4. Verificar que el frame `lidar_link` del driver coincide con el montaje real.
5. Confirmar si `/imu/data` esta en ENU o si conserva conversiones MAVLink/NED.
6. Comparar roll/pitch de IMU contra nivel fisico.
7. Medir latencia aproximada entre `/scan_3d` e `/imu/data`.
8. Confirmar que no entran partes del robot en `/scan_3d`.

Checks concretos:

```bash
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 topic echo --once /imu/data
ros2 topic hz /scan_3d
ros2 topic hz /imu/data
```

Entregables:

- tabla con transform medida y transform configurada;
- decision escrita: usar o no usar IMU en la primera V2;
- RViz con nube cruda alineada al robot;
- lista de partes del robot que deben filtrarse con self-filter.

Criterio de salida:

- error de altura del LiDAR menor a 5 cm;
- error de yaw/pitch/roll de montaje menor a 2 grados;
- convencion de IMU documentada;
- self-filter definido por dimensiones reales del robot.

## Etapa 3: Banco Offline De Percepcion
Objetivo: probar filtros contra bags sin mover el robot.

Implementar un runner offline:

```bash
ros2 launch navegacion_gps lidar_v2_replay.launch.py \
  bag:=/path/al/bag \
  profile:=rs16_v2_default
```

El runner debe:

- reproducir `/scan_3d`, `/tf`, `/imu/data`;
- ejecutar V1 y V2 en paralelo si es posible;
- publicar nubes debug;
- guardar metricas por frame;
- permitir cambiar parametros sin recompilar.

Metricas obligatorias:

```text
input_points
roi_points
self_filtered_points
ground_points
non_ground_points
obstacle_points
obstacle_clusters
clusters_near_left
clusters_near_right
nearest_obstacle_front_m
nearest_obstacle_left_m
nearest_obstacle_right_m
processing_ms
```

Entregables:

- launch de replay;
- CSV/JSON de metricas;
- script para comparar perfiles;
- RViz config de debug V2.

Criterio de salida:

- se puede reproducir cada bag sin robot real;
- se puede comparar V1 vs V2 en el mismo bag;
- el procesamiento reporta `processing_ms`;
- no se avanza a pruebas reales sin pasar este banco.

## Etapa 4: Nodo V2 De Preprocesamiento 3D
Objetivo: sanear la nube antes de segmentar suelo.

Implementar `lidar_obstacle_filter_v2`.

Lenguaje:

- iniciar en Python + NumPy solo si la tasa real alcanza;
- migrar a C++/PCL si `processing_ms` supera el presupuesto.

Presupuesto inicial:

```text
max_processing_ms: 80
target_rate_hz: 10
```

Pipeline obligatorio:

1. Transformar nube a `base_footprint`.
2. Aplicar crop box:

```text
x: -1.0 .. 20.0
y: -8.0 .. 8.0
z: -2.0 .. 3.0
```

3. Self-filter del robot:

```text
x: -0.6 .. 1.4
y: -0.7 .. 0.7
z: -0.3 .. 1.8
```

4. Voxel downsample:

```text
voxel_x: 0.12
voxel_y: 0.12
voxel_z: 0.08
```

5. Outlier filter:

```text
radius_m: 0.35
min_neighbors: 2
```

6. Publicar:

```text
/lidar/roi_cloud
/lidar/self_filtered_cloud
```

Entregables:

- nodo V2 con parametros;
- tests unitarios de crop/self-filter/outlier;
- diagnostico de tiempo por frame.

Criterio de salida:

- en bag `plano_limpio_detenido`, no quedan puntos del robot;
- en bag `laterales_libres`, los fantasmas aislados bajan al menos 70%;
- tasa efectiva minima 8 Hz en Raspberry Pi 5 o decision de migrar a C++.

## Etapa 5: Segmentacion De Suelo Por Rayos/Anillos
Objetivo: reemplazar RANSAC de plano unico.

Implementar ground segmentation basada en geometria local.

Metodo:

1. Convertir puntos a coordenadas polares:

```text
range, azimuth, z
```

2. Agrupar por rayos de azimut:

```text
azimuth_bin_deg: 1.0
```

3. Ordenar cada rayo por distancia.
4. Clasificar suelo por continuidad local:

```text
max_local_slope_deg: 10
max_global_slope_deg: 18
max_height_jump_m: 0.18
ground_seed_z_tolerance_m: 0.20
```

5. Un punto es obstaculo si:

```text
height_above_local_ground_m >= 0.22
height_above_local_ground_m <= 1.60
```

6. Publicar:

```text
/lidar/ground_cloud
/lidar/non_ground_cloud
```

Reglas obligatorias:

- no usar un unico plano para toda la ROI;
- no depender de IMU para clasificar suelo en la primera version aceptable;
- usar IMU solo como diagnostico o compensacion opcional despues de validar frames.

Entregables:

- segmentador por rayos/anillos;
- tests con suelo plano, pendiente y obstaculo vertical;
- visualizacion ground/non-ground en RViz.

Criterio de salida:

- `plano_limpio_detenido`: obstaculos falsos persistentes = 0;
- `pendiente_transitable`: suelo no aparece como obstaculo en mas del 95% de frames;
- `pared_auto_cono`: obstaculos se conservan en mas del 90% de frames.

## Etapa 6: Obstaculos, Clusters Y Persistencia Temporal
Objetivo: impedir que un frame ruidoso afecte la navegacion.

Pipeline:

1. Tomar `/lidar/non_ground_cloud`.
2. Crear clusters por voxel/grid 2.5D.
3. Rechazar clusters chicos:

```text
min_cluster_points: 4
min_cluster_width_m: 0.20
min_cluster_persistence_frames: 2
```

4. Decaimiento temporal:

```text
obstacle_decay_s: 0.5
max_track_age_s: 1.0
```

5. Publicar:

```text
/lidar/obstacles_cloud
/scan_filtered_v2
/lidar/diagnostics
```

Reglas:

- un cluster de un solo frame no debe llegar al global planner;
- collision monitor puede recibir informacion mas conservadora y cercana;
- global costmap no debe reaccionar a ruido lateral de baja persistencia.

Entregables:

- tracker temporal simple por celdas/cluster;
- diagnostico de cantidad de clusters;
- test de punto fantasma de 1 frame;
- test de obstaculo real persistente.

Criterio de salida:

- fantasmas de 1 frame no aparecen en `/scan_filtered_v2`;
- obstaculo real visible por 2 frames aparece en `/scan_filtered_v2`;
- procesamiento total sigue bajo 80 ms o se migra a C++.

## Etapa 7: Integracion Nav2 Sin Sobreactuar Global
Objetivo: que Nav2 use obstaculos confiables sin generar rodeos gigantes.

Cambios a implementar:

1. Local costmap:

```text
fuente principal: /lidar/obstacles_cloud o /scan_filtered_v2
persistencia baja
raytracing agresivo
```

2. Collision monitor:

```text
fuente cercana y conservadora
rango frontal limitado
prioridad seguridad
```

3. Global costmap:

```text
mantener rolling window realista
no agrandar width/height para esconder problemas
evaluar reducir influencia de obstaculos dinamicos LiDAR
```

Decision concreta a evaluar en bag:

- Opcion A: global costmap consume `/scan_filtered_v2` con rango y persistencia bajos.
- Opcion B: global costmap no consume LiDAR dinamico; usa keepout/static y el local maneja dinamicos.

La opcion que genere menos rodeos falsos sin comprometer seguridad queda como default.

Entregables:

- dos perfiles Nav2 comparables: `global_lidar_dynamic` y `local_lidar_only`;
- replay con ambos perfiles sobre los mismos bags;
- tabla de resultados.

Criterio de salida:

- un fantasma no genera plan global con rodeo grande;
- un auto/pared real si modifica la trayectoria;
- collision monitor sigue frenando ante obstaculo cercano.

## Etapa 8: Validacion En Robot Real
Objetivo: aprobar V2 para patrullaje.

Orden de pruebas:

1. Robot detenido, sin goals.
2. Robot con teleop lento.
3. Nav2 goal corto en plano.
4. Nav2 goal corto en pendiente.
5. Ruta con waypoints cercanos.
6. Ruta con obstaculos reales.

En cada prueba grabar bag y guardar:

- video o captura RViz;
- logs Nav2;
- metricas V2;
- resultado: pasa/falla.

Criterios de aceptacion final:

- piso plano: 0 obstaculos persistentes falsos;
- pendiente transitable: no bloquea ruta;
- laterales libres: no aparecen fantasmas persistentes junto al robot;
- obstaculo real a 3-10 m: detectado estable;
- punto/cluster fantasma de 1 frame: no altera global planner;
- CPU Raspberry Pi 5 dentro de margen operativo;
- rollback a `/scan` sigue funcionando.

## Orden De Implementacion
No saltear etapas.

Orden obligatorio:

1. Etapa 0: rollback y seguridad.
2. Etapa 1: bags reales.
3. Etapa 2: geometria y tiempos.
4. Etapa 3: replay offline.
5. Etapa 4: preprocesamiento.
6. Etapa 5: suelo por rayos/anillos.
7. Etapa 6: clusters y persistencia.
8. Etapa 7: Nav2.
9. Etapa 8: robot real.

La primera etapa de codigo grande debe ser el banco offline, no el filtro nuevo. Sin bags y replay, cualquier tuning vuelve a ser prueba y error.

## Resultado Esperado
Al terminar, SALUS debe tener una percepcion LiDAR 3D modular y medible:

```text
RS16
-> nube 3D saneada
-> suelo separado
-> obstaculos persistentes
-> costmap local/collision robustos
-> global planner menos sensible a fantasmas
```

El exito no es que RViz se vea limpio una vez. El exito es que el mismo filtro pase bags reales, pendiente, terreno irregular y pruebas de robot sin generar obstaculos fantasma persistentes.
