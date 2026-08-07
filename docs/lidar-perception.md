# Percepcion LiDAR RS16

Estado: transicion
Alcance: diagnostico, arquitectura actual, fallas observadas y plan de mejora para percepcion LiDAR en navegacion Global V2
Fuente de verdad: `real_global_v2`, `sim_global_v2`, configs Nav2, nodo `lidar_obstacle_filter`, pruebas en simulacion/robot real y referencias tecnicas listadas al final

## Resumen ejecutivo
La percepcion LiDAR es un frente abierto. El cambio reciente de filtrado 3D mejoro algunos falsos positivos en simulacion, pero fallo en pruebas reales: el robot detecto muchos obstaculos fantasma y Nav2 genero caminos grandes para rodearlos. La conclusion operativa es que el filtro actual no debe considerarse validado para patrullaje real.

Plan inmediato recomendado: [docs/lidar-noise-reduction-plan.md](/home/leo/codigo/ROS2_SALUS/docs/lidar-noise-reduction-plan.md).

Plan V2 futuro, no a implementar de momento: [docs/investigaciones/lidar-percepcion-v2-plan.md](/home/leosole/Desktop/AEye/ROS2_SALUS/docs/investigaciones/lidar-percepcion-v2-plan.md).

Para volver al fallback legacy puro durante pruebas reales, usar:

```bash
./tools/launch_real_global_v2_wifi.sh enable_lidar_obstacle_filter:=False enable_scan_noise_filter:=False
```

La siguiente iteracion no deberia ser seguir ajustando el RANSAC actual. Como el pipeline anterior funcionaba razonablemente bien y el problema inmediato era ruido puntual, la recomendacion actual es una V1.5 conservadora: volver al pipeline legacy `/scan`, agregar filtrado simple de `LaserScan`, ajustar persistencia/clearing de costmaps con cambios pequeños y mantener rollback inmediato. El rediseño 3D completo queda como plan futuro si V1.5 no alcanza.

## Hardware y topicos
- LiDAR: RoboSense RS16.
- Driver: `rslidar_sdk`, lanzado desde `sensores/launch/rs16.launch.py`.
- Config principal: `src/sensores/config/rs16.yaml`.
- Frame publicado por el driver: `lidar_link`.
- Nube principal: `/scan_3d`.
- Rango configurado en driver: `min_distance: 0.4`, `max_distance: 20`.
- FOV configurado en driver: `start_angle: 270`, `end_angle: 90`, equivalente al frente del robot.

## Cadena historica
Antes del filtro 3D, la cadena era:

```text
RS16
-> /scan_3d (PointCloud2)
-> pointcloud_to_laserscan
-> /scan (LaserScan)
-> Nav2 costmaps + collision monitor
```

En real se usaba `pointcloud_to_laserscan_real.yaml`:

- `target_frame: base_footprint`
- `min_height: 0.30`
- `max_height: 1.20`
- `range_min: 0.4`
- `range_max: 20.0`
- `use_inf: true`

Esta cadena es simple y pierde informacion 3D, pero era predecible. El recorte por altura evita parte del suelo, aunque no resuelve bien pendientes, pasto, reflejos ni retornos raros.

## Cadena actual con filtro 3D
El cambio reciente agrego esta ruta:

```text
RS16
-> /scan_3d (PointCloud2)
-> lidar_obstacle_filter
-> /obstacles_cloud (PointCloud2 debug)
-> /scan_filtered (LaserScan)
-> Nav2 costmaps + collision monitor
```

Tambien se mantiene el pipeline legacy:

```text
/scan_3d -> pointcloud_to_laserscan -> /scan
```

La ruta V1.5 conservadora actualmente recomendada agrega un filtro simple
despues de `/scan`:

```text
/scan_3d -> pointcloud_to_laserscan -> /scan -> scan_noise_filter -> /scan_clean
```

Launch args relevantes:

- `enable_lidar_obstacle_filter:=True|False`
- `lidar_scan_topic:=/scan_filtered`
- `enable_scan_noise_filter:=True|False`
- `scan_noise_filter_output:=/scan_clean`
- `lidar_filter_roi_x_min`
- `lidar_filter_roi_x_max`
- `lidar_filter_roi_y_min`
- `lidar_filter_roi_y_max`
- `lidar_filter_roi_z_min`
- `lidar_filter_roi_z_max`
- `lidar_filter_ground_distance_threshold`
- `lidar_filter_min_obstacle_height`
- `lidar_filter_max_obstacle_height`
- `lidar_filter_min_voxel_points`

Cuando el filtro RANSAC esta activo, `nav_global_v2.launch.py` reescribe los
topicos de costmap/collision monitor para usar `/scan_filtered`. Cuando RANSAC
esta desactivado y `enable_scan_noise_filter:=True`, usa `/scan_clean`.
Cuando ambos filtros estan desactivados, vuelve a `/scan`.

## Nota Sobre `laser_filters`

`laser_filters` existe en el contenedor Humble y podria usarse con
`scan_to_scan_filter_chain`, pero la V1.5 conservadora usa un nodo propio:

```text
src/navegacion_gps/navegacion_gps/scan_noise_filter.py
```

Motivos practicos:

- el ejemplo instalado de `LaserScanSpeckleFilter` usa una estructura YAML
  antigua que no carga directamente como parametros ROS 2;
- `scan_to_scan_filter_chain` publica por defecto en `/scan_filtered`, topico
  que en SALUS queda reservado para la ruta RANSAC experimental;
- el speckle de `laser_filters` marca puntos eliminados como `NaN`, mientras
  que el filtro actual publica `+inf` para lecturas invalidas/fuera de
  rango/sin vecinos, que encaja mejor con el fallback y clearing esperados en
  esta etapa;
- la logica propia es pequena, parametrizable desde launch y tiene tests
  unitarios para puntos aislados, clusters, NaN/Inf, limites min/max y
  preservacion de metadata.

Por eso `laser_filters` queda como alternativa futura, no como bloqueo para
esta implementacion.

## Que hace el filtro RANSAC V1 experimental
Nodo: `src/navegacion_gps/navegacion_gps/lidar_obstacle_filter.py`.

Proceso actual:

1. Consume `/scan_3d`.
2. Transforma la nube a `base_footprint`.
3. Toma roll/pitch desde `/imu/data` si la IMU esta fresca.
4. Aplica una ROI fija:
   - `x: -0.4..12.0`
   - `y: -2.5..2.5`
   - `z: -1.0..2.0`
5. Compensa roll/pitch.
6. Ajusta un unico plano de suelo con RANSAC/SVD dentro de la ROI.
7. Clasifica como obstaculo puntos entre:
   - `min_obstacle_height: 0.22`
   - `max_obstacle_height: 1.40`
8. Aplica filtro de densidad por voxel XY:
   - `voxel_size_x: 0.25`
   - `voxel_size_y: 0.25`
   - `min_voxel_points: 3`
9. Publica:
   - `/obstacles_cloud`
   - `/scan_filtered`

## Cambios realizados hasta ahora
- Se creo `lidar_obstacle_filter`.
- Se agrego salida debug `/obstacles_cloud`.
- Se agrego salida `/scan_filtered` para Nav2 y collision monitor.
- Se agrego fallback con `enable_lidar_obstacle_filter:=False`.
- Se integro el filtro en:
  - `real_global_v2.launch.py`
  - `real_global_v2_wifi.launch.py`
  - `sim_global_v2.launch.py`
  - `sim_global_v2_wifi.launch.py`
- Se ajustaron costmaps para separar fuentes de marcado y limpieza:
  - `scan_marking`: marca obstaculos, no limpia.
  - `scan_clearing`: limpia espacio libre, no marca.
- Se agrego healthcheck de `/scan_filtered` y `/obstacles_cloud`.
- Se agrego script de bag de debug con topicos LiDAR.
- Se agrego mundo de simulacion con rampa:
  - `src/navegacion_gps/worlds/slope_lidar.world`
  - `tools/launch_sim_global_v2_wifi_slope.sh`
- Se agregaron tests unitarios y de contrato de launch para el filtro.

## Que fallo en robot real
Sintoma observado:

- Muchos obstaculos fantasma.
- El planner intentaba generar caminos grandes para sortear esos obstaculos.
- El comportamiento no fue aceptable para navegacion real.

Lectura tecnica probable:

- Algunos retornos espurios sobreviven al filtro 3D.
- Al proyectarse a `LaserScan`, cada retorno sobreviviente se convierte en una medicion 2D valida.
- Nav2 infla esas celdas.
- El global costmap puede retenerlas alrededor de 1 s.
- El planner global intenta rodearlas como si fueran objetos reales.

## Por que la simulacion no anticipo el problema
La simulacion fue util para validar wiring, topicos, launch args y errores obvios, pero no reprodujo bien el RS16 real.

Diferencias importantes:

- La nube simulada es mas limpia y estable.
- No aparecen reflejos reales, vegetacion, polvo, pasto, bordes de terreno ni retornos por material.
- No se simula bien vibracion del chasis.
- No se simula bien movimiento durante el barrido mecanico.
- No se modelan errores finos de extrinsecos `lidar_link -> base_footprint`.
- No se modela una IMU real con offsets, latencia, convenciones de frame o ruido.
- Un RS16 real es escaso verticalmente: 16 anillos. Eso hace que pocos puntos mal clasificados puedan pesar mucho.

## Hipotesis principales
### 1. RANSAC de plano unico demasiado fragil
El filtro actual busca un unico plano de suelo en toda la ROI. En terreno real, el suelo puede tener pendiente local, lomadas, laterales, cunetas, pasto, bordes o irregularidades. Un plano global puede ajustar mal y dejar pedazos de suelo como obstaculos.

### 2. Compensacion IMU no suficientemente confiable
El filtro usa roll/pitch de `/imu/data`, pero no valida en profundidad:

- Frame exacto de la IMU.
- Convencion ENU/NED.
- Alineacion fisica de la Pixhawk respecto del chasis.
- Timestamp de IMU contra timestamp de LiDAR.
- Offset entre `imu_link`, `base_footprint` y `lidar_link`.

Si esa compensacion esta unos grados mal, el suelo puede entrar como obstaculo.

### 3. Filtrado de outliers insuficiente
El filtro de densidad actual agrupa por voxel XY y pide 3 puntos. En RS16 real, un objeto fantasma puede generar varios puntos cercanos en un frame, especialmente por anillos vecinos, bordes o reflejos. Eso puede pasar el filtro.

### 4. Proyeccion final a LaserScan
Aunque el procesamiento inicial sea 3D, la salida que consume Nav2 es 2D. Al perder altura, anillo, densidad y forma, un pequeno cluster superviviente se convierte en obstaculo plano.

### 5. Obstaculos dinamicos en global costmap
El global costmap real WiFi usa obstacle layer con alcance amplio e inflacion. Esto hace que falsos positivos provoquen replanning grande. Para robots outdoor, suele ser mas estable dejar los obstaculos dinamicos fuertes en local/collision monitor y limitar su peso en global.

Nota operativa despues de pruebas reales:
en el perfil `real_global_v2_wifi`, el LiDAR puede quedar levemente inclinado y
convertir retornos rasantes del suelo lejano en obstaculos cuando el marcado
llega demasiado lejos. Para mantener el clearing largo sin usar esos retornos
para planificar rodeos, el perfil real WiFi copia la logica conservadora del
perfil real no-WiFi:

```text
local_costmap scan_marking obstacle_max_range: 10.0
local_costmap scan_clearing obstacle_max_range: 10.0
global_costmap obstacle_layer obstacle_max_range: 8.0
raytrace_max_range: 20.0
```

`raytrace_max_range` queda en 20 m para limpiar espacio libre; lo que se limita
es el alcance de marcado que puede meter obstaculos en costmaps y afectar la
ruta global.

## Que se suele hacer de forma estandar
La practica usual en robots con LiDAR 3D no es hacer solamente `RANSAC plano unico -> LaserScan`. Una cadena mas comun incluye:

1. Crop box / ROI.
2. Self-filter del robot.
3. Downsampling con voxel grid.
4. Filtro de outliers por radio o estadistica.
5. Transformacion correcta de frames.
6. Correccion temporal o deskew si el robot se mueve durante el barrido.
7. Segmentacion de suelo por metodo robusto:
   - por rayos/anillos;
   - por pendiente local;
   - por distancia radial;
   - o libreria probada.
8. Publicacion de nube de obstaculos como `PointCloud2`.
9. Costmap 3D/temporal local.
10. Decaimiento temporal y persistencia controlada.

Referencias externas utiles:

- Nav2 recomienda `SpatioTemporalVoxelLayer` para LiDAR 3D con decaimiento temporal, especialmente cuando la capa voxel estandar no alcanza:
  - https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html
  - https://docs.nav2.org/tuning/index.html
- `pointcloud_to_laserscan` es util para convertir nubes a `LaserScan`, pero esa conversion depende fuertemente de `target_frame`, `min_height`, `max_height` y rangos:
  - https://docs.ros.org/en/ros2_packages/humble/api/pointcloud_to_laserscan/
- Autoware separa preprocesamiento de nube, filtrado de outliers, transformacion y segmentacion de suelo:
  - https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/components/sensing/data-types/point-cloud/
  - https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_ground_segmentation/
  - https://autowarefoundation.github.io/autoware_core/main/perception/autoware_ground_filter/docs/ground-filter/
- PCL documenta filtros comunes para outliers:
  - https://pointclouds.org/documentation/tutorials/statistical_outlier.html
  - https://pointclouds.org/documentation/tutorials/remove_outliers.html

## Recomendacion operativa inmediata
Mientras no exista una version 2 validada:

1. En robot real, probar con filtro desactivado:

```bash
./tools/launch_real_global_v2_wifi.sh enable_lidar_obstacle_filter:=False
```

2. Grabar bags cuando aparezcan obstaculos fantasma:

```bash
./tools/record_nav_debug_bag.sh
```

3. Incluir al menos:
   - `/scan_3d`
   - `/scan`
   - `/scan_filtered`
   - `/obstacles_cloud`
   - `/imu/data`
   - `/tf`
   - `/tf_static`
   - `/odom`
   - `/local_costmap/costmap`
   - `/global_costmap/costmap`
   - `/cmd_vel`
   - `/cmd_vel_safe`
   - `/cmd_vel_final`

4. En RViz comparar:
   - `/scan_3d`
   - `/scan`
   - `/obstacles_cloud`
   - `/scan_filtered`
   - local costmap
   - global costmap

## Plan de mejora propuesto
### Fase 0: seguridad y rollback
Objetivo: que las pruebas reales no dependan del filtro fallido.

- Mantener `enable_lidar_obstacle_filter:=False` para pruebas reales hasta validar una version 2.
- Considerar cambiar el default real a `False` y dejar el filtro como experimental.
- Mantener el filtro activo solo en sim/desarrollo o cuando se pida explicitamente.
- Documentar en launch que `lidar_obstacle_filter` V1 no esta validado para real.

### Fase 1: dataset real
Objetivo: dejar de tunear con intuicion.

- Grabar bags en:
  - plano limpio;
  - pendiente;
  - pasto/tierra;
  - bordes laterales;
  - cerca de paredes/autos;
  - con robot detenido;
  - con robot en movimiento lento.
- Crear scripts offline para reproducir filtros contra bags.
- Medir por frame:
  - cantidad de puntos en `/scan_3d`;
  - cantidad de puntos clasificados como obstaculo;
  - clusters por distancia/angulo;
  - puntos laterales cercanos;
  - costo generado en local/global costmap.

### Fase 2: validar frames y calibracion
Objetivo: eliminar errores geometricos antes de tocar algoritmos.

- Verificar `lidar_link -> base_footprint` fisicamente.
- Confirmar altura real del LiDAR.
- Confirmar pitch/roll de montaje del LiDAR.
- Confirmar frame de `/imu/data`.
- Confirmar si la IMU ya esta convertida a ENU.
- Comparar roll/pitch IMU contra nivel fisico del robot.
- Revisar timestamps de `/scan_3d` y `/imu/data`.
- Confirmar que `use_lidar_clock: false` no introduzca problemas de timing.

### Fase 3: pipeline V2 de nube
Objetivo: robustecer antes de segmentar.

Propuesta de pipeline:

```text
/scan_3d
-> transform/crop/self-filter
-> voxel downsample
-> radius/statistical outlier filter
-> ground segmentation por rayos/anillos
-> obstacle cloud
-> costmap local temporal 3D
```

Preferencia tecnica:

- Implementar V2 en C++/PCL si la Raspberry Pi 5 no soporta Python a la tasa real.
- Mantener debug rico:
  - `/lidar/roi_cloud`
  - `/lidar/non_ground_cloud`
  - `/lidar/ground_cloud`
  - `/lidar/obstacles_cloud`
  - `/lidar/diagnostics`

### Fase 4: segmentacion de suelo por rayos/anillos
Objetivo: reemplazar plano unico.

Metodo esperado:

- Convertir puntos a coordenadas polares.
- Agrupar por azimut o anillo/rayo.
- Ordenar por distancia.
- Clasificar suelo por continuidad, pendiente local y altura relativa.
- Rechazar puntos que sean suelo aunque esten en pendiente.
- Preservar obstaculos verticales reales.

Este enfoque se parece mas a los filtros usados en stacks outdoor/autonomos que el plano unico global.

### Fase 5: costmaps Nav2
Objetivo: que un falso positivo no dispare caminos gigantes.

Cambios a evaluar:

- Local costmap:
  - usar `PointCloud2` en vez de `LaserScan` si es posible;
  - evaluar `SpatioTemporalVoxelLayer`;
  - decaimiento rapido;
  - persistencia baja para marcado;
  - limpieza agresiva con raytracing.
- Global costmap:
  - reducir o desactivar obstaculos dinamicos del LiDAR;
  - mantener keepout/static como fuente global principal;
  - si se usan obstaculos en global, bajar alcance y persistencia;
  - evitar que ruido puntual provoque replanning global grande.
- Collision monitor:
  - usar fuente conservadora y cercana;
  - priorizar seguridad sobre planificacion global.

### Fase 6: criterios de aceptacion
La nueva percepcion debe pasar pruebas en bag y robot real:

- Piso plano sin obstaculos: no debe generar obstaculos persistentes.
- Pendiente transitable: no debe bloquear la ruta.
- Pared/caja/auto: debe aparecer estable y con forma razonable.
- Objeto pequeno relevante: debe detectarse si esta dentro de la zona de seguridad.
- Punto aislado o cluster de 1 frame: no debe afectar global planner.
- Perdida temporal de LiDAR: debe limpiar o degradar de forma segura.
- Robot detenido y en movimiento: resultados comparables.

## Preguntas abiertas
- Cual es la transformacion exacta y medida de `lidar_link` respecto a `base_footprint`.
- Si `/imu/data` esta en ENU final o conserva convenciones MAVLink/NED en algun punto.
- Si el RS16 esta publicando todos los anillos esperados y sin packet loss.
- Si hay reflexiones o partes del robot entrando en la nube.
- Si el global costmap necesita obstaculos LiDAR o puede operar principalmente con mapa/keepout y dejar dinamicos al local.
- Si conviene integrar `spatio_temporal_voxel_layer` como dependencia o mantener implementacion propia.

## Decision actual
El filtro LiDAR V1 queda documentado como experimental. Sirvio para aprender, pero no debe ser la base final de percepcion real. La direccion recomendada es rehacer la percepcion con pipeline 3D robusto, validado con bags reales y con menor impacto de falsos positivos en el global planner.
