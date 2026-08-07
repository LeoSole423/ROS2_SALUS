# Plan Inmediato De Reduccion De Ruido LiDAR

Estado: plan operativo inmediato
Alcance: mejoras conservadoras sobre el pipeline LiDAR legacy para reducir obstaculos fantasma sin rediseñar toda la percepcion
Fuente de verdad: pruebas reales previas, `docs/lidar-perception.md`, `pointcloud_to_laserscan_real.yaml`, configs Nav2 Global V2 y comportamiento observado antes/despues del filtro RANSAC

## Objetivo
Reducir obstaculos fantasma manteniendo el comportamiento que ya funcionaba bastante bien antes del filtro RANSAC.

Este plan NO busca resolver toda la percepcion outdoor 3D. Busca una mejora pragmatica y de bajo riesgo:

```text
/scan_3d
-> pointcloud_to_laserscan
-> /scan
-> filtros simples /scan
-> /scan_clean
-> Nav2 costmaps + collision monitor
```

La prioridad es volver a una base estable y eliminar ruido puntual sin introducir un filtro 3D fragil.

## Decision Actual
No implementar ahora el plan V2 completo tipo Autoware/PCL/Nav2 3D.

El plan V2 queda documentado como direccion futura en:

```text
docs/investigaciones/lidar-percepcion-v2-plan.md
```

Para la siguiente iteracion se trabajara sobre este plan inmediato.

## Principios
- No usar el filtro RANSAC V1 como default real.
- No agrandar rolling windows ni cambiar Nav2 para esconder ruido.
- No reescribir percepcion 3D completa todavia.
- No tocar control ni localizacion salvo que una medicion lo justifique.
- Cambiar una cosa por vez y validar contra `/scan`, costmaps y RViz.

## Pipeline Objetivo V1.5
Pipeline recomendado:

```text
RS16
-> /scan_3d
-> pointcloud_to_laserscan
-> /scan
-> laser_filters / scan cleanup
-> /scan_clean
-> Nav2 local/global costmaps
-> collision monitor
```

Fallback:

```text
/scan
```

Salida nueva esperada:

```text
/scan_clean
```

`/scan_clean` debe ser reversible por launch arg. Si algo falla, Nav2 debe poder volver a `/scan`.

## Etapa 0: Restaurar Base Estable
Objetivo: dejar de depender del filtro RANSAC V1 en robot real.

Tareas:

1. Confirmar que el robot real puede arrancar con:

```bash
enable_lidar_obstacle_filter:=False
```

2. Confirmar que Nav2 y collision monitor vuelven a consumir `/scan`.
3. Mantener `/scan_filtered` y `/obstacles_cloud` solo para debug/desarrollo.
4. Documentar que RANSAC V1 no es default para real.

Entregables:

- launch real con fallback claro a `/scan`;
- healthcheck que muestre que topico LiDAR consume Nav2;
- nota en docs.

Criterio de salida:

- el robot navega igual que antes del filtro RANSAC;
- `/scan` aparece en RViz y costmaps;
- no se usa `/scan_filtered` en real salvo pedido explicito.

## Etapa 1: Medicion Minima, No Dataset Grande
Objetivo: obtener evidencia suficiente sin frenar el avance con una campaña larga.

Grabar pocos bags, solo los necesarios para este plan:

1. `plano_limpio_detenido`
   - robot quieto;
   - sin obstaculos cercanos;
   - confirma ruido base.

2. `laterales_libres`
   - espacio libre a izquierda/derecha;
   - reproduce puntos fantasma laterales si existen.

3. `obstaculo_real_simple`
   - cono, caja, pared o auto;
   - confirma que el filtrado no borra obstaculos reales.

4. `pendiente_corta` solo si esta disponible
   - no es obligatorio para V1.5;
   - sirve para saber si el pipeline legacy sigue aceptable en pendiente.

Topicos minimos:

```text
/scan_3d
/scan
/imu/data
/tf
/tf_static
/local_costmap/costmap
/global_costmap/costmap
/cmd_vel
/cmd_vel_safe
/cmd_vel_final
```

Duracion minima:

- 20 s detenido por escenario;
- 30 s en movimiento lento si el escenario lo permite.

Entregables:

- 3 bags minimos;
- una captura RViz o nota por bag;
- observacion: si aparecieron fantasmas, donde y por cuanto tiempo.

Criterio de salida:

- existe al menos un bag donde se vea el ruido que queremos reducir;
- existe al menos un bag con obstaculo real;
- se puede comparar `/scan` contra costmaps.

## Etapa 2: Filtrado Simple De LaserScan
Objetivo: eliminar puntos aislados sin tocar segmentacion 3D.

Implementar un nodo o configuracion de filtros sobre `/scan`.

Orden recomendado:

```text
/scan
-> range filter
-> speckle filter
-> median filter opcional
-> /scan_clean
```

Filtros candidatos:

- `laser_filters/LaserScanRangeFilter`
- `laser_filters/LaserScanSpeckleFilter`
- `laser_filters/LaserScanMedianFilter`
- filtro propio minimo si `laser_filters` no esta disponible en Humble del contenedor.

Nota de implementacion V1.5:
`laser_filters` esta disponible en el contenedor Humble, pero la primera
implementacion quedo como nodo propio `scan_noise_filter`. La razon practica es
que el filtro de speckles de `laser_filters` no era plug-and-play para este
caso: el ejemplo instalado de speckle usa una estructura YAML antigua que el
parser ROS 2 rechaza, el nodo de cadena publica por defecto en `/scan_filtered`
(topico reservado en SALUS para el RANSAC experimental) y la semantica no es
identica a la deseada para Nav2. En particular, `LaserScanSpeckleFilter` marca
speckles con `NaN`, mientras que el filtro V1.5 propio convierte lecturas
invalidas/fuera de rango/sin vecinos a `+inf`, preserva metadata del
`LaserScan`, conserva clusters anchos y queda cubierto por tests unitarios.
Migrar a `laser_filters` sigue siendo posible, pero no es requisito para esta
etapa conservadora.

Parametros iniciales sugeridos:

```text
range_min: 0.4
range_max: 20.0
speckle_max_range: 12.0
speckle_filter_window: 2..4 beams
speckle_max_deviation_m: 0.20..0.35
median_window: 3 beams
```

Reglas:

- no eliminar clusters anchos;
- no tocar obstaculos persistentes;
- no filtrar por altura aca, porque `/scan` ya perdio altura.

Entregables:

- config de filtros;
- launch arg:

```bash
enable_scan_noise_filter:=True|False
scan_noise_filter_output:=/scan_clean
```

- RViz mostrando `/scan` y `/scan_clean`.

Criterio de salida:

- puntos aislados desaparecen o bajan claramente;
- obstaculo real simple se conserva;
- si el filtro falla, `enable_scan_noise_filter:=False` vuelve a `/scan`.

## Etapa 3: Ajuste Conservador De Costmaps
Objetivo: que un punto fantasma corto no genere rodeos grandes.

Cambios a evaluar, uno por vez:

1. Reducir persistencia de observaciones LiDAR:

```text
observation_persistence: 0.2..0.4
```

2. Mantener clearing agresivo:

```text
clearing: True
raytrace_max_range: 20.0
```

3. Separar comportamiento local/global:

```text
local_costmap: puede marcar obstaculos dinamicos
global_costmap: menor sensibilidad a ruido dinamico
collision_monitor: conserva seguridad cercana
```

4. Si el global planner sigue rodeando fantasmas:
   - bajar rango de marcado en global;
   - mantener rango completo en local/collision;
   - evaluar que global use menos LiDAR dinamico.

Reglas:

- no agrandar rolling window;
- no subir inflacion para tapar ruido;
- no bajar seguridad del collision monitor.

Entregables:

- cambios pequenos y documentados en YAML;
- comparacion antes/despues con el mismo bag;
- decision de parametros default.

Criterio de salida:

- fantasma corto no genera plan global absurdo;
- obstaculo real cercano sigue bloqueando o desviando;
- collision monitor sigue frenando.

## Etapa 4: Validacion En Robot
Objetivo: probar V1.5 con bajo riesgo.

Orden:

1. Robot detenido, sin goal.
2. Teleop lento.
3. Goal corto en plano.
4. Goal corto con obstaculo real simple.
5. Pendiente solo si el caso anterior paso.

En cada prueba mirar:

- `/scan`;
- `/scan_clean`;
- local costmap;
- global costmap;
- `/cmd_vel_safe`;
- logs de planner.

Criterios de aceptacion:

- no aparecen obstaculos persistentes donde no hay nada;
- los puntos aislados desaparecen mas rapido que antes;
- un obstaculo real sigue visible;
- Nav2 no genera rodeos gigantes por ruido puntual;
- rollback a `/scan` funciona.

## Cuando Pasar Al Plan V2
Pasar al plan V2 futuro solo si V1.5 no alcanza.

Condiciones para activar V2:

- el suelo en pendiente sigue bloqueando la ruta;
- el ruido no se puede eliminar con filtros `/scan`;
- aparecen fantasmas persistentes de varios frames;
- el global planner sigue sobreactuando a pesar de costmaps conservadores;
- se necesita navegar en terreno irregular de forma frecuente.

Si esas condiciones no se cumplen, mantener V1.5. Es mas simple, mas barato y mas facil de validar.

## Resultado Esperado
Al terminar este plan, SALUS debe volver a comportarse como antes del RANSAC, pero con menos ruido:

```text
pipeline simple
menos puntos fantasma
menos rodeos falsos
rollback inmediato
sin rediseño 3D grande
```

Este plan es el camino de corto plazo. El plan V2 queda como arquitectura definitiva futura si el robot necesita una percepcion 3D outdoor mas robusta.
