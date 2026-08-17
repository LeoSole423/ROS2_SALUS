# Simulación de cobertura por waypoints

Estado: implementado para `sim_global_v2`, con preview y compuerta de seguridad
nominal en la sección **CAMPO** de Cockpit. No está validado todavía en el
vehículo físico ni con GPS degradado.

Alcance: generación de un recorrido tipo cortadora de césped para un lote
cuadrado, visualización de la polilínea completa y envío de sus
extremos de pasada más una guía exterior por cabecera al `route_executor`, sólo
después de una revalidación.

Fuente de verdad: `coverage_waypoint_core.py`, `coverage_waypoint_mission.py`,
`route_executor.py`, `web_zone_server.py`, `GenerateCoveragePlanLL.srv`,
`CoverageService.ts` y `nav2_global_v2_sim_rolling_params.yaml`.

## Regla geométrica y topología

Las pasadas son paralelas, cubren el ancho solicitado y alternan el sentido. La
separación máxima entre centros es:

```text
ancho_de_corte * (1 - solape)
```

La última separación se ajusta para no dejar una franja fina. Cada transición
usa un camino Dubins sin marcha atrás y con el radio mínimo indicado.

### El orden de pasadas es fila por fila

El lote se recorre pasada por pasada, desde la del vehículo hacia el borde
opuesto, sin saltear ninguna. Es el recorrido que se espera de una cosechadora,
deja el trabajo sin huecos aunque la misión se corte a mitad de camino y mantiene
juntas en el tiempo dos pasadas vecinas, que es lo que acota el error de solape
frente a la deriva de la localización.

El precio lo paga la cabecera. Encadenar la pasada vecina da una U simple sólo
cuando la separación llega al diámetro de giro; por debajo, el enlace es un
omega, que sobresale mucho más de lo que mide el radio. Medido sobre la geometría
Dubins real con `R = 4 m`:

| separación | tipo | largo del giro | desborde de cabecera |
| --- | --- | --- | --- |
| 1.64 m | omega | 27.4 m | 10.4 m |
| 5.00 m | omega | 22.5 m | 8.7 m |
| 8.00 m (`2R`) | U simple | 12.6 m | 4.0 m |
| 9.82 m | U simple | 14.4 m | 4.0 m |

El omega es un giro válido y el vehículo lo ejecuta; sólo pide cabecera libre. Lo
que **no** hace el planificador es cambiar el orden para evitarlo.

`coverage_allow_row_skipping` (parámetro de `route_executor`,
`allow_row_skipping` en el core, `--allow-row-skipping` en el CLI) habilita la
búsqueda del orden que menos sobresale, apagada por defecto y también en
`sim_global_v2`. Con ella, el candidato clave es `max_separation_row_order`:
parte las pasadas en dos bloques y los intercala (`0, b, 1, b+1, ...` con
`b = ceil(N/2)`), lo que maximiza la separación mínima posible para un recorrido
que arranca en la pasada `0` —arrancar ahí no es negociable, el preflight de
`start_coverage` exige que la primera meta esté al lado del vehículo—. En el lote
de 20 × 20 con corte 2 y solape 0.15 eso convierte 11 omegas en 11 U simples:
cabecera 10.4 → 4.0 m, desborde lateral 3.2 → 0 m y recorrido 541 → 389 m. El
costo es el orden de cobertura: el lote queda cubierto en dos bloques
intercalados, y por eso es una decisión del perfil y no un ajuste automático.

Con la búsqueda habilitada, la seguridad topológica manda sobre el desborde: se
recorre la preferencia de mejor a peor y se devuelve el primer candidato sin
conflictos en el alcance auditado. Si ninguno está limpio se devuelve el de menor
cabecera, y el bloqueo queda aguas arriba.

Saltar pasadas **no** elimina los cruces de cabecera: los enlaces de un mismo
extremo se entrelazan igual, porque todo emparejamiento no cruzado contiene un
par de pasadas adyacentes. Lo que sí elimina es el omega. Los cruces que quedan
caen enteros en la cabecera, fuera del lote.

### El radio de giro es lo que fija el tamaño del omega

Con las pasadas más angostas que el diámetro de giro, el largo del omega y la
cabecera que necesita escalan de forma lineal con el radio. Medido sobre la
geometría Dubins real para pasadas a 1.53 m:

| radio | largo del giro | cabecera |
| --- | --- | --- |
| 4.00 m | 27.5 m | 10.4 m |
| 3.50 m | 23.8 m | 9.1 m |
| 2.89 m | 19.3 m | 7.4 m |
| 2.00 m | 12.7 m | 4.9 m |

El radio físico del vehículo es ~1.63 m (wheelbase 0.94 m, dirección a 30°). Los
4.0 m del perfil histórico salían del límite angular de 0.4 rad/s a 1.6 m/s, no
de la mecánica. `sim_global_v2` planifica a **2.9 m** (Smac, smoother y
`coverage_planner_min_turning_radius_m` se mueven juntos) y sube el límite
operativo de dirección de 18° a 25°, que da un radio efectivo de 2.02 m: la
diferencia es el margen de corrección del controlador. Con los 18° anteriores el
seguimiento del omega quedaba exactamente sobre el tope de dirección.

El perfil real sigue en 4.0 m y 18°: el radio corto no está validado en el
vehículo físico.

La condición para que el recorrido completo, incluidas las cabeceras, no se
pise a sí mismo es:

```text
separación entre pasadas consecutivas del recorrido >= min_turning_radius_m
```

Es invariante de escala: medida sobre la geometría Dubins real y verificada para
`R = 2, 3, 4 y 6 m`. Ojo con la diferencia: **`separación >= 2R` no es el
criterio de seguridad**, es la condición de la U simple. Usarlo como criterio
descarta todo el rango `[R, 2R)`, que sí es válido.

`clean_uturn_count` es una condición local de cada transición; no prueba que el
recorrido completo sea simple. Por eso el plan también expone:

- `strict_crossing_count`
- `nonadjacent_touch_count`
- `collinear_overlap_count`
- `topology_conflict_count`
- `is_topologically_safe`
- `minimum_safe_lane_spacing_m`
- `topology_audit_spacing_m`

El planificador remuestrea a `0.4 m` y calcula dos auditorías: una global y otra
recortada al interior abierto del rectángulo físico. El muestreo de preview no
sirve para auditar: recorta las curvas y puede ocultar un cruce real.

En `sim_global_v2`, `coverage_allow_headland_conflicts=true`: la compuerta usa la
auditoría interior. Los cruces, contactos, solapes y giros omega exteriores se
muestran como advertencia pero no bloquean. El perfil real conserva la política
global: sólo acepta un orden sin ningún conflicto, y si la geometría no lo
permite el arranque queda bloqueado.

El caso `20 x 20 m`, corte `2 m` y solape `0.15` deja `12` pasadas separadas
`1.636 m`, muy por debajo del diámetro de giro. Recorridas de a una, las `11`
cabeceras son omegas: con `R = 4 m` piden `10.4 m` de cabecera a cada lado y
`3.2 m` de invasión lateral, y el camino sale `541 m` para `240 m` de trabajo;
con `R = 2.9 m` bajan a `7.4 m` y `2.1 m`, y el camino a `452 m`. Los cruces que
quedan son de cabecera y caen fuera del rectángulo; uno que invada el lote sigue
bloqueando el inicio. Con `coverage_allow_row_skipping` encendido el mismo lote
sale con `11` U simples y `389 m`, a costa de cubrirlo en dos bloques
intercalados.

## Uso desde Cockpit

La sección **CAMPO** aparece junto a los demás acordeones de navegación.

El lote es **siempre un cuadrado y no se dibuja**: se arma desde la pose del
vehículo, que queda en una esquina, y el rumbo del vehículo es la dirección de
las pasadas. No hay figura libre que marcar en el mapa, ni modo rectángulo, ni
dos medidas que ajustar: sólo el lado.

1. Pulsar **ARMAR CUADRADO**. Necesita pose y rumbo del vehículo; sin eso el
   botón queda deshabilitado en vez de inventar una coordenada.
2. Escribir el **lado exacto** si el valor por defecto no sirve. Cualquiera de
   los dos lados que se toque mueve los dos.
3. Ubicarlo: en el mapa el cuadrado tiene dos tiradores. El del **centro** lo
   mueve sin cambiarle el lado ni el rumbo; el de la **esquina opuesta a la de
   arranque** le cambia el lado. Mover el cuadrado lo desengancha de la pose del
   vehículo.
4. **INVERTIR INICIO** mueve el comienzo a la esquina opuesta y cambia el sentido
   de las pasadas; también invierte de qué lado del vehículo crece el cuadrado.
5. Pulsar **GENERAR PREVIEW**.

El cuadrado se dibuja en azul. La trayectoria nominal aparece magenta si pasa
la auditoría y naranja si contiene conflictos. El primer punto recibido por el
generador es una esquina física: el backend desplaza el centro de la primera
pasada media cuchilla hacia delante y media cuchilla hacia el interior, y reduce
el largo de centro de línea en un ancho de cuchilla. Así la huella nominal queda
dentro del cuadrado. Por eso **ARMAR CUADRADO** no manda la pose del vehículo
como esquina sino media cuchilla por detrás y hacia el otro lado: con ese
corrimiento el inset deja la primera pasada exactamente bajo el vehículo y la
misión arranca alineada, sin un rulo previo para tomar medio ancho de desvío.

`preview_coverage` es de sólo lectura y nunca mueve el vehículo. Antes de iniciar,
`start_coverage` vuelve a generar el plan en el servidor. En simulación falla
cerrado si se cumple cualquiera de estas condiciones:

- existe un cruce, contacto no adyacente o solape colineal dentro del campo;
- el radio solicitado es menor que el mínimo del planner de simulación (`2.9 m`);
- el resultado ROS contiene conteos, flags key o `topology_safe` contradictorios;
- el robot no tiene pose y rumbo globales frescos;
- el robot está a más de `50 m` de la primera meta o difiere más de `30°` de su
  rumbo. Este margen amplio es sólo para simulación y permite que Nav2 haga el
  acercamiento desde el punto de spawn;
- se superan `100` pasadas, `200` metas key o `2000` puntos de poligonal (ver
  «El conteo de puntos no depende del largo de la pasada»).

La auditoría usa un muestreo de `0.5 m` o más fino, aunque el preview visual se
haya solicitado con menor detalle.

### El conteo de puntos no depende del largo de la pasada

La auditoría de cruces remuestrea el trazado, y antes partía también las pasadas
cada `0.5 m`. Eso ataba el conteo al largo del lote: un cuadrado de `37.8 m` con
corte `2 m` daba `23` pasadas de `35.8 m` y `2581` puntos contra un tope de
`2000`, así que el preview quedaba bloqueado con
`coverage sampled waypoint upper bound 2581 exceeds limit 2000`.

Partir una recta no cambia ningún cruce: dos segmentos se cortan o no, y
muestrearlos más fino sólo agrega puntos. El detalle hace falta únicamente en las
curvas, donde la poligonal recorta el arco real. Ahora las pasadas entran enteras,
con sus dos extremos, y el muestreo fino queda para las cabeceras —que es también
donde se ubica la guía, así que la poligonal ejecutable no pierde precisión—.

| lado del lote | pasadas | antes | ahora |
| --- | --- | --- | --- |
| 20 m | 12 | 0.03 s | 0.03 s |
| 37.8 m | 23 | 0.29 s | 0.08 s |
| 70 m | 41 | 1.31 s | 0.27 s |
| 140 m | 83 | 10.47 s | 0.63 s |
| 300 m | 177 | — | 3.47 s |

Los conteos de conflictos no cambian: para el mismo lote dan lo mismo con detalle
de preview `2.0`, `0.5` o `0.25 m`, porque la auditoría siempre remuestrea por su
cuenta. Eso hizo redundante la segunda construcción del plan que hacía
`route_executor`, que ahora sólo sirve para ubicar la guía de cabecera.

El tope sigue en `2000` puntos y ahora se mide por separado sobre las dos
poligonales —la de preview, que depende del **detalle** pedido, y la de auditoría,
que depende de las **cabeceras**—; el error nombra las dos y qué mover.

### Un lote corrido del vehículo se ejecutaba por el medio

Mover el cuadrado destapó dos defectos de ejecución, los dos fuera del generador
de cobertura. Con el lote apoyado en el vehículo no se notaban; con el lote
corrido unos metros la misión dejaba pasadas sin hacer.

**1. La ruta se enganchaba en un tramo del medio.** `prepare_route_waypoints`
descarta las metas anteriores si el robot está a menos de
`route_segment_start_tolerance_m` (`5 m`) de cualquier tramo de la ruta. Sirve
para retomar una ruta a medio hacer, y en cobertura es un error: los lazos de
cabecera pasan a pocos metros del vehículo, así que un lote corrido arrancaba por
la pasada 3. Medido: `12` metas entraban y `5` quedaban. `SetRouteMissionLL` ganó
`start_from_first_waypoint`, que `start_coverage` manda siempre en `true`; el
resto de las rutas conserva el comportamiento anterior.

**2. Nav2 borraba la meta de inicio de cada pasada.** El árbol de
`NavigateThroughPoses` tenía `RemovePassedGoals radius="2.5"`, más grande que la
separación entre pasadas (`1.5` a `2.4 m`). Al llegar al final de una pasada, la
meta de inicio de la siguiente caía dentro del radio y Nav2 la borraba: el plan
pasaba a ser una diagonal hasta el final de la pasada siguiente, sin hacer la
cabecera ni recorrer la línea. El radio bajó a `1.2 m`, que es la misma
tolerancia de llegada que usa el resto del stack
(`route_waypoint_reached_tolerance_m` y `xy_goal_tolerance`).

De ahí sale una condición nueva del recorrido: **la separación entre pasadas tiene
que superar el radio de `RemovePassedGoals`**. Con corte `2 m` y solape `0.15` la
separación es `1.64 m` contra `1.2 m`, y el caso está medido. Un solape mucho más
alto la haría bajar y el síntoma volvería: pasadas recorridas en diagonal.

### El giro va entero como una sola meta

Al ejecutar se envían **sólo** los dos extremos `key` de cada pasada. El giro de
cabecera viaja como un único tramo `fin de pasada → inicio de la siguiente`, sin
puntos intermedios. `coverage_use_headland_guides` está apagado en los dos
perfiles.

La guía exterior parecía razonable —un punto que obliga a Smac a tomar el lóbulo
correcto— y es exactamente lo que hay que evitar. Al partir el giro, el último
tramo pide un arco del radio mínimo exacto, que cae justo en el borde del
conjunto alcanzable del Hybrid-A*; cualquier error de cuantización lo vuelve
inalcanzable y la búsqueda lo cierra con una vuelta completa de `2πR = 25 m`.
Medido contra el `planner_server` de este repositorio, barriendo el rumbo de la
pasada sobre `0, 8.4, 33, 90, 160, 200°`:

| giro | nominal | una sola meta | partido con guía |
| --- | --- | --- | --- |
| U simple, sep. `9.82 m` | 14.4 m | 14.7–18.0 m | 39.5–63.8 m |
| U simple, sep. `8.00 m` | 12.6 m | 16.5–17.7 m | — |
| omega, sep. `5.00 m` | 22.5 m | 23.5–24.7 m | 25.6–51.0 m |
| omega, sep. `1.64 m` | 27.4 m | 27.9–29.1 m | 54.2–55.3 m |

Con guía el resultado además depende del rumbo: el mismo giro sale limpio en un
rumbo y con rulo en otro. Agrandar el radio de diseño o agregar una recta de
salida lo mueve de lugar pero no lo elimina. Enviado como una sola meta el plan
se mantiene cerca del nominal en todos los casos probados.

La misión conserva `loop=false` y usa un `leg_spacing_m` mayor que el largo de
pasada, para que `route_executor` no agregue interpolaciones rectas dentro de la
cabecera. Un timeout al enviar se informa como estado incierto y no debe
reintentarse sin consultar o cancelar primero la ruta.

`real_global_v2` mantiene radio de planner `4.0 m`, dirección automática limitada
a `18°`, política topológica global y preflight de aproximación de `5 m`/`30°`.
Sólo la simulación admite conflictos de cabecera fuera del campo y una
aproximación inicial de hasta `50 m`.

## Resultado medido en simulación

### Radio 4.0 m contra 2.9 m, mismo lote y mismo orden

`sim_global_v2`, lote `10.5 x 12.5 m`, corte `1.8 m`, solape `0.15`, `8` pasadas
a `1.53 m`, orden `0..7`. Dos corridas completas de punta a punta:

| medida | `R = 4.0 m` | `R = 2.9 m` |
| --- | --- | --- |
| recorrido nominal | 274.8 m | 217.9 m |
| recorrido ejecutado | 260.7 m | 229.2 m |
| duración | 319 s | 241 s |
| plan por chunk (pasada + cabecera) | 39.5–40.8 m, uno de 54.9 m | 30.8–32.4 m |
| huella del recorrido | 32.6 × 30.0 m | 27.1 × 15.7 m |
| error de seguimiento del plan | 0.058 m medio, 0.189 m máx | 0.072 m medio, 0.279 m máx |
| distancia meta → trayectoria | 0.459 m media, 1.469 m máx | 0.164 m media, 0.417 m máx |
| desvío de la pasada nominal | 0.215–0.374 m medio | 0.120–0.299 m medio |

Las dos corridas terminan en `route completed` y recorren las `8` pasadas en
orden. El radio corto no empeora el seguimiento —el error contra el plan de Nav2
sube `0.014 m` de media y ninguna muestra pasa de `0.28 m`— y sí mejora el
trabajo: la trayectoria pasa tres veces más cerca de sus metas, porque cada giro
acumula menos deriva antes de entrar a la pasada siguiente. Con `R = 4 m` un giro
se fue `15 m` de más; con `2.9 m` no hay ningún plan atípico.

### El flujo del cockpit, punta a punta

Cuadrado de lado `15 m` armado desde el vehículo, corte `3 m`, solape `0.15`:
`6` pasadas a `2.40 m`, orden `0..5`, `5` omegas y `161.6 m` nominales. Arrancado
desde el mismo flujo termina en `route completed`, y comparado contra el trazado
nominal convertido con `/fromLL`:

| medida | valor |
| --- | --- |
| largo nominal del preview | 161.6 m |
| largo ejecutado dentro del lote | 169.6 m |
| desvío contra el nominal | 0.239 m medio, 0.720 m p95, 1.217 m máx |
| puntos nominales visitados a menos de 1 m | 91 / 92 |
| distancia a cada meta key | 0.173 m media, 0.894 m máx |
| cobertura por pasada | 100, 92, 100, 100, 100, 96 % |

Once de las doce metas quedan a menos de `0.25 m`; la peor es la última
(`0.89 m`), el cierre de la misión. El sobrante de `8 m` sale de que el vehículo
abre un poco más los omegas que el nominal.

### El mismo lote corrido del vehículo

El mismo cuadrado de `15 m`, movido `12 m` al este y `6 m` al norte con el tirador
del centro. La cobertura por pasada es la medida que importa: qué fracción de cada
línea nominal quedó a menos de `0.6 m` de la huella.

| medida | radio 2.5 m, sin bandera | con `start_from_first_waypoint` | + radio 1.2 m |
| --- | --- | --- | --- |
| metas entregadas a la ruta | 5 de 12 | 12 de 12 | 12 de 12 |
| cobertura por pasada | 100, 100, 24, 60, 44, 96 % | 100, 100, 24, 60, 44, 96 % | 100, 100, 100, 100, 100, 96 % |
| largo ejecutado (nominal 161.7 m) | 127.2 m | 136.2 m | 163.8 m |
| desvío contra el nominal | 0.547 m medio | 0.432 m medio | 0.219 m medio, 1.082 m máx |
| distancia a cada meta key | 0.679 m media, 2.34 m máx | 0.529 / 1.71 m | 0.207 / 1.07 m |
| puntos nominales a menos de 1 m | 53 / 92 | 57 / 92 | 91 / 92 |

Las dos correcciones son necesarias: la bandera devuelve las metas descartadas,
pero sin bajar el radio las pasadas del medio se siguen recorriendo en diagonal.

El caso más ajustado, `20 x 20 m` con corte `2 m` (separación `1.64 m` contra el
radio de `1.2 m`), movido `8 m` al este y `5 m` al norte: `24` de `24` metas,
`460.8 m` ejecutados contra `425.0 m` nominales, `232` de `241` puntos nominales a
menos de `1 m`, metas key a `0.199 m` de media y `1.118 m` la peor, y cobertura por
pasada de `100 %` en nueve de las doce, con `97.3`, `94.6` y `91.9 %` en las otras
tres.

### Orden de pasadas y guía de cabecera

`sim_global_v2`, lote `20 x 20 m`, corte `2 m`, solape `0.15`, `R = 4 m`,
`12` pasadas.

| corrida | nominal | ejecutado | duración | plan por chunk |
| --- | --- | --- | --- | --- |
| pasadas vecinas + guía | 538 m | — | — | — |
| máxima separación + guía | 389 m | 547 m | 523 s | 59–85 m |
| máxima separación, sin guía | 389 m | 432 m | 413 s | 37.5–39.0 m |

Con el giro entero cada chunk queda en `37.5–39.0 m` contra `34.4 m` nominales:
sin vueltas de más. Las `12` pasadas se recorren completas, con un residuo de
recta de `0.066 m` de media y `0.355 m` de máximo. El orden de máxima separación
de esa tabla hoy queda detrás de `coverage_allow_row_skipping`, apagado.

Un segundo lote, `40 x 20 m` con corte `5 m` (4 pasadas, dos U simples y un
omega), da `214.9 m` ejecutados contra `211.3 m` nominales: el omega sin guía
también reproduce su nominal.

Queda un residuo que no es geométrico: la separación real entre pasadas vecinas
varía hasta `0.39 m` contra un margen de solape de `0.36 m`. Es error de
localización. Apagar la guía lo bajó de `0.43 m` de media a `0.17 m`, y el radio
corto lo baja otra vez, a `0.164 m` de distancia media entre meta y trayectoria.
Antes de ir a campo con solape bajo conviene medirlo con el GPS real.

## Validación por consola

Recompilar el contrato y sus consumidores dentro del contenedor:

```bash
./tools/compile-ros.sh interfaces navegacion_gps map_tools
```

Para inspeccionar un plan sin mover el vehículo:

```bash
./tools/demo_coverage.sh --dry-run \
  --field-length-m 20 \
  --field-width-m 20 \
  --cutter-width-m 2 \
  --overlap-ratio 0.15 \
  --min-turning-radius-m 2.9
```

Para ejercitar el flujo del cockpit sin navegador —el cuadrado desde la pose, el
preview y el arranque, con el `CoverageService` de producción contra el
`web_zone_server` real— desde `cockpit/`:

```bash
npx vite-node scripts/campo-square-check.ts 20 2           # solo preview
npx vite-node scripts/campo-square-check.ts 15 3 --start   # arranca la mision
```

Los dos números son el lado del cuadrado y el ancho de corte. `CAMPO_DUMP=<archivo>`
guarda el trazado nominal en lat/lon, que con `/fromLL` pasa a coordenadas de mapa
y permite comparar contra `/odometry/global` sin estimar ningún marco.

`coverage_waypoint_mission.py` y `run_coverage_waypoints.sh` son herramientas de
consola separadas: `--send-route` puede iniciar sin la compuerta atómica de
`start_coverage`. Para la garantía nominal descrita en este documento se debe
usar **CAMPO**. El CLI conserva la política global y no representa la excepción
de cabeceras exteriores que habilita el launch de simulación.

## Límites de la garantía

La auditoría certifica la polilínea Dubins nominal generada, no el camino real.
Nav2 recibe sólo las metas `key` y Smac todavía puede replanificar cada tramo por
error de pose, costmaps u obstáculos: una replanificación dinámica no queda
certificada por el preview. Para comprobar el resultado de ejecución hay que
registrar y analizar la unión de los planes de Nav2 y la trayectoria de
`/odometry/global`.

La simulación tampoco representa un RTK real: el perfil `f9p_rtk` inyecta ruido
sintético y publica la etiqueta `RTK_FIXED`. Antes de ir a campo hay que medir el
radio operativo a velocidad de trabajo, usar margen sobre el radio configurado
(como punto de partida, `1.2–1.3 x R_planner`) y repetir la prueba con GPS
degradado. Que Gazebo complete una ruta no demuestra espacio, adherencia ni
precisión suficientes en el vehículo real.
