# MEMORIA CAMPO — contexto para retomar en otra máquina

Volcado de lo que aprendí trabajando la cobertura CAMPO, para que otra sesión no
tenga que redescubrirlo. Es texto plano, editable a mano: corregí lo que esté
mal, es la idea.

Última actualización: 2026-08-19.


## 1. Cómo trabajar

- Respuestas directas, sin teoría de más. Comandos listos para copiar y pegar.
- Nada de proponer "optimizaciones" que ya fueron rechazadas (ver §3).
- Antes de dar por buena una prueba en simulación, verificar que no haya nodos
  zombis (§5.1). Es el error más caro que cometí.


## 2. Dónde está cada cosa

| repo                  | rama                     | commit    | qué tiene                    |
|-----------------------|--------------------------|-----------|------------------------------|
| LeoSole423/ROS2_SALUS | `campo-cobertura-logica` | `82f6fdc` | toda la lógica de cobertura  |
| AEyeSecurity/cockpit  | `cuadro-campo-arrastre`  | `a750ee4` | panel CAMPO y el cuadrado    |

Carpeta limpia con las dos ramas y un script que levanta todo:
`/home/franco/campo` → `./campo.sh` (ver `COMO_EJECUTAR.md`).

Archivos centrales del backend:

- `src/navegacion_gps/navegacion_gps/coverage_waypoint_core.py` — geometría de
  la cobertura: pasadas, giros, cabecera, orden de recorrido.
- `src/navegacion_gps/navegacion_gps/route_executor.py` — arma el plan y lo
  ejecuta contra Nav2.
- `src/interfaces/srv/GenerateCoveragePlanLL.srv` y `SetRouteMissionLL.srv`.
- `src/map_tools/map_tools/web_zone_server.py` — el puente con el cockpit.
- `tools/run_campo.sh` — todo el ciclo desde la terminal, sin cockpit.

Del lado del cockpit:

- `src/packages/nav2/modules/navigation/service/impl/CoverageService.ts` — el
  cuadrado: armarlo, moverlo, girarlo, redimensionarlo, y el preview.
- `src/packages/nav2/modules/map/frontend/index.tsx` — el dibujo en el mapa y
  los tiradores.
- `scripts/campo-square-check.ts` — arnés que maneja el servicio real contra la
  sim y verifica que el trazado caiga dentro del cuadrado.


## 3. Decisiones tomadas, y por qué

### 3.1 El recorrido va pasada por pasada, sin saltear
`coverage_allow_row_skipping` queda en **False**. Va de la pasada del vehículo
hacia el borde opuesto, de a una. Los rulos (omega) de cabecera no molestan
aunque salgan del lote.

Franco lo pidió explícito el 2026-08-17 viendo el preview. Un orden que salteaba
pasadas (`0, 6, 1, 7, …`) achicaba la cabecera de 10.4 m a 4 m y el camino de
541 m a 389 m, y aun así lo rechazó. **La cabecera más chica no manda sobre el
orden de cobertura.** No volver a proponer el salteo como optimización.

### 3.2 El lote es un cuadrado, no una figura libre
Se probó dejar dibujar el polígono y se descartó: "que la figura sea un cuadro
no que me permita dibujar". El cuadrado sale de la pose del vehículo (la esquina
es donde está parado) y se ajusta con los tiradores.

### 3.3 Radio de giro 2.9 m
`minimum_turning_radius: 2.9` en el planner y en el smoother, en los dos YAML de
sim, y `coverage_planner_min_turning_radius_m: 2.9` en el launch.

Medido en sim contra 4.0: camino nominal 274.8 → 217.9 m, tiempo 319 → 241 s,
distancia a la meta 0.459 → 0.164 m, y el seguimiento prácticamente igual.

**Ojo:** `docs/sim-real-parity.md:18` pide que `operational_steering_limit_rad` y
los dos radios de Nav2 se muevan juntos. Dejé
`regulated_linear_scaling_min_radius` en 4.0 a propósito.

### 3.4 `RemovePassedGoals radius="1.2"`
Estaba en 2.5 y **ese era el motivo de que el vehículo cruzara las filas en
diagonal**: 2.5 m es más que la separación entre pasadas (1.5–2.4 m), así que
descartaba metas de la fila siguiente antes de llegar. Con 1.2 la cobertura por
fila pasó de 100/100/24/60/44/96 % a 100/100/100/100/100/96 %.

El valor 1.2 era el original del repo; lo habían subido a 2.5 en `4a1a2b4`.

### 3.5 `start_from_first_waypoint`
Campo nuevo del `.srv`. Sin él, `prepare_route_waypoints` unía el primer tramo
con la pose del vehículo y **perdía metas** cuando el lote quedaba unos metros
al costado (síntoma: "12 metas, 5 expandidas").

### 3.6 El cuadrado tiene que seguir al mouse
Guardar estado en cada `mousemove` obliga a rearmar toda la capa del mapa y el
cuadrado queda atrás del puntero: se siente como que "le erra". La geometría
nunca estuvo mal — verificado contra el backend moviendo, girando y agrandando
desde las cuatro esquinas, el trazado siempre cayó dentro del lote.

La solución fue separar calcular de guardar: `geometryForMove`,
`geometryForRotate` y `geometryForResize` devuelven la figura **sin tocar el
estado**, el mapa corre las formas de Leaflet a mano durante el gesto, y recién
al soltar se guarda. Si alguien vuelve a meter un `commit` adentro del
`mousemove`, vuelve el problema.


## 4. Números que hay que tener a mano

- Distancia entre pasadas = ancho de corte × (1 − solape), después repartida
  pareja para que entren enteras en el lote. **No está en ningún `.yml`**: sale
  del ancho de corte que se pide en el panel.
- Wheelbase 0.94 m. `steering_limit_rad` 30° (sale del URDF).
  `operational_steering_limit_rad` 25° (0.4363 rad).
- Radio = wheelbase / tan(ángulo). Con 30° → 1.63 m; con 18° → 2.89 m.


## 5. Trampas conocidas

### 5.1 Zombis de simulación
`tools/stop_sim_global_v2.sh` mata por patrón de nombre y **le faltan**
`scan_noise_filter`, `nav_observability`, `nav_trace_recorder`,
`path_clearance_validator` y `polygon_stamped_republisher`. Cada lanzamiento
deja esos cinco vivos; el 16-ago-2026 había 11 copias de cada uno, todas
publicando en los mismos topics.

**Es la causa de "la simulación corre el código viejo".** No es que el build
falle. Antes de creerle a cualquier prueba:

```bash
docker exec ros2_salus ps -eo pid=,args=
docker exec ros2_salus bash -lc 'source /opt/ros/humble/setup.bash && ros2 node list | sort | uniq -c'
```

Para limpiar: `kill -9` los PIDs de esos cinco nombres y después
`ros2 daemon stop && ros2 daemon start`.

### 5.2 La guía de cabecera va en el último cambio de curvatura
La guía no-key que viaja dentro del `NavigateThroughPoses` de cada giro omega va
en el **arranque del tercer arco del Dubins**, no en el medio.

Medido contra Smac Hybrid con radio 4.0 m: una meta a mitad de arco hace saltar
el plan del tramo de 22.5 m a 55–58 m, agregando vueltas enteras. Barrido de
fracciones (16-ago-2026): 0.40–0.80 rompen siempre; 0.20 rompe en 3 de 6
perturbaciones; **0.889 da 12/12 correcto**. Agregar el ápice como segunda guía
lo rompe de nuevo en 4/4. Smac falla en tramos cortos con cambio de rumbo de
~90°, no en el giro completo de 180°.

Antes de mover dónde cae la guía, reproducir el barrido con
`compute_path_through_poses` (`use_start=True`) y comparar contra el nominal. No
confiar en la geometría Dubins exacta, que sí admite la guía en el medio.

### 5.3 `/route_executor/mission_path` está en otro marco
Ese topic sale desplazado **y rotado** respecto del mundo. Me equivoqué una vez
alineándolo solo por traslación y reporté que el vehículo cortaba diagonales y
manejaba 128 m de más. Era falso. Para comparar contra el plan nominal, convertir
con `/fromLL`, que es la verdad de terreno. El desfase de `mission_path` es un
defecto de visualización que ya estaba y sigue sin arreglar.


## 6. Lo que NO está medido

**No hay ninguna prueba real ni documentación de un radio de giro medido en el
vehículo físico.** Lo busqué en todo el repo y en la memoria de codex.

- Los 30° salen del URDF, no de una medición.
- 2.89 / 1.63 / 4.0 son cuentas, no mediciones.
- El perfil real corría `minimum_turning_radius: 1.9` hasta que `3457b65` lo
  subió a 4.0 buscando margen de seguimiento, sin medición que lo respalde.

Antes de bajar el radio en el vehículo real, hay que medirlo de verdad.


## 7. Pendiente

- Probar el cuadrado nuevo en el cockpit con las manos (yo lo verifiqué con el
  arnés, no con el mouse).
- Decidir si `cuadro-campo-arrastre` se mergea a la rama de cobertura.
- En la otra sesión quedaron sin subir: Fields2Cover, zonas no-go y el catálogo
  de fuentes RTK (rama local `feature/zonas-no-go-cobertura` y dos stashes).
- Faltan dos tests del core de RTK.
- Documentación aparte del RTK en `/home/franco/final/estudio-rtk` (no está en
  ningún repo).


## 8. Verificar que algo funciona

```bash
cd /home/franco/campo && ./campo.sh          # levanta todo
./campo.sh chequeo                            # trazado dentro del cuadrado
./campo.sh test                               # colcon test + vitest + tsc
```

El chequeo tiene que imprimir `trazado dentro del cuadrado: SI`. Convierte cada
meta a coordenadas del propio lote (avance/costado desde la esquina de arranque);
en un cuadrado de 20 m las pasadas tienen que caer entre 1 y 19 m, que es el
margen de media cuchilla.
