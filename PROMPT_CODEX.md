# ROS2_SALUS — modo CAMPO (cobertura agricola). Handoff completo.

Sos un agente trabajando en un robot cortacesped/agricola Ackermann con ROS 2
Humble. Este documento es autocontenido: no hay conversacion previa que consultar.

---

## 1. Que es el sistema

Robot Ackermann (no diferencial: NO puede girar en el lugar) con GPS RTK, LiDAR
RS16 y Nav2. Tiene cuatro modalidades de mision, independientes entre si:

- **AUTOMATIC ROUTE** — seguir una lista de waypoints.
- **PATROL** — patrulla estructurada con vuelta a HOME.
- **Goals sueltos** — ir a un punto.
- **CAMPO** — cobertura agricola: barrer un lote entero con un implemento de
  corte, en pasadas paralelas. **Es lo unico que toca este trabajo.**

Parametros fisicos que importan: wheelbase 0.94 m, limite de direccion 30 grados
(radio fisico ~1.63 m), limite operativo del perfil 25 grados (radio ~2.02 m).

---

## 2. La cadena de datos de CAMPO, punta a punta

```
[cockpit React/Vite]  el operador dibuja un poligono, elige ancho de corte,
        |             solape, radio de giro y detalle de preview
        |  WebSocket puerto 8766, op "preview_coverage" / "start_coverage"
        v
[web_zone_server]  (src/map_tools/map_tools/web_zone_server.py)
        |  llama /route_executor/generate_coverage_plan_ll
        v
[route_executor]  _on_generate_coverage_plan -> _generate_coverage_plan_fields2cover
        |  arma la meta y la manda por action
        v
[coverage_server]  OpenNav Coverage / Fields2Cover, overlay en
        |          /opt/salus_coverage_ws. Devuelve swaths (pasadas) + turns (giros)
        v
[route_executor]  traduce a CoverageBodyWaypoint, REEMPLAZA los giros por la
        |          cabecera propia, recorta contra zonas no-go, georreferencia
        v
[web_zone_server]  arma la ruta ejecutable y llama /route_executor/set_route_ll
        v
[route_executor]  trocea en chunks y despacha
        v
[nav_command_server]  /nav_command_server/set_goal_ll
        v
[Nav2]  planner Smac Hybrid + smoother Constrained + controller RPP
        v
[controller_server propio]  traduce cmd_vel a Ackermann (src/controller_server/)
```

Servicios y topicos relevantes:

- `/route_executor/generate_coverage_plan_ll` — `interfaces/srv/GenerateCoveragePlanLL`
- `/route_executor/set_route_ll` — `interfaces/srv/SetRouteMissionLL`
- `/zones_manager/get_state` — `interfaces/srv/GetZonesState` (zonas no-go en GeoJSON)
- `/nav_command_server/set_goal_ll` — `interfaces/srv/SetNavGoalLL`
- Actions de Nav2: `/follow_waypoints`, `/navigate_through_poses`, `/backup`
- `/diagnostics` — por ahi salen los eventos de ruta (`ROUTE_WAYPOINT_ACTION_STARTED`,
  `COVERAGE_BACKUP_DONE`, `COVERAGE_BACKUP_FAILED`, ...)

---

## 3. Mapa de archivos

**Backend** (`/home/franco/final/ROS2_SALUS`, branch `feature/zonas-no-go-cobertura`):

| archivo | que hace |
|---|---|
| `src/navegacion_gps/navegacion_gps/route_executor.py` | ~6200 lineas. El nodo de misiones. CAMPO vive en `_on_generate_coverage_plan`, `_generate_coverage_plan_fields2cover`, `_fill_fields2cover_response`, `_coverage_no_go_polygons`, `_coverage_warmup`, `_fields2cover_planner` |
| `src/navegacion_gps/navegacion_gps/coverage_fields2cover.py` | cliente del Coverage Server con executor propio en un hilo aparte + la geometria de la cabecera |
| `src/navegacion_gps/navegacion_gps/coverage_waypoint_core.py` | `CoverageBodyWaypoint`, `CoveragePlan` y el planificador propio (legacy, zigzag rectangular) |
| `src/navegacion_gps/navegacion_gps/coverage_nogo.py` | recorte contra zonas no-go: borra waypoints adentro e inserta rodeos por el contorno. Idempotente a proposito |
| `src/navegacion_gps/navegacion_gps/coverage_nogo_zones.py` | GeoJSON -> poligonos en marco del cuerpo (`ll_to_body`) |
| `src/navegacion_gps/navegacion_gps/coverage_field_polygon.py` | validacion del poligono del lote |
| `src/map_tools/map_tools/web_zone_server.py` | puente WebSocket con el cockpit + editor de zonas |
| `src/interfaces/srv/GenerateCoveragePlanLL.srv` | contrato del plan de cobertura |
| `src/navegacion_gps/config/nav2_global_v2_{sim,real}_rolling{,_wifi}_params.yaml` | Nav2 de los perfiles global v2 |
| `src/navegacion_gps/launch/sim_global_v2.launch.py` | entrada de simulacion |
| `src/navegacion_gps/launch/nav_global_v2.launch.py` | trae Nav2 + lifecycle managers |

**Cockpit** (`/home/franco/final/ROS2_SALUS/cockpit`, branch `feature/zonas-no-go-cobertura`,
vite dev server ya corriendo con HMR — NO hace falta compilar, toma los cambios solo):

| archivo | que hace |
|---|---|
| `src/packages/nav2/modules/navigation/service/impl/CoverageService.ts` | 81 KB. El panel de CAMPO |
| `src/packages/nav2/modules/navigation/service/impl/coverageNoGo.ts` | 17 KB. Port a TS del recorte no-go: `clipPathToNoGo`, `inflatePolygon`, `detourAlongContour`. El backend ASUME que el cockpit repite el recorte para detectar que el backend no lo aplico |
| `src/packages/nav2/modules/map/frontend/index.tsx` | mapa Leaflet, editor de poligono y de zonas |
| `src/test/{coverageService,coverageNoGo,coveragePolygon}.test.ts` | tests |

**IGNORAR `cockpit-main/`**: es otro checkout, de otro repo (AEyeSecurity/cockpit),
que no usa nadie. El que sirve el dev server es `cockpit/`.

---

## 4. El problema original y por que se llego aca (leer, explica casi todo)

El operador trabaja con ancho de corte 2 m y solape 15%, o sea **separacion entre
pasadas de 1.65-1.7 m**. El radio de giro que pide es 2.9 o 4 m.

Cuando la separacion entre pasadas es **menor que el diametro de giro** (2R), no
existe NINGUNA curva hacia adelante que lleve del final de una pasada al inicio de
la siguiente. Fields2Cover resuelve eso con una **omega**: un lazo enorme que sale
del lote. Medido sobre un lote de 40x40: 23 giros, cada uno acumulando entre 384 y
536 grados de rumbo, con 391 poses de giro y 11 de ellas cayendo DENTRO del lote.
El operador lo ve como "petalos y circulos gigantes y feos" en las cabeceras.

Decisiones historicas y su motivo (varias YA NO APLICAN, ojo):

- **`route_type: SNAKE`** — se puso porque BOUSTROPHEDON recorre pasadas vecinas y
  a 1.65 m con radio 2.9 daba giros de 0.32 m de radio, fisicamente inejecutables.
  Saltear pasadas era la unica forma de agrandar el giro. **Ya no aplica**: ahora
  los giros de F2C se descartan enteros, asi que el route_type solo elige el ORDEN.
  Cambiado a BOUSTROPHEDON (fila por fila, que es como se trabaja un lote).
- **`path_type: DUBIN` y no REEDS_SHEPP** — Reeds-Shepp mete marcha atras y cuspides
  en las cabeceras. Sigue en DUBIN, pero ver el punto de la marcha atras mas abajo.
- **`generate_headland: false`** — el lote entero es superficie de trabajo; reservar
  una banda interior achicaria el area cubierta.
- **Smac `minimum_turning_radius: 4.0`** — estaba puesto para que entrara la cabecera
  de cobertura hacia adelante. **Ya no aplica**: bajado a 2.9.
- **Coverage Server hay que reciclarle el lifecycle** para que relea parametros: arma
  el objeto de robot al configurarse y no vuelve a mirarlos. Medido: con radio 2.9
  seteado por parametro pero sin reciclar, los giros salian con radio 0.32 m.
- **El cliente de Fields2Cover vive en su propio nodo con su propio executor** en un
  hilo aparte. No es paranoia: bloquear dentro de un callback de servicio esperando
  otro servicio se va a timeout justo cuando hay trafico (siempre que el cockpit
  esta conectado), y planificar con F2C tarda segundos.

**Prioridad absoluta del operador**: que los surcos DENTRO del poligono se recorran
exactos. Lo que pase afuera del lote en la cabecera no le importa la precision.

---

## 5. Lo que YA esta hecho (no rehacer)

1. `coverage_fields2cover.py::replace_turns_with_flexible_headlands(plan, margin_m, *, min_turning_radius_m)`:
   descarta TODOS los giros de Fields2Cover y arma la cabecera propia. Los waypoints
   `phase="row"` NO se tocan (hay test que lo fija). Pura e idempotente.
2. **Cabecera de tres puntos con marcha atras**, para cuando la separacion es menor
   que el diametro de giro: `reverse_leg_length_m(R, d) = max(0, 2R - d)`. Con R=4 y
   d=1.7 da 6.3 m de reversa. La maniobra es: arco de 90 grados, reversa recta, otro
   arco de 90 grados. Cierra exacta (error 4e-16). Necesita R metros de cabecera en
   vez de los 7.3 m que pedia el omega.
   Emite **2 guias** por cabecera: el pivote (lleva `backup_m`) y la reentrada.
   Antes emitia 4 y eso causaba lazos: la guia posterior a la reversa queda DETRAS
   del pivote, asi que si la marcha atras no corre, hacia adelante solo se llega
   dando la vuelta.
3. `CoverageBodyWaypoint.backup_m: float = 0.0` en `coverage_waypoint_core.py`.
4. Accion de waypoint **`coverage_backup`** en `route_executor.py`: se parsea
   (`_parse_route_action_json`), se serializa (`_serialize_route_actions`) y se
   ejecuta contra `/backup` (`nav2_msgs/action/BackUp`) en `_run_coverage_backup()`.
   Si falla NO aborta la mision, a proposito: la cabecera es transito afuera del lote.
5. `GenerateCoveragePlanLL.srv`: campo nuevo `string[] route_action_jsons`, alineado
   con `route_lats/lons`. `web_zone_server` lo reenvia como
   `SetRouteMissionLL.waypoint_action_jsons` via la clave `action_json` del waypoint.
   El vertice con `backup_m > 0` esta protegido de la decimacion de guias.
6. Radios de 4.0 a **2.9** en los 4 YAML `nav2_global_v2_*_rolling*.yaml` (Smac,
   smoother, `regulated_linear_scaling_min_radius`) y en
   `coverage_planner_min_turning_radius_m` (route_executor + sim_global_v2.launch.py).
   Es un PISO, no un objetivo: el operador puede pedir 4 y funciona.
7. `coverage_f2c_route_type` default `SNAKE` -> `BOUSTROPHEDON`.
8. `bond_timeout: 20.0` en los dos lifecycle managers de `nav_global_v2.launch.py`.
   Estaba en 4.0 (default de fabrica) y con Gazebo cargando la maquina algun nodo
   tardaba mas, el manager abandonaba la secuencia y Nav2 quedaba con
   `planner_server` en `inactive` y todo lo demas en `unconfigured`. Se manifestaba
   como **"FollowWaypoints action server not available"** al pedir una mision.
9. Tests nuevos: `src/navegacion_gps/test/test_coverage_fields2cover_headlands.py`
   (24 pasan).

**Medido contra el Coverage Server real**, lote 40x40, corte 2 m, solape 0.15, radio 4:

```
24 pasadas, orden 0..23 (fila por fila)     2 guias por transicion
23 marchas atras de 6.30 m                  0 guias dentro del lote
radio minimo de la polilinea: 4.36 m        0 lazos
largo de transiciones: 256 m (el omega crudo daba 626 m)
```

Con una zona no-go real: borra 14 waypoints, mete 4 rodeos, deja 0 adentro.

---

## 6. Entorno de ejecucion

Todo corre en el contenedor **`ros2_salus`**. El workspace es `/ros2_ws`, con el
codigo montado desde el repo (`--symlink-install`, o sea que editar Python NO
requiere recompilar, solo reiniciar el nodo; cambiar un `.srv` SI requiere build).

```bash
# build
docker exec ros2_salus bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-up-to navegacion_gps map_tools --symlink-install'

# tests (correr por paquete: juntar paquetes colisiona en test_copyright.py)
docker exec ros2_salus bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && \
  source install/setup.bash && cd src/navegacion_gps && python3 -m pytest test/ -q'

# levantar la sim
./tools/stop_sim_global_v2.sh
ros2 launch navegacion_gps sim_global_v2.launch.py gps_profile:=f9p_rtk \
  launch_web_app:=True use_keepout:=False
```

El overlay de Fields2Cover esta en `/opt/salus_coverage_ws` — hay que hacerle source
ademas del workspace para hablar con el Coverage Server desde un script.

### Gotchas del runtime (te van a morder)

- **Hay un nodo `/route_executor` fantasma en el grafo**: `ros2 node list` lo muestra
  dos veces y las llamadas a `ros2 param get/set` caen en uno u otro al azar. El
  fantasma responde `Parameter not set` o
  `Invalid access to undeclared parameter(s)`. **Reintentar 5-8 veces** hasta que
  salga; no es que el parametro no exista.
- `ros2 daemon stop && ros2 daemon start` limpia entradas viejas del grafo despues
  de reiniciar nodos.
- Si Nav2 quedo a medio levantar, se rescata a mano sin relanzar:
  ```bash
  ros2 service call /lifecycle_manager_global_navigation_v2/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"   # RESET
  ros2 service call /lifecycle_manager_global_navigation_v2/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"   # STARTUP
  ```
  Verificar con `ros2 lifecycle get /controller_server` (tiene que decir `active`).
- `coverage_f2c_route_type`, `coverage_f2c_path_type` y `coverage_f2c_path_continuity`
  se leen **frescos en cada pedido**: se pueden cambiar con `ros2 param set` sin
  relanzar ni recompilar. Muy util para comparar SNAKE contra BOUSTROPHEDON.

---

## 7. Lo que hay que hacer

### A. BUG PRINCIPAL: los surcos se salen del poligono

Con un poligono de **8 vertices** dibujado en el cockpit (tamano 40 m, ancho de
corte 2, solape 15%, radio 4), el preview muestra surcos que se extienden bastante
mas alla del borde del poligono y se cruzan entre si. El operador lo describe como
"hizo lo que quiso adentro, esta mal mal". El panel reportaba: 24 pasadas, 63 metas
key, trayectoria 1290.9 m, 0 giros omega, 0 conflictos de auditoria.

**NO esta diagnosticado.** Hipotesis a descartar, en orden:

1. Que lo que se ve **no sean surcos sino las transiciones dibujadas como rectas**
   (row_end -> pivote -> reentrada -> row_start). El pivote esta a `lead + R` mas
   alla del extremo del surco y `R` al costado; con R=4 y lead=0.5 son 4.5 m afuera
   y 4 m de lado. Si lo que se ve excede eso por mucho, NO son las transiciones y
   hay que seguir con las otras hipotesis.
2. Que Fields2Cover reciba un poligono distinto del dibujado. Verificar `a_cuerpo()`
   en `_generate_coverage_plan_fields2cover` y `_ring_to_coordinates()` (cierra el
   anillo repitiendo el primer vertice; sin eso F2C devuelve INVALID_COORDS 803).
3. Que `coverage_f2c_swath_angle_deg` (el launch lo fija en **0.0**, no NaN) fuerce
   un angulo de pasada incompatible con un poligono no rectangular. Con NaN, F2C usa
   BRUTE_FORCE y busca el angulo optimo. Se fijo porque la guarda de aproximacion
   compara el rumbo del vehiculo contra el de la primera meta.
4. Que el cambio a BOUSTROPHEDON haya alterado el recorte de swaths. Comparar en
   caliente: `ros2 param set /route_executor coverage_f2c_route_type SNAKE`.

**Como diagnosticarlo**: escribir un probe que pida el plan al servicio vivo con un
octogono y mida cuantos waypoints `phase="row"` caen fuera del poligono y a que
distancia, separado de los `phase="turn"`. Ver seccion 9 para el esqueleto.

### B. `lane_spacing_m` no se reporta en la rama Fields2Cover

El cockpit muestra **"SEPARACION 0.00 m"**. Causa confirmada: el dataclass
`Fields2CoverPlan` solo tiene `waypoints`, `swath_count`, `work_length_m`,
`transition_length_m`, `route_type`, `path_type` — no tiene `lane_spacing_m`, y
`_fill_fields2cover_response` nunca setea `response.lane_spacing_m` (la linea que lo
setea vive en la rama legacy). Agregar el campo, calcularlo en
`plan_to_body_waypoints` (distancia perpendicular entre swaths consecutivos) y
publicarlo. Es dato de reporte: no afecta la geometria, pero el operador lo usa para
decidir y verlo en cero le hace desconfiar de todo el plan.

### C. Tolerancia de llegada flexible para los waypoints de AFUERA

Pedido textual del operador: *"tiene que haber dos waypoints, uno que este dentro del
poligono y otros los que estan afuera; los que estan afuera el radio de acierto debe
ser mas flexible; los unicos que deben ser mas importantes son los de la vuelta"*.

Estado: **la distincion existe** — rol `coverage` adentro, `coverage_transit` afuera,
la asigna `web_zone_server.py` (`"coverage" if waypoint.key else "coverage_transit"`).
`should_follow_exact_coverage_chunk()` exige que TODOS los roles del chunk sean
`coverage` para seguir el path exacto, asi que las cabeceras ya se navegan flexible.

**Lo que NO existe es la tolerancia real.** `coverage_transit_reached_tolerance_m`
(3.0) solo afecta a `skip_reached_chunk_start`, o sea saltear waypoints ya
alcanzados al empezar un chunk. La tolerancia que de verdad decide si una meta se dio
por alcanzada es el goal checker de Nav2: `general_goal_checker.xy_goal_tolerance:
1.2`, global para todos. Con Ackermann, clavar una guia de cabecera a 1.2 m obliga a
maniobrar.

Implementarlo **sin romper el aislamiento** y **sin mandar un `set_parameters` por
cada waypoint** (se probo y se descarto por chatty: serian ~46 llamadas por mision).
Opciones a evaluar: setear el parametro una vez por cabecera y restaurarlo al entrar
al surco (~2 cambios por cabecera), o directamente no mandar las guias como metas
separadas. Hay un helper reusable para parametros remotos:
`route_executor.py::_set_costmap_inflation_parameters`.

### D. Verificar que la marcha atras se ejecute de verdad

**Nunca corrio.** El backend la emite bien (verificado contra el servicio vivo: 23
acciones `coverage_backup` de 6.3 m en `route_action_jsons`) y `web_zone_server` la
reenvia, pero no se observo ningun evento `COVERAGE_BACKUP_DONE` ni
`COVERAGE_BACKUP_FAILED` en `/diagnostics` durante una mision. Confirmar de punta a
punta y arreglar lo que falte. Ojo que el `/backup` de Nav2 va **recto hacia atras**
(sin direccion), que es justo lo que pide la maniobra de tres puntos.

---

## 8. Restricciones NO NEGOCIABLES

1. **AISLAMIENTO**: cobertura entra SOLO por CAMPO. AUTOMATIC ROUTE, PATROL y los
   goals sueltos no pueden importar ni depender de cobertura, ni siquiera al importar
   el modulo. Los imports de cobertura viven DENTRO de los handlers de Campo, nunca
   arriba del archivo. `test/test_coverage_planner_isolation.py` TIENE que seguir
   pasando; si agregas un simbolo de cobertura nuevo, sumalo a `SIMBOLOS_DE_COBERTURA`.
   El motivo es concreto: cuando el overlay de Fields2Cover no estaba instalado, un
   import arriba mataba el nodo ENTERO y se caian con el la ruta automatica y la
   patrulla, que no tienen nada que ver con cobertura.
2. **Ninguna excepcion puede escapar de un callback de rclpy**: mata el nodo y se
   lleva puestas la ruta y la patrulla. Todo lo de CAMPO va envuelto en try/except
   que devuelve `ok=False` con mensaje.
3. **La marcha atras es EXCLUSIVA de Campo.** Smac sigue en
   `motion_model_for_search: DUBIN`, el smoother con `reversing_enabled: false` y RPP
   con `allow_reversing: false`. NO habilitar reversa global en Nav2: el operador
   eligio explicitamente que Ruta, Patrol y goals no puedan retroceder ni por
   accidente. La reversa se hace con una accion puntual y acotada.
4. **El recorte de zonas no-go tiene que seguir corriendo DESPUES de la cabecera y
   sin cambios.** Ademas el cockpit **repite el mismo recorte** sobre lo que llega
   para detectar que el backend no lo aplico; si el backend hace otra cosa, los dos
   dibujos no coinciden y el arranque queda bloqueado por una diferencia que no es un
   problema real.
5. **Los waypoints `phase="row"` no se tocan.** Es lo unico que el operador necesita
   exacto.
6. No hacer cambios de estilo ni fuera del alcance de cobertura.

---

## 9. Como validar

Ademas de los tests, hay que medir contra el sistema real. Esqueleto de probe (correr
dentro del contenedor con los tres source):

```python
import math, rclpy
from rclpy.node import Node
from interfaces.srv import GenerateCoveragePlanLL
from interfaces.msg import GeoRing, NoGoPoint

LAT0, LON0 = -31.485802, -64.241050
MLAT = 111320.0; MLON = 111320.0 * math.cos(math.radians(LAT0))
v = lambda x, y: NoGoPoint(lat=LAT0 + y/MLAT, lon=LON0 + x/MLON)
xy = lambda la, lo: ((lo-LON0)*MLON, (la-LAT0)*MLAT)

rclpy.init(); n = Node("probe")
c = n.create_client(GenerateCoveragePlanLL, "/route_executor/generate_coverage_plan_ll")
c.wait_for_service(timeout_sec=20.0)
r = GenerateCoveragePlanLL.Request()
r.start_lat, r.start_lon, r.start_yaw_deg = LAT0, LON0, 0.0
r.field_length_m = r.field_width_m = 40.0
r.cutter_width_m, r.overlap_ratio = 2.0, 0.15
r.min_turning_radius_m, r.waypoint_spacing_m = 4.0, 2.0
r.side = "left"
octo = [(25*math.cos(2*math.pi*i/8), 25*math.sin(2*math.pi*i/8)) for i in range(8)]
ring = GeoRing(); ring.vertices = [v(x, y) for x, y in octo]
r.coverage_polygon = ring
f = c.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=120.0)
res = f.result()
# medir: cuantos sampled_phases=="row" caen fuera de `octo` y a que distancia,
# separado de los "turn". Ahi esta la respuesta al bug A.
```

Tambien se puede saltear el route_executor y hablar directo con el Coverage Server
instanciando `Fields2CoverPlanner` de `coverage_fields2cover.py`, que es util para
aislar si el problema es de F2C o del post-proceso.

---

## 10. Estado de git

Hay **trabajo sin commitear** en las dos branches (backend y cockpit). No commitear
ni pushear salvo que el usuario lo pida. Hay respaldos en
`.respaldo-pull-20260819-153515/` (parches de ambos repos).

Convencion del equipo: una feature por branch con nombre descriptivo; cockpit y
backend van en dos branches que se mergean juntas.

---

## 11. Fallas preexistentes, ajenas a este trabajo (no las persigas, no son tuyas)

- `test_nav_command_server_loop_helper.py`: **9 tests fallan** porque `_FakeLoopNode`
  no tiene `_nav_action_results_to_ignore`. Viene de otro cambio en
  `nav_command_server.py` en esta misma branch.
- `test_flake8` y `test_pep257` fallan en todo el repo desde antes: 34 E501 en
  `route_executor.py` y D213 masivo (el repo entero usa ese estilo de docstring).
  Verificar que TU cambio no sume errores nuevos comparando contra
  `git show HEAD:<archivo>`, no que el test pase.
