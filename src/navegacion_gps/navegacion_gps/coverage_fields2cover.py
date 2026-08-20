"""Puente entre el modo Campo de SALUS y el Coverage Server de OpenNav.

Fields2Cover planifica sobre el poligono real del lote y trata las exclusiones
como anillos interiores, asi que las pasadas que cruzan una exclusion salen
partidas de fabrica en vez de parchearse despues.

Este modulo hace tres cosas y nada mas: arma la meta de la accion, la manda, y
traduce el resultado al mismo ``CoverageBodyWaypoint`` que ya produce el
planificador propio. Todo lo que viene despues de ``route_executor`` —el
troceado de la ruta, ``nav_command_server``, Nav2, el control Ackermann— no se
entera de que cambio el planificador.

Trabaja en el marco del cuerpo del lote, en metros: se le manda al server
coordenadas cartesianas y vuelven en el mismo marco, asi que la
georreferenciacion sigue siendo la de siempre.

**Executor propio.** El cliente vive en su propio nodo con su propio executor en
un hilo aparte. No es paranoia: el executor del ``route_executor`` tiene dos
hilos, y ya se midio que bloquear dentro de un callback de servicio esperando
otro servicio se va a timeout justo cuando hay trafico —que es siempre que el
cockpit esta conectado—. Planificar con Fields2Cover tarda segundos, o sea
bastante mas que aquella llamada que fallaba.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

try:
    from opennav_coverage_msgs.action import ComputeCoveragePath
    from opennav_coverage_msgs.msg import Coordinate, Coordinates
    _IMPORT_ERROR = ""
except ImportError as _exc:  # pragma: no cover - depende del overlay instalado
    # Importar esto arriba y sin red mataba al nodo ENTERO cuando el overlay de
    # Fields2Cover no estaba: route_executor no llegaba ni a arrancar y se caian
    # con el la ruta automatica, la patrulla y los goals, que no tienen nada que
    # ver con cobertura. El modulo tiene que poder importarse siempre; lo unico
    # que puede fallar es planificar Campo con fields2cover, y falla con un
    # mensaje que dice que instalar.
    ComputeCoveragePath = None  # type: ignore[assignment]
    Coordinate = None  # type: ignore[assignment]
    Coordinates = None  # type: ignore[assignment]
    _IMPORT_ERROR = str(_exc)

from navegacion_gps.coverage_waypoint_core import CoverageBodyWaypoint

Point = Tuple[float, float]

# Fases, iguales a las del planificador propio para que nada aguas abajo tenga
# que aprender vocabulario nuevo.
WORK_PHASE = "row"
TRANSITION_PHASE = "turn"


class Fields2CoverError(RuntimeError):
    """Falla al planificar con el Coverage Server.

    Siempre se atrapa en el llamador: una excepcion que escape de un callback de
    servicio de rclpy mata el nodo entero, y eso dejaria sin ruta y sin patrulla
    a alguien que solo queria un preview de cobertura.
    """


@dataclass
class Fields2CoverPlan:
    """Resultado ya traducido al vocabulario de SALUS."""

    waypoints: List[CoverageBodyWaypoint] = field(default_factory=list)
    swath_count: int = 0
    lane_spacing_m: float = 0.0
    work_length_m: float = 0.0
    transition_length_m: float = 0.0
    route_type: str = ""
    path_type: str = ""

    @property
    def total_length_m(self) -> float:
        """Largo de trabajo mas largo de transiciones."""
        return float(self.work_length_m + self.transition_length_m)


def _yaw_from_quaternion(orientation: Any) -> float:
    """Rumbo en grados a partir del cuaternion de una pose."""
    z = float(orientation.z)
    w = float(orientation.w)
    x = float(orientation.x)
    y = float(orientation.y)
    siny = 2.0 * ((w * z) + (x * y))
    cosy = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.degrees(math.atan2(siny, cosy))


def _sample_segment(start: Point, end: Point, spacing_m: float) -> List[Point]:
    """Puntos sobre el segmento, incluidos los dos extremos."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(length / max(0.05, float(spacing_m)))))
    return [
        (
            start[0] + ((end[0] - start[0]) * (index / steps)),
            start[1] + ((end[1] - start[1]) * (index / steps)),
        )
        for index in range(steps + 1)
    ]


def _lane_spacing_from_swaths(swaths: Sequence[Any]) -> float:
    """Separacion nominal entre las lineas paralelas de trabajo.

    Se ordenan los offsets geometricos en vez de usar el orden de visita. Asi
    el reporte sigue dando el ancho entre surcos cuando ``SNAKE`` saltea filas
    o una exclusion parte una misma linea en dos swaths.
    """
    swath_list = list(swaths)
    if len(swath_list) < 2:
        return 0.0

    reference = None
    for swath in swath_list:
        dx = float(swath.end.x) - float(swath.start.x)
        dy = float(swath.end.y) - float(swath.start.y)
        length = math.hypot(dx, dy)
        if length > 1.0e-9:
            reference = (-dy / length, dx / length)
            break
    if reference is None:
        return 0.0

    offsets = sorted(
        (
            (0.5 * (float(swath.start.x) + float(swath.end.x)) * reference[0])
            + (0.5 * (float(swath.start.y) + float(swath.end.y)) * reference[1])
        )
        for swath in swath_list
    )
    unique_offsets: List[float] = []
    for offset in offsets:
        if not unique_offsets or abs(offset - unique_offsets[-1]) > 1.0e-6:
            unique_offsets.append(float(offset))
    gaps = sorted(
        unique_offsets[index + 1] - unique_offsets[index]
        for index in range(len(unique_offsets) - 1)
        if unique_offsets[index + 1] - unique_offsets[index] > 1.0e-6
    )
    if not gaps:
        return 0.0
    middle = len(gaps) // 2
    if len(gaps) % 2:
        return float(gaps[middle])
    return float(0.5 * (gaps[middle - 1] + gaps[middle]))


def fields2cover_disponible() -> bool:
    """Si el overlay de Fields2Cover esta instalado en este entorno."""
    return not _IMPORT_ERROR


def _ring_to_coordinates(ring: Sequence[Point]) -> Any:
    """Anillo en metros locales al tipo que espera la accion.

    Se cierra repitiendo el primer vertice al final. SALUS maneja los anillos
    abiertos, pero Fields2Cover los pasa a OGR y ahi un anillo sin cerrar es
    invalido: la accion responde INVALID_COORDS (803) sin mas explicacion.
    """
    puntos = [(float(x), float(y)) for x, y in ring]
    if puntos and puntos[0] != puntos[-1]:
        puntos.append(puntos[0])
    out = Coordinates()
    out.coordinates = [Coordinate(axis1=x, axis2=y) for x, y in puntos]
    return out


def plan_to_body_waypoints(
    result: Any,
    *,
    waypoint_spacing_m: float,
) -> Fields2CoverPlan:
    """Traducir el resultado de la accion a waypoints de SALUS.

    Se usa ``coverage_path`` y no ``nav_path`` porque el primero viene separado
    en ``swaths`` (trabajo) y ``turns`` (transiciones). Esa separacion es la que
    despues necesita el implemento para saber cuando cortar, y la que permite
    medir trabajo y giros por separado. El ``nav_path`` es la misma trayectoria
    pero ya aplanada, sin decir que es cada tramo.
    """
    components = result.coverage_path
    swaths = list(components.swaths)
    turns = list(components.turns)
    if not swaths:
        raise Fields2CoverError("Fields2Cover no devolvio ninguna pasada")

    plan = Fields2CoverPlan(
        swath_count=len(swaths),
        lane_spacing_m=_lane_spacing_from_swaths(swaths),
    )
    waypoints: List[CoverageBodyWaypoint] = []

    for index, swath in enumerate(swaths):
        start = (float(swath.start.x), float(swath.start.y))
        end = (float(swath.end.x), float(swath.end.y))
        plan.work_length_m += math.hypot(end[0] - start[0], end[1] - start[1])
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        muestras = _sample_segment(start, end, waypoint_spacing_m)
        for sample_index, (x, y) in enumerate(muestras):
            # Solo los extremos de pasada son metas de parada, igual que en el
            # planificador propio: los intermedios son para dibujar y auditar.
            es_extremo = sample_index in (0, len(muestras) - 1)
            waypoints.append(
                CoverageBodyWaypoint(
                    forward_m=float(x),
                    left_m=float(y),
                    yaw_delta_deg=float(heading),
                    phase=WORK_PHASE,
                    row_index=int(index),
                    is_key=bool(es_extremo),
                )
            )

        if index < len(turns):
            poses = list(turns[index].poses)
            previa: Optional[Point] = end
            for pose in poses:
                punto = (float(pose.pose.position.x), float(pose.pose.position.y))
                if previa is not None:
                    plan.transition_length_m += math.hypot(
                        punto[0] - previa[0], punto[1] - previa[1]
                    )
                previa = punto
                waypoints.append(
                    CoverageBodyWaypoint(
                        forward_m=punto[0],
                        left_m=punto[1],
                        yaw_delta_deg=_yaw_from_quaternion(pose.pose.orientation),
                        phase=TRANSITION_PHASE,
                        row_index=int(index),
                        is_key=False,
                        # La curva no es una meta de trabajo, pero si una guia
                        # obligatoria: sin ella el ejecutor une dos extremos de
                        # pasada con una recta y no sigue el preview de F2C.
                        is_guide=True,
                    )
                )

    plan.waypoints = waypoints
    return plan


# Tramo recto que despeja el implemento antes y despues de los dos arcos.
DEFAULT_HEADLAND_LEAD_M = 0.5

# Distancia por debajo de la cual dos puntos son el mismo y no se agrega guia.
_EPSILON_M = 1.0e-6


def _direction_from_yaw(yaw_deg: float) -> Point:
    """Versor a partir de un rumbo en grados."""
    radianes = math.radians(float(yaw_deg))
    return (math.cos(radianes), math.sin(radianes))


def _unit(desde: Point, hasta: Point) -> Optional[Point]:
    """Versor de ``desde`` a ``hasta``, o None si son el mismo punto."""
    dx = float(hasta[0]) - float(desde[0])
    dy = float(hasta[1]) - float(desde[1])
    largo = math.hypot(dx, dy)
    if largo <= _EPSILON_M:
        return None
    return (dx / largo, dy / largo)


def _work_runs(
    waypoints: Sequence[CoverageBodyWaypoint],
) -> List[List[CoverageBodyWaypoint]]:
    """Tramos contiguos de trabajo, en orden de visita.

    Se agrupa por contiguidad y no por ``row_index`` a proposito: el orden de la
    lista ES el orden de recorrido, y agrupar por indice mezclaria dos visitas a
    la misma pasada si alguna vez las hubiera.
    """
    runs: List[List[CoverageBodyWaypoint]] = []
    actual: List[CoverageBodyWaypoint] = []
    for waypoint in waypoints:
        if str(waypoint.phase) == WORK_PHASE:
            actual.append(waypoint)
            continue
        if actual:
            runs.append(actual)
            actual = []
    if actual:
        runs.append(actual)
    return runs


def _exit_direction(run: Sequence[CoverageBodyWaypoint]) -> Point:
    """Direccion con la que se abandona la pasada."""
    ultimo = run[-1]
    fin = (float(ultimo.forward_m), float(ultimo.left_m))
    for previo in reversed(run[:-1]):
        versor = _unit((float(previo.forward_m), float(previo.left_m)), fin)
        if versor is not None:
            return versor
    return _direction_from_yaw(ultimo.yaw_delta_deg)


def _entry_direction(run: Sequence[CoverageBodyWaypoint]) -> Point:
    """Direccion con la que se entra a la pasada."""
    primero = run[0]
    inicio = (float(primero.forward_m), float(primero.left_m))
    for siguiente in run[1:]:
        versor = _unit(inicio, (float(siguiente.forward_m), float(siguiente.left_m)))
        if versor is not None:
            return versor
    return _direction_from_yaw(primero.yaw_delta_deg)


# Cabecera de tres puntos. Cuando las pasadas quedan mas juntas que el diametro
# de giro (2R), NO existe ninguna curva hacia adelante que lleve del final de un
# surco al inicio del siguiente: Fields2Cover resuelve eso con una omega, que se
# come 7.3 m de cabecera y dibuja los petalos. Retrocediendo, la misma
# transicion entra en R metros de cabecera.
#
# La maniobra, con R el radio y d la separacion entre pasadas:
#
#   1. salir derecho del surco `lead` metros          (despeja el implemento)
#   2. arco de 90 grados hacia el surco siguiente
#   3. marcha atras recta de L = 2R - d metros
#   4. otro arco de 90 grados, que deja el rumbo invertido
#   5. entrar derecho al surco siguiente `lead` metros
#
# L sale de cerrar la geometria: los dos arcos desplazan 2R de costado, y la
# reversa descuenta lo que sobra hasta la separacion real. Si d >= 2R la cuenta
# da L <= 0, que es justo el caso en que el giro hacia adelante SI existe y no
# hace falta retroceder.
REVERSE_ARC_DEG = 90.0


def _rotate(vector: Point, degrees: float) -> Point:
    """Rotar un versor en el plano."""
    radianes = math.radians(float(degrees))
    coseno = math.cos(radianes)
    seno = math.sin(radianes)
    return (
        (vector[0] * coseno) - (vector[1] * seno),
        (vector[0] * seno) + (vector[1] * coseno),
    )


def reverse_leg_length_m(min_turning_radius_m: float, lane_spacing_m: float) -> float:
    """Metros de marcha atras que pide la cabecera de tres puntos.

    Cero cuando las pasadas estan mas separadas que el diametro de giro: ahi el
    giro hacia adelante existe y no hay nada que retroceder.
    """
    radio = max(0.0, float(min_turning_radius_m))
    separacion = abs(float(lane_spacing_m))
    return max(0.0, (2.0 * radio) - separacion)


def _three_point_turn(
    fin: Point,
    salida: Point,
    inicio: Point,
    *,
    radio_m: float,
    lead_m: float,
) -> Optional[Tuple[List[Tuple[Point, Point, float]], float]]:
    """Vertices de la cabecera de tres puntos, o None si no hace falta.

    Devuelve ``[(punto, versor_de_rumbo, marcha_atras_m), ...]`` y el largo
    recorrido. La marcha atras viaja en el vertice donde hay que hacerla.
    """
    normal = (-salida[1], salida[0])
    hacia = (inicio[0] - fin[0], inicio[1] - fin[1])
    lateral = (hacia[0] * normal[0]) + (hacia[1] * normal[1])
    separacion = abs(lateral)
    reversa = reverse_leg_length_m(radio_m, separacion)
    if reversa <= _EPSILON_M:
        return None

    # Se dobla hacia donde esta el surco siguiente.
    sentido = 1.0 if lateral >= 0.0 else -1.0
    giro = (sentido * normal[0], sentido * normal[1])

    salida_arranque = (
        fin[0] + (salida[0] * lead_m),
        fin[1] + (salida[1] * lead_m),
    )
    # Fin del primer arco: R adelante y R al costado, con el rumbo ya girado 90.
    pivote = (
        salida_arranque[0] + (radio_m * salida[0]) + (radio_m * giro[0]),
        salida_arranque[1] + (radio_m * salida[1]) + (radio_m * giro[1]),
    )
    rumbo_pivote = giro
    tras_reversa = (
        pivote[0] - (reversa * rumbo_pivote[0]),
        pivote[1] - (reversa * rumbo_pivote[1]),
    )
    # Fin del segundo arco: R hacia atras del eje del surco y R mas al costado.
    reentrada = (
        tras_reversa[0] - (radio_m * salida[0]) + (radio_m * giro[0]),
        tras_reversa[1] - (radio_m * salida[1]) + (radio_m * giro[1]),
    )
    rumbo_reentrada = (-salida[0], -salida[1])

    # Dos guias y nada mas. Las otras dos que salian de la construccion no
    # aportan y una hacia dano:
    #
    #   salida_arranque  el vehiculo ya viene con ese rumbo al terminar el
    #                    surco, asi que pedirle que pase por ahi no agrega nada.
    #   tras_reversa     queda DETRAS del pivote. Si la marcha atras no llega a
    #                    correr, hacia adelante solo se llega dando un lazo: son
    #                    los circulos que aparecian en el preview.
    #
    # Afuera del lote la precision no importa, asi que cuantos menos puntos haya
    # que clavar, menos maniobra. El pivote se conserva porque ahi va la marcha
    # atras; la reentrada, porque alinea el vehiculo con el surco siguiente.
    vertices = [
        (pivote, rumbo_pivote, float(reversa)),
        (reentrada, rumbo_reentrada, 0.0),
    ]
    # Largo real: los arcos son cuartos de circunferencia, no cuerdas.
    arcos = 2.0 * (math.pi * radio_m / 2.0)
    recorrido = (2.0 * lead_m) + arcos + reversa
    recorrido += math.hypot(
        inicio[0] - reentrada[0] - (salida[0] * -lead_m),
        inicio[1] - reentrada[1] - (salida[1] * -lead_m),
    )
    return vertices, recorrido


def replace_turns_with_flexible_headlands(
    plan: Fields2CoverPlan,
    margin_m: float,
    *,
    min_turning_radius_m: float = 0.0,
) -> Fields2CoverPlan:
    """Cambiar los giros de Fields2Cover por transiciones exteriores simples.

    Fields2Cover resuelve la cabecera con curvas Dubins: cuando la separacion
    entre pasadas es menor que dos veces el radio minimo —2.0 m de ancho con 15%
    de solape dan 1.7 m, contra 4 m de radio— la unica solucion posible es una
    omega, y el preview se llena de petalos gigantes fuera del lote. Al operador
    no le importa por donde pasa el vehiculo afuera del lote: le importa que los
    surcos se recorran exactos. Asi que el surco se deja tal cual y solo se
    reemplaza la cabecera por tres tramos rectos:

        salir derecho por el eje de la pasada -> desplazarse afuera ->
        reentrar derecho por el eje de la siguiente

    Los dos vertices de ese recorrido quedan como guias no-key: el ejecutor las
    respeta como puntos de paso, pero Nav2 planifica libremente entre ellas, que
    es exactamente la flexibilidad que se busca afuera del lote. Los waypoints
    de trabajo no se tocan.

    ``margin_m`` es el tramo recto que despeja el implemento antes y despues
    de los arcos (ver ``DEFAULT_HEADLAND_LEAD_M``).

    La funcion es pura e idempotente: no toca el plan que recibe y aplicarla dos
    veces da lo mismo, porque vuelve a derivar las guias de los surcos, que son
    los que nunca cambian.

    Nota de alcance: el tramo exterior se traza recto entre las dos guias. En un
    lote muy concavo ese tramo podria rozar el poligono; se acepta a proposito,
    porque el recorte de zonas no-go corre despues y es el que decide por donde
    no se puede pasar.
    """
    margen = max(0.0, float(margin_m))
    radio = max(0.0, float(min_turning_radius_m))
    runs = _work_runs(plan.waypoints)
    if not runs:
        return replace(plan, waypoints=list(plan.waypoints))

    waypoints: List[CoverageBodyWaypoint] = []
    transicion_m = 0.0
    for indice, run in enumerate(runs):
        waypoints.extend(run)
        if indice + 1 >= len(runs):
            continue

        siguiente = runs[indice + 1]
        salida = _exit_direction(run)
        entrada = _entry_direction(siguiente)
        fin = (float(run[-1].forward_m), float(run[-1].left_m))
        inicio = (float(siguiente[0].forward_m), float(siguiente[0].left_m))

        tres_puntos = (
            _three_point_turn(fin, salida, inicio, radio_m=radio, lead_m=margen)
            if radio > 0.0
            else None
        )
        if tres_puntos is not None:
            vertices = tres_puntos[0]
        else:
            # Separacion mayor que el diametro de giro, o reversa apagada: el
            # giro hacia adelante existe y alcanza con salir, correrse y entrar.
            vertices = [
                (
                    (fin[0] + (salida[0] * margen), fin[1] + (salida[1] * margen)),
                    salida,
                    0.0,
                ),
                (
                    (
                        inicio[0] - (entrada[0] * margen),
                        inicio[1] - (entrada[1] * margen),
                    ),
                    entrada,
                    0.0,
                ),
            ]

        row_index = int(run[-1].row_index)
        previo = fin
        for punto, versor, reversa_m in vertices:
            largo = math.hypot(punto[0] - previo[0], punto[1] - previo[1])
            hasta_inicio = math.hypot(punto[0] - inicio[0], punto[1] - inicio[1])
            if largo <= _EPSILON_M or hasta_inicio <= _EPSILON_M:
                # Con margen nulo la guia cae encima de un extremo de surco, y un
                # waypoint repetido es una meta de largo cero aguas abajo.
                continue
            transicion_m += largo
            previo = punto
            waypoints.append(
                CoverageBodyWaypoint(
                    forward_m=float(punto[0]),
                    left_m=float(punto[1]),
                    yaw_delta_deg=float(math.degrees(math.atan2(versor[1], versor[0]))),
                    phase=TRANSITION_PHASE,
                    row_index=row_index,
                    is_key=False,
                    is_guide=True,
                    backup_m=float(reversa_m),
                )
            )
        transicion_m += math.hypot(inicio[0] - previo[0], inicio[1] - previo[1])

    return replace(plan, waypoints=waypoints, transition_length_m=float(transicion_m))


class Fields2CoverPlanner:
    """Cliente del Coverage Server con executor propio."""

    def __init__(
        self,
        *,
        action_name: str = "compute_coverage_path",
        parameter_service: str = "/coverage_server/set_parameters",
        change_state_service: str = "/coverage_server/change_state",
        node_name: str = "route_executor_fields2cover_client",
        logger: Any = None,
    ) -> None:
        """Levantar el nodo cliente y su executor en un hilo aparte."""
        if _IMPORT_ERROR:
            raise Fields2CoverError(
                "el overlay de Fields2Cover no esta instalado en este entorno "
                f"({_IMPORT_ERROR}); source del workspace de opennav_coverage "
                "antes de lanzar, o coverage_planner:=legacy"
            )
        self._logger = logger
        # Este cliente vive dentro del proceso route_executor, cuyo launch
        # agrega ``__node:=route_executor``. Si hereda los argumentos globales,
        # el helper tambien se renombra /route_executor y sus servicios de
        # parametros compiten con los del nodo real (respuestas vacias al azar).
        self._node = rclpy.create_node(node_name, use_global_arguments=False)
        self._action = ActionClient(self._node, ComputeCoveragePath, action_name)
        self._parameters = self._node.create_client(SetParameters, parameter_service)
        self._change_state = self._node.create_client(ChangeState, change_state_service)
        # Ultimos parametros fisicos aplicados de verdad, para no reciclar el
        # servidor en cada pedido.
        self._applied: Dict[str, float] = {}
        # Warmup, previews de dos clientes y cambios de parametros comparten un
        # unico Coverage Server lifecycle. Las transiciones no son idempotentes:
        # dos CONFIGURE concurrentes dejan a uno rechazado aunque el otro haya
        # funcionado. Serializar tambien la accion evita reciclar el robot
        # mientras otro pedido todavia esta calculando con esos parametros.
        self._server_lock = threading.RLock()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, name=node_name, daemon=True
        )
        self._thread.start()
        # Levantar el Coverage Server ahora y no en el primer pedido: viene del
        # lanzamiento en `unconfigured`, y configurarlo tarda mas que el timeout
        # del cockpit. Sin esto el primer preview de la sesion falla siempre y
        # el segundo anda, que es la peor forma de andar. En un hilo aparte para
        # no demorar el arranque del route_executor.
        threading.Thread(
            target=self._warmup, name=f"{node_name}_warmup", daemon=True
        ).start()

    def _warmup(self) -> None:
        """Dejar el Coverage Server activo, si es que hay uno."""
        try:
            with self._server_lock:
                if self.available(timeout_s=10.0):
                    return
                self._cycle_server(10.0)
                # Los parametros del vehiculo todavia no se saben —dependen del
                # pedido—, asi que el primero que llegue va a reciclar igual.
                self._applied = {}
            if self._logger is not None:
                self._logger.info("Coverage Server activado desde el arranque")
        except Exception as exc:  # pragma: no cover - depende del entorno
            if self._logger is not None:
                self._logger.warning(f"no se pudo activar el Coverage Server: {exc}")

    def shutdown(self) -> None:
        """Bajar el executor y el nodo cliente."""
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:  # pragma: no cover - solo en el apagado
            pass

    def available(self, timeout_s: float = 2.0) -> bool:
        """Decir si el Coverage Server esta escuchando."""
        return bool(self._action.wait_for_server(timeout_sec=float(timeout_s)))

    def _cycle_server(self, timeout_s: float) -> None:
        """Reciclar el Coverage Server para que relea los parametros.

        Sin esto los parametros del vehiculo no tienen efecto: el servidor arma
        su objeto de robot al configurarse y no vuelve a mirarlos. Medido: con
        radio minimo 2.9 m seteado por parametro pero sin reciclar, los giros
        salian con radio de 0.32 m —fisicamente inejecutables— porque adentro
        seguia el valor de la configuracion anterior. Despues del ciclo, 2.90 m
        exactos y ningun punto por debajo.
        """
        if not self._change_state.wait_for_service(timeout_sec=float(timeout_s)):
            raise Fields2CoverError(
                "el Coverage Server no expone change_state; no se pueden "
                "aplicar los parametros del vehiculo"
            )
        # Bajar solo tiene sentido si el server ya estaba arriba. Recien
        # lanzado esta en `unconfigured` y ahi DEACTIVATE y CLEANUP se rechazan
        # —correctamente—, asi que exigirlas dejaria a Campo sin funcionar
        # cuando el server viene del lanzamiento y nadie lo activo a mano.
        # Configurar y activar, en cambio, tienen que salir si o si.
        transiciones = (
            (Transition.TRANSITION_DEACTIVATE, False),
            (Transition.TRANSITION_CLEANUP, False),
            (Transition.TRANSITION_CONFIGURE, True),
            (Transition.TRANSITION_ACTIVATE, True),
        )
        for transicion, obligatoria in transiciones:
            request = ChangeState.Request()
            request.transition.id = int(transicion)
            future = self._change_state.call_async(request)
            if not self._wait(future, timeout_s):
                raise Fields2CoverError(
                    "timeout reciclando el Coverage Server para aplicar los "
                    "parametros del vehiculo"
                )
            respuesta = future.result()
            if respuesta is None or not bool(respuesta.success):
                if obligatoria:
                    raise Fields2CoverError(
                        f"el Coverage Server rechazo la transicion {int(transicion)}"
                    )

    def _push_parameters(self, values: Dict[str, float], timeout_s: float) -> None:
        """Mandarle al server los parametros fisicos del vehiculo."""
        if not self._parameters.wait_for_service(timeout_sec=float(timeout_s)):
            raise Fields2CoverError(
                "el Coverage Server no expone set_parameters; "
                "revisa que este activo"
            )
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=str(name),
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                ),
            )
            for name, value in values.items()
        ]
        future = self._parameters.call_async(request)
        if not self._wait(future, timeout_s):
            raise Fields2CoverError("timeout mandando parametros al Coverage Server")
        response = future.result()
        fallidos = [
            request.parameters[index].name
            for index, result in enumerate(response.results)
            if not result.successful
        ]
        if fallidos:
            raise Fields2CoverError(
                f"el Coverage Server rechazo los parametros {fallidos}"
            )
        # Solo se recicla cuando algo cambio de verdad: el ciclo tarda y no
        # tiene sentido pagarlo en cada preview con los mismos numeros.
        if any(
            abs(float(values[name]) - float(self._applied.get(name, float("nan"))))
            > 1.0e-9
            or name not in self._applied
            for name in values
        ):
            self._cycle_server(timeout_s)
            self._applied = {name: float(value) for name, value in values.items()}

    @staticmethod
    def _wait(future: Any, timeout_s: float) -> bool:
        """Esperar un future que resuelve el executor propio."""
        evento = threading.Event()
        future.add_done_callback(lambda _: evento.set())
        return bool(evento.wait(timeout=float(timeout_s)))

    def _plan_locked(
        self,
        *,
        polygon_body: Sequence[Point],
        exclusions_body: Sequence[Sequence[Point]] = (),
        cutter_width_m: float,
        robot_width_m: float,
        overlap_ratio: float,
        min_turning_radius_m: float,
        waypoint_spacing_m: float,
        swath_angle_deg: Optional[float] = None,
        route_type: str = "BOUSTROPHEDON",
        path_type: str = "DUBIN",
        path_continuity: str = "CONTINUOUS",
        turn_point_distance_m: float = 0.5,
        headland_width_m: float = 0.0,
        server_timeout_s: float = 30.0,
    ) -> Fields2CoverPlan:
        """Planificar el lote y devolver los waypoints en marco del cuerpo."""
        if len(polygon_body) < 3:
            raise Fields2CoverError("el poligono del lote necesita al menos 3 vertices")
        if not self.available(timeout_s=min(5.0, server_timeout_s)):
            # Recien lanzado, el Coverage Server esta en `unconfigured` y su
            # action server todavia no existe: nadie lo activo. Antes de darlo
            # por caido se lo intenta levantar, que es lo que el lanzamiento
            # espera que pase en el primer pedido de CAMPO. Si tampoco asi
            # aparece, entonces si no esta.
            try:
                self._cycle_server(min(5.0, server_timeout_s))
                self._applied = {}
            except Fields2CoverError:
                pass
            if not self.available(timeout_s=min(5.0, server_timeout_s)):
                raise Fields2CoverError(
                    "el Coverage Server no responde; revisa que opennav_coverage "
                    "este corriendo y activo"
                )

        # El solape se traduce a un ancho de operacion menor, que es como lo
        # expresa Fields2Cover. Da la misma separacion entre pasadas que la
        # cuenta del planificador propio: ancho * (1 - solape).
        operation_width = float(cutter_width_m) * (1.0 - float(overlap_ratio))
        if operation_width <= 0.0:
            raise Fields2CoverError("el solape deja un ancho de trabajo nulo")
        self._push_parameters(
            {
                "operation_width": operation_width,
                "robot_width": float(robot_width_m),
                "min_turning_radius": float(min_turning_radius_m),
            },
            timeout_s=min(5.0, server_timeout_s),
        )

        goal = ComputeCoveragePath.Goal()
        goal.use_gml_file = False
        goal.frame_id = "map"
        goal.polygons = [_ring_to_coordinates(polygon_body)] + [
            _ring_to_coordinates(ring) for ring in exclusions_body
        ]
        # Sin cabecera interior: el lote entero es superficie de trabajo y los
        # giros pueden salirse. Reservar una banda interior achicaria el area
        # cubierta, que es justo lo que no se quiere.
        goal.generate_headland = float(headland_width_m) > 0.0
        goal.headland_mode.width = float(headland_width_m)
        goal.generate_route = True
        goal.generate_path = True
        goal.swath_mode.mode = "BRUTE_FORCE" if swath_angle_deg is None else "SET_ANGLE"
        goal.swath_mode.objective = "LENGTH"
        if swath_angle_deg is not None:
            goal.swath_mode.best_angle = float(math.radians(swath_angle_deg))
        goal.route_mode.mode = str(route_type)
        goal.path_mode.mode = str(path_type)
        goal.path_mode.continuity_mode = str(path_continuity)
        # Fields2Cover muestrea los giros cada 10 cm por defecto. Sobre un lote
        # de 40 m eso son ~1400 poses solo de cabeceras, que se comen el tope de
        # waypoints sin aportar nada: el arco queda igual de fiel con un paso
        # varias veces mayor.
        goal.path_mode.turn_point_distance = float(turn_point_distance_m)

        enviado = self._action.send_goal_async(goal)
        if not self._wait(enviado, server_timeout_s):
            raise Fields2CoverError("timeout esperando que el Coverage Server acepte")
        handle = enviado.result()
        if handle is None or not handle.accepted:
            raise Fields2CoverError("el Coverage Server rechazo la meta")

        resultado = handle.get_result_async()
        if not self._wait(resultado, server_timeout_s):
            raise Fields2CoverError("timeout esperando el plan de Fields2Cover")
        envuelto = resultado.result()
        if envuelto is None:
            raise Fields2CoverError("el Coverage Server no devolvio resultado")
        respuesta = envuelto.result
        if int(getattr(respuesta, "error_code", 0)) != 0:
            raise Fields2CoverError(
                f"Fields2Cover fallo con error_code={int(respuesta.error_code)}"
            )

        plan = plan_to_body_waypoints(
            respuesta, waypoint_spacing_m=float(waypoint_spacing_m)
        )
        plan.route_type = str(route_type)
        plan.path_type = str(path_type)
        return plan

    def plan(
        self,
        *,
        polygon_body: Sequence[Point],
        exclusions_body: Sequence[Sequence[Point]] = (),
        cutter_width_m: float,
        robot_width_m: float,
        overlap_ratio: float,
        min_turning_radius_m: float,
        waypoint_spacing_m: float,
        swath_angle_deg: Optional[float] = None,
        route_type: str = "BOUSTROPHEDON",
        path_type: str = "DUBIN",
        path_continuity: str = "CONTINUOUS",
        turn_point_distance_m: float = 0.5,
        headland_width_m: float = 0.0,
        server_timeout_s: float = 30.0,
    ) -> Fields2CoverPlan:
        """Planificar sin competir con warmup ni con otro pedido de Campo."""
        with self._server_lock:
            return self._plan_locked(
                polygon_body=polygon_body,
                exclusions_body=exclusions_body,
                cutter_width_m=cutter_width_m,
                robot_width_m=robot_width_m,
                overlap_ratio=overlap_ratio,
                min_turning_radius_m=min_turning_radius_m,
                waypoint_spacing_m=waypoint_spacing_m,
                swath_angle_deg=swath_angle_deg,
                route_type=route_type,
                path_type=path_type,
                path_continuity=path_continuity,
                turn_point_distance_m=turn_point_distance_m,
                headland_width_m=headland_width_m,
                server_timeout_s=server_timeout_s,
            )
