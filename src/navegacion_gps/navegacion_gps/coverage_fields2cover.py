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
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates

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


def _ring_to_coordinates(ring: Sequence[Point]) -> Coordinates:
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

    plan = Fields2CoverPlan(swath_count=len(swaths))
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
                        is_guide=False,
                    )
                )

    plan.waypoints = waypoints
    return plan


class Fields2CoverPlanner:
    """Cliente del Coverage Server con executor propio."""

    def __init__(
        self,
        *,
        action_name: str = "compute_coverage_path",
        parameter_service: str = "/coverage_server/set_parameters",
        node_name: str = "route_executor_fields2cover_client",
        logger: Any = None,
    ) -> None:
        """Levantar el nodo cliente y su executor en un hilo aparte."""
        self._logger = logger
        self._node = rclpy.create_node(node_name)
        self._action = ActionClient(self._node, ComputeCoveragePath, action_name)
        self._parameters = self._node.create_client(SetParameters, parameter_service)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, name=node_name, daemon=True
        )
        self._thread.start()

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

    @staticmethod
    def _wait(future: Any, timeout_s: float) -> bool:
        """Esperar un future que resuelve el executor propio."""
        evento = threading.Event()
        future.add_done_callback(lambda _: evento.set())
        return bool(evento.wait(timeout=float(timeout_s)))

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
        """Planificar el lote y devolver los waypoints en marco del cuerpo."""
        if len(polygon_body) < 3:
            raise Fields2CoverError("el poligono del lote necesita al menos 3 vertices")
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
