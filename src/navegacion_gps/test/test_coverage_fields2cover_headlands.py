"""Cabeceras flexibles: los surcos se respetan, los giros omega se van.

Fields2Cover, con la separacion entre pasadas por debajo de dos radios minimos,
resuelve cada cabecera con una omega: un lazo enorme fuera del lote. Estos tests
fijan el reemplazo por una transicion de tres tramos rectos y, sobre todo, que
el reemplazo NO toque un solo waypoint de trabajo, que es lo unico que el
operador necesita exacto.

No arrancan ROS: la funcion bajo prueba es pura.
"""

import ast
import math
import pathlib
from types import SimpleNamespace

from navegacion_gps.coverage_fields2cover import (
    Fields2CoverPlan,
    TRANSITION_PHASE,
    WORK_PHASE,
    plan_to_body_waypoints,
    replace_turns_with_flexible_headlands,
    reverse_leg_length_m,
)
from navegacion_gps.coverage_waypoint_core import CoverageBodyWaypoint


_FUENTE_ROUTE_EXECUTOR = (
    pathlib.Path(__file__).resolve().parents[1] / "navegacion_gps" / "route_executor.py"
)


def _surco(row_index: int, inicio, fin, muestras: int = 5):
    """Waypoints de una pasada recta, con los extremos como metas key."""
    puntos = []
    for paso in range(muestras):
        fraccion = paso / (muestras - 1)
        puntos.append(
            CoverageBodyWaypoint(
                forward_m=inicio[0] + ((fin[0] - inicio[0]) * fraccion),
                left_m=inicio[1] + ((fin[1] - inicio[1]) * fraccion),
                yaw_delta_deg=math.degrees(
                    math.atan2(fin[1] - inicio[1], fin[0] - inicio[0])
                ),
                phase=WORK_PHASE,
                row_index=row_index,
                is_key=paso in (0, muestras - 1),
            )
        )
    return puntos


def _omega(row_index: int, centro, radio: float = 6.0, poses: int = 24):
    """Un lazo cerrado, que es lo que Fields2Cover devuelve como giro."""
    return [
        CoverageBodyWaypoint(
            forward_m=centro[0] + (radio * math.cos(2.0 * math.pi * paso / poses)),
            left_m=centro[1] + (radio * math.sin(2.0 * math.pi * paso / poses)),
            yaw_delta_deg=float(360.0 * paso / poses),
            phase=TRANSITION_PHASE,
            row_index=row_index,
            is_key=False,
            is_guide=True,
        )
        for paso in range(poses)
    ]


def _plan_con_omegas() -> Fields2CoverPlan:
    """Dos surcos paralelos a 1.7 m unidos por una omega, como el caso real."""
    surco_0 = _surco(0, (0.0, 0.0), (40.0, 0.0))
    surco_1 = _surco(1, (40.0, 1.7), (0.0, 1.7))
    waypoints = surco_0 + _omega(0, (44.0, 0.85)) + surco_1
    return Fields2CoverPlan(
        waypoints=waypoints,
        swath_count=2,
        work_length_m=80.0,
        transition_length_m=37.7,
        route_type="SNAKE",
        path_type="DUBIN",
    )


def _trabajo(plan):
    return [punto for punto in plan.waypoints if punto.phase == WORK_PHASE]


def _transiciones(plan):
    return [punto for punto in plan.waypoints if punto.phase == TRANSITION_PHASE]


def test_los_waypoints_de_trabajo_no_cambian() -> None:
    original = _plan_con_omegas()
    salida = replace_turns_with_flexible_headlands(original, margin_m=6.0)
    assert _trabajo(salida) == _trabajo(original)
    # Y el plan de entrada queda intacto: la funcion es pura.
    assert len(original.waypoints) == 5 + 24 + 5


def test_cada_transicion_queda_en_dos_guias_exteriores() -> None:
    salida = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    transiciones = _transiciones(salida)
    assert len(transiciones) == 2
    for punto in transiciones:
        assert punto.phase == TRANSITION_PHASE
        assert punto.is_key is False
        assert punto.is_guide is True
    # El row_index es el del surco del que se sale, para que el tramo quede
    # atribuido a la cabecera que lo genero.
    assert [punto.row_index for punto in transiciones] == [0, 0]


def test_no_sobrevive_ningun_punto_de_la_omega() -> None:
    original = _plan_con_omegas()
    omega = {
        (round(p.forward_m, 6), round(p.left_m, 6))
        for p in original.waypoints
        if p.phase == TRANSITION_PHASE
    }
    salida = replace_turns_with_flexible_headlands(original, margin_m=6.0)
    quedaron = {
        (round(p.forward_m, 6), round(p.left_m, 6)) for p in salida.waypoints
    } & omega
    assert not quedaron, f"la omega sobrevivio en {sorted(quedaron)}"


def test_el_margen_exterior_es_el_pedido() -> None:
    # Radio 4 m por 1.5 = 6 m fuera del extremo del surco, sobre el eje del
    # surco: ni antes (cortaria trabajo) ni al costado (seria otra maniobra).
    salida = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    salida_guia, entrada_guia = _transiciones(salida)
    assert salida_guia.forward_m == 46.0
    assert salida_guia.left_m == 0.0
    assert entrada_guia.forward_m == 46.0
    assert entrada_guia.left_m == 1.7
    # Ambas quedan fuera del lote, que termina en forward=40.
    assert salida_guia.forward_m > 40.0
    assert entrada_guia.forward_m > 40.0


def test_el_margen_escala_con_el_radio() -> None:
    for margen in (4.0, 6.0, 8.0):
        salida = replace_turns_with_flexible_headlands(
            _plan_con_omegas(), margin_m=margen
        )
        primera = _transiciones(salida)[0]
        assert math.isclose(primera.forward_m, 40.0 + margen, abs_tol=1.0e-9)


def test_las_guias_apuntan_a_salir_y_a_entrar() -> None:
    salida_guia, entrada_guia = _transiciones(
        replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    )
    # Se sale mirando como iba el surco; se reentra mirando como va el proximo.
    assert math.isclose(salida_guia.yaw_delta_deg, 0.0, abs_tol=1.0e-6)
    assert math.isclose(abs(entrada_guia.yaw_delta_deg), 180.0, abs_tol=1.0e-6)


def test_la_transicion_no_tiene_curvatura() -> None:
    """Tres tramos rectos: sin lazo no hay forma de que aparezca un circulo."""
    salida = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    fin_surco = _trabajo(salida)[4]
    inicio_siguiente = _trabajo(salida)[5]
    guia_salida, guia_entrada = _transiciones(salida)
    recorrido = [
        (fin_surco.forward_m, fin_surco.left_m),
        (guia_salida.forward_m, guia_salida.left_m),
        (guia_entrada.forward_m, guia_entrada.left_m),
        (inicio_siguiente.forward_m, inicio_siguiente.left_m),
    ]
    # Una omega vuelve sobre si misma: su recorrido mide varias veces la
    # distancia entre extremos. Esta transicion no puede pasar de un rodeo
    # rectangular de dos margenes mas la separacion entre surcos.
    largo = sum(
        math.dist(recorrido[i], recorrido[i + 1]) for i in range(len(recorrido) - 1)
    )
    assert largo <= (2.0 * 6.0) + 1.7 + 1.0e-6
    # Y ningun punto se mete de vuelta en el lote a mitad de la cabecera.
    assert all(punto[0] >= 40.0 for punto in recorrido)


def test_el_largo_de_transiciones_se_recalcula() -> None:
    salida = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    # 6 m de salida + 1.7 m de corrimiento + 6 m de reentrada.
    assert math.isclose(salida.transition_length_m, 13.7, abs_tol=1.0e-9)
    # El trabajo no lo toca nadie.
    assert math.isclose(salida.work_length_m, 80.0, abs_tol=1.0e-9)


def test_es_idempotente() -> None:
    una = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=6.0)
    dos = replace_turns_with_flexible_headlands(una, margin_m=6.0)
    assert dos.waypoints == una.waypoints
    assert math.isclose(dos.transition_length_m, una.transition_length_m, abs_tol=1e-9)


def test_con_un_solo_surco_no_inventa_transiciones() -> None:
    plan = Fields2CoverPlan(waypoints=_surco(0, (0.0, 0.0), (40.0, 0.0)), swath_count=1)
    salida = replace_turns_with_flexible_headlands(plan, margin_m=6.0)
    assert not _transiciones(salida)
    assert salida.transition_length_m == 0.0


def test_margen_nulo_no_deja_waypoints_repetidos() -> None:
    """Un waypoint encima de otro es una meta de largo cero aguas abajo."""
    salida = replace_turns_with_flexible_headlands(_plan_con_omegas(), margin_m=0.0)
    posiciones = [(p.forward_m, p.left_m) for p in salida.waypoints]
    assert len(posiciones) == len(set(posiciones))


def test_la_rama_de_campo_aplica_las_cabeceras_antes_del_recorte() -> None:
    """El recorte de zonas tiene que ver el plan que se va a ejecutar."""
    arbol = ast.parse(_FUENTE_ROUTE_EXECUTOR.read_text(encoding="utf-8"))
    metodo = next(
        nodo
        for nodo in ast.walk(arbol)
        if isinstance(nodo, (ast.FunctionDef, ast.AsyncFunctionDef))
        and nodo.name == "_generate_coverage_plan_fields2cover"
    )
    lineas = {}
    for nodo in ast.walk(metodo):
        if isinstance(nodo, ast.Call) and isinstance(nodo.func, ast.Name):
            lineas.setdefault(nodo.func.id, nodo.lineno)
    assert "replace_turns_with_flexible_headlands" in lineas
    assert "clip_plan_to_nogo" in lineas
    assert lineas["replace_turns_with_flexible_headlands"] < lineas["clip_plan_to_nogo"]


def test_las_cabeceras_flexibles_se_pueden_apagar_por_parametro() -> None:
    fuente = _FUENTE_ROUTE_EXECUTOR.read_text(encoding="utf-8")
    assert '"coverage_f2c_flexible_turns"' in fuente
    assert '"coverage_f2c_turn_lead_m"' in fuente


def test_fields2cover_reporta_la_separacion_geometrica_entre_surcos() -> None:
    def punto(x, y):
        return SimpleNamespace(x=float(x), y=float(y))

    def swath(inicio, fin):
        return SimpleNamespace(start=punto(*inicio), end=punto(*fin))

    # Desordenados, con sentidos alternados y una linea partida en dos. La
    # separacion informada tiene que seguir siendo 1.7 m, no el salto de visita.
    swaths = [
        swath((0.0, 0.0), (10.0, 0.0)),
        swath((10.0, 3.4), (0.0, 3.4)),
        swath((0.0, 1.7), (4.0, 1.7)),
        swath((10.0, 1.7), (6.0, 1.7)),
    ]
    result = SimpleNamespace(
        coverage_path=SimpleNamespace(swaths=swaths, turns=[])
    )

    plan = plan_to_body_waypoints(result, waypoint_spacing_m=2.0)

    assert math.isclose(plan.lane_spacing_m, 1.7, abs_tol=1.0e-9)


# ---------------------------------------------------------------------------
# Cabecera de tres puntos. Con las pasadas mas juntas que el diametro de giro no
# existe ninguna curva hacia adelante que lleve de un surco al siguiente: por eso
# Fields2Cover devolvia omegas. Retrocediendo, la misma transicion entra en R
# metros de cabecera en vez de 7.3.
# ---------------------------------------------------------------------------

RADIO_M = 2.9
SEPARACION_M = 1.7


def _plan_apretado() -> Fields2CoverPlan:
    """Dos surcos a 1.7 m: menos que el diametro de giro de 5.8 m."""
    surco_0 = _surco(0, (0.0, 0.0), (40.0, 0.0))
    surco_1 = _surco(1, (40.0, SEPARACION_M), (0.0, SEPARACION_M))
    return Fields2CoverPlan(
        waypoints=surco_0 + _omega(0, (44.0, 0.85)) + surco_1,
        swath_count=2,
        work_length_m=80.0,
    )


def _flexible(plan, margen=0.5, radio=RADIO_M):
    return replace_turns_with_flexible_headlands(
        plan, margin_m=margen, min_turning_radius_m=radio
    )


def test_la_reversa_mide_dos_radios_menos_la_separacion() -> None:
    # Cerrar la geometria obliga: los dos arcos corren 2R de costado y la
    # reversa descuenta hasta la separacion real.
    assert math.isclose(reverse_leg_length_m(2.9, 1.7), 4.1, abs_tol=1.0e-9)
    assert math.isclose(reverse_leg_length_m(4.0, 1.65), 6.35, abs_tol=1.0e-9)


def test_sin_reversa_cuando_las_pasadas_entran_por_delante() -> None:
    """Separacion mayor al diametro: el giro hacia adelante existe."""
    assert reverse_leg_length_m(2.9, 5.8) == 0.0
    assert reverse_leg_length_m(2.9, 9.0) == 0.0


def test_la_cabecera_apretada_emite_la_marcha_atras() -> None:
    salida = _flexible(_plan_apretado())
    transiciones = _transiciones(salida)
    con_reversa = [punto for punto in transiciones if punto.backup_m > 0.0]
    assert len(con_reversa) == 1, "la maniobra tiene un solo vertice de reversa"
    assert math.isclose(con_reversa[0].backup_m, 4.1, abs_tol=1.0e-9)
    # Sigue siendo una guia de transito, no una meta de trabajo.
    assert con_reversa[0].is_key is False
    assert con_reversa[0].is_guide is True
    assert con_reversa[0].phase == TRANSITION_PHASE


def test_la_maniobra_deja_el_vehiculo_en_el_surco_siguiente() -> None:
    """La cabecera tiene que cerrar: si no, el surco arranca torcido."""
    salida = _flexible(_plan_apretado())
    trabajo = _trabajo(salida)
    inicio_siguiente = trabajo[5]
    ultima_guia = _transiciones(salida)[-1]
    # La ultima guia queda sobre el eje del surco siguiente, a `margen` de su
    # inicio, y mirando hacia adentro del lote.
    assert math.isclose(ultima_guia.left_m, inicio_siguiente.left_m, abs_tol=1.0e-9)
    assert math.isclose(
        ultima_guia.forward_m - inicio_siguiente.forward_m, 0.5, abs_tol=1.0e-9
    )
    assert math.isclose(abs(ultima_guia.yaw_delta_deg), 180.0, abs_tol=1.0e-6)


def test_la_cabecera_de_tres_puntos_entra_en_menos_lugar_que_el_omega() -> None:
    salida = _flexible(_plan_apretado())
    fuera = [
        punto.forward_m - 40.0
        for punto in salida.waypoints
        if punto.phase == TRANSITION_PHASE
    ]
    # margen + radio = 0.5 + 2.9. El omega de Fields2Cover pedia 7.3 m.
    assert math.isclose(max(fuera), 3.4, abs_tol=1.0e-9)
    assert max(fuera) < 7.3


def test_con_pasadas_separadas_vuelve_a_la_transicion_recta() -> None:
    """Si el giro entra por delante no se retrocede: seria gratuito."""
    surco_0 = _surco(0, (0.0, 0.0), (40.0, 0.0))
    surco_1 = _surco(1, (40.0, 9.0), (0.0, 9.0))
    plan = Fields2CoverPlan(waypoints=surco_0 + _omega(0, (44.0, 4.5)) + surco_1)
    salida = _flexible(plan)
    transiciones = _transiciones(salida)
    assert len(transiciones) == 2, "la transicion recta de siempre"
    assert all(punto.backup_m == 0.0 for punto in transiciones)


def test_sin_radio_no_hay_reversa() -> None:
    """El comportamiento viejo sigue disponible con radio 0."""
    salida = replace_turns_with_flexible_headlands(
        _plan_apretado(), margin_m=6.0, min_turning_radius_m=0.0
    )
    assert all(punto.backup_m == 0.0 for punto in _transiciones(salida))
    assert len(_transiciones(salida)) == 2


def test_los_surcos_siguen_intactos_con_la_maniobra() -> None:
    original = _plan_apretado()
    salida = _flexible(original)
    assert _trabajo(salida) == _trabajo(original)


def test_la_maniobra_es_idempotente() -> None:
    una = _flexible(_plan_apretado())
    dos = _flexible(una)
    assert dos.waypoints == una.waypoints


def test_la_maniobra_no_deja_lazos() -> None:
    """Tres tramos y dos arcos: el rumbo gira 180, no 360."""
    salida = _flexible(_plan_apretado())
    trabajo = _trabajo(salida)
    recorrido = [(trabajo[4].forward_m, trabajo[4].left_m)]
    recorrido += [(p.forward_m, p.left_m) for p in _transiciones(salida)]
    recorrido += [(trabajo[5].forward_m, trabajo[5].left_m)]
    # Ningun punto vuelve sobre el lote: toda la maniobra pasa el borde.
    assert all(punto[0] >= 40.0 for punto in recorrido)


def test_el_backend_publica_la_accion_de_marcha_atras() -> None:
    """El vertice de la reversa tiene que viajar como accion de waypoint."""
    fuente = _FUENTE_ROUTE_EXECUTOR.read_text(encoding="utf-8")
    assert "route_action_jsons" in fuente
    assert '"coverage_backup"' in fuente
    # Y no puede decimarse: sin ese vertice la maniobra queda a medias.
    assert 'float(item.get("backup_m", 0.0)) > 0.0' in fuente
