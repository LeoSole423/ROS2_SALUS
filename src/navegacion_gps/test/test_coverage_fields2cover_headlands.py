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

import pytest

from navegacion_gps.coverage_fields2cover import (
    FORWARD_TURN_PHASE,
    Fields2CoverPlan,
    ForwardOnlyTurnError,
    TRANSITION_PHASE,
    WORK_PHASE,
    _forward_omega_turn,
    plan_to_body_waypoints,
    _lane_change_anticipation_m,
    _lane_change_radius_for_run_m,
    reorder_internal_nogo_swaths,
    replace_turns_with_flexible_headlands,
    reverse_leg_length_m,
)
from navegacion_gps.coverage_waypoint_core import CoverageBodyWaypoint
from navegacion_gps.coverage_nogo import plan_nogo_conflicts


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
    return [
        punto
        for punto in plan.waypoints
        if punto.phase in {TRANSITION_PHASE, FORWARD_TURN_PHASE}
    ]


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


# ---------------------------------------------------------------------------
# Politica forward-only. En simulacion el vehiculo no puede retroceder: la
# cabecera de tres puntos queda prohibida y la misma transicion se resuelve con
# una omega hacia adelante, que sale del lote por la cabecera y vuelve a entrar.
# ---------------------------------------------------------------------------


def _forward_only(plan, margen=0.5, radio=RADIO_M):
    return replace_turns_with_flexible_headlands(
        plan,
        margin_m=margen,
        min_turning_radius_m=radio,
        allow_reverse=False,
    )


def test_sin_reversa_ninguna_guia_pide_marcha_atras() -> None:
    """La condicion que apaga coverage_backup aguas abajo."""
    salida = _forward_only(_plan_apretado())
    transiciones = _transiciones(salida)
    assert transiciones, "la cabecera tiene que existir igual"
    assert all(punto.backup_m == 0.0 for punto in transiciones)
    # Tangencias y alineacion exterior flexibles; solo el apice central es la
    # ancla moderada que impide recortar la omega.
    assert sum(bool(punto.is_key) for punto in transiciones) == 1
    assert sum(bool(punto.is_guide) for punto in transiciones) == 3
    assert all(punto.phase == FORWARD_TURN_PHASE for punto in transiciones)


def test_con_reversa_la_misma_cabecera_si_retrocede() -> None:
    """Contraste explicito: el perfil real no cambia."""
    con = _flexible(_plan_apretado())
    assert any(punto.backup_m > 0.0 for punto in _transiciones(con))


def test_la_omega_sale_del_lote_y_vuelve_al_surco_siguiente() -> None:
    salida = _forward_only(_plan_apretado())
    transiciones = _transiciones(salida)
    # El bucle abre hacia adelante del surco, o sea afuera del lote de 40 m.
    assert max(punto.forward_m for punto in transiciones) > 40.0 + RADIO_M
    assert len(transiciones) == 4
    trabajo = _trabajo(salida)
    inicio_siguiente = trabajo[5]
    assert math.isclose(abs(inicio_siguiente.yaw_delta_deg), 180.0, abs_tol=1.0e-6)


def test_la_omega_encadena_arcos_del_radio_minimo() -> None:
    """Cada tramo tiene que ser recorrible con el radio configurado.

    Se mide el cambio de rumbo contra la cuerda entre guias consecutivas: el
    radio implicito de un arco de cuerda ``c`` y giro ``theta`` es
    ``c / (2 sin(theta/2))``. Si diera menos que el radio minimo, Nav2 no
    podria seguir la maniobra hacia adelante.
    """
    salida = _forward_only(_plan_apretado())
    trabajo = _trabajo(salida)
    recorrido = [trabajo[4]] + _transiciones(salida) + [trabajo[5]]
    for previo, siguiente in zip(recorrido, recorrido[1:]):
        cuerda = math.hypot(
            siguiente.forward_m - previo.forward_m,
            siguiente.left_m - previo.left_m,
        )
        giro = abs(
            ((siguiente.yaw_delta_deg - previo.yaw_delta_deg) + 180.0) % 360.0
            - 180.0
        )
        if giro <= 1.0e-6:
            continue
        # Media vuelta como maximo por tramo: si un tramo barriera mas, el
        # enlace mas corto dejaria de ser el arco y Nav2 cortaria camino.
        assert giro <= 180.0 + 1.0e-6
        radio_implicito = cuerda / (2.0 * math.sin(math.radians(giro) / 2.0))
        assert radio_implicito >= RADIO_M - 1.0e-6


def test_la_omega_no_vuelve_sobre_las_pasadas_ya_trabajadas() -> None:
    """El bucle abre hacia la cabecera, nunca hacia atras del surco."""
    salida = _forward_only(_plan_apretado())
    fin_surco = _trabajo(salida)[4].forward_m
    assert all(
        punto.forward_m >= fin_surco - 1.0e-6
        for punto in _transiciones(salida)
    )


def test_sin_reversa_y_con_pasadas_separadas_sigue_la_transicion_recta() -> None:
    """La omega es para la separacion corta; con lugar no se paga el bucle."""
    surco_0 = _surco(0, (0.0, 0.0), (40.0, 0.0))
    surco_1 = _surco(1, (40.0, 9.0), (0.0, 9.0))
    plan = Fields2CoverPlan(waypoints=surco_0 + _omega(0, (44.0, 4.5)) + surco_1)
    transiciones = _transiciones(_forward_only(plan))
    assert len(transiciones) == 2
    assert all(punto.backup_m == 0.0 for punto in transiciones)


def test_sin_reversa_y_sin_radio_no_inventa_marcha_atras() -> None:
    salida = _forward_only(_plan_apretado(), radio=0.0)
    assert all(punto.backup_m == 0.0 for punto in _transiciones(salida))


def test_la_omega_es_idempotente() -> None:
    una = _forward_only(_plan_apretado())
    assert _forward_only(una).waypoints == una.waypoints


def test_los_surcos_no_se_tocan_con_la_omega() -> None:
    original = _plan_apretado()
    assert _trabajo(_forward_only(original)) == _trabajo(original)


def test_dos_tramos_de_la_misma_pasada_no_generan_ninguna_maniobra() -> None:
    """Una zona no-go parte la pasada; los dos tramos siguen derecho.

    Era el modo de falla que rompia la simulacion: la separacion lateral entre
    los dos tramos es cero, la regla vieja lo leia como "cabecera apretada" y
    pedia una maniobra de media vuelta en el medio de un surco recto.
    """
    tramo_a = _surco(0, (0.0, 0.0), (15.0, 0.0))
    tramo_b = _surco(0, (25.0, 0.0), (40.0, 0.0))
    plan = Fields2CoverPlan(
        waypoints=tramo_a + _omega(0, (20.0, 4.0)) + tramo_b
    )
    transiciones = _transiciones(_forward_only(plan))
    assert len(transiciones) == 2, "solo salir derecho y volver a entrar"
    assert all(punto.backup_m == 0.0 for punto in transiciones)
    # Y con la reversa habilitada tampoco: la regla es la misma.
    assert all(punto.backup_m == 0.0 for punto in _transiciones(_flexible(plan)))


def test_pasadas_vecinas_lejos_en_el_eje_del_surco_van_derecho() -> None:
    """Con 2R de distancia entre poses ya existe la media vuelta Dubins.

    Otra consecuencia de la zona no-go: dos tramos de pasadas vecinas pueden
    estar pegados de costado y muy separados a lo largo del surco. Ahi el
    enlace CSC existe y el bucle sobraria.
    """
    surco_0 = _surco(0, (0.0, 0.0), (15.0, 0.0))
    surco_1 = _surco(1, (40.0, 1.7), (25.0, 1.7))
    plan = Fields2CoverPlan(
        waypoints=surco_0 + _omega(0, (20.0, 4.0)) + surco_1
    )
    transiciones = _transiciones(_forward_only(plan))
    assert len(transiciones) == 2
    # Salir 0.5 m derecho del primer tramo y entrar 0.5 m derecho al segundo.
    # Sin bucle: nada se aleja del eje de los surcos.
    assert [
        (round(punto.forward_m, 6), round(punto.left_m, 6))
        for punto in transiciones
    ] == [(15.5, 0.0), (40.5, 1.7)]


def test_sin_enlace_hacia_adelante_falla_en_vez_de_retroceder() -> None:
    """Nunca se reemplaza en silencio por marcha atras.

    El criterio de ``_needs_headland_maneuver`` no le entrega al constructor
    ninguna geometria imposible, asi que esto es una red de seguridad. Se
    prueba llamandolo directo: si un dia el criterio cambia y deja pasar un
    caso sin solucion, el resultado tiene que ser un error explicito y no una
    marcha atras silenciosa.
    """
    with pytest.raises(ForwardOnlyTurnError) as fallo:
        _forward_omega_turn(
            (0.0, 0.0),
            (1.0, 0.0),
            (-60.0, 1.7),
            (-1.0, 0.0),
            radio_m=RADIO_M,
            lead_m=0.5,
        )
    assert "reversa" in str(fallo.value)


def test_el_backend_apaga_la_accion_de_marcha_atras_en_forward_only() -> None:
    """La accion coverage_backup no puede emitirse con la politica activa."""
    fuente = _FUENTE_ROUTE_EXECUTOR.read_text(encoding="utf-8")
    assert 'self.declare_parameter("coverage_f2c_allow_reverse", True)' in fuente
    assert "allow_reverse=permite_reversa" in fuente
    # La accion solo se emite si el perfil permite reversa.
    assert (
        'if permite_reversa and float(i.get("backup_m", 0.0)) > 0.0' in fuente
    )
    # Y hay barrera de admision y de ejecucion, no solo de planificacion.
    assert "perfil forward-only: la ruta trae coverage_backup" in fuente
    assert "perfil forward-only: coverage_backup deshabilitado" in fuente


def _plan_con_dos_filas_partidas() -> Fields2CoverPlan:
    """Caso real reducido: dos lineas cortadas por el mismo circulo."""
    runs = [
        _surco(0, (2.0, 0.0), (2.0, 22.0)),
        _surco(1, (6.0, 22.0), (6.0, 0.0)),
        _surco(2, (10.0, 14.5), (10.0, 22.0)),
        _surco(3, (10.0, 7.5), (10.0, 0.0)),
        _surco(4, (14.0, 14.5), (14.0, 22.0)),
        _surco(5, (14.0, 7.5), (14.0, 0.0)),
        _surco(6, (18.0, 0.0), (18.0, 22.0)),
        _surco(7, (22.0, 22.0), (22.0, 0.0)),
    ]
    waypoints = []
    for index, run in enumerate(runs):
        waypoints.extend(run)
        if index + 1 < len(runs):
            waypoints.extend(_omega(index, (0.0, 0.0), poses=1))
    return Fields2CoverPlan(
        waypoints=waypoints,
        swath_count=8,
        lane_spacing_m=4.0,
    )


def _circulo_interno(radius: float = 4.0):
    return [
        (
            12.0 + (radius * math.cos(2.0 * math.pi * index / 32.0)),
            11.0 + (radius * math.sin(2.0 * math.pi * index / 32.0)),
        )
        for index in range(32)
    ]


def _contorno_lote():
    return [(0.0, 0.0), (24.0, 0.0), (24.0, 22.0), (0.0, 22.0)]


def _largo_de_la_ese(salida) -> float:
    """Cuanto antes de la zona arranca el cambio de fila.

    Cada esquive aporta cuatro puntos ``nogo_lane_change``: inicio de la S,
    cambio de curvatura, fin de la S y el escape recto hasta la cabecera. Lo
    que mide el rodeo es el primero contra el tercero; el escape depende del
    contorno y no del radio.
    """
    lane = [
        (float(punto.forward_m), float(punto.left_m))
        for punto in salida.waypoints
        if str(getattr(punto, "phase", "")) == "nogo_lane_change"
    ]
    extents = [
        math.dist(lane[index], lane[index + 2])
        for index in range(0, len(lane) - 3, 4)
    ]
    return max(extents) if extents else 0.0


def test_la_estrategia_headland_descarta_la_fila_bloqueada() -> None:
    # Estrategia OPCIONAL, no el default: la fila que la zona parte al medio no
    # se recorre. Se descarta la pasada entera de punta a punta, asi que una
    # zona de 7 m2 llega a costar 272 m2 sin cubrir. Queda solo para casos donde
    # no se tolere ninguna maniobra adentro del cultivo.
    salida = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
        internal_strategy="headland",
    )
    assert not [
        punto for punto in salida.waypoints
        if str(getattr(punto, "phase", "")) == "nogo_lane_change"
    ]
    # Y se contabiliza lo que se dejo sin recorrer, no se pierde en silencio.
    assert salida.internal_nogo_dropped_waypoint_count > 0


def test_por_default_la_fila_bloqueada_se_recorre_con_una_ese() -> None:
    # Default: cada mitad de la fila bloqueada se recorre desde SU cabecera
    # hacia la zona, y recien ahi se corre de fila. Asi se cubren las dos
    # mitades salvo el tramo de anticipacion, en vez de perder la pasada entera.
    salida = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
    )
    assert [
        punto for punto in salida.waypoints
        if str(getattr(punto, "phase", "")) == "nogo_lane_change"
    ]


def test_la_anticipacion_de_la_ese_sigue_la_formula_cerrada() -> None:
    # anticipacion = sqrt(s * (4R - s)). Son los valores que se miden en el
    # plan: 7.38 m con el radio de cabecera y 5.66 m con el propio.
    assert _lane_change_anticipation_m(4.4, 4.0) == pytest.approx(7.376, abs=0.01)
    assert _lane_change_anticipation_m(3.0, 4.0) == pytest.approx(5.657, abs=0.01)
    # Por debajo del piso geometrico s/2 la S no cierra el cambio de fila.
    assert not math.isfinite(_lane_change_anticipation_m(1.5, 4.0))
    # La inversa devuelve el radio mas grande que entra en el largo disponible.
    assert _lane_change_radius_for_run_m(5.657, 4.0) == pytest.approx(3.0, abs=0.01)
    assert _lane_change_radius_for_run_m(4.899, 4.0) == pytest.approx(2.5, abs=0.01)


def test_el_esquive_achica_el_radio_en_vez_de_rechazar_el_plan() -> None:
    # Media fila corta: con el radio preferido de 3.0 m la S necesita 5.66 m y
    # no entra, pero achicandola a 2.5 m entran 4.90 m. Antes esto rechazaba el
    # plan entero; ahora tiene que resolverlo.
    disponible = 5.20
    assert _lane_change_anticipation_m(3.0, 4.0) > disponible
    achicado = _lane_change_radius_for_run_m(disponible, 4.0)
    assert 2.5 <= achicado < 3.0
    assert _lane_change_anticipation_m(achicado, 4.0) == pytest.approx(disponible, abs=0.01)


def test_el_esquive_no_go_usa_su_propio_radio_y_queda_mas_corto() -> None:
    # Con el radio de cabecera la S arranca 7.4 m antes de la zona; ese rodeo
    # se lleva puestas filas que no hacia falta tocar. El radio propio la deja
    # mas compacta sin cambiar el radio con que se planifica la cabecera.
    ancho = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
            internal_strategy="lane_change",
    )
    compacto = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
        lane_change_radius_m=3.0,
            internal_strategy="lane_change",
    )
    assert _largo_de_la_ese(compacto) < _largo_de_la_ese(ancho)


def test_el_radio_del_esquive_nunca_baja_del_piso_geometrico() -> None:
    # Con radio menor que la mitad de la separacion la S no cierra el cambio de
    # una fila: el piso tiene que sostenerla igual, no romper el plan.
    salida = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
        lane_change_radius_m=0.5,
            internal_strategy="lane_change",
    )
    assert _largo_de_la_ese(salida) > 0.0


def test_nogo_interno_cambia_una_fila_y_conserva_los_medios_swaths() -> None:
    salida = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
            internal_strategy="lane_change",
    )
    runs = []
    actual = []
    for punto in salida.waypoints:
        if actual and punto.row_index != actual[-1].row_index:
            runs.append(actual)
            actual = []
        actual.append(punto)
    if actual:
        runs.append(actual)
    endpoints = [
        (
            (run[0].forward_m, run[0].left_m),
            (run[-1].forward_m, run[-1].left_m),
        )
        for run in runs
    ]
    assert endpoints == [
        ((2.0, 0.0), (2.0, 22.0)),
        ((6.0, 22.0), (6.0, 0.0)),
        ((10.0, 0.0), (6.0, 22.0)),
        ((10.0, 22.0), (6.0, 0.0)),
        ((14.0, 0.0), (18.0, 22.0)),
        ((14.0, 22.0), (18.0, 0.0)),
        ((18.0, 0.0), (18.0, 22.0)),
        ((22.0, 22.0), (22.0, 0.0)),
    ]
    assert salida.swath_count == 8
    assert salida.internal_nogo_dropped_waypoint_count > 0


def test_nogo_interno_agrupa_medias_filas_aunque_lleguen_intercaladas() -> None:
    runs = [
        _surco(0, (2.0, 0.0), (2.0, 22.0)),
        _surco(1, (6.0, 22.0), (6.0, 0.0)),
        _surco(2, (10.0, 14.5), (10.0, 22.0)),
        # Fields2Cover puede entregar la primera mitad de la fila vecina antes
        # de volver a la segunda mitad de x=10.
        _surco(4, (14.0, 14.5), (14.0, 22.0)),
        _surco(3, (10.0, 7.5), (10.0, 0.0)),
        _surco(5, (14.0, 7.5), (14.0, 0.0)),
        _surco(6, (18.0, 0.0), (18.0, 22.0)),
        _surco(7, (22.0, 22.0), (22.0, 0.0)),
    ]
    waypoints = []
    for index, run in enumerate(runs):
        waypoints.extend(run)
        if index + 1 < len(runs):
            waypoints.extend(_omega(index, (0.0, 0.0), poses=1))
    plan = Fields2CoverPlan(
        waypoints=waypoints,
        swath_count=8,
        lane_spacing_m=4.0,
    )

    salida = reorder_internal_nogo_swaths(
        plan,
        [_circulo_interno()],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
            internal_strategy="lane_change",
    )

    lane_changes = [
        point
        for point in salida.waypoints
        if point.phase == "nogo_lane_change"
    ]
    assert len(lane_changes) == 16
    assert salida.internal_nogo_dropped_waypoint_count > 0


def test_nogo_interno_no_genera_ningun_giro_dentro_del_lote() -> None:
    exclusion = _circulo_interno()
    reordered = reorder_internal_nogo_swaths(
        _plan_con_dos_filas_partidas(),
        [exclusion],
        field_boundary=_contorno_lote(),
        min_turning_radius_m=4.0,
            internal_strategy="lane_change",
    )
    salida = replace_turns_with_flexible_headlands(
        reordered,
        margin_m=0.5,
        min_turning_radius_m=4.0,
        allow_reverse=False,
        avoid_polygons=[exclusion],
    )
    assert plan_nogo_conflicts(salida.waypoints, [exclusion]) == ([], [])
    assert not [
        punto for punto in salida.waypoints if punto.phase == "nogo_transition"
    ]
    lane_changes = [
        point
        for point in salida.waypoints
        if point.phase == "nogo_lane_change"
    ]
    lane_guides = [point for point in lane_changes if point.is_guide]
    lane_change_keys = [
        point for point in lane_changes if point.is_key and not point.is_guide
    ]
    # Cada S conserva tres vertices geometricos, y los tres son guias: entrada,
    # cambio de curvatura y salida. La unica key es el escape, que se alcanza
    # yendo derecho hasta la cabecera. Asi la fila recta y la S entera viajan en
    # un unico FollowPath y no se le pide al Ackermann parar sobre el apex con
    # la tolerancia de trabajo, que barriendo la curva no puede cumplir.
    assert len(lane_guides) == 12
    assert len(lane_change_keys) == 4
    assert all(point.backup_m == 0.0 for point in lane_changes)
    # Cada media fila hace una sola S de exactamente una separacion de pasada;
    # despues conserva la coordenada lateral hasta la cabecera opuesta.
    changed_runs = {}
    for point in lane_changes:
        changed_runs.setdefault(point.row_index, []).append(point)
    assert len(changed_runs) == 4
    for points in changed_runs.values():
        assert len(points) == 4
        heading = (
            math.cos(math.radians(points[0].yaw_delta_deg)),
            math.sin(math.radians(points[0].yaw_delta_deg)),
        )
        displacement = (
            points[2].forward_m - points[0].forward_m,
            points[2].left_m - points[0].left_m,
        )
        lateral_shift = abs(
            (displacement[0] * -heading[1])
            + (displacement[1] * heading[0])
        )
        assert math.isclose(lateral_shift, 4.0, abs_tol=1.0e-6)
        escape_displacement = (
            points[3].forward_m - points[2].forward_m,
            points[3].left_m - points[2].left_m,
        )
        assert math.isclose(
            (escape_displacement[0] * -heading[1])
            + (escape_displacement[1] * heading[0]),
            0.0,
            abs_tol=1.0e-6,
        )
