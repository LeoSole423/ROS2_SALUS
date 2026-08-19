import ast
import json
import math
import pathlib

import pytest

from navegacion_gps.coverage_nogo import NOGO_DETOUR_PHASE
from navegacion_gps.coverage_nogo import clip_plan_to_nogo
from navegacion_gps.coverage_nogo import detour_along_contour
from navegacion_gps.coverage_nogo import inflate_polygon
from navegacion_gps.coverage_nogo import point_in_polygon
from navegacion_gps.coverage_nogo import segment_polygon_intersections
from navegacion_gps.coverage_nogo_zones import ll_to_body
from navegacion_gps.coverage_nogo_zones import polygons_from_geojson
from navegacion_gps.coverage_waypoint_core import CoverageBodyWaypoint
from navegacion_gps.coverage_waypoint_core import build_lawnmower_waypoints
from navegacion_gps.nav_benchmarking import body_relative_offsets_to_north_east
from navegacion_gps.nav_benchmarking import offset_lat_lon

# Fixture compartido con el test de TypeScript del cockpit
# (`cockpit/src/test/coverageNoGo.test.ts`). Si cambia aca tiene que cambiar
# alla: es lo unico que mantiene honestas a las dos implementaciones del
# recorte, que corren la misma cuenta en los dos lados a proposito.
ZONA_CUADRADA = [(10.0, 10.0), (20.0, 10.0), (20.0, 20.0), (10.0, 20.0)]
FILA_QUE_ATRAVIESA = [(0.0, 15.0), (10.0, 15.0), (15.0, 15.0), (20.0, 15.0), (30.0, 15.0)]
MARGEN_M = 0.5


def _waypoint(forward_m: float, left_m: float, **kwargs: object) -> CoverageBodyWaypoint:
    campos = {
        "yaw_delta_deg": 0.0,
        "phase": "row",
        "row_index": 0,
        "is_key": True,
    }
    campos.update(kwargs)
    return CoverageBodyWaypoint(forward_m=forward_m, left_m=left_m, **campos)


def _fila(puntos) -> list:
    return [_waypoint(forward_m, left_m) for forward_m, left_m in puntos]


def _posiciones(waypoints) -> list:
    return [
        (round(float(item.forward_m), 6), round(float(item.left_m), 6), str(item.phase))
        for item in waypoints
    ]


def test_point_in_polygon_distingue_adentro_afuera_y_borde() -> None:
    assert point_in_polygon((15.0, 15.0), ZONA_CUADRADA) is True
    assert point_in_polygon((5.0, 15.0), ZONA_CUADRADA) is False
    assert point_in_polygon((25.0, 25.0), ZONA_CUADRADA) is False
    # El borde cuenta como adentro: el poligono ya viene inflado por el margen,
    # asi que dudar hacia adentro es el lado conservador.
    assert point_in_polygon((10.0, 15.0), ZONA_CUADRADA) is True


def test_point_in_polygon_ignora_poligonos_degenerados() -> None:
    assert point_in_polygon((0.0, 0.0), [(0.0, 0.0), (1.0, 1.0)]) is False


def test_inflate_polygon_respeta_el_margen_en_las_dos_orientaciones() -> None:
    antihorario = inflate_polygon(ZONA_CUADRADA, 1.0)
    horario = inflate_polygon(list(reversed(ZONA_CUADRADA)), 1.0)
    assert antihorario == [(9.0, 9.0), (21.0, 9.0), (21.0, 21.0), (9.0, 21.0)]
    # El sentido de los vertices no puede cambiar hacia donde se infla: el
    # cockpit dibuja el rectangulo en el sentido en que arrastra el mouse.
    assert sorted(horario) == sorted(antihorario)


def test_inflate_polygon_sin_margen_no_toca_nada() -> None:
    assert inflate_polygon(ZONA_CUADRADA, 0.0) == [
        (float(x), float(y)) for x, y in ZONA_CUADRADA
    ]


def test_segment_polygon_intersections_ordena_desde_el_inicio() -> None:
    cortes = segment_polygon_intersections((0.0, 15.0), (30.0, 15.0), ZONA_CUADRADA)
    assert [round(punto[0], 3) for _, punto, _ in cortes] == [10.0, 20.0]
    assert cortes[0][0] < cortes[-1][0]


def test_zona_lejos_del_lote_deja_el_plan_intacto() -> None:
    lejos = [(100.0, 100.0), (110.0, 100.0), (110.0, 110.0), (100.0, 110.0)]
    plan = _fila(FILA_QUE_ATRAVIESA)
    recortado, descartados, rodeos = clip_plan_to_nogo(plan, [lejos], margin_m=MARGEN_M)
    assert _posiciones(recortado) == _posiciones(plan)
    assert (descartados, rodeos) == (0, 0)


def test_sin_zonas_no_se_toca_el_plan() -> None:
    plan = _fila(FILA_QUE_ATRAVIESA)
    recortado, descartados, rodeos = clip_plan_to_nogo(plan, [], margin_m=MARGEN_M)
    assert _posiciones(recortado) == _posiciones(plan)
    assert (descartados, rodeos) == (0, 0)


def test_zona_sobre_una_pasada_descarta_los_puntos_y_bordea_el_contorno() -> None:
    recortado, descartados, rodeos = clip_plan_to_nogo(
        _fila(FILA_QUE_ATRAVIESA), [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    assert descartados == 3
    assert rodeos == 1
    assert _posiciones(recortado) == [
        (0.0, 15.0, "row"),
        (9.5, 15.0, NOGO_DETOUR_PHASE),
        (9.5, 9.5, NOGO_DETOUR_PHASE),
        (20.5, 9.5, NOGO_DETOUR_PHASE),
        (20.5, 15.0, NOGO_DETOUR_PHASE),
        (30.0, 15.0, "row"),
    ]
    # Los puntos del rodeo son metas de parada: si no fueran key, el
    # route_executor los saltearia y la ruta volveria a cruzar la zona.
    assert all(
        bool(item.is_key)
        for item in recortado
        if str(item.phase) == NOGO_DETOUR_PHASE
    )


def test_ningun_punto_del_recorte_queda_dentro_de_la_zona() -> None:
    recortado, _, _ = clip_plan_to_nogo(
        _fila(FILA_QUE_ATRAVIESA), [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    for item in recortado:
        assert not point_in_polygon(
            (float(item.forward_m), float(item.left_m)), ZONA_CUADRADA
        )


@pytest.mark.parametrize(
    "left_m, esperado_left",
    [
        (18.0, 20.5),  # cruce alto: el lado corto es por arriba
        (12.0, 9.5),  # cruce bajo: el lado corto es por abajo
    ],
)
def test_el_rodeo_elige_el_lado_mas_corto(left_m: float, esperado_left: float) -> None:
    recortado, _, _ = clip_plan_to_nogo(
        _fila([(0.0, left_m), (30.0, left_m)]), [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    laterales = {
        round(float(item.left_m), 3)
        for item in recortado
        if str(item.phase) == NOGO_DETOUR_PHASE
    }
    assert esperado_left in laterales
    # Nunca se toca el lado largo: si apareciera, el rodeo dio la vuelta entera.
    assert (9.5 if esperado_left > 15.0 else 20.5) not in laterales


def test_el_rodeo_hereda_el_row_index_del_destino() -> None:
    plan = [
        _waypoint(0.0, 15.0, row_index=3),
        _waypoint(30.0, 15.0, row_index=3),
    ]
    recortado, _, _ = clip_plan_to_nogo(plan, [ZONA_CUADRADA], margin_m=MARGEN_M)
    assert {int(item.row_index) for item in recortado} == {3}
    assert not any(bool(item.is_guide) for item in recortado)


def test_el_yaw_del_rodeo_apunta_al_tramo_siguiente() -> None:
    recortado, _, _ = clip_plan_to_nogo(
        _fila(FILA_QUE_ATRAVIESA), [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    rodeo = [item for item in recortado if str(item.phase) == NOGO_DETOUR_PHASE]
    # Baja, cruza por debajo y vuelve a subir.
    assert [round(float(item.yaw_delta_deg), 3) for item in rodeo] == [
        -90.0,
        0.0,
        90.0,
        0.0,
    ]


def test_un_tramo_pegado_al_borde_no_genera_rodeo() -> None:
    # El tramo corre exactamente sobre un lado del poligono inflado: hay dos
    # cortes con los lados perpendiculares, pero nada que rodear. Si esto se
    # tratara como cruce, el rodeo se reinsertaria en cada pasada.
    inflado = inflate_polygon(ZONA_CUADRADA, MARGEN_M)
    assert detour_along_contour((0.0, 9.5), (30.0, 9.5), inflado) == []


def test_el_recorte_es_idempotente() -> None:
    primera, _, _ = clip_plan_to_nogo(
        _fila(FILA_QUE_ATRAVIESA), [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    segunda, descartados, rodeos = clip_plan_to_nogo(
        primera, [ZONA_CUADRADA], margin_m=MARGEN_M
    )
    # Es lo que permite que el cockpit recorte lo que ya recorto el backend sin
    # pisarlo: la segunda pasada tiene que ser un no-op exacto.
    assert _posiciones(segunda) == _posiciones(primera)
    assert (descartados, rodeos) == (0, 0)


def test_zona_que_cubre_todo_el_lote_da_error_y_no_una_ruta_vacia() -> None:
    todo = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]
    with pytest.raises(ValueError, match="no dejan superficie"):
        clip_plan_to_nogo(_fila(FILA_QUE_ATRAVIESA), [todo], margin_m=MARGEN_M)


def test_dos_zonas_seguidas_se_rodean_las_dos() -> None:
    segunda_zona = [(40.0, 10.0), (50.0, 10.0), (50.0, 20.0), (40.0, 20.0)]
    recortado, descartados, rodeos = clip_plan_to_nogo(
        _fila([(0.0, 15.0), (60.0, 15.0)]),
        [ZONA_CUADRADA, segunda_zona],
        margin_m=MARGEN_M,
    )
    assert descartados == 0
    assert rodeos == 2
    for zona in (ZONA_CUADRADA, segunda_zona):
        for item in recortado:
            assert not point_in_polygon(
                (float(item.forward_m), float(item.left_m)), zona
            )


def _plan_de_cobertura(**kwargs: object):
    return build_lawnmower_waypoints(
        start_lat=-31.42,
        start_lon=-64.19,
        start_yaw_deg=0.0,
        field_length_m=60.0,
        field_width_m=40.0,
        cutter_width_m=3.0,
        overlap_ratio=0.1,
        min_turning_radius_m=3.0,
        waypoint_spacing_m=2.0,
        **kwargs,
    )


def test_build_lawnmower_waypoints_sin_zonas_no_cambia() -> None:
    plan_base, waypoints_base = _plan_de_cobertura()
    plan_none, waypoints_none = _plan_de_cobertura(no_go_polygons_body=None)
    plan_vacio, waypoints_vacio = _plan_de_cobertura(no_go_polygons_body=[])
    assert waypoints_none == waypoints_base
    assert waypoints_vacio == waypoints_base
    assert plan_none == plan_base
    assert plan_vacio == plan_base
    assert plan_base.nogo_polygon_count == 0
    assert plan_base.nogo_dropped_count == 0
    assert plan_base.nogo_detour_count == 0


def test_build_lawnmower_waypoints_recorta_y_reporta() -> None:
    zona = [(20.0, -5.0), (40.0, -5.0), (40.0, 15.0), (20.0, 15.0)]
    _, waypoints_base = _plan_de_cobertura()
    plan, waypoints = _plan_de_cobertura(
        no_go_polygons_body=[zona], no_go_margin_m=1.5
    )
    assert plan.nogo_polygon_count == 1
    assert plan.nogo_dropped_count > 0
    assert plan.nogo_detour_count > 0
    assert len(waypoints) < len(waypoints_base)
    assert any(str(item["phase"]) == NOGO_DETOUR_PHASE for item in waypoints)
    # La topologia se mide sobre el trazado sin recortar: un rodeo es una
    # desviacion deliberada y no tiene por que marcar el plan como inseguro.
    plan_base, _ = _plan_de_cobertura()
    assert plan.strict_crossing_count == plan_base.strict_crossing_count
    assert plan.row_count == plan_base.row_count


@pytest.mark.parametrize("yaw_deg", [0.0, 37.0, 90.0, -120.0])
def test_el_recorte_no_depende_del_rumbo_del_lote(yaw_deg: float) -> None:
    # La zona se define en marco del cuerpo y se lleva a lat/lon con el mismo
    # ancla que usa la georreferenciacion, asi que el resultado tiene que ser el
    # mismo con el lote derecho que en diagonal.
    zona_body = [(20.0, -5.0), (40.0, -5.0), (40.0, 15.0), (20.0, 15.0)]
    plan, _ = build_lawnmower_waypoints(
        start_lat=-31.42,
        start_lon=-64.19,
        start_yaw_deg=float(yaw_deg),
        field_length_m=60.0,
        field_width_m=40.0,
        cutter_width_m=3.0,
        overlap_ratio=0.1,
        min_turning_radius_m=3.0,
        waypoint_spacing_m=2.0,
        no_go_polygons_body=[zona_body],
        no_go_margin_m=1.5,
    )
    assert (plan.nogo_dropped_count, plan.nogo_detour_count) == (77, 7)


@pytest.mark.parametrize("yaw_deg", [0.0, 37.0, 90.0, -120.0])
def test_ll_to_body_es_la_inversa_de_la_georreferenciacion(yaw_deg: float) -> None:
    origen_lat, origen_lon = -31.42, -64.19
    for forward_m, left_m in ((0.0, 0.0), (50.0, -20.0), (-15.0, 33.0), (120.0, 80.0)):
        north_m, east_m = body_relative_offsets_to_north_east(
            start_yaw_deg=yaw_deg, forward_m=forward_m, left_m=left_m
        )
        lat, lon = offset_lat_lon(
            lat_deg=origen_lat, lon_deg=origen_lon, north_m=north_m, east_m=east_m
        )
        vuelta = ll_to_body(
            lat,
            lon,
            origin_lat=origen_lat,
            origin_lon=origen_lon,
            origin_yaw_deg=yaw_deg,
        )
        assert math.isclose(vuelta[0], forward_m, abs_tol=1.0e-6)
        assert math.isclose(vuelta[1], left_m, abs_tol=1.0e-6)


def _geojson_de_zona(zona_body, *, enabled=True, zone_type="no_go"):
    origen_lat, origen_lon = -31.42, -64.19
    anillo = []
    for forward_m, left_m in zona_body:
        north_m, east_m = body_relative_offsets_to_north_east(
            start_yaw_deg=0.0, forward_m=forward_m, left_m=left_m
        )
        lat, lon = offset_lat_lon(
            lat_deg=origen_lat, lon_deg=origen_lon, north_m=north_m, east_m=east_m
        )
        anillo.append([lon, lat])
    return json.dumps(
        {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "id": "z1",
                        "type": zone_type,
                        "enabled": enabled,
                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [anillo + [anillo[0]]],
                    },
                }
            ],
        }
    )


def test_polygons_from_geojson_proyecta_el_rectangulo_del_cockpit() -> None:
    texto = _geojson_de_zona(ZONA_CUADRADA)
    poligonos = polygons_from_geojson(
        texto, origin_lat=-31.42, origin_lon=-64.19, origin_yaw_deg=0.0
    )
    assert len(poligonos) == 1
    for obtenido, esperado in zip(poligonos[0], ZONA_CUADRADA):
        assert math.isclose(obtenido[0], esperado[0], abs_tol=1.0e-6)
        assert math.isclose(obtenido[1], esperado[1], abs_tol=1.0e-6)


def test_polygons_from_geojson_saltea_zonas_apagadas_y_de_otro_tipo() -> None:
    apagada = _geojson_de_zona(ZONA_CUADRADA, enabled=False)
    otro_tipo = _geojson_de_zona(ZONA_CUADRADA, zone_type="slow_down")
    for texto in (apagada, otro_tipo, "", "   "):
        assert (
            polygons_from_geojson(
                texto, origin_lat=-31.42, origin_lon=-64.19, origin_yaw_deg=0.0
            )
            == []
        )


# Lote de prueba en marco del cuerpo para los casos de borde: forward 0..38,
# left 0..40. Compartido con `cockpit/src/test/coverageNoGo.test.ts`.
LOTE = (0.0, 38.0, 0.0, 40.0)


def _zona_centrada(forward_c: float, left_c: float):
    return [
        (forward_c - 2.6, left_c - 1.6),
        (forward_c + 2.6, left_c - 1.6),
        (forward_c + 2.6, left_c + 1.6),
        (forward_c - 2.6, left_c + 1.6),
    ]


@pytest.mark.parametrize("left_c", [1.0, 2.5, 4.0, 6.0])
def test_una_zona_pegada_al_borde_no_hace_salir_el_rodeo_del_lote(left_c: float) -> None:
    # Sin el rectangulo del lote, el lado corto del contorno cae afuera y el
    # rodeo se dibuja saliendo del campo. Con zonas cerca del borde tiene que
    # ganar la vuelta larga por adentro aunque sea mas larga.
    zona = _zona_centrada(21.4, left_c)
    plan = _fila([(f, left_c) for f in (0.0, 10.0, 21.4, 30.0, 38.0)])
    recortado, _, rodeos = clip_plan_to_nogo(
        plan, [zona], margin_m=4.4, bounds=LOTE
    )
    assert rodeos >= 1
    for item in recortado:
        assert -0.5 <= float(item.forward_m) <= 38.5
        assert -0.5 <= float(item.left_m) <= 40.5


def test_sin_lote_el_rodeo_puede_salirse() -> None:
    # El comportamiento de antes, que es el que hay que evitar: se documenta para
    # que quede claro que el rectangulo es lo unico que lo impide.
    zona = _zona_centrada(21.4, 2.5)
    plan = _fila([(0.0, 2.5), (38.0, 2.5)])
    recortado, _, _ = clip_plan_to_nogo(plan, [zona], margin_m=4.4)
    assert min(float(item.left_m) for item in recortado) < -0.5


def test_el_lote_no_cambia_el_rodeo_cuando_la_zona_esta_en_el_medio() -> None:
    zona = _zona_centrada(21.4, 20.0)
    plan = _fila([(0.0, 20.0), (38.0, 20.0)])
    con, _, _ = clip_plan_to_nogo(plan, [zona], margin_m=4.4, bounds=LOTE)
    sin, _, _ = clip_plan_to_nogo(plan, [zona], margin_m=4.4)
    assert _posiciones(con) == _posiciones(sin)


def test_zona_que_cruza_el_lote_de_lado_a_lado_igual_devuelve_un_rodeo() -> None:
    # Los dos lados se salen; no hay por donde bordear, pero no se puede dejar
    # el tramo cruzando la zona: se toma el mas corto igual.
    zona = [(18.0, -50.0), (24.0, -50.0), (24.0, 50.0), (18.0, 50.0)]
    plan = _fila([(0.0, 20.0), (38.0, 20.0)])
    _, _, rodeos = clip_plan_to_nogo(plan, [zona], margin_m=1.0, bounds=LOTE)
    assert rodeos >= 1


# ---------------------------------------------------------------------------
# Campo con Fields2Cover tambien recorta.
#
# Se lee el fuente en vez de levantar el nodo: lo que hay que garantizar no es
# que en un escenario concreto se recorte, sino que no exista un camino por el
# que Campo planifique ignorando las zonas. Si el recorte desaparece de esa
# rama, el cockpit lo repite por su cuenta, ve que el backend no lo hizo y
# bloquea el arranque con "el backend no aplico las zonas no-go" — que es
# exactamente el sintoma que este test evita que vuelva.
# ---------------------------------------------------------------------------

_FUENTE_ROUTE_EXECUTOR = (
    pathlib.Path(__file__).resolve().parents[1] / "navegacion_gps" / "route_executor.py"
)


def _cuerpo_de_metodo(nombre: str) -> ast.AST:
    arbol = ast.parse(_FUENTE_ROUTE_EXECUTOR.read_text(encoding="utf-8"))
    for nodo in ast.walk(arbol):
        if isinstance(nodo, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if nodo.name == nombre:
                return nodo
    raise AssertionError(f"{nombre} ya no existe en route_executor.py")


def _nombres_llamados(nodo: ast.AST) -> set:
    salida = set()
    for hijo in ast.walk(nodo):
        if isinstance(hijo, ast.Call):
            func = hijo.func
            if isinstance(func, ast.Name):
                salida.add(func.id)
            elif isinstance(func, ast.Attribute):
                salida.add(func.attr)
    return salida


def test_la_rama_de_fields2cover_aplica_el_recorte_de_zonas() -> None:
    llamados = _nombres_llamados(_cuerpo_de_metodo("_generate_coverage_plan_fields2cover"))
    assert "_coverage_no_go_polygons" in llamados
    assert "clip_plan_to_nogo" in llamados


def test_la_respuesta_de_fields2cover_informa_las_zonas_aplicadas() -> None:
    metodo = _cuerpo_de_metodo("_fill_fields2cover_response")
    asignados = {
        destino.attr
        for nodo in ast.walk(metodo)
        if isinstance(nodo, ast.Assign)
        for destino in nodo.targets
        if isinstance(destino, ast.Attribute)
    }
    for campo in (
        "nogo_polygon_count",
        "nogo_dropped_count",
        "nogo_detour_count",
        "nogo_note",
    ):
        assert campo in asignados, f"la respuesta de Fields2Cover no informa {campo}"
