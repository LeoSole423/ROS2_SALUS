import pytest

from navegacion_gps.coverage_field_polygon import point_in_ring
from navegacion_gps.coverage_field_polygon import ring_is_simple
from navegacion_gps.coverage_field_polygon import ring_signed_area_m2
from navegacion_gps.coverage_field_polygon import ring_to_local_m
from navegacion_gps.coverage_field_polygon import validate_coverage_field

# Lote de prueba: un cuadrado de ~40 m cerca de Cordoba, y una L irregular para
# los casos que un rectangulo no puede representar.
LAT0, LON0 = -31.4859, -64.2409
DLAT = 40.0 / 111_320.0
DLON = 40.0 / (111_320.0 * 0.853)  # cos(31.4859 grados)

CUADRADO = [
    (LAT0, LON0),
    (LAT0, LON0 + DLON),
    (LAT0 + DLAT, LON0 + DLON),
    (LAT0 + DLAT, LON0),
]
LOTE_EN_L = [
    (LAT0, LON0),
    (LAT0, LON0 + DLON),
    (LAT0 + DLAT / 2, LON0 + DLON),
    (LAT0 + DLAT / 2, LON0 + DLON / 2),
    (LAT0 + DLAT, LON0 + DLON / 2),
    (LAT0 + DLAT, LON0),
]


def _exclusion(centro_lat_frac: float, centro_lon_frac: float, lado_frac: float = 0.1):
    lat = LAT0 + DLAT * centro_lat_frac
    lon = LON0 + DLON * centro_lon_frac
    dlat, dlon = DLAT * lado_frac, DLON * lado_frac
    return [
        (lat - dlat, lon - dlon),
        (lat - dlat, lon + dlon),
        (lat + dlat, lon + dlon),
        (lat + dlat, lon - dlon),
    ]


def test_sin_poligono_no_es_error_es_modo_legacy() -> None:
    # Un pedido viejo llega sin poligono: eso no puede fallar, tiene que caer al
    # rectangulo de siempre.
    exterior, exclusiones, error = validate_coverage_field([], [])
    assert (exterior, exclusiones, error) == (None, [], "")


def test_acepta_un_poligono_de_mas_de_cuatro_vertices() -> None:
    # Lo que un rectangulo no puede representar y es el motivo del cambio.
    exterior, exclusiones, error = validate_coverage_field(LOTE_EN_L)
    assert error == ""
    assert exterior is not None and len(exterior) == 6
    assert exclusiones == []


def test_saca_el_cierre_repetido() -> None:
    cerrado = list(CUADRADO) + [CUADRADO[0]]
    exterior, _, error = validate_coverage_field(cerrado)
    assert error == ""
    assert exterior is not None and len(exterior) == 4


@pytest.mark.parametrize("vertices", [[], [(LAT0, LON0)], [(LAT0, LON0), (LAT0, LON0 + DLON)]])
def test_menos_de_tres_vertices(vertices) -> None:
    if not vertices:
        pytest.skip("el anillo vacio es modo legacy, no un error")
    _, _, error = validate_coverage_field(vertices)
    assert "al menos 3 vertices" in error


def test_rechaza_un_poligono_que_se_cruza() -> None:
    # Un moño: el operador cerro el poligono cruzando dos lados.
    mono = [
        (LAT0, LON0),
        (LAT0 + DLAT, LON0 + DLON),
        (LAT0 + DLAT, LON0),
        (LAT0, LON0 + DLON),
    ]
    _, _, error = validate_coverage_field(mono)
    assert "se cruza a si mismo" in error


def test_rechaza_vertices_colineales() -> None:
    linea = [(LAT0, LON0), (LAT0, LON0 + DLON / 2), (LAT0, LON0 + DLON)]
    _, _, error = validate_coverage_field(linea)
    assert "degenerado" in error


def test_rechaza_vertices_repetidos() -> None:
    repetido = [CUADRADO[0], CUADRADO[0], CUADRADO[1], CUADRADO[2]]
    _, _, error = validate_coverage_field(repetido)
    assert "repetidos" in error


def test_rechaza_lat_lon_fuera_de_rango() -> None:
    _, _, error = validate_coverage_field([(0.0, 0.0), (0.0, 1.0), (95.0, 1.0)])
    assert "fuera del rango" in error


def test_acepta_exclusiones_adentro_del_lote() -> None:
    exterior, exclusiones, error = validate_coverage_field(
        CUADRADO, [_exclusion(0.5, 0.5), _exclusion(0.25, 0.25)]
    )
    assert error == ""
    assert exterior is not None
    assert len(exclusiones) == 2


def test_rechaza_una_exclusion_fuera_del_lote() -> None:
    _, _, error = validate_coverage_field(CUADRADO, [_exclusion(2.0, 2.0)])
    assert "fuera del poligono del lote" in error


def test_rechaza_una_exclusion_que_cruza_el_borde() -> None:
    # A caballo del borde: la mitad adentro y la mitad afuera. Con solo mirar
    # los vertices podria pasar, por eso tambien se chequean los cortes.
    _, _, error = validate_coverage_field(CUADRADO, [_exclusion(0.5, 1.0, 0.2)])
    assert "fuera del poligono del lote" in error or "cruza el borde" in error


def test_rechaza_una_exclusion_que_se_cruza_a_si_misma() -> None:
    mono = [
        (LAT0 + DLAT * 0.4, LON0 + DLON * 0.4),
        (LAT0 + DLAT * 0.6, LON0 + DLON * 0.6),
        (LAT0 + DLAT * 0.6, LON0 + DLON * 0.4),
        (LAT0 + DLAT * 0.4, LON0 + DLON * 0.6),
    ]
    _, _, error = validate_coverage_field(CUADRADO, [mono])
    assert "se cruza a si mismo" in error


def test_una_exclusion_vacia_se_ignora_sin_error() -> None:
    _, exclusiones, error = validate_coverage_field(CUADRADO, [[], _exclusion(0.5, 0.5)])
    assert error == ""
    assert len(exclusiones) == 1


def test_la_validacion_no_depende_del_sentido_de_los_vertices() -> None:
    # El operador dibuja en el sentido que le sale; los dos tienen que valer.
    _, _, error_ccw = validate_coverage_field(CUADRADO)
    _, _, error_cw = validate_coverage_field(list(reversed(CUADRADO)))
    assert error_ccw == "" and error_cw == ""


def test_los_chequeos_corren_en_metros_no_en_grados() -> None:
    # A esta latitud un grado de longitud mide 0.85 de uno de latitud. Sobre
    # grados crudos el area del cuadrado saldria distorsionada; en metros tiene
    # que dar 40 x 40 = 1600 m2.
    local = ring_to_local_m(CUADRADO, CUADRADO[0])
    assert abs(abs(ring_signed_area_m2(local)) - 1600.0) < 5.0


def test_helpers_de_geometria() -> None:
    local = ring_to_local_m(CUADRADO, CUADRADO[0])
    assert ring_is_simple(local) is True
    assert point_in_ring((20.0, 20.0), local) is True
    assert point_in_ring((100.0, 20.0), local) is False
