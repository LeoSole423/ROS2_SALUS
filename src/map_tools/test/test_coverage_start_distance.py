"""El preflight de CAMPO mide la distancia al LOTE, no a la primera pasada.

Fields2Cover elige el swath inicial por la forma del lote, no por donde esta
parado el vehiculo. Medir contra esa pasada rechazaba arranques legitimos: en un
lote de 22 m el sorteo podia caer en el extremo opuesto y sumar la diagonal
entera. El criterio que se quiere conservar -no arrancar la cobertura de un lote
lejano- se responde contra el poligono del lote.
"""

import math
from pathlib import Path

import pytest

from map_tools.web_zone_server import WebZoneServerNode, _ring_vertices


# Un cuadrado de ~20 m de lado, con el vertice inferior izquierdo en el origen.
# A esta latitud un grado de longitud son ~95 km, asi que los numeros de abajo
# se eligieron en metros y se convirtieron, no al reves.
_LAT0 = -31.4857
_LON0 = -64.2406
_M_POR_GRADO_LAT = 111_320.0
_M_POR_GRADO_LON = _M_POR_GRADO_LAT * abs(math.cos(math.radians(_LAT0)))


def _punto(este_m: float, norte_m: float) -> dict:
    return {
        "lat": _LAT0 + norte_m / _M_POR_GRADO_LAT,
        "lon": _LON0 + este_m / _M_POR_GRADO_LON,
    }


def _lote_20m() -> list:
    return [_punto(0.0, 0.0), _punto(20.0, 0.0), _punto(20.0, 20.0), _punto(0.0, 20.0)]


def _distancia_al_lote(este_m: float, norte_m: float, lote=None) -> float:
    referencia = _punto(este_m, norte_m)
    referencia["yaw_deg"] = 0.0
    anillo = _lote_20m() if lote is None else lote
    return WebZoneServerNode._distance_to_ring_m(
        WebZoneServerNode._offsets_to_reference_m(referencia, anillo)
    )


def test_adentro_del_lote_da_cero():
    assert _distancia_al_lote(10.0, 10.0) == 0.0


def test_sobre_el_borde_da_cero_o_casi():
    assert _distancia_al_lote(0.0, 10.0) < 0.05


def test_afuera_mide_al_borde_mas_cercano_no_al_centro():
    # 5 m al oeste del borde oeste. Al centro habria 15 m: si el numero fuera 15
    # estaria midiendo contra otra cosa.
    assert _distancia_al_lote(-5.0, 10.0) == pytest.approx(5.0, abs=0.05)


def test_esquina_diagonal():
    # 3 m al oeste y 4 m al sur de la esquina (0,0): 5 m por Pitagoras.
    assert _distancia_al_lote(-3.0, -4.0) == pytest.approx(5.0, abs=0.05)


def test_la_diagonal_del_lote_no_cuenta_como_distancia():
    # Este es el caso que rompia en el robot: el vehiculo parado en una esquina
    # del lote mientras Fields2Cover elige la pasada del extremo opuesto. Contra
    # la pasada serian ~28 m; contra el lote es cero, porque esta adentro.
    assert _distancia_al_lote(0.5, 0.5) == 0.0


def test_sin_poligono_devuelve_inf_para_que_el_llamador_use_el_fallback():
    # Menos de tres vertices no es un poligono. Devolver inf -y no un numero
    # grande- es lo que deja al preflight distinguir "lejos" de "no habia lote".
    assert math.isinf(_distancia_al_lote(0.0, 0.0, lote=[]))
    assert math.isinf(_distancia_al_lote(0.0, 0.0, lote=[_punto(0.0, 0.0)]))
    assert math.isinf(
        _distancia_al_lote(0.0, 0.0, lote=[_punto(0.0, 0.0), _punto(1.0, 0.0)])
    )


def test_ring_vertices_acepta_las_dos_formas_del_cockpit():
    vertices = _lote_20m()
    assert _ring_vertices({"vertices": vertices}) == vertices
    assert _ring_vertices(vertices) == vertices
    assert _ring_vertices(None) == []
    assert _ring_vertices("no es un anillo") == []


def test_vertices_rotos_no_rompen_la_medicion():
    lote = _lote_20m()
    lote.insert(2, {"lat": "nan", "lon": None})
    lote.insert(0, {"sin": "lat ni lon"})
    assert _distancia_al_lote(10.0, 10.0, lote=lote) == 0.0


def test_el_perfil_real_le_pasa_el_umbral_al_cockpit():
    # Sin esto el nodo se queda en su default de 5.0 m y CAMPO planifica bien
    # pero nunca arranca en el robot, que es exactamente lo que fallaba.
    raiz = Path(__file__).resolve().parents[3]
    perfil = raiz / "navegacion_gps" / "launch" / "real_global_v2.launch.py"
    if not perfil.exists():
        perfil = raiz / "src" / "navegacion_gps" / "launch" / "real_global_v2.launch.py"
    texto = perfil.read_text()
    assert '"coverage_start_max_distance_m"' in texto, (
        "el perfil real no le pasa coverage_start_max_distance_m al cockpit"
    )
