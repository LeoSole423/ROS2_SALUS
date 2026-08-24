"""El arbol de comportamiento no puede perderle la carrera a su servicio.

`navigate_through_poses_w_replanning_and_recovery_no_spin.xml` usa el nodo
`IsPathClearanceValid`, que es un `BtServiceNode` de Nav2. Esos nodos esperan el
servicio AL CONSTRUIR EL ARBOL y, si no aparece, tiran `std::runtime_error`: el
XML no carga, `bt_navigator` no configura y el lifecycle manager aborta el
bringup entero. La navegacion global queda muerta aunque el servicio systemd
siga "active".

Medido en la Jetson Orin: con el default de 1 s de Nav2, `path_clearance_validator`
no llegaba a publicar su servicio y el arranque fallaba. En una maquina de
escritorio no se reproduce porque el servicio aparece antes del segundo.
"""

from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

PERFILES = (
    "nav2_global_v2_real_rolling_params",
    "nav2_global_v2_real_rolling_wifi_params",
    "nav2_global_v2_sim_rolling_params",
    "nav2_global_v2_sim_rolling_wifi_params",
)


@pytest.mark.parametrize("perfil", PERFILES)
def test_el_arbol_espera_su_servicio_lo_suficiente(perfil: str) -> None:
    contenido = (PACKAGE_ROOT / "config" / f"{perfil}.yaml").read_text(encoding="utf-8")
    assert "wait_for_service_timeout" in contenido, (
        f"{perfil} no fija wait_for_service_timeout: con el default de 1 s de "
        "Nav2, bt_navigator falla al cargar el arbol en una maquina lenta"
    )
    for linea in contenido.splitlines():
        if "wait_for_service_timeout" in linea and not linea.strip().startswith("#"):
            valor_ms = int(linea.split(":", 1)[1].strip())
            assert valor_ms >= 10000, (
                f"{perfil} espera solo {valor_ms} ms; en la Jetson el servicio "
                "de path_clearance_validator tarda mas que eso en aparecer"
            )
            break


def test_el_validador_se_lanza_antes_que_bt_navigator() -> None:
    """El orden no garantiza, pero no hay razon para darle la desventaja."""
    launch = (PACKAGE_ROOT / "launch" / "nav_global_v2.launch.py").read_text(
        encoding="utf-8"
    )
    validador = launch.index('executable="path_clearance_validator"')
    bt = launch.index('executable="bt_navigator"')
    assert validador < bt, (
        "path_clearance_validator se lanza despues de bt_navigator; el arbol "
        "consume su servicio al construirse"
    )


def test_el_arbol_sigue_usando_el_nodo_de_clearance() -> None:
    """Si el BT dejara de usarlo, los otros dos tests perderian sentido."""
    arbol = (
        PACKAGE_ROOT
        / "config"
        / "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    ).read_text(encoding="utf-8")
    assert "IsPathClearanceValid" in arbol
    for perfil in PERFILES:
        contenido = (PACKAGE_ROOT / "config" / f"{perfil}.yaml").read_text(
            encoding="utf-8"
        )
        assert "nav2_is_path_clearance_valid_condition_bt_node" in contenido, (
            f"{perfil} no declara el plugin que el arbol necesita"
        )
