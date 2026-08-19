"""El planner agricola entra SOLO por Campo.

Estos tests no arrancan ROS: leen el fuente del route_executor y comprueban
donde puede y donde no puede aparecer el planificador de cobertura. Es a
proposito. Un test de integracion probaria que en un escenario concreto no se
usa; esto prueba que no hay ningun camino de codigo por el que pueda usarse,
que es lo que se quiere garantizar.
"""

import ast
import pathlib

FUENTE = pathlib.Path(__file__).resolve().parents[1] / "navegacion_gps" / "route_executor.py"

# Todo lo que planifica cobertura. Si aparece fuera de los metodos de Campo, es
# que se filtro a otra modalidad.
SIMBOLOS_DE_COBERTURA = {
    # Planificador propio
    "build_lawnmower_waypoints",
    "resolve_row_visit_order",
    "headland_turn_length_m",
    "validate_coverage_field",
    "clip_plan_to_nogo",
    # Planificador agricola
    "Fields2CoverPlanner",
    "Fields2CoverError",
    "_fields2cover_planner",
    "_generate_coverage_plan_fields2cover",
    "_fill_fields2cover_response",
}

# Los unicos metodos que pueden tocarlos: los del servicio de Campo.
METODOS_DE_CAMPO = {
    "_validate_generate_coverage_request",
    "_on_generate_coverage_plan",
    "_coverage_no_go_polygons",
    "_coverage_warmup",
    "_fields2cover_planner",
    "_generate_coverage_plan_fields2cover",
    "_fill_fields2cover_response",
}

# Entradas de las otras modalidades. Ninguna puede terminar en cobertura.
METODOS_DE_OTRAS_MISIONES = {
    "_on_set_route",              # AUTOMATIC ROUTE
    "_on_cancel_route",
    "_on_get_state",
    "_on_set_patrol",             # PATROL
    "_on_cancel_patrol",
    "_on_get_patrol_state",
    "_on_request_return_home",    # vuelta a HOME
    "_on_set_navigation_profile",
}


def _metodos(arbol):
    salida = {}
    for nodo in ast.walk(arbol):
        if isinstance(nodo, ast.ClassDef):
            for hijo in nodo.body:
                if isinstance(hijo, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    salida[hijo.name] = hijo
    return salida


def _nombres_usados(nodo):
    """Nombres que el metodo USA de verdad, sin contar anotaciones de tipo.

    `self._fields2cover: Optional[Fields2CoverPlanner] = None` declara el
    atributo, no planifica nada. Se descartan las anotaciones en vez de
    exceptuar `__init__` entero, para que un uso real ahi tampoco pase.
    """
    anotaciones = {
        id(hijo.annotation)
        for hijo in ast.walk(nodo)
        if isinstance(hijo, (ast.AnnAssign, ast.arg)) and hijo.annotation is not None
    }
    ignorados = set()
    for hijo in ast.walk(nodo):
        if id(hijo) in anotaciones:
            ignorados |= {id(sub) for sub in ast.walk(hijo)}
    return {
        n.id for n in ast.walk(nodo)
        if isinstance(n, ast.Name) and id(n) not in ignorados
    } | {
        n.attr for n in ast.walk(nodo)
        if isinstance(n, ast.Attribute) and id(n) not in ignorados
    }


def test_solo_campo_usa_el_planificador_de_cobertura() -> None:
    arbol = ast.parse(FUENTE.read_text())
    metodos = _metodos(arbol)
    culpables = {
        nombre: sorted(_nombres_usados(nodo) & SIMBOLOS_DE_COBERTURA)
        for nombre, nodo in metodos.items()
        if nombre not in METODOS_DE_CAMPO
        and (_nombres_usados(nodo) & SIMBOLOS_DE_COBERTURA)
    }
    assert not culpables, (
        "el planificador de cobertura se filtro fuera de Campo: "
        f"{culpables}. Campo tiene que seguir siendo el unico que planifica "
        "cobertura; Ruta, Patrol y goals no pueden tocarlo."
    )


def test_ruta_patrol_y_goals_no_planifican_cobertura() -> None:
    arbol = ast.parse(FUENTE.read_text())
    metodos = _metodos(arbol)
    faltantes = METODOS_DE_OTRAS_MISIONES - set(metodos)
    assert not faltantes, (
        f"no existen estos handlers: {sorted(faltantes)}. Si se renombraron hay "
        "que actualizar la lista: si no, este test pasaria sin comprobar nada."
    )
    presentes = METODOS_DE_OTRAS_MISIONES
    for nombre in sorted(presentes):
        usados = _nombres_usados(metodos[nombre]) & SIMBOLOS_DE_COBERTURA
        assert not usados, f"{nombre}() usa {sorted(usados)}, que es de Campo"


def test_el_parametro_coverage_planner_solo_se_lee_en_campo() -> None:
    # Si otra modalidad empieza a mirar `coverage_planner`, es senal de que la
    # bifurcacion del planner agricola se escapo de Campo.
    arbol = ast.parse(FUENTE.read_text())
    metodos = _metodos(arbol)
    permitidos = METODOS_DE_CAMPO | {"__init__", "_read_parameters", "_load_parameters"}
    culpables = [
        nombre
        for nombre, nodo in metodos.items()
        if nombre not in permitidos and "coverage_planner" in _nombres_usados(nodo)
    ]
    assert not culpables, f"coverage_planner se lee fuera de Campo en {culpables}"


def test_el_modulo_de_poligonos_no_lo_importa_nadie_mas() -> None:
    raiz = FUENTE.parent
    ofensores = []
    for archivo in raiz.glob("*.py"):
        if archivo.name in {
            "route_executor.py",
            "coverage_field_polygon.py",
            "coverage_waypoint_core.py",
            "coverage_waypoint_mission.py",
            "coverage_nogo.py",
            "coverage_nogo_zones.py",
        }:
            continue
        if "coverage_field_polygon" in archivo.read_text():
            ofensores.append(archivo.name)
    assert not ofensores, f"modulos ajenos a Campo importan el poligono: {ofensores}"


def test_el_planner_agricola_solo_se_construye_en_campo() -> None:
    # Lo que pidio Franco explicitamente: que falle si Fields2Cover se empieza a
    # usar fuera de Campo. Se mira la construccion del cliente, que es la unica
    # puerta de entrada al planner agricola.
    arbol = ast.parse(FUENTE.read_text())
    metodos = _metodos(arbol)
    culpables = [
        nombre
        for nombre, nodo in metodos.items()
        if nombre not in METODOS_DE_CAMPO
        and nombre != "__init__"
        and "Fields2CoverPlanner" in _nombres_usados(nodo)
    ]
    assert not culpables, (
        f"{culpables} construye el planner agricola. Fields2Cover entra "
        "unicamente por el flujo de Campo."
    )


def test_el_modulo_de_fields2cover_no_lo_importa_nadie_mas() -> None:
    raiz = FUENTE.parent
    permitidos = {"route_executor.py", "coverage_fields2cover.py"}
    ofensores = [
        archivo.name
        for archivo in raiz.glob("*.py")
        if archivo.name not in permitidos
        and "coverage_fields2cover" in archivo.read_text()
    ]
    assert not ofensores, f"modulos ajenos a Campo importan Fields2Cover: {ofensores}"
