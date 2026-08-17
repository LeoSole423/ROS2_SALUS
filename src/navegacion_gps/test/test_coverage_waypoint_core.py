import math

import pytest

from navegacion_gps.coverage_waypoint_core import analyze_polyline_topology
from navegacion_gps.coverage_waypoint_core import analyze_polyline_topology_in_bounds
from navegacion_gps.coverage_waypoint_core import build_lawnmower_body_plan
from navegacion_gps.coverage_waypoint_core import build_lawnmower_waypoints
from navegacion_gps.coverage_waypoint_core import evaluate_row_visit_order
from navegacion_gps.coverage_waypoint_core import headland_turn_length_m
from navegacion_gps.coverage_waypoint_core import is_clean_uturn
from navegacion_gps.coverage_waypoint_core import max_separation_row_order
from navegacion_gps.coverage_waypoint_core import minimum_safe_lane_spacing_m
from navegacion_gps.coverage_waypoint_core import resolve_row_visit_order


def test_lawnmower_plan_covers_width_without_exceeding_requested_spacing() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=8.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=2.0,
    )

    assert plan.row_count == 5
    assert plan.lane_spacing_m == pytest.approx(1.5)
    assert plan.lane_spacing_m <= 2.0 * (1.0 - 0.15)

    rows = {
        point.row_index: [
            item
            for item in plan.waypoints
            if item.phase == "row" and item.row_index == point.row_index
        ]
        for point in plan.waypoints
    }
    assert min(item.left_m for item in rows[0]) == pytest.approx(0.0)
    assert max(item.left_m for item in rows[4]) == pytest.approx(6.0)


def test_lawnmower_plan_alternates_rows_and_uses_external_headlands() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=8.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=1.0,
    )

    # El sentido alterna con el orden de visita. Por defecto ese orden es la
    # serpentina adyacente, pero el recorrido puede saltar pasadas si el perfil
    # lo habilita, y entonces el sentido sigue al orden y no al indice.
    assert plan.row_visit_order == tuple(range(plan.row_count))
    for visit_index, row_index in enumerate(plan.row_visit_order):
        row_points = [
            point
            for point in plan.waypoints
            if point.phase == "row" and point.row_index == row_index
        ]
        start_x, end_x = (0.0, 20.0) if visit_index % 2 == 0 else (20.0, 0.0)
        heading_deg = 0.0 if visit_index % 2 == 0 else 180.0
        assert row_points[0].forward_m == pytest.approx(start_x)
        assert row_points[-1].forward_m == pytest.approx(end_x)
        assert row_points[0].yaw_delta_deg == pytest.approx(heading_deg)

    assert plan.headland_before_m > plan.min_turning_radius_m
    assert plan.headland_after_m > plan.min_turning_radius_m


def test_lawnmower_plan_respects_waypoint_spacing() -> None:
    spacing_m = 1.25
    plan = build_lawnmower_body_plan(
        field_length_m=16.0,
        field_width_m=6.0,
        cutter_width_m=2.0,
        overlap_ratio=0.1,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=spacing_m,
    )

    distances = [
        math.hypot(
            current.forward_m - previous.forward_m,
            current.left_m - previous.left_m,
        )
        for previous, current in zip(plan.waypoints, plan.waypoints[1:])
    ]
    assert distances
    assert max(distances) <= spacing_m + 1.0e-9


def test_lawnmower_plan_can_place_rows_on_vehicle_right() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=10.0,
        field_width_m=5.0,
        cutter_width_m=2.0,
        overlap_ratio=0.0,
        min_turning_radius_m=3.0,
        side="right",
    )

    row_points = [point for point in plan.waypoints if point.phase == "row"]
    assert min(point.left_m for point in row_points) == pytest.approx(-3.0)
    assert max(point.left_m for point in row_points) == pytest.approx(0.0)


def test_lawnmower_geographic_waypoints_preserve_phases_and_headings() -> None:
    plan, waypoints = build_lawnmower_waypoints(
        start_lat=-31.48,
        start_lon=-64.24,
        start_yaw_deg=30.0,
        field_length_m=12.0,
        field_width_m=4.0,
        cutter_width_m=2.0,
        overlap_ratio=0.0,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=2.0,
    )

    assert len(waypoints) == len(plan.waypoints)
    assert waypoints[0]["lat"] == pytest.approx(-31.48)
    assert waypoints[0]["lon"] == pytest.approx(-64.24)
    assert waypoints[0]["yaw_deg"] == pytest.approx(30.0)
    assert waypoints[0]["phase"] == "row"
    assert any(item["phase"] == "turn" for item in waypoints)


def test_headland_turn_needs_two_radii_of_separation_for_a_clean_uturn() -> None:
    radius = 4.0

    assert headland_turn_length_m(2.0 * radius, min_turning_radius_m=radius) == pytest.approx(
        math.pi * radius, abs=1.0e-6
    )
    assert is_clean_uturn(2.0 * radius, min_turning_radius_m=radius)
    assert not is_clean_uturn(1.5, min_turning_radius_m=radius)
    # Por debajo del diametro de giro el camino mas corto es un omega, mucho mas largo.
    assert headland_turn_length_m(1.5, min_turning_radius_m=radius) > 2.0 * math.pi * radius


def test_row_skipping_widens_the_turn_even_when_no_uturn_fits() -> None:
    # 5 pasadas a 1.5 m cubren 6 m; el diametro de giro son 8 m, no entra ninguna
    # U simple. Aun asi separar el giro paga: el desborde del omega decrece de
    # forma monotona con la separacion, asi que el orden de maxima separacion deja
    # una cabecera mas chica y un camino mas corto que la serpentina adyacente.
    order = resolve_row_visit_order(
        row_count=5,
        lane_spacing_m=1.5,
        min_turning_radius_m=4.0,
        allow_row_skipping=True,
    )

    assert order == max_separation_row_order(5) == (0, 3, 1, 4, 2)

    row_offsets = [1.5 * index for index in range(5)]
    adjacent_metrics = evaluate_row_visit_order(
        field_length_m=20.0,
        row_offsets=row_offsets,
        visit_order=(0, 1, 2, 3, 4),
        turning_radius_m=4.0,
    )
    chosen_metrics = evaluate_row_visit_order(
        field_length_m=20.0,
        row_offsets=row_offsets,
        visit_order=order,
        turning_radius_m=4.0,
    )

    assert chosen_metrics.minimum_separation_m > adjacent_metrics.minimum_separation_m
    assert chosen_metrics.headland_depth_m < adjacent_metrics.headland_depth_m
    assert chosen_metrics.path_length_m < adjacent_metrics.path_length_m


def test_max_separation_row_order_starts_at_the_vehicle_row_and_covers_every_row() -> None:
    # Arrancar en la pasada 0 es un requisito del preflight de start_coverage: la
    # primera meta tiene que caer al lado del vehiculo.
    for row_count in range(2, 40):
        order = max_separation_row_order(row_count)

        assert sorted(order) == list(range(row_count))
        assert order[0] == 0
        gaps = [abs(order[index + 1] - order[index]) for index in range(row_count - 1)]
        # Maximo alcanzable para un recorrido que empieza en la pasada 0.
        assert min(gaps) == max(1, (row_count - 1) // 2)


def test_coverage_walks_the_rows_one_by_one_unless_the_profile_allows_skipping() -> None:
    """Regresion CAMPO: el lote se recorre pasada por pasada, sin saltear.

    Es el recorrido que se espera de una cosechadora: se arranca en la pasada
    del vehiculo y se avanza hacia el borde opuesto, de a una. Ninguna metrica de
    cabecera puede cambiar ese orden; lo unico que lo cambia es que el perfil
    pida explicitamente el salteo.
    """

    for row_count, spacing in ((3, 9.0), (5, 1.5), (12, 1.636), (40, 0.5)):
        order = resolve_row_visit_order(
            row_count=row_count,
            lane_spacing_m=spacing,
            min_turning_radius_m=4.0,
            field_length_m=20.0,
            cutter_width_m=2.0,
        )

        assert order == tuple(range(row_count))
        gaps = {order[index + 1] - order[index] for index in range(row_count - 1)}
        assert gaps == {1}

    # El caso de la captura: 12 pasadas a 1.64 m, donde el criterio de cabecera
    # prefiere el salteo por 10.4 m contra 4 m y sin embargo no manda.
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=0.5,
        allow_headland_conflicts=True,
    )

    assert plan.row_count == 12
    assert plan.row_visit_order == tuple(range(12))
    assert resolve_row_visit_order(
        row_count=12,
        lane_spacing_m=float(plan.lane_spacing_m),
        min_turning_radius_m=4.0,
        field_length_m=20.0,
        cutter_width_m=2.0,
        allow_row_skipping=True,
    ) != tuple(range(12))


def test_row_visit_order_without_field_length_only_shortens_turns() -> None:
    # Sin field_length_m no se puede construir la poligonal, asi que la consulta
    # cae al criterio de costo y devuelve el salto-de-pasada sin auditarlo. Es una
    # consulta informativa: la autoridad es la variante que recibe el largo.
    order = resolve_row_visit_order(
        row_count=12,
        lane_spacing_m=1.636,
        min_turning_radius_m=4.0,
        allow_row_skipping=True,
    )

    assert sorted(order) == list(range(12))
    separations = [
        abs(order[index + 1] - order[index]) * 1.636 for index in range(len(order) - 1)
    ]
    assert all(
        is_clean_uturn(separation, min_turning_radius_m=4.0) for separation in separations
    )


def test_square_field_plan_uses_clean_uturns_and_only_row_endpoints_are_key() -> None:
    # Un implemento que deja una separacion mayor al diametro de giro permite la
    # U simple con pasadas adyacentes, que es el unico patron sin cruces.
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=32.0,
        cutter_width_m=8.0,
        overlap_ratio=0.0,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=0.5,
    )

    assert plan.lane_spacing_m >= 2.0 * plan.min_turning_radius_m
    assert plan.row_visit_order == tuple(range(plan.row_count))
    assert plan.clean_uturn_count == len(plan.turn_separations_m)
    assert plan.is_topologically_safe
    assert len(plan.key_waypoints) == 2 * plan.row_count
    assert all(point.phase == "row" for point in plan.key_waypoints)
    # Las cabeceras de una U simple solo necesitan un radio de cabecera libre.
    assert plan.headland_after_m == pytest.approx(plan.min_turning_radius_m, abs=0.05)


def test_key_waypoints_are_the_ends_of_every_pass() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=8.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=2.0,
    )

    key_points = plan.key_waypoints
    assert len(key_points) == 2 * plan.row_count
    for index in range(0, len(key_points), 2):
        start, end = key_points[index], key_points[index + 1]
        assert start.row_index == end.row_index
        assert abs(end.forward_m - start.forward_m) == pytest.approx(20.0)
        assert start.left_m == pytest.approx(end.left_m)


def test_route_waypoints_add_one_outer_guide_per_adjacent_headland() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=8.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=0.5,
        allow_headland_conflicts=True,
    )

    route_points = plan.route_waypoints
    guides = [point for point in route_points if point.is_guide]
    assert len(route_points) == (2 * plan.row_count) + (plan.row_count - 1)
    assert len(guides) == plan.row_count - 1
    assert all(point.phase == "turn" for point in guides)
    assert all(not point.is_key for point in guides)
    assert [point.is_key for point in route_points] == [
        flag
        for row_index in range(plan.row_count)
        for flag in (
            ([True, True, False] if row_index < plan.row_count - 1 else [True, True])
        )
    ]

    # La guia es el ultimo cambio de curvatura del giro, no el punto medio de
    # arco. Lo unico que se exige aca es que sea un punto propio, separado de
    # ambas metas key: tiene que viajar dentro del chunk, no confundirse con una
    # parada. Su ubicacion exacta la fija
    # test_headland_guide_sits_on_the_last_curvature_switch.
    for route_index, guide in (
        (index, point)
        for index, point in enumerate(route_points)
        if point.is_guide
    ):
        before = route_points[route_index - 1]
        after = route_points[route_index + 1]
        assert before.is_key
        assert after.is_key
        assert math.hypot(
            guide.forward_m - before.forward_m,
            guide.left_m - before.left_m,
        ) > 0.5
        assert math.hypot(
            after.forward_m - guide.forward_m,
            after.left_m - guide.left_m,
        ) > 0.5


def test_headland_guide_sits_on_the_last_curvature_switch() -> None:
    """La guia cae sobre el cambio de curvatura, no sobre una muestra cualquiera.

    Por eso no depende de ``waypoint_spacing_m``: el muestreo Dubins reinicia en
    cada tramo, asi que el arranque del ultimo arco siempre es un punto exacto.
    """

    def guides_for(waypoint_spacing_m: float) -> list[float]:
        plan = build_lawnmower_body_plan(
            field_length_m=35.0,
            field_width_m=20.0,
            cutter_width_m=5.0,
            overlap_ratio=0.0,
            min_turning_radius_m=4.0,
            waypoint_spacing_m=waypoint_spacing_m,
            allow_headland_conflicts=True,
        )
        guides = [
            point for point in plan.route_waypoints if point.is_guide
        ]
        assert len(guides) == plan.row_count - 1
        return [
            value
            for point in guides
            for value in (point.forward_m, point.left_m, point.yaw_delta_deg)
        ]

    reference = guides_for(2.0)
    assert guides_for(0.5) == pytest.approx(reference, abs=1.0e-9)
    assert guides_for(0.1) == pytest.approx(reference, abs=1.0e-9)

    plan = build_lawnmower_body_plan(
        field_length_m=35.0,
        field_width_m=20.0,
        cutter_width_m=5.0,
        overlap_ratio=0.0,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=0.1,
        allow_headland_conflicts=True,
    )
    points = list(plan.waypoints)
    guide_indices = [index for index, point in enumerate(points) if point.is_guide]
    assert len(guide_indices) == plan.row_count - 1
    for guide_index in guide_indices:
        guide = points[guide_index]
        assert guide.phase == "turn"
        assert not guide.is_key
        # El paso previo y el siguiente tienen curvatura distinta: el punto es
        # exactamente donde arranca el arco de salida. En un omega el signo se da
        # vuelta; en una U simple el tramo previo es la recta y no gira.
        before = points[guide_index].yaw_delta_deg - points[guide_index - 1].yaw_delta_deg
        after = points[guide_index + 1].yaw_delta_deg - points[guide_index].yaw_delta_deg
        assert after != pytest.approx(0.0, abs=1.0e-9)
        assert before * after <= 0.0
        assert before != pytest.approx(after, abs=1.0e-9)


def test_geographic_waypoints_expose_the_key_flag() -> None:
    _, waypoints = build_lawnmower_waypoints(
        start_lat=-31.48,
        start_lon=-64.24,
        start_yaw_deg=0.0,
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
    )

    key_waypoints = [item for item in waypoints if item["key"]]
    assert len(key_waypoints) == 24
    assert all(item["phase"] == "row" for item in key_waypoints)
    assert waypoints[0]["key"] is True
    assert sum(bool(item["guide"]) for item in waypoints) == 11


def test_sampled_turn_polyline_never_exceeds_the_minimum_turning_radius() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=8.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=1.0,
    )

    worst_curvature = 0.0
    points = plan.waypoints
    for first, second, third in zip(points, points[1:], points[2:]):
        side_a = math.hypot(second.forward_m - first.forward_m, second.left_m - first.left_m)
        side_b = math.hypot(third.forward_m - second.forward_m, third.left_m - second.left_m)
        side_c = math.hypot(third.forward_m - first.forward_m, third.left_m - first.left_m)
        if side_a * side_b * side_c < 1.0e-12:
            continue
        double_area = abs(
            (second.forward_m - first.forward_m) * (third.left_m - first.left_m)
            - (second.left_m - first.left_m) * (third.forward_m - first.forward_m)
        )
        worst_curvature = max(worst_curvature, 2.0 * double_area / (side_a * side_b * side_c))

    assert worst_curvature <= (1.0 / plan.min_turning_radius_m) + 1.0e-6


def test_polyline_topology_distinguishes_global_conflict_types() -> None:
    safe = analyze_polyline_topology([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)])
    crossing = analyze_polyline_topology(
        [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]
    )
    touching = analyze_polyline_topology(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 0.0)]
    )
    overlapping = analyze_polyline_topology(
        [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 0.0), (2.0, 0.0)]
    )

    assert safe.is_safe
    assert safe.conflict_count == 0
    assert crossing.strict_crossing_count == 1
    assert not crossing.is_safe
    assert touching.nonadjacent_touch_count == 1
    assert not touching.is_safe
    assert overlapping.collinear_overlap_count == 1
    assert not overlapping.is_safe


def test_bounded_topology_ignores_headland_crossings_outside_field() -> None:
    crossing = [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]

    assert analyze_polyline_topology_in_bounds(
        crossing,
        bounds_m=(0.5, 1.5, 0.5, 1.5),
    ).strict_crossing_count == 1
    assert analyze_polyline_topology_in_bounds(
        crossing,
        bounds_m=(2.1, 3.0, 0.5, 1.5),
    ).is_safe


def test_square_field_can_allow_conflicts_only_outside_its_physical_bounds() -> None:
    # 20x20 con corte 2 y solape 0.15 deja pasadas a 1.64 m, muy por debajo del
    # radio: cada cabecera es un omega que se cruza con el de la pasada siguiente.
    # Todos esos cruces caen fuera del lote, y el plan tiene que decir las dos
    # cosas por separado.
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=2.0,
        allow_headland_conflicts=True,
    )

    assert plan.lane_spacing_m < plan.minimum_safe_lane_spacing_m
    assert plan.strict_crossing_count > 0
    assert plan.topology_conflict_count == (
        plan.strict_crossing_count
        + plan.nonadjacent_touch_count
        + plan.collinear_overlap_count
    )
    assert not plan.is_topologically_safe
    assert plan.is_field_topologically_safe
    assert plan.field_topology_conflict_count == 0
    # Regla CAMPO: las pasadas se recorren de a una, sin saltos, y la cabecera
    # paga el omega. El plan declara cuanto necesita: 10.4 m de largo y 3.2 m de
    # costado, muy por encima del radio.
    assert plan.row_visit_order == tuple(range(plan.row_count))
    assert plan.clean_uturn_count == 0
    assert plan.headland_before_m == pytest.approx(10.39, abs=0.05)
    assert plan.headland_after_m == pytest.approx(10.39, abs=0.05)
    assert plan.lateral_overflow_m == pytest.approx(3.18, abs=0.05)


def test_row_skipping_replaces_every_omega_with_a_clean_uturn() -> None:
    """Que compra el salteo de pasadas cuando el perfil lo habilita.

    Con 12 pasadas a 1.64 m y radio 4 m, encadenar pasadas vecinas obliga a un
    omega por cabecera: 10.4 m de desborde, 3.2 m de invasion lateral y 541 m de
    camino para 240 m de trabajo. Saltando a la pasada opuesta cada giro entra
    como U simple y el recorrido se apoya en la cabecera minima. El precio es el
    orden de cobertura, y por eso queda apagado por defecto.
    """

    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=0.4,
        allow_headland_conflicts=True,
        allow_row_skipping=True,
    )
    row_offsets = [plan.lane_spacing_m * index for index in range(plan.row_count)]
    adjacent = evaluate_row_visit_order(
        field_length_m=20.0,
        row_offsets=row_offsets,
        visit_order=tuple(range(plan.row_count)),
        turning_radius_m=4.0,
    )

    assert plan.row_count == 12
    assert plan.clean_uturn_count == len(plan.turn_separations_m) == 11
    assert min(plan.turn_separations_m) >= 2.0 * plan.min_turning_radius_m
    # La cabecera baja de 10.4 m a un radio y el recorrido deja de invadir el
    # costado del lote.
    assert adjacent.headland_depth_m > 10.0
    assert plan.headland_before_m <= 1.05 * plan.min_turning_radius_m
    assert plan.headland_after_m <= 1.05 * plan.min_turning_radius_m
    assert plan.lateral_overflow_m == pytest.approx(0.0, abs=1.0e-6)
    # 541 m -> 390 m sobre 240 m de pasadas utiles.
    assert plan.estimated_path_length_m < 0.75 * adjacent.path_length_m
    assert plan.is_field_topologically_safe


def test_headland_conflicts_remain_blocking_when_simulation_override_is_disabled() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=2.0,
    )

    assert plan.topology_conflict_count > 0
    assert not plan.is_topologically_safe


def test_topology_beats_a_smaller_headland_when_choosing_the_order() -> None:
    # Pasadas a 5 m con radio 4: la serpentina adyacente todavia gira con omega y
    # necesita 8.7 m de cabecera, pero es la unica que no se pisa en ningun lado.
    # El salto reduce la cabecera al radio y aun asi debe perder mientras la
    # auditoria sea global; con la auditoria acotada al lote, gana.
    row_offsets = [5.0 * index for index in range(6)]
    adjacent = evaluate_row_visit_order(
        field_length_m=30.0,
        row_offsets=row_offsets,
        visit_order=(0, 1, 2, 3, 4, 5),
        turning_radius_m=4.0,
    )
    skipping = evaluate_row_visit_order(
        field_length_m=30.0,
        row_offsets=row_offsets,
        visit_order=max_separation_row_order(6),
        turning_radius_m=4.0,
    )

    assert adjacent.is_safe
    assert not skipping.is_safe
    assert skipping.headland_depth_m < adjacent.headland_depth_m

    assert resolve_row_visit_order(
        row_count=6,
        lane_spacing_m=5.0,
        min_turning_radius_m=4.0,
        field_length_m=30.0,
        allow_row_skipping=True,
    ) == (0, 1, 2, 3, 4, 5)
    assert resolve_row_visit_order(
        row_count=6,
        lane_spacing_m=5.0,
        min_turning_radius_m=4.0,
        field_length_m=30.0,
        cutter_width_m=5.0,
        allow_row_skipping=True,
    ) == max_separation_row_order(6)


def test_topology_audit_resolution_is_finer_than_half_a_metre() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=40.0,
        cutter_width_m=10.0,
        overlap_ratio=0.15,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=3.0,
    )

    assert plan.topology_audit_spacing_m <= 0.5
    assert plan.is_topologically_safe


def test_minimum_safe_lane_spacing_is_one_turning_radius() -> None:
    for radius in (1.5, 3.0, 4.0, 6.0):
        assert minimum_safe_lane_spacing_m(radius) == pytest.approx(radius)

    # Justo por encima del radio la serpentina adyacente ya no se pisa.
    safe = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=20.0,
        cutter_width_m=2.0,
        overlap_ratio=0.15,
        min_turning_radius_m=1.5,
        waypoint_spacing_m=1.0,
    )
    assert safe.lane_spacing_m >= safe.minimum_safe_lane_spacing_m
    assert safe.is_topologically_safe


def test_adjacent_rows_at_two_turning_radii_have_safe_global_topology() -> None:
    plan = build_lawnmower_body_plan(
        field_length_m=20.0,
        field_width_m=32.0,
        cutter_width_m=8.0,
        overlap_ratio=0.0,
        min_turning_radius_m=4.0,
        waypoint_spacing_m=1.0,
    )

    assert plan.row_count == 4
    assert plan.lane_spacing_m == pytest.approx(8.0)
    assert plan.row_visit_order == (0, 1, 2, 3)
    assert plan.clean_uturn_count == len(plan.turn_separations_m)
    assert plan.strict_crossing_count == 0
    assert plan.nonadjacent_touch_count == 0
    assert plan.collinear_overlap_count == 0
    assert plan.topology_conflict_count == 0
    assert plan.is_topologically_safe


@pytest.mark.parametrize(
    "argument,value",
    [
        ("field_length_m", 0.0),
        ("field_width_m", -1.0),
        ("cutter_width_m", 0.0),
        ("overlap_ratio", 1.0),
        ("min_turning_radius_m", 0.0),
        ("waypoint_spacing_m", 0.0),
    ],
)
def test_lawnmower_plan_rejects_invalid_dimensions(argument: str, value: float) -> None:
    kwargs = {
        "field_length_m": 10.0,
        "field_width_m": 5.0,
        "cutter_width_m": 2.0,
        "overlap_ratio": 0.1,
        "min_turning_radius_m": 4.0,
        "waypoint_spacing_m": 2.0,
    }
    kwargs[argument] = value

    with pytest.raises(ValueError):
        build_lawnmower_body_plan(**kwargs)
