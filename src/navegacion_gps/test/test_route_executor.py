from navegacion_gps.route_executor import (
    RouteWaypoint,
    build_chunk_waypoints,
    expand_route_waypoints,
    next_chunk_start_index,
)


def test_expand_route_waypoints_inserts_intermediate_points_for_long_legs():
    base = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
    ]

    expanded = expand_route_waypoints(base, leg_spacing_m=30.0, loop=False)

    assert len(expanded) >= 4
    assert expanded[0] == base[0]
    assert expanded[-1] == base[-1]


def test_expand_route_waypoints_handles_loop_closure_without_duplicating_start():
    base = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    expanded = expand_route_waypoints(base, leg_spacing_m=40.0, loop=True)

    assert expanded[0] == base[0]
    assert len(expanded) > len(base)
    assert expanded.count(base[0]) == 1


def test_build_chunk_waypoints_limits_chunk_by_span_and_keeps_progress_overlap():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0008, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=False,
        chunk_span_m=50.0,
        chunk_max_waypoints=5,
    )

    assert len(chunk) in (2, 3)
    assert end_index == len(chunk) - 1

    next_chunk, next_end_index = build_chunk_waypoints(
        route,
        start_index=end_index,
        loop=False,
        chunk_span_m=50.0,
        chunk_max_waypoints=5,
    )

    assert next_chunk[0] == route[end_index]
    assert next_end_index >= end_index


def test_build_chunk_waypoints_wraps_for_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=2,
        loop=True,
        chunk_span_m=80.0,
        chunk_max_waypoints=3,
    )

    assert chunk[0] == route[2]
    assert chunk[1] == route[0]
    assert end_index == 1


def test_next_chunk_start_index_keeps_overlap_for_non_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=6, loop=False) == 3


def test_next_chunk_start_index_advances_for_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=4, loop=True) == 0
    assert next_chunk_start_index(current_target_index=1, route_size=4, loop=True) == 2
