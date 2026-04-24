from navegacion_gps.route_executor import (
    RouteWaypoint,
    build_chunk_waypoints,
    expand_route_waypoints,
    next_chunk_start_index,
    prepare_route_waypoints,
    should_suppress_chunk_success_brake,
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


def test_build_chunk_waypoints_limits_chunk_by_span_and_advances_after_target():
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
        start_index=end_index + 1,
        loop=False,
        chunk_span_m=50.0,
        chunk_max_waypoints=5,
    )

    assert next_chunk[0] == route[end_index + 1]
    assert next_end_index >= end_index + 1


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


def test_next_chunk_start_index_advances_for_non_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=6, loop=False) == 4
    assert next_chunk_start_index(current_target_index=5, route_size=6, loop=False) == 6


def test_next_chunk_start_index_advances_for_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=4, loop=True) == 0
    assert next_chunk_start_index(current_target_index=1, route_size=4, loop=True) == 2


def test_should_suppress_chunk_success_brake_for_intermediate_non_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=3,
            route_size=6,
            loop=False,
        )
        is True
    )


def test_should_not_suppress_chunk_success_brake_for_final_non_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=5,
            route_size=6,
            loop=False,
        )
        is False
    )


def test_should_suppress_chunk_success_brake_for_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=5,
            route_size=6,
            loop=True,
        )
        is True
    )


def test_prepare_route_waypoints_skips_reached_prefix_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.00001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0005, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.skipped_waypoints == 2
    assert prepared.note == "skipped 2 reached waypoints"
    assert prepared.waypoints == [route[2]]


def test_prepare_route_waypoints_joins_nearest_segment_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.002, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.start_index == 1
    assert prepared.skipped_waypoints == 1
    assert prepared.note == "joined nearest segment 1->2"
    assert prepared.waypoints == [route[1], route[2]]


def test_prepare_route_waypoints_does_not_join_far_segment_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.002, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0001,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.start_index == 0
    assert prepared.note == ""
    assert prepared.waypoints == route


def test_prepare_route_waypoints_rejects_non_loop_when_final_is_already_reached():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
    )

    assert prepared is None
    assert "final waypoint already within 1.20 m" in error


def test_prepare_route_waypoints_rotates_loop_to_next_useful_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0003, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0003,
        waypoint_reached_tolerance_m=1.2,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.rotated is True
    assert prepared.start_index == 1
    assert prepared.note == "loop rotated to waypoint 2"
    assert prepared.waypoints == [route[1], route[2], route[0]]


def test_prepare_route_waypoints_joins_nearest_segment_for_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.rotated is True
    assert prepared.start_index == 1
    assert prepared.note == "loop joined nearest segment 1->2"
    assert prepared.waypoints == [route[1], route[2], route[0]]
