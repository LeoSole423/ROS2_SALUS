import asyncio
import json
import threading
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
import pytest

from map_tools.web_zone_server import ROSBAG_TOPIC_PROFILES, WebSocketApi, WebZoneServerNode


def _valid_coverage_service_response(**overrides):
    values = {
        "ok": True,
        "error": "",
        "route_start_lat": -31.0,
        "route_start_lon": -64.0,
        "route_start_yaw_deg": 0.0,
        "centerline_length_m": 18.0,
        "sampled_lats": [-31.0, -31.0, -31.0],
        "sampled_lons": [-64.0, -63.9999, -63.9998],
        "sampled_yaws_deg": [0.0, 0.0, 0.0],
        "sampled_phases": ["row", "row", "row"],
        "sampled_row_indices": [0, 0, 0],
        "sampled_key_flags": [True, False, True],
        "key_lats": [-31.0, -31.0],
        "key_lons": [-64.0, -63.9998],
        "key_yaws_deg": [0.0, 0.0],
        "route_lats": [-31.0, -31.0],
        "route_lons": [-64.0, -63.9998],
        "route_yaws_deg": [0.0, 0.0],
        "route_key_flags": [True, True],
        "route_phases": ["row", "row"],
        "row_count": 1,
        "lane_spacing_m": 0.0,
        "row_visit_order": [0],
        "turn_separations_m": [],
        "clean_uturn_count": 0,
        "omega_turn_count": 0,
        "estimated_path_length_m": 18.0,
        "headland_before_m": 0.0,
        "headland_after_m": 0.0,
        "lateral_overflow_m": 0.0,
        "strict_crossing_count": 0,
        "nonadjacent_touch_count": 0,
        "collinear_overlap_count": 0,
        "topology_conflict_count": 0,
        "field_strict_crossing_count": 0,
        "field_nonadjacent_touch_count": 0,
        "field_collinear_overlap_count": 0,
        "field_topology_conflict_count": 0,
        "topology_safe": True,
        "topology_scope": "global",
        "topology_audit_spacing_m": 0.5,
        "planner_min_turning_radius_m": 4.0,
        "recommended_leg_spacing_m": 21.0,
        "recommended_chunk_span_m": 60.0,
        "recommended_chunk_max_waypoints": 25,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _coverage_parameters():
    return {
        "start_lat": -31.0,
        "start_lon": -64.0,
        "start_yaw_deg": 0.0,
        "field_length_m": 20.0,
        "field_width_m": 2.0,
        "cutter_width_m": 2.0,
        "overlap_ratio": 0.15,
        "min_turning_radius_m": 4.0,
        "waypoint_spacing_m": 2.0,
        "side": "left",
    }


def test_preview_coverage_parser_uses_explicit_reference_and_defaults():
    node = SimpleNamespace()
    api = WebSocketApi(node)

    parameters, error = api._parse_coverage_parameters(
        {
            "reference": {"lat": -31.485, "lon": -64.241, "yaw_deg": 90.0},
            "field_length_m": 20.0,
            "field_width_m": 20.0,
        }
    )

    assert error == ""
    assert parameters is not None
    assert parameters["start_lat"] == pytest.approx(-31.485)
    assert parameters["start_yaw_deg"] == pytest.approx(90.0)
    assert parameters["cutter_width_m"] == pytest.approx(2.0)
    assert parameters["overlap_ratio"] == pytest.approx(0.15)
    assert parameters["min_turning_radius_m"] == pytest.approx(4.0)
    assert parameters["side"] == "left"
    # El operador marco la esquina fisica: el proveedor aplica el inset.
    assert parameters["start_is_field_corner"] is True


def test_preview_coverage_parser_uses_fresh_current_reference():
    node = SimpleNamespace(
        current_coverage_reference=lambda: (
            {"lat": -31.1, "lon": -64.2, "yaw_deg": 12.0},
            "",
        )
    )
    api = WebSocketApi(node)

    parameters, error = api._parse_coverage_parameters(
        {"field_length_m": 20.0, "field_width_m": 10.0, "side": "right"}
    )

    assert error == ""
    assert parameters is not None
    assert parameters["start_lon"] == pytest.approx(-64.2)
    assert parameters["start_yaw_deg"] == pytest.approx(12.0)
    assert parameters["side"] == "right"
    # Sin referencia explicita la pose del vehiculo es el centro de la primera
    # pasada. Tomarla como esquina dejaria la primera meta media pasada adelante
    # y media al costado: un corrimiento lateral que con radio minimo obliga a un
    # rulo de aproximacion antes de empezar la primera pasada.
    assert parameters["start_is_field_corner"] is False


def test_current_coverage_reference_rejects_stale_heading(monkeypatch):
    node = object.__new__(WebZoneServerNode)
    node._lock = threading.Lock()
    node._last_robot_pose = {"lat": -31.1, "lon": -64.2, "heading_deg": 12.0}
    node._last_robot_pose_monotonic = 100.0
    node._last_robot_heading_monotonic = 90.0
    node.coverage_reference_max_age_s = 5.0
    monkeypatch.setattr("map_tools.web_zone_server.time.monotonic", lambda: 102.0)

    reference, error = WebZoneServerNode.current_coverage_reference(node)

    assert reference is None
    assert error == "current robot map heading is stale"


def test_generate_coverage_plan_maps_typed_arrays_to_preview_and_route_request():
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    captured = {}

    def call_service(client, request, timeout_s):
        captured["client"] = client
        captured["request"] = request
        captured["timeout_s"] = timeout_s
        return SimpleNamespace(
            ok=True,
            error="",
            route_start_lat=-31.0,
            route_start_lon=-64.0,
            route_start_yaw_deg=90.0,
            centerline_length_m=18.0,
            sampled_lats=[-31.0, -30.9999, -30.9998],
            sampled_lons=[-64.0, -64.0, -64.0],
            sampled_yaws_deg=[90.0, 90.0, 90.0],
            sampled_phases=["row", "row", "row"],
            sampled_row_indices=[0, 0, 0],
            sampled_key_flags=[True, False, True],
            key_lats=[-31.0, -30.9998],
            key_lons=[-64.0, -64.0],
            key_yaws_deg=[90.0, 90.0],
            route_lats=[-31.0, -30.9998],
            route_lons=[-64.0, -64.0],
            route_yaws_deg=[90.0, 90.0],
            route_key_flags=[True, True],
            route_phases=["row", "row"],
            row_count=1,
            lane_spacing_m=0.0,
            row_visit_order=[0],
            turn_separations_m=[],
            clean_uturn_count=0,
            omega_turn_count=0,
            estimated_path_length_m=20.0,
            headland_before_m=0.0,
            headland_after_m=0.0,
            lateral_overflow_m=0.0,
            strict_crossing_count=0,
            nonadjacent_touch_count=0,
            collinear_overlap_count=0,
            topology_conflict_count=0,
            field_strict_crossing_count=0,
            field_nonadjacent_touch_count=0,
            field_collinear_overlap_count=0,
            field_topology_conflict_count=0,
            topology_safe=True,
            topology_scope="global",
            topology_audit_spacing_m=0.5,
            planner_min_turning_radius_m=4.0,
            recommended_leg_spacing_m=21.0,
            recommended_chunk_span_m=60.0,
            recommended_chunk_max_waypoints=25,
        )

    node._call_service = call_service
    parameters = {
        "start_lat": -31.0,
        "start_lon": -64.0,
        "start_yaw_deg": 90.0,
        "field_length_m": 20.0,
        "field_width_m": 2.0,
        "cutter_width_m": 2.0,
        "overlap_ratio": 0.15,
        "min_turning_radius_m": 4.0,
        "waypoint_spacing_m": 2.0,
        "side": "left",
    }

    ok, error, payload = node.generate_coverage_plan(parameters)

    assert ok is True
    assert error == ""
    assert len(payload["sampled_waypoints"]) == 3
    assert len(payload["key_waypoints"]) == 2
    assert payload["topology_safe"] is True
    assert payload["parameters"]["field_length_m"] == pytest.approx(20.0)
    assert payload["parameters"]["field_width_m"] == pytest.approx(2.0)
    assert payload["parameters"]["centerline_length_m"] == pytest.approx(18.0)
    assert payload["route_start_reference"]["lat"] == pytest.approx(-31.0)
    assert payload["headland_guidance_enabled"] is False
    assert payload["route_request"] == {
        "op": "set_route_ll",
        "waypoints": [
            {
                **waypoint,
                "key": True,
                "guide": False,
                "phase": "row",
                "action_json": "",
                "role": "coverage",
            }
            for waypoint in payload["key_waypoints"]
        ],
        "loop": False,
        "leg_spacing_m": 21.0,
        "chunk_span_m": 60.0,
        "chunk_max_waypoints": 25,
    }
    assert captured["request"].field_length_m == pytest.approx(20.0)
    assert captured["request"].start_is_field_corner is True
    assert captured["request"].side == "left"
    assert captured["timeout_s"] == pytest.approx(2.0)


def test_generate_coverage_plan_uses_one_headland_guide_only_when_enabled():
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    node.coverage_use_headland_guides = True
    node._call_service = lambda *_args: _valid_coverage_service_response(
        sampled_lats=[-31.0, -31.0, -30.99999, -30.99999],
        sampled_lons=[-64.0, -63.9998, -63.9998, -64.0],
        sampled_yaws_deg=[0.0, 0.0, 180.0, 180.0],
        sampled_phases=["row", "row", "row", "row"],
        sampled_row_indices=[0, 0, 1, 1],
        sampled_key_flags=[True, True, True, True],
        key_lats=[-31.0, -31.0, -30.99999, -30.99999],
        key_lons=[-64.0, -63.9998, -63.9998, -64.0],
        key_yaws_deg=[0.0, 0.0, 180.0, 180.0],
        route_lats=[-31.0, -31.0, -30.9999, -30.99999, -30.99999],
        route_lons=[-64.0, -63.9998, -63.9997, -63.9998, -64.0],
        route_yaws_deg=[0.0, 0.0, 90.0, 180.0, 180.0],
        route_key_flags=[True, True, False, True, True],
        route_phases=["row", "row", "turn", "row", "row"],
        row_count=2,
        lane_spacing_m=1.5,
        row_visit_order=[0, 1],
        turn_separations_m=[1.5],
        omega_turn_count=1,
        strict_crossing_count=1,
        topology_conflict_count=1,
        topology_safe=True,
        topology_scope="field_interior",
    )

    ok, error, payload = node.generate_coverage_plan(_coverage_parameters())

    assert ok is True
    assert error == ""
    assert payload["headland_guidance_enabled"] is True
    assert [
        {key: value for key, value in waypoint.items() if key != "role"}
        for waypoint in payload["route_request"]["waypoints"]
    ] == payload["route_waypoints"]
    assert {
        waypoint["role"] for waypoint in payload["route_request"]["waypoints"]
    } == {"coverage", "coverage_transit"}
    assert payload["route_request"]["waypoints"][2]["role"] == "coverage_transit"
    assert [waypoint["key"] for waypoint in payload["route_request"]["waypoints"]] == [
        True,
        True,
        False,
        True,
        True,
    ]


def test_generate_coverage_plan_keeps_nogo_transition_block_with_guides_off():
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    node.coverage_use_headland_guides = False
    node._call_service = lambda *_args: _valid_coverage_service_response(
        sampled_lats=[-31.0] * 6,
        sampled_lons=[
            -64.0,
            -63.99995,
            -63.9999,
            -63.9998,
            -63.99975,
            -63.9997,
        ],
        sampled_yaws_deg=[0.0, 15.0, 30.0, -30.0, -15.0, 0.0],
        sampled_phases=[
            "row",
            "nogo_transition",
            "nogo_transition",
            "nogo_transition",
            "nogo_transition",
            "row",
        ],
        sampled_row_indices=[0] * 6,
        sampled_key_flags=[True, False, False, True, False, True],
        key_lats=[-31.0, -31.0, -31.0],
        key_lons=[-64.0, -63.9998, -63.9997],
        key_yaws_deg=[0.0, -30.0, 0.0],
        route_lats=[-31.0] * 6,
        route_lons=[
            -64.0,
            -63.99995,
            -63.9999,
            -63.9998,
            -63.99975,
            -63.9997,
        ],
        route_yaws_deg=[0.0, 15.0, 30.0, -30.0, -15.0, 0.0],
        route_key_flags=[True, False, False, True, False, True],
        route_phases=[
            "row",
            "nogo_transition",
            "nogo_transition",
            "nogo_transition",
            "nogo_transition",
            "row",
        ],
        nogo_polygon_count=1,
        nogo_dropped_count=2,
        nogo_detour_count=1,
        topology_audited=False,
        topology_scope="fields2cover",
    )

    ok, error, payload = node.generate_coverage_plan(_coverage_parameters())

    assert ok is True
    assert error == ""
    assert payload["headland_guidance_enabled"] is False
    waypoints = payload["route_request"]["waypoints"]
    assert [waypoint["key"] for waypoint in waypoints] == [
        True,
        False,
        False,
        True,
        False,
        True,
    ]
    assert [waypoint["role"] for waypoint in waypoints] == [
        "coverage",
        "coverage_transit",
        "coverage_transit",
        "coverage_curve",
        "coverage_transit",
        "coverage_transit",
    ]


@pytest.mark.parametrize("required_phase", ["forward_turn", "nogo_lane_change"])
def test_generate_coverage_plan_keeps_required_guides_off(required_phase):
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    node.coverage_use_headland_guides = False
    node._call_service = lambda *_args: _valid_coverage_service_response(
        sampled_lats=[-31.0] * 6,
        sampled_lons=[
            -64.0,
            -63.99995,
            -63.9999,
            -63.9998,
            -63.99975,
            -63.9997,
        ],
        sampled_yaws_deg=[0.0, 45.0, 90.0, -90.0, -135.0, 180.0],
        sampled_phases=[
            "row",
            required_phase,
            required_phase,
            required_phase,
            required_phase,
            "row",
        ],
        sampled_row_indices=[0] * 6,
        sampled_key_flags=[
            True,
            False,
            False,
            required_phase == "forward_turn",
            False,
            True,
        ],
        key_lats=(
            [-31.0, -31.0, -31.0]
            if required_phase == "forward_turn"
            else [-31.0, -31.0]
        ),
        key_lons=(
            [-64.0, -63.9998, -63.9997]
            if required_phase == "forward_turn"
            else [-64.0, -63.9997]
        ),
        key_yaws_deg=(
            [0.0, -90.0, 180.0]
            if required_phase == "forward_turn"
            else [0.0, 180.0]
        ),
        route_lats=[-31.0] * 6,
        route_lons=[
            -64.0,
            -63.99995,
            -63.9999,
            -63.9998,
            -63.99975,
            -63.9997,
        ],
        route_yaws_deg=[0.0, 45.0, 90.0, -90.0, -135.0, 180.0],
        route_key_flags=[
            True,
            False,
            False,
            required_phase == "forward_turn",
            False,
            True,
        ],
        route_phases=[
            "row",
            required_phase,
            required_phase,
            required_phase,
            required_phase,
            "row",
        ],
        topology_audited=False,
        topology_scope="fields2cover",
    )

    ok, error, payload = node.generate_coverage_plan(_coverage_parameters())

    assert ok is True
    assert error == ""
    waypoints = payload["route_request"]["waypoints"]
    assert [waypoint["phase"] for waypoint in waypoints] == [
        "row",
        required_phase,
        required_phase,
        required_phase,
        required_phase,
        "row",
    ]
    if required_phase == "nogo_lane_change":
        assert {waypoint["role"] for waypoint in waypoints} == {"coverage"}
    else:
        assert [waypoint["role"] for waypoint in waypoints] == [
            "coverage",
            "coverage_transit",
            "coverage_transit",
            "coverage_curve",
            "coverage_transit",
            "coverage_transit",
        ]


@pytest.mark.parametrize(
    ("response", "expected_error"),
    [
        (
            _valid_coverage_service_response(
                omega_turn_count=1,
                topology_safe=True,
            ),
            "topology_safe invariant is inconsistent",
        ),
        (
            _valid_coverage_service_response(
                strict_crossing_count=1,
                topology_conflict_count=0,
                topology_safe=False,
            ),
            "topology conflict count is inconsistent",
        ),
        (
            _valid_coverage_service_response(key_lons=[-64.0, -63.7]),
            "key arrays do not match sampled key flags",
        ),
        (
            _valid_coverage_service_response(row_count=2),
            "two endpoints per row",
        ),
    ],
)
def test_generate_coverage_plan_rejects_contradictory_service_response(
    response,
    expected_error,
):
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    node._call_service = lambda *_args: response

    ok, error, payload = node.generate_coverage_plan(_coverage_parameters())

    assert ok is False
    assert expected_error in error
    assert payload == {}


def test_generate_coverage_plan_allows_only_field_safe_headland_conflicts_in_sim():
    node = object.__new__(WebZoneServerNode)
    node._coverage_plan_client = object()
    node.coverage_plan_timeout_s = 2.0
    node._call_service = lambda *_args: _valid_coverage_service_response(
        strict_crossing_count=4,
        topology_conflict_count=4,
        field_strict_crossing_count=0,
        field_topology_conflict_count=0,
        omega_turn_count=3,
        topology_safe=True,
        topology_scope="field_interior",
    )

    ok, error, payload = node.generate_coverage_plan(_coverage_parameters())

    assert ok is True
    assert error == ""
    assert payload["topology_safe"] is True
    assert payload["metrics"]["topology_conflicts"]["total"] == 0
    assert payload["metrics"]["global_topology_conflicts"]["total"] == 4
    assert payload["metrics"]["topology_scope"] == "field_interior"


def test_start_coverage_rejects_unsafe_plan_without_route_submission():
    node = object.__new__(WebZoneServerNode)
    node.generate_coverage_plan = lambda _parameters: (
        True,
        "",
        {
            "topology_safe": False,
            "metrics": {
                "omega_turn_count": 0,
                "topology_conflicts": {
                    "strict_crossings": 1,
                    "nonadjacent_touches": 0,
                    "collinear_overlaps": 0,
                    "total": 1,
                },
            },
        },
    )
    route_calls = []
    node.set_route_mission = lambda *_args, **_kwargs: route_calls.append(_args)

    ok, error, result = node.start_coverage(_coverage_parameters())

    assert ok is False
    assert "topology conflicts" in error
    assert result["route_started"] is False
    assert result["route_submission_state"] == "not_started"
    assert route_calls == []


def test_start_coverage_rejects_far_or_misaligned_approach_without_route_submission():
    node = object.__new__(WebZoneServerNode)
    node.coverage_start_max_distance_m = 5.0
    node.coverage_start_max_heading_error_deg = 30.0
    plan = {
        "topology_safe": True,
        "metrics": {
            "omega_turn_count": 0,
            "topology_conflicts": {
                "strict_crossings": 0,
                "nonadjacent_touches": 0,
                "collinear_overlaps": 0,
                "total": 0,
            },
        },
        "key_waypoints": [{"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0}],
        "route_request": {
            "waypoints": [],
            "leg_spacing_m": 21.0,
            "chunk_span_m": 60.0,
            "chunk_max_waypoints": 25,
        },
    }
    node.generate_coverage_plan = lambda _parameters: (True, "", plan)
    node.current_coverage_reference = lambda **_kwargs: (
        {"lat": -31.001, "lon": -64.0, "yaw_deg": 90.0},
        "",
    )
    route_calls = []
    node.set_route_mission = lambda *_args, **_kwargs: route_calls.append(_args)

    ok, error, result = node.start_coverage(_coverage_parameters())

    assert ok is False
    assert "distance=" in error
    assert "limit=5.00 m" in error
    assert "heading_error=90.0 deg" in error
    assert "limit=30.0 deg" in error
    assert result["approach"]["distance_m"] > 100.0
    assert route_calls == []


def test_start_coverage_regenerates_checks_approach_and_submits_recommended_route():
    node = object.__new__(WebZoneServerNode)
    node.coverage_start_max_distance_m = 5.0
    node.coverage_start_max_heading_error_deg = 30.0
    plan = {
        "topology_safe": True,
        "metrics": {
            "omega_turn_count": 0,
            "topology_conflicts": {
                "strict_crossings": 0,
                "nonadjacent_touches": 0,
                "collinear_overlaps": 0,
                "total": 0,
            },
        },
        "key_waypoints": [
            {"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0},
            {"lat": -31.0, "lon": -63.9998, "yaw_deg": 0.0},
        ],
        "route_request": {
            "waypoints": [
                {"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0},
                {"lat": -31.0, "lon": -63.9998, "yaw_deg": 0.0},
            ],
            "leg_spacing_m": 21.0,
            "chunk_span_m": 60.0,
            "chunk_max_waypoints": 25,
        },
    }
    generated = []
    node.generate_coverage_plan = lambda parameters: (
        generated.append(parameters) or True,
        "",
        plan,
    )
    node.current_coverage_reference = lambda **_kwargs: (
        {"lat": -31.0, "lon": -64.0, "yaw_deg": 5.0},
        "",
    )
    route_calls = []

    def set_route(*args, **kwargs):
        route_calls.append((args, kwargs))
        return True, "", 2, 2

    node.set_route_mission = set_route

    ok, error, result = node.start_coverage(_coverage_parameters())

    assert ok is True
    assert error == ""
    assert len(generated) == 1
    assert len(route_calls) == 1
    assert route_calls[0][0][1:] == (False, 21.0, 60.0, 25)
    # La cobertura se recorre completa: sin esto route_executor puede enganchar la
    # ruta en un tramo del medio y descartar las primeras pasadas.
    assert route_calls[0][1] == {"start_from_first_waypoint": True}
    assert result["route_started"] is True
    assert result["route_submission_state"] == "started"
    assert result["input_waypoint_count"] == 2


def test_start_coverage_reports_route_service_timeout_as_unknown():
    node = object.__new__(WebZoneServerNode)
    node.coverage_start_max_distance_m = 5.0
    node.coverage_start_max_heading_error_deg = 30.0
    plan = {
        "topology_safe": True,
        "metrics": {
            "omega_turn_count": 0,
            "topology_conflicts": {
                "strict_crossings": 0,
                "nonadjacent_touches": 0,
                "collinear_overlaps": 0,
                "total": 0,
            },
        },
        "key_waypoints": [{"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0}],
        "route_request": {
            "waypoints": [{"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0}],
            "leg_spacing_m": 21.0,
            "chunk_span_m": 60.0,
            "chunk_max_waypoints": 25,
        },
    }
    node.generate_coverage_plan = lambda _parameters: (True, "", plan)
    node.current_coverage_reference = lambda **_kwargs: (
        {"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0},
        "",
    )
    node.set_route_mission = lambda *_args, **_kwargs: (
        False,
        "set_route_ll timeout",
        1,
        0,
    )

    ok, error, result = node.start_coverage(_coverage_parameters())

    assert ok is False
    assert error == "set_route_ll timeout"
    assert result["route_started"] is None
    assert result["route_submission_state"] == "unknown_timeout"


def test_start_fields2cover_no_exige_rumbo_actual_del_robot():
    """El rumbo no participa en el approach de Fields2Cover estacionario."""
    node = object.__new__(WebZoneServerNode)
    node.coverage_start_max_distance_m = 5.0
    node.coverage_start_max_heading_error_deg = 30.0
    plan = {
        "topology_safe": True,
        "topology_audited": False,
        "metrics": {
            "topology_conflicts": {
                "strict_crossings": 0,
                "nonadjacent_touches": 0,
                "collinear_overlaps": 0,
                "total": 0,
            },
        },
        "key_waypoints": [{"lat": -31.0, "lon": -64.0, "yaw_deg": 90.0}],
        "route_request": {
            "waypoints": [{"lat": -31.0, "lon": -64.0, "yaw_deg": 90.0}],
            "leg_spacing_m": 21.0,
            "chunk_span_m": 60.0,
            "chunk_max_waypoints": 25,
        },
    }
    node.generate_coverage_plan = lambda _parameters: (True, "", plan)
    heading_requests = []
    node.current_coverage_reference = lambda **kwargs: (
        heading_requests.append(kwargs.get("require_heading")) or {
            "lat": -31.0,
            "lon": -64.0,
            "yaw_deg": 0.0,
        },
        "",
    )
    node.set_route_mission = lambda *_args, **_kwargs: (True, "", 1, 1)

    ok, error, result = node.start_coverage(_coverage_parameters())

    assert ok is True
    assert error == ""
    assert heading_requests == [False]
    assert result["approach"]["checks_heading"] is False


def test_preview_coverage_is_not_a_motion_control_operation():
    node = SimpleNamespace(enable_control_lock=True)
    api = WebSocketApi(node)

    assert api._is_controlled_robot_op("preview_coverage", {}) is False
    assert api._is_controlled_robot_op("start_coverage", {}) is True


def test_preview_coverage_websocket_operation_returns_correlated_ack():
    sent_payloads = []

    class FakeNode:
        enable_control_lock = False

        def get_logger(self):
            return SimpleNamespace(
                info=lambda *_args, **_kwargs: None,
                warning=lambda *_args, **_kwargs: None,
            )

        def generate_coverage_plan(self, parameters):
            assert parameters["field_length_m"] == pytest.approx(20.0)
            return True, "", {"topology_safe": True, "route_request": {}}

        async def send_ws_json(self, _ws, payload):
            sent_payloads.append(payload)
            return True

    api = WebSocketApi(FakeNode())
    asyncio.run(
        api._handle_message(
            object(),
            json.dumps(
                {
                    "op": "preview_coverage",
                    "client_req_id": "coverage-1",
                    "reference": {"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0},
                    "field_length_m": 20.0,
                    "field_width_m": 2.0,
                }
            ),
        )
    )

    assert sent_payloads == [
        {
            "op": "ack",
            "ok": True,
            "request": "preview_coverage",
            "error": None,
            "client_req_id": "coverage-1",
            "coverage_plan": {"topology_safe": True, "route_request": {}},
        }
    ]


def test_start_coverage_websocket_operation_returns_explicit_started_state():
    sent_payloads = []
    broadcasts = []

    class FakeNode:
        enable_control_lock = False

        def get_logger(self):
            return SimpleNamespace(
                info=lambda *_args, **_kwargs: None,
                warning=lambda *_args, **_kwargs: None,
            )

        def is_ui_control_locked(self):
            return False

        def get_ui_control_lock_reason(self):
            return ""

        def start_coverage(self, parameters):
            assert parameters["field_length_m"] == pytest.approx(20.0)
            return (
                True,
                "",
                {
                    "route_started": True,
                    "route_submission_state": "started",
                    "input_waypoint_count": 2,
                    "expanded_waypoint_count": 2,
                },
            )

        def snapshot_state(self):
            return {"op": "state"}

        async def _broadcast(self, payload):
            broadcasts.append(payload)

        async def send_ws_json(self, _ws, payload):
            sent_payloads.append(payload)
            return True

    api = WebSocketApi(FakeNode())
    asyncio.run(
        api._handle_message(
            object(),
            json.dumps(
                {
                    "op": "start_coverage",
                    "client_req_id": "coverage-start-1",
                    "reference": {"lat": -31.0, "lon": -64.0, "yaw_deg": 0.0},
                    "field_length_m": 20.0,
                    "field_width_m": 2.0,
                }
            ),
        )
    )

    assert sent_payloads == [
        {
            "op": "ack",
            "ok": True,
            "request": "start_coverage",
            "error": None,
            "client_req_id": "coverage-start-1",
            "route_started": True,
            "route_submission_state": "started",
            "input_waypoint_count": 2,
            "expanded_waypoint_count": 2,
            "control_locked": False,
            "control_lock_reason": "",
            "locked": False,
            "lock_reason": "",
        }
    ]
    assert broadcasts == [{"op": "state"}]


def test_normalize_waypoint_actions_accepts_navigation_profiles():
    actions, error = WebSocketApi._normalize_waypoint_actions(
        [{"type": "set_navigation_profile", "profile": "RURAL"}],
        2,
    )

    assert error == ""
    assert actions == [{"type": "set_navigation_profile", "profile": "rural"}]


def test_normalize_waypoint_actions_rejects_unknown_navigation_profile():
    actions, error = WebSocketApi._normalize_waypoint_actions(
        [{"type": "set_navigation_profile", "profile": "forest"}],
        2,
    )

    assert actions == []
    assert "must be 'urban' or 'rural'" in error


def test_set_navigation_profile_forwards_valid_profile_to_ros_service():
    node = object.__new__(WebZoneServerNode)
    node._navigation_profile_client = object()
    node.request_timeout_s = 1.0
    captured = {}

    def call_service(client, request, timeout_s):
        captured["client"] = client
        captured["profile"] = request.profile
        captured["timeout_s"] = timeout_s
        return SimpleNamespace(ok=True, error="", active_profile="rural")

    node._call_service = call_service

    ok, error, active_profile = WebZoneServerNode.set_navigation_profile(node, "RURAL")

    assert ok is True
    assert error == ""
    assert active_profile == "rural"
    assert captured["profile"] == "rural"


def test_set_navigation_profile_rejects_invalid_value_before_ros_call():
    node = object.__new__(WebZoneServerNode)

    ok, error, active_profile = WebZoneServerNode.set_navigation_profile(node, "forest")

    assert ok is False
    assert "urban' or 'rural" in error
    assert active_profile == ""


def test_patrol_parser_accepts_empty_optional_connectors():
    api = object.__new__(WebSocketApi)
    patrol, error = api._parse_patrol_mission_from_message(
        {
            "patrol_mission": {
                "loop_waypoints": [
                    {"lat": -31.0, "lon": -64.0},
                    {"lat": -31.0, "lon": -64.001},
                ],
                "home_waypoint": {"lat": -31.001, "lon": -64.0},
                "return_waypoints": [],
                "depart_waypoints": [],
                "depart_entry_loop_index": 0,
            }
        }
    )

    assert error == ""
    assert patrol is not None
    assert len(patrol["loop_waypoints"]) == 2
    assert patrol["return_waypoints"] == []
    assert patrol["depart_waypoints"] == []


def _diag_level(value) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


class _FakeNode:
    _diag_level_value = staticmethod(WebZoneServerNode._diag_level_value)
    _should_surface_diagnostic = WebZoneServerNode._should_surface_diagnostic
    _rosbag_topics_for_profile = staticmethod(WebZoneServerNode._rosbag_topics_for_profile)
    _normalize_gps_status_text = staticmethod(WebZoneServerNode._normalize_gps_status_text)
    _build_gps_status_payload = staticmethod(WebZoneServerNode._build_gps_status_payload)
    _build_gps_status_payload_from_navsat = staticmethod(
        WebZoneServerNode._build_gps_status_payload_from_navsat
    )


class _FakeStatus:
    def __init__(self, name: str, level, message: str) -> None:
        self.name = name
        self.level = level
        self.message = message


class _FakeSensorNode(_FakeNode):
    _build_default_datum_snapshot = WebZoneServerNode._build_default_datum_snapshot
    _precision_from_gps_snapshot = staticmethod(WebZoneServerNode._precision_from_gps_snapshot)
    _derive_mode = staticmethod(WebZoneServerNode._derive_mode)
    _connection_status_locked = WebZoneServerNode._connection_status_locked
    _build_general_sensor_snapshot = WebZoneServerNode._build_general_sensor_snapshot
    build_sensor_info_message = WebZoneServerNode.build_sensor_info_message
    is_ui_control_locked = WebZoneServerNode.is_ui_control_locked
    get_ui_control_lock_reason = WebZoneServerNode.get_ui_control_lock_reason

    def _build_datums_state_payload(self):
        return {
            "datums": [],
            "selected_id": "",
            "selected": None,
            "runtime": {
                "lat": self.fixed_datum_lat,
                "lon": self.fixed_datum_lon,
                "yaw_deg": self.fixed_datum_yaw_deg,
                "source": self.fixed_datum_source,
                "available": True,
                "already_set": True,
            },
            "pending_restart": False,
            "apply_mode": "next_restart",
            "file_path": "",
            "error": "",
        }

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.fixed_datum_lat = -31.4858037
        self.fixed_datum_lon = -64.2410570
        self.fixed_datum_yaw_deg = 0.0
        self.fixed_datum_source = "real_global_v2_fixed"
        self._goal_active = False
        self._manual_control = {"enabled": False}
        self._control_locked = False
        self._control_lock_reason = ""
        self.sensor_bridge_enabled = True
        self._gps_status_payload = {
            "available": True,
            "raw": "RTK_FIXED",
            "normalized": "rtk_fixed",
            "label": "RTK FIXED",
            "level": "good",
            "source": "rtk_status",
        }
        self._sensor_bridge_ok = True
        self._sensor_bridge_error = ""
        self._sensor_bridge_snapshot = {
            "gps_meta": {
                "fix_type_name": "RTK_FIXED",
                "rtk_status": "rtk_fixed",
                "satellites_visible": 18,
                "eph": 85,
            },
            "rtk_source_state": {
                "connected": True,
                "active_source_label": "Base Norte",
                "rtcm_age_s": 0.4,
            },
            "rtk_sources": [{"id": "base-norte", "label": "Base Norte"}],
            "gps": {
                "position_covariance": [0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.0]
            },
            "diagnostics": {"yaw_delta_deg": 1.7},
        }
        self._rtk_source_status = {}
        self._rtk_sources_list = []
        self._datum_snapshot = self._build_default_datum_snapshot()


class _FakeWaypointYawNode(_FakeNode):
    _normalize_yaw_deg = staticmethod(WebZoneServerNode._normalize_yaw_deg)
    _bearing_deg_between_ll = staticmethod(WebZoneServerNode._bearing_deg_between_ll)
    _route_tangent_bearing_deg = staticmethod(WebZoneServerNode._route_tangent_bearing_deg)
    _resolve_waypoint_yaws = WebZoneServerNode._resolve_waypoint_yaws

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_robot_pose = None
        self._last_robot_heading_deg = None


class _FakeRouteStateNode(_FakeNode):
    _build_default_route_mission_payload = staticmethod(
        WebZoneServerNode._build_default_route_mission_payload
    )
    _route_waypoints_from_state = staticmethod(WebZoneServerNode._route_waypoints_from_state)
    _update_route_state = WebZoneServerNode._update_route_state

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._route_mission = self._build_default_route_mission_payload()


class _FakeMissionSessionNode(_FakeNode):
    _MISSION_START_CODES = WebZoneServerNode._MISSION_START_CODES
    _MISSION_STOP_CODES = WebZoneServerNode._MISSION_STOP_CODES
    _nav_event_details_to_dict = staticmethod(WebZoneServerNode._nav_event_details_to_dict)
    _nav_event_to_payload = WebZoneServerNode._nav_event_to_payload
    _on_nav_event = WebZoneServerNode._on_nav_event

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._recent_nav_events = []
        self._mission_active = False
        self._mission_start_count = 0
        self._mission_stop_count = 0
        self._mission_records = []
        self._loop = None

    def _mission_start(self) -> None:
        self._mission_active = True
        self._mission_start_count += 1

    def _mission_stop(self) -> None:
        self._mission_active = False
        self._mission_stop_count += 1

    def _mission_record(self, record: dict) -> None:
        if self._mission_active:
            self._mission_records.append(record)

    def _build_nav_telemetry_payload(self):
        return {"op": "nav_telemetry"}

    async def _broadcast(self, payload):
        return payload


class _FakeBatteryTelemetryNode(_FakeNode):
    _derive_mode = staticmethod(WebZoneServerNode._derive_mode)
    _connection_status_locked = WebZoneServerNode._connection_status_locked
    _on_controller_telemetry = WebZoneServerNode._on_controller_telemetry
    _on_battery_state = WebZoneServerNode._on_battery_state
    _safe_float = staticmethod(WebZoneServerNode._safe_float)
    _safe_int = staticmethod(WebZoneServerNode._safe_int)

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._goal_active = False
        self._manual_control = {"enabled": False}
        self._control_locked = False
        self._control_lock_reason = ""
        self._battery_pct = None
        self._battery_voltage_v = None
        self._battery_state = ""
        self._battery_mission_state = ""
        self._battery_return_home_recommended = None
        self._battery_recovered_voltage_v = None
        self._battery_loaded_voltage_v = None
        self._battery_present = None
        self._battery_updated_age_s = None
        self._battery_ws_key = None
        self._battery_use_controller_telemetry = False
        self._mission_last_controller_telemetry_key = None
        self._broadcast_payloads = []
        self._mission_records = []

    def _build_nav_telemetry_payload(self):
        return {"op": "nav_telemetry", **self._connection_status_locked()}

    def _broadcast_from_thread(self, payload):
        self._broadcast_payloads.append(payload)

    def _mission_record(self, record: dict) -> None:
        self._mission_records.append(record)

    def get_logger(self):
        return SimpleNamespace(error=lambda *args, **kwargs: None)


def _nav_event(code: str):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=0, nanosec=0),
        severity=0,
        component="nav_command_server",
        code=code,
        message=code,
        event_id=1,
        details=[],
    )


def _discard_scheduled_coroutine(coro, loop):
    _ = loop
    coro.close()
    return None


def test_should_surface_diagnostic_accepts_navigation_errors():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/nav_command_server",
        DiagnosticStatus.ERROR,
        "failure=GOAL_RESULT_ABORTED",
    )

    assert node._should_surface_diagnostic(status) is True


def test_should_surface_diagnostic_filters_non_navigation_status():
    node = _FakeNode()
    status = _FakeStatus("ekf_filter_node_map", DiagnosticStatus.ERROR, "stale")

    assert node._should_surface_diagnostic(status) is False


def test_should_surface_diagnostic_filters_idle_collision_monitor_warning():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/collision_monitor",
        DiagnosticStatus.WARN,
        "no collision monitor state yet",
    )

    assert node._should_surface_diagnostic(status) is False


def test_rosbag_topics_for_profile_matches_declared_profiles():
    topics = _FakeNode._rosbag_topics_for_profile("core")

    assert topics == ROSBAG_TOPIC_PROFILES["core"]
    assert "/global_position/raw/fix" in topics
    assert "/gps/rtk_status_mavros" in topics
    assert "/gps/odometry_map" in topics
    assert "/gps/course_heading" in topics
    assert "/gps/course_heading/debug" in topics
    assert "/odometry/global" in topics
    assert "/controller/drive_telemetry" in topics
    assert "/diagnostics" in topics
    assert "/nav_command_server/events" in topics
    assert _FakeNode._rosbag_topics_for_profile("missing") is None


def test_normalize_gps_status_text_handles_common_variants():
    assert _FakeNode._normalize_gps_status_text("RTK_FIXED") == "rtk_fixed"
    assert _FakeNode._normalize_gps_status_text("3D-FIX") == "3d_fix"
    assert _FakeNode._normalize_gps_status_text(" waiting for gps ") == "waiting_for_gps"


def test_build_gps_status_payload_maps_quality_to_label_and_level():
    payload = _FakeNode._build_gps_status_payload(
        raw="RTK_FLOAT",
        source="rtk_status",
        available=True,
    )

    assert payload["label"] == "RTK FLOAT"
    assert payload["level"] == "warn"
    assert payload["normalized"] == "rtk_float"
    assert payload["source"] == "rtk_status"


def test_build_gps_status_payload_from_navsat_falls_back_to_3d_fix():
    payload = _FakeNode._build_gps_status_payload_from_navsat(0)

    assert payload["label"] == "3D FIX"
    assert payload["level"] == "warn"
    assert payload["source"] == "gps_fix"


def test_build_default_datum_snapshot_uses_fixed_global_v2_values():
    node = _FakeSensorNode()

    snapshot = node._build_default_datum_snapshot()

    assert snapshot["available"] is True
    assert snapshot["already_set"] is True
    assert snapshot["datum_lat"] == node.fixed_datum_lat
    assert snapshot["datum_lon"] == node.fixed_datum_lon
    assert snapshot["last_set_source"] == "real_global_v2_fixed"


def test_build_general_sensor_snapshot_merges_bridge_state_and_precision():
    node = _FakeSensorNode()

    snapshot = node._build_general_sensor_snapshot()

    assert snapshot["gps_meta"]["fix_type_name"] == "RTK_FIXED"
    assert snapshot["gps_meta"]["estimated_precision_m"] == 0.85
    assert snapshot["rtk_source_state"]["active_source_label"] == "Base Norte"
    assert snapshot["datum"]["last_set_source"] == "real_global_v2_fixed"


def test_build_sensor_info_message_reports_bridge_errors_for_pixhawk_tab():
    node = _FakeSensorNode()
    node._sensor_bridge_ok = False
    node._sensor_bridge_error = "bridge offline"

    payload = node.build_sensor_info_message(tab="pixhawk_gps", interval_s=0.5)

    assert payload["implemented"] is True
    assert payload["ok"] is False
    assert payload["error"] == "bridge offline"


def test_resolve_waypoint_yaws_uses_route_tangent_for_auto_points():
    node = _FakeWaypointYawNode()
    waypoints = [
        {"lat": -31.0, "lon": -64.0},
        {"lat": -30.999, "lon": -64.0},
        {"lat": -30.999, "lon": -63.999},
    ]

    yaws = node._resolve_waypoint_yaws(waypoints, loop=False)

    assert yaws[0] == pytest.approx(90.0)
    assert yaws[1] == pytest.approx(45.0)
    assert yaws[2] == pytest.approx(0.0)


def test_resolve_waypoint_yaws_uses_loop_bearing_for_last_auto_point():
    node = _FakeWaypointYawNode()
    waypoints = [
        {"lat": -31.0, "lon": -64.0},
        {"lat": -30.999, "lon": -64.0},
    ]

    yaws = node._resolve_waypoint_yaws(waypoints, loop=True)

    assert yaws[0] == 90.0
    assert yaws[1] == -90.0


def test_resolve_waypoint_yaws_preserves_manual_and_uses_robot_for_single_auto():
    node = _FakeWaypointYawNode()
    node._last_robot_pose = {"lat": -31.0, "lon": -64.0, "heading_deg": 42.0}

    assert node._resolve_waypoint_yaws([{"lat": -30.999, "lon": -64.0}], loop=False) == [90.0]
    assert node._resolve_waypoint_yaws([{"lat": -31.0, "lon": -64.0}], loop=False) == [42.0]
    assert node._resolve_waypoint_yaws([{"lat": -31.0, "lon": -64.0, "yaw_deg": 181.0}], loop=False) == [-179.0]


def test_default_route_mission_payload_includes_blocked_fields():
    payload = _FakeRouteStateNode._build_default_route_mission_payload()

    assert payload["low_battery_active"] is False
    assert payload["return_home_requested"] is False
    assert payload["return_home_active"] is False
    assert payload["return_home_exit_waypoint_index"] == -1
    assert payload["return_home_phase"] == "idle"
    assert payload["home_available"] is False
    assert payload["home_waypoint"] is None
    assert payload["blocked_state"] == ""
    assert payload["blocked_reason_code"] == ""
    assert payload["blocked_reason_text"] == ""
    assert payload["blocked_retry_attempt"] == 0
    assert payload["blocked_retry_max_attempts"] == 0
    assert payload["blocked_wait_remaining_s"] == 0.0


def test_update_route_state_exposes_blocked_fields_for_websocket_payload():
    node = _FakeRouteStateNode()
    response = SimpleNamespace(
        active=True,
        paused=False,
        loop=False,
        low_battery_active=True,
        return_home_requested=True,
        return_home_active=False,
        return_home_exit_waypoint_index=7,
        return_home_phase="waiting_exit",
        home_available=True,
        input_waypoint_count=2,
        expanded_waypoint_count=3,
        current_start_index=1,
        current_target_index=2,
        active_chunk_size=2,
        leg_spacing_m=30.0,
        chunk_span_m=80.0,
        chunk_max_waypoints=4,
        status="route blocked: waiting",
        blocked_state="BLOCKED_WAITING",
        blocked_reason_code="NO_VALID_PATH",
        blocked_reason_text="no valid path found",
        blocked_retry_attempt=1,
        blocked_retry_max_attempts=3,
        blocked_wait_remaining_s=7.5,
        home_lat=-31.002,
        home_lon=-64.002,
        home_yaw_deg=180.0,
        mission_lats=[-31.0, -31.001],
        mission_lons=[-64.0, -64.001],
        mission_yaws_deg=[0.0, 90.0],
        mission_waypoint_roles=["normal", "normal"],
        active_lats=[-31.001],
        active_lons=[-64.001],
        active_yaws_deg=[90.0],
    )

    node._update_route_state(response)

    assert node._route_mission["blocked_state"] == "BLOCKED_WAITING"
    assert node._route_mission["blocked_reason_code"] == "NO_VALID_PATH"
    assert node._route_mission["blocked_reason_text"] == "no valid path found"
    assert node._route_mission["blocked_retry_attempt"] == 1
    assert node._route_mission["blocked_retry_max_attempts"] == 3
    assert node._route_mission["blocked_wait_remaining_s"] == pytest.approx(7.5)
    assert node._route_mission["low_battery_active"] is True
    assert node._route_mission["return_home_requested"] is True
    assert node._route_mission["return_home_exit_waypoint_index"] == 7
    assert node._route_mission["return_home_phase"] == "waiting_exit"
    assert node._route_mission["home_available"] is True
    assert node._route_mission["home_waypoint"]["role"] == "home"


def test_mission_session_starts_only_after_goal_accepted(monkeypatch):
    monkeypatch.setattr(
        "map_tools.web_zone_server.asyncio.run_coroutine_threadsafe",
        _discard_scheduled_coroutine,
    )
    node = _FakeMissionSessionNode()

    node._on_nav_event(_nav_event("GOAL_REQUESTED"))
    node._on_nav_event(_nav_event("ACTION_SERVER_UNAVAILABLE"))

    assert node._mission_active is False
    assert node._mission_start_count == 0
    assert node._mission_records == []

    node._on_nav_event(_nav_event("GOAL_ACCEPTED"))

    assert node._mission_active is True
    assert node._mission_start_count == 1
    assert node._mission_records[-1]["data"]["code"] == "GOAL_ACCEPTED"


def test_mission_session_stops_on_terminal_nav_event(monkeypatch):
    monkeypatch.setattr(
        "map_tools.web_zone_server.asyncio.run_coroutine_threadsafe",
        _discard_scheduled_coroutine,
    )
    node = _FakeMissionSessionNode()

    node._on_nav_event(_nav_event("GOAL_ACCEPTED"))
    node._on_nav_event(_nav_event("GOAL_RESULT_ABORTED"))

    assert node._mission_active is False


def test_connection_status_exposes_battery_metadata():
    node = _FakeBatteryTelemetryNode()
    node._battery_pct = 84.5
    node._battery_voltage_v = 61.3
    node._battery_state = "OK"
    node._battery_present = True
    node._battery_updated_age_s = 0.8

    payload = node._connection_status_locked()

    assert payload["battery_pct"] == pytest.approx(84.5)
    assert payload["battery_voltage_v"] == pytest.approx(61.3)
    assert payload["battery_state"] == "OK"
    assert payload["battery_present"] is True
    assert payload["battery_updated_age_s"] == pytest.approx(0.8)


def test_controller_telemetry_updates_battery_fields_and_broadcasts():
    node = _FakeBatteryTelemetryNode()
    msg = SimpleNamespace(
        data=(
            '{"source":"auto","telemetry":{"ready":true},"requested_auto_command":{"drive_enabled":true},'
            '"battery":{"filtered_voltage_v":61.87,"filtered_percentage":0.92,"state":"OK","mission_guard_state":"OK",'
            '"return_home_recommended":false,"loaded_voltage_slow_v":61.87,"recovered_voltage_v":61.95,"ready":true,"link_age_s":0.6}}'
        )
    )

    node._on_controller_telemetry(msg)

    assert node._battery_pct == pytest.approx(92.0)
    assert node._battery_voltage_v == pytest.approx(61.87)
    assert node._battery_state == "OK"
    assert node._battery_mission_state == "OK"
    assert node._battery_return_home_recommended is False
    assert node._battery_recovered_voltage_v == pytest.approx(61.95)
    assert node._battery_loaded_voltage_v == pytest.approx(61.87)
    assert node._battery_present is True
    assert node._battery_updated_age_s == pytest.approx(0.6)
    assert node._broadcast_payloads[-1]["battery_voltage_v"] == pytest.approx(61.87)
    assert node._broadcast_payloads[-1]["battery_state"] == "OK"
    assert node._broadcast_payloads[-1]["battery_mission_state"] == "OK"


def test_battery_state_callback_does_not_desync_controller_battery_snapshot():
    node = _FakeBatteryTelemetryNode()
    msg = SimpleNamespace(
        data=(
            '{"source":"auto","telemetry":{"ready":true},"requested_auto_command":{"drive_enabled":true},'
            '"battery":{"filtered_voltage_v":61.87,"filtered_percentage":0.92,"state":"OK","mission_guard_state":"OK",'
            '"return_home_recommended":false,"loaded_voltage_slow_v":61.87,"recovered_voltage_v":61.95,"ready":true,"link_age_s":0.6}}'
        )
    )

    node._on_controller_telemetry(msg)
    node._on_battery_state(SimpleNamespace(percentage=0.15))

    assert node._battery_use_controller_telemetry is True
    assert node._battery_pct == pytest.approx(92.0)
    assert node._battery_state == "OK"
    assert node._battery_voltage_v == pytest.approx(61.87)
