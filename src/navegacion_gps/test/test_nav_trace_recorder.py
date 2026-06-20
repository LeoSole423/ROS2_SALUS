import math

from navegacion_gps.nav_trace_recorder import (
    analyze_path_points,
    detect_replan_bursts,
    render_trace_summary,
)


def test_straight_path_has_no_o_signature():
    points = [(float(index), 0.0) for index in range(21)]

    metrics = analyze_path_points(points, robot_pose=(0.0, 0.0, 0.0))

    assert metrics["length_m"] == 20.0
    assert metrics["initial_heading_error_deg"] == 0.0
    assert metrics["turn_first_20m_deg"] == 0.0
    assert metrics["self_intersections"] == 0
    assert metrics["suspected_o_path"] is False


def test_circle_path_is_flagged_as_possible_o():
    radius = 2.0
    points = [
        (
            radius * math.cos(step * 2.0 * math.pi / 72.0),
            radius * math.sin(step * 2.0 * math.pi / 72.0),
        )
        for step in range(73)
    ]

    metrics = analyze_path_points(points, robot_pose=(radius, 0.0, math.pi / 2.0))

    assert metrics["turn_first_20m_deg"] >= 300.0
    assert metrics["suspected_o_path"] is True


def test_path_comparison_reports_large_replan_change():
    previous = [(float(index), 0.0) for index in range(10)]
    current = [(float(index), 4.0) for index in range(10)]

    metrics = analyze_path_points(current, previous_points=previous)

    assert metrics["mean_change_from_previous_m"] == 4.0
    assert metrics["max_change_from_previous_m"] == 4.0


def test_detect_replan_bursts_groups_three_events_inside_ten_seconds():
    records = [
        {"t_wall": 100.0, "code": "REPLAN_STARTED", "data": {"reason": "goal_updated"}},
        {"t_wall": 103.0, "code": "REPLAN_STARTED", "data": {"reason": "path_invalid"}},
        {"t_wall": 108.0, "code": "REPLAN_STARTED", "data": {"reason": "path_invalid"}},
        {"t_wall": 120.0, "code": "REPLAN_STARTED", "data": {"reason": "clearance_invalid"}},
    ]

    bursts = detect_replan_bursts(records)

    assert len(bursts) == 1
    assert bursts[0]["count"] == 3
    assert bursts[0]["reasons"] == ["goal_updated", "path_invalid", "path_invalid"]


def test_summary_is_self_contained_for_agent_review():
    records = [
        {
            "t_wall": 1.0,
            "code": "REPLAN_STARTED",
            "kind": "replan",
            "data": {"reason": "path_invalid"},
        },
        {
            "t_wall": 2.0,
            "code": "ROUTE_CHECKPOINT_REACHED",
            "kind": "route_event",
            "data": {"input_index": 1},
        },
        {
            "t_wall": 3.0,
            "code": "PLAN_PUBLISHED",
            "kind": "plan",
            "data": {
                "plan_id": 2,
                "suspected_o_path": True,
                "turn_first_20m_deg": 310.0,
                "self_intersections": 0,
                "initial_heading_error_deg": 90.0,
            },
        },
        {
            "t_wall": 4.0,
            "code": "CLEARANCE_INVALID",
            "kind": "clearance",
            "data": {"reason": "lethal_cost", "max_cost": "254"},
        },
    ]

    summary = render_trace_summary(
        {"mission_id": "mission-test", "status": "active", "started_at_utc": "now"},
        records,
    )

    assert "Navigation trace mission-test" in summary
    assert "`path_invalid`: 1" in summary
    assert "## Clearance Validator" in summary
    assert "`lethal_cost`: 1" in summary
    assert "Paths con posible O: `1`" in summary
    assert "timeline.jsonl" in summary
