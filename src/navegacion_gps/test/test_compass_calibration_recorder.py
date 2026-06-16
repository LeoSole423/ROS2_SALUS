import json
import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from navegacion_gps.compass_calibration_recorder import (  # noqa: E402
    CalibrationThresholds,
    CalibrationTopics,
    CompassCalibrationCollector,
    parse_gps_debug_payload,
)


def _collector(
    *,
    min_valid_samples: int = 2,
    max_bias_std_deg: float = 3.0,
) -> CompassCalibrationCollector:
    return CompassCalibrationCollector(
        label="test",
        duration_s=60.0,
        thresholds=CalibrationThresholds(
            min_valid_samples=min_valid_samples,
            min_speed_mps=0.5,
            max_abs_steer_deg=5.0,
            max_abs_yaw_rate_rps=0.08,
            max_bias_std_deg=max_bias_std_deg,
            max_pair_age_s=1.0,
        ),
        topics=CalibrationTopics(),
    )


def _add_valid_pair(
    collector: CompassCalibrationCollector,
    *,
    stamp_s: float,
    compass_hdg_deg: float,
    gps_yaw_deg: float,
    speed_mps: float = 1.0,
    steer_deg: float = 0.0,
    yaw_rate_rps: float = 0.0,
) -> None:
    collector.add_gps_debug(
        {
            "valid": True,
            "rtk_status": "RTK_FIXED",
            "rtk_status_normalized": "rtk_fixed",
        }
    )
    collector.add_drive_telemetry(speed_mps=speed_mps, steer_deg=steer_deg)
    collector.add_imu_yaw_rate(yaw_rate_rps)
    collector.add_compass(compass_hdg_deg, stamp_s)
    collector.add_gps_course_heading(gps_yaw_deg, stamp_s + 0.1)


def test_parse_gps_debug_payload_handles_valid_json() -> None:
    payload = parse_gps_debug_payload('{"valid": true, "yaw_deg": -5.0}')

    assert payload["valid"] is True
    assert payload["yaw_deg"] == -5.0


def test_parse_gps_debug_payload_marks_invalid_json() -> None:
    payload = parse_gps_debug_payload("not-json")

    assert payload["valid"] is False
    assert payload["parse_error"] is True


def test_collector_recommends_bias_from_stable_delta() -> None:
    collector = _collector(min_valid_samples=2)
    _add_valid_pair(
        collector,
        stamp_s=10.0,
        compass_hdg_deg=116.0,
        gps_yaw_deg=-5.0,
    )
    _add_valid_pair(
        collector,
        stamp_s=11.0,
        compass_hdg_deg=117.0,
        gps_yaw_deg=-6.0,
    )
    collector.finish()

    report = collector.build_report()

    delta = report["summaries"]["delta_compass_minus_gps_yaw_deg"]
    assert delta["count"] == 2
    assert math.isclose(delta["mean_deg"], -21.0, abs_tol=1.0e-6)
    assert report["recommendation"]["enough_data"] is True
    assert math.isclose(
        report["recommendation"]["recommended_yaw_bias_deg"],
        21.0,
        abs_tol=1.0e-6,
    )
    assert math.isclose(
        report["recommendation"]["recommended_compass_hdg_bias_deg"],
        -21.0,
        abs_tol=1.0e-6,
    )


def test_collector_filters_invalid_samples_by_motion_and_debug() -> None:
    collector = _collector(min_valid_samples=1)

    collector.add_gps_debug({"valid": False})
    collector.add_drive_telemetry(speed_mps=1.0, steer_deg=0.0)
    collector.add_imu_yaw_rate(0.0)
    collector.add_compass(90.0, 10.0)
    collector.add_gps_course_heading(0.0, 10.0)

    _add_valid_pair(
        collector,
        stamp_s=11.0,
        compass_hdg_deg=90.0,
        gps_yaw_deg=0.0,
        speed_mps=0.1,
    )
    _add_valid_pair(
        collector,
        stamp_s=12.0,
        compass_hdg_deg=90.0,
        gps_yaw_deg=0.0,
        steer_deg=8.0,
    )
    _add_valid_pair(
        collector,
        stamp_s=13.0,
        compass_hdg_deg=90.0,
        gps_yaw_deg=0.0,
        yaw_rate_rps=0.2,
    )

    report = collector.build_report()

    assert report["sample_counts"]["valid_comparison"] == 0
    assert report["invalid_reasons"]["gps_debug_invalid"] == 1
    assert report["invalid_reasons"]["speed_low"] == 1
    assert report["invalid_reasons"]["steer_high"] == 1
    assert report["invalid_reasons"]["yaw_rate_high"] == 1
    assert report["recommendation"]["enough_data"] is False


def test_collector_filters_stale_compass_pair() -> None:
    collector = _collector(min_valid_samples=1)
    collector.add_gps_debug({"valid": True})
    collector.add_drive_telemetry(speed_mps=1.0, steer_deg=0.0)
    collector.add_imu_yaw_rate(0.0)
    collector.add_compass(90.0, 10.0)
    collector.add_gps_course_heading(0.0, 12.0)

    report = collector.build_report()

    assert report["sample_counts"]["valid_comparison"] == 0
    assert report["invalid_reasons"]["compass_pair_stale"] == 1


def test_report_has_agent_stable_json_fields() -> None:
    collector = _collector(min_valid_samples=1)
    _add_valid_pair(
        collector,
        stamp_s=10.0,
        compass_hdg_deg=90.0,
        gps_yaw_deg=0.0,
    )
    report = collector.build_report(include_samples=True)

    rendered = json.dumps(report, sort_keys=True)
    reparsed = json.loads(rendered)

    assert reparsed["schema_version"] == 1
    assert reparsed["tool"] == "compass_calibration_recorder"
    assert "metadata" in reparsed
    assert "sample_counts" in reparsed
    assert "summaries" in reparsed
    assert "comparison" in reparsed
    assert "recommendation" in reparsed
    assert len(reparsed["valid_samples"]) == 1


def test_mag_norm_is_reported_in_microtesla() -> None:
    collector = _collector(min_valid_samples=1)
    collector.add_mag(10.0e-6, 20.0e-6, 20.0e-6)

    report = collector.build_report()

    summary = report["summaries"]["mag_norm_uT"]
    assert summary["count"] == 1
    assert math.isclose(summary["mean"], 30.0, abs_tol=1.0e-6)
