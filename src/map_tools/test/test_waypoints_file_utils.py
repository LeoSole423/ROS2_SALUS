from pathlib import Path

from map_tools.waypoints_file_utils import (
    load_waypoints_yaml_file,
    parse_waypoints_yaml_text,
    save_waypoints_yaml_file,
)


def test_parse_waypoints_yaml_text_canonical():
    text = """
waypoints:
  - latitude: -31.0
    longitude: -64.0
    yaw: 10.0
  - latitude: -31.1
    longitude: -64.1
    yaw: 20.0
"""
    waypoints, patrol_profile, err = parse_waypoints_yaml_text(text)
    assert err == ""
    assert patrol_profile is None
    assert waypoints == [
        {"lat": -31.0, "lon": -64.0, "yaw_deg": 10.0},
        {"lat": -31.1, "lon": -64.1, "yaw_deg": 20.0},
    ]


def test_parse_waypoints_yaml_text_variant_keys():
    text = """
waypoints:
  - lat: -31.2
    lon: -64.2
    yaw_deg: 30.0
"""
    waypoints, patrol_profile, err = parse_waypoints_yaml_text(text)
    assert err == ""
    assert patrol_profile is None
    assert waypoints == [{"lat": -31.2, "lon": -64.2, "yaw_deg": 30.0}]


def test_parse_waypoints_yaml_text_accepts_auto_yaw():
    text = """
waypoints:
  - latitude: -31.2
    longitude: -64.2
"""
    waypoints, patrol_profile, err = parse_waypoints_yaml_text(text)
    assert err == ""
    assert patrol_profile is None
    assert waypoints == [{"lat": -31.2, "lon": -64.2}]


def test_parse_waypoints_yaml_text_invalid():
    waypoints, patrol_profile, err = parse_waypoints_yaml_text("waypoints: [")
    assert waypoints is None
    assert patrol_profile is None
    assert "invalid yaml" in err


def test_save_and_load_waypoints_yaml_file(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {"lat": -31.4, "lon": -64.4, "yaw_deg": 5.0},
        {"lat": -31.5, "lon": -64.5, "yaw_deg": -15.0},
    ]
    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src)
    assert ok_save
    assert err_save == ""
    assert count == 2

    ok_load, err_load, loaded, patrol_profile = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src
    assert patrol_profile is None

    raw_text = file_path.read_text(encoding="utf-8")
    assert "latitude" in raw_text
    assert "longitude" in raw_text
    assert "yaw" in raw_text


def test_save_and_load_waypoints_yaml_file_preserves_auto_yaw(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {"lat": -31.4, "lon": -64.4},
        {"lat": -31.5, "lon": -64.5, "yaw_deg": -15.0},
    ]
    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src)
    assert ok_save
    assert err_save == ""
    assert count == 2

    raw_text = file_path.read_text(encoding="utf-8")
    first_entry = raw_text.split("- latitude:", maxsplit=2)[1]
    assert "yaw" not in first_entry
    assert "yaw: -15.0" in raw_text

    ok_load, err_load, loaded, patrol_profile = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src
    assert patrol_profile is None


def test_save_and_load_waypoints_yaml_file_preserves_actions(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {
            "lat": -31.4,
            "lon": -64.4,
            "actions": [{"type": "brake_hold", "duration_s": 5.0, "brake_pct": 100}],
        },
    ]

    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src)
    assert ok_save
    assert err_save == ""
    assert count == 1

    ok_load, err_load, loaded, patrol_profile = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src
    assert patrol_profile is None


def test_save_and_load_waypoints_yaml_file_preserves_home_role(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {"lat": -31.4, "lon": -64.4, "role": "home"},
        {"lat": -31.5, "lon": -64.5, "yaw_deg": -15.0},
    ]

    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src)
    assert ok_save
    assert err_save == ""
    assert count == 2

    raw_text = file_path.read_text(encoding="utf-8")
    assert "role: home" in raw_text

    ok_load, err_load, loaded, patrol_profile = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src
    assert patrol_profile is None


def test_parse_waypoints_yaml_text_rejects_multiple_home_roles():
    text = """
waypoints:
  - latitude: -31.0
    longitude: -64.0
    role: home
  - latitude: -31.1
    longitude: -64.1
    role: home
"""
    waypoints, patrol_profile, err = parse_waypoints_yaml_text(text)
    assert waypoints is None
    assert patrol_profile is None
    assert "only one HOME waypoint" in err


def test_load_waypoints_yaml_file_missing(tmp_path: Path):
    missing = tmp_path / "missing.yaml"
    ok, err, waypoints, patrol_profile = load_waypoints_yaml_file(missing)
    assert not ok
    assert "not found" in err
    assert waypoints == []
    assert patrol_profile is None


def test_save_waypoints_yaml_file_rejects_empty(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    ok, err, count = save_waypoints_yaml_file(file_path, [])
    assert not ok
    assert "non-empty" in err
    assert count == 0


def test_save_and_load_waypoints_yaml_file_preserves_patrol_profile(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {"lat": -31.4, "lon": -64.4, "role": "home"},
        {"lat": -31.5, "lon": -64.5},
        {"lat": -31.6, "lon": -64.6},
    ]
    patrol_profile = {
        "home_waypoint_index": 0,
        "loop_waypoint_indices": [1, 2],
        "return_waypoint_indices": [],
        "depart_waypoint_indices": [],
        "depart_entry_waypoint_index": 1,
    }

    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src, patrol_profile)
    assert ok_save
    assert err_save == ""
    assert count == 3

    ok_load, err_load, loaded, loaded_profile = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src
    assert loaded_profile == patrol_profile
