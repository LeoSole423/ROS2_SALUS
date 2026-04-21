from pathlib import Path

from map_tools.datum_file_utils import (
    load_datums_yaml_file,
    normalize_datum_profile,
    parse_datums_yaml_text,
    save_datums_yaml_file,
)


def test_normalize_datum_profile_validates_coordinates():
    profile, err = normalize_datum_profile(
        {"name": "Base Norte", "lat": -31.4, "lon": -64.1, "yaw_deg": 361.0}
    )

    assert err == ""
    assert profile is not None
    assert profile["id"] == "base-norte"
    assert profile["yaw_deg"] == 1.0


def test_parse_datums_yaml_rejects_missing_name():
    doc, err = parse_datums_yaml_text(
        """
version: 1
datums:
- lat: -31.4
  lon: -64.1
"""
    )

    assert doc is None
    assert "name is required" in err


def test_save_and_load_datums_yaml_file(tmp_path: Path):
    file_path = tmp_path / "datums.yaml"
    ok, err = save_datums_yaml_file(
        file_path,
        {
            "version": 1,
            "selected_id": "campo-sur",
            "datums": [
                {
                    "id": "campo-sur",
                    "name": "Campo Sur",
                    "lat": -31.5,
                    "lon": -64.2,
                    "yaw_deg": 0.0,
                }
            ],
        },
    )

    assert ok is True
    assert err == ""

    ok, err, doc = load_datums_yaml_file(file_path)

    assert ok is True
    assert err == ""
    assert doc["selected_id"] == "campo-sur"
    assert doc["datums"][0]["name"] == "Campo Sur"
