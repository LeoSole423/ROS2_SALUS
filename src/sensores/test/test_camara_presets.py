from pathlib import Path

from sensores.camara import CamaraNode, _load_preset_overrides_file


class _LoggerStub:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def _bind(node: CamaraNode, name: str):
    return getattr(CamaraNode, name).__get__(node, CamaraNode)


def _build_test_node(tmp_path: Path) -> tuple[CamaraNode, _LoggerStub]:
    node = CamaraNode.__new__(CamaraNode)
    logger = _LoggerStub()
    node.get_logger = lambda: logger  # type: ignore[attr-defined]
    node._az_min = 0.0
    node._az_max = 355.0
    node._el_min = 0.0
    node._el_max = 90.0
    node._zoom_min = 1.0
    node._zoom_max = 4.0
    node._zoom_zero_level = 1.0
    node._zoom_in = False
    node._last_command = "none"
    node._presets_file = tmp_path / ".camera_presets.json"
    node._base_presets = {
        "home": (0.0, 0.0, 1.0),
        "front": (0.0, 0.0, 1.0),
        "left": (90.0, 0.0, 1.0),
        "right": (270.0, 0.0, 1.0),
        "rear": (180.0, 0.0, 1.0),
    }
    node._preset_overrides = {}
    node._presets = dict(node._base_presets)
    node._clamp = CamaraNode._clamp  # type: ignore[method-assign]
    node._normalize_azimuth = _bind(node, "_normalize_azimuth")
    node._normalize_preset_state = _bind(node, "_normalize_preset_state")
    node._refresh_presets_from_sources = _bind(node, "_refresh_presets_from_sources")
    node._load_preset_overrides = _bind(node, "_load_preset_overrides")
    node._write_preset_overrides = _bind(node, "_write_preset_overrides")
    node._resolve_preset = _bind(node, "_resolve_preset")
    node._state_to_payload = _bind(node, "_state_to_payload")
    node._match_preset = _bind(node, "_match_preset")
    node._save_preset_from_current_state = _bind(node, "_save_preset_from_current_state")
    return node, logger


def test_load_preset_overrides_file_parses_valid_entries(tmp_path: Path) -> None:
    presets_file = tmp_path / ".camera_presets.json"
    presets_file.write_text(
        '{"home":{"pan_deg":12,"tilt_deg":7,"zoom_level":3},"left":{"pan_deg":95,"tilt_deg":5,"zoom_level":1.0}}',
        encoding="utf-8",
    )

    overrides, err = _load_preset_overrides_file(presets_file)

    assert err == ""
    assert overrides["home"] == (12.0, 7.0, 3.0)
    assert overrides["left"] == (95.0, 5.0, 1.0)


def test_load_preset_overrides_tolerates_corrupt_file(tmp_path: Path) -> None:
    node, logger = _build_test_node(tmp_path)
    node._presets_file.write_text("{not-json", encoding="utf-8")

    node._load_preset_overrides()

    assert node._presets == node._base_presets
    assert logger.warnings
    assert "cannot read preset overrides file" in logger.warnings[0]


def test_load_preset_overrides_overlays_base_presets(tmp_path: Path) -> None:
    node, logger = _build_test_node(tmp_path)
    node._presets_file.write_text(
        '{"home":{"pan_deg":33,"tilt_deg":11,"zoom_level":2.5},"unknown":{"pan_deg":1,"tilt_deg":2,"zoom_level":3}}',
        encoding="utf-8",
    )

    node._load_preset_overrides()

    assert node._presets["home"] == (33.0, 11.0, 2.5)
    assert node._presets["left"] == (90.0, 0.0, 1.0)
    assert "unknown" not in node._preset_overrides
    assert logger.warnings == []


def test_save_home_preset_persists_current_zoom(tmp_path: Path) -> None:
    node, _ = _build_test_node(tmp_path)
    node._get_absolute_state = lambda: ((12.0, 34.0, 3.5), "")  # type: ignore[attr-defined]

    ok, err, payload = node._save_preset_from_current_state("home", save_zoom=True)

    assert ok is True
    assert err == ""
    assert payload is not None
    assert payload["saved_preset"] == "home"
    assert node._presets["home"] == (34.0, 12.0, 3.5)
    overrides, load_err = _load_preset_overrides_file(node._presets_file)
    assert load_err == ""
    assert overrides["home"] == (34.0, 12.0, 3.5)


def test_save_left_preset_preserves_existing_zoom(tmp_path: Path) -> None:
    node, _ = _build_test_node(tmp_path)
    node._presets["left"] = (90.0, 0.0, 1.0)
    node._get_absolute_state = lambda: ((8.0, 120.0, 3.2), "")  # type: ignore[attr-defined]

    ok, err, payload = node._save_preset_from_current_state("left", save_zoom=False)

    assert ok is True
    assert err == ""
    assert payload is not None
    assert payload["saved_preset"] == "left"
    assert node._presets["left"] == (120.0, 8.0, 1.0)
    overrides, load_err = _load_preset_overrides_file(node._presets_file)
    assert load_err == ""
    assert overrides["left"] == (120.0, 8.0, 1.0)


def test_save_preset_rejects_invalid_name(tmp_path: Path) -> None:
    node, _ = _build_test_node(tmp_path)
    node._get_absolute_state = lambda: ((8.0, 120.0, 3.2), "")  # type: ignore[attr-defined]

    ok, err, payload = node._save_preset_from_current_state("rear", save_zoom=False)

    assert ok is False
    assert "cannot be overwritten" in err
    assert payload is None


def test_save_preset_does_not_mutate_memory_if_write_fails(tmp_path: Path) -> None:
    node, _ = _build_test_node(tmp_path)
    node._get_absolute_state = lambda: ((8.0, 120.0, 3.2), "")  # type: ignore[attr-defined]
    node._write_preset_overrides = lambda overrides: (False, "disk full")  # type: ignore[method-assign]

    ok, err, payload = node._save_preset_from_current_state("home", save_zoom=True)

    assert ok is False
    assert err == "disk full"
    assert payload is None
    assert node._preset_overrides == {}
    assert node._presets["home"] == (0.0, 0.0, 1.0)
