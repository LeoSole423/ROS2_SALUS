#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
import requests
from ament_index_python.packages import get_package_share_directory
from requests.auth import HTTPDigestAuth
from requests.exceptions import RequestException
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import (
    CameraPan,
    CameraPreset,
    CameraSavePreset,
    CameraPtz,
    CameraPtzState,
    CameraStatus,
)


def _parse_env_file(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except Exception:
        return values

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        if key:
            values[key] = value
    return values


def _resolve_default_env_file() -> Path:
    try:
        share_dir = Path(get_package_share_directory("sensores"))
        candidates = [share_dir / ".env"]
        try:
            workspace_root = share_dir.parents[3]
            candidates.append(workspace_root / "src" / "sensores" / ".env")
        except IndexError:
            pass

        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]
    except Exception:
        return Path(__file__).resolve().parents[2] / ".env"


def _resolve_default_presets_file(env_file: Path) -> Path:
    env_parent = env_file.parent if str(env_file.parent) else Path(".")
    return env_parent / ".camera_presets.json"


def _compact_body(body: str, max_len: int = 280) -> str:
    compact = " ".join((body or "").strip().split())
    if not compact:
        return "<empty>"
    if len(compact) <= max_len:
        return compact
    return compact[: max_len - 3] + "..."


def _local_xml_text(root: ET.Element, local_name: str) -> Optional[str]:
    for elem in root.iter():
        if elem.tag.rsplit("}", 1)[-1] == local_name and elem.text is not None:
            return elem.text.strip()
    return None


def _to_float(value: str) -> float:
    return float(value.replace(",", ".").strip())


def _serialize_preset_map(presets: Dict[str, Tuple[float, float, float]]) -> Dict[str, Dict[str, float]]:
    return {
        name: {
            "pan_deg": round(float(values[0]), 4),
            "tilt_deg": round(float(values[1]), 4),
            "zoom_level": round(float(values[2]), 4),
        }
        for name, values in sorted(presets.items())
    }


def _parse_preset_entry(raw: Any) -> Optional[Tuple[float, float, float]]:
    if not isinstance(raw, dict):
        return None
    pan_deg = raw.get("pan_deg")
    tilt_deg = raw.get("tilt_deg")
    zoom_level = raw.get("zoom_level")
    values = (pan_deg, tilt_deg, zoom_level)
    if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values):
        return None
    return (float(pan_deg), float(tilt_deg), float(zoom_level))


def _load_preset_overrides_file(path: Path) -> Tuple[Dict[str, Tuple[float, float, float]], str]:
    if not path.exists():
        return {}, ""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {}, f"cannot read preset overrides file '{path}': {exc}"
    if not isinstance(raw, dict):
        return {}, f"preset overrides file '{path}' must contain a JSON object"

    overrides: Dict[str, Tuple[float, float, float]] = {}
    invalid_names = []
    for name, value in raw.items():
        if not isinstance(name, str):
            invalid_names.append(str(name))
            continue
        parsed = _parse_preset_entry(value)
        if parsed is None:
            invalid_names.append(name)
            continue
        overrides[name.strip().lower()] = parsed

    if invalid_names:
        return overrides, (
            f"preset overrides file '{path}' contains invalid entries: "
            + ", ".join(sorted(invalid_names))
        )
    return overrides, ""


class CamaraNode(Node):
    _PRESET_ALIASES = {
        "center": "home",
        "home": "home",
        "front": "front",
        "left": "left",
        "right": "right",
        "rear": "rear",
        "back": "rear",
    }
    _SAVEABLE_PRESETS = {"home", "left", "right"}

    def __init__(self) -> None:
        super().__init__("camara")

        default_env_file = str(_resolve_default_env_file())
        self.declare_parameter("env_file", default_env_file)
        self._env_file = Path(str(self.get_parameter("env_file").value))
        self._env_data = _parse_env_file(self._env_file)
        self.declare_parameter(
            "camera_presets_file",
            str(_resolve_default_presets_file(self._env_file)),
        )
        self._presets_file = Path(str(self.get_parameter("camera_presets_file").value))

        self.declare_parameter("camera_host", self._env_cfg("CAMERA_HOST", "192.168.1.64"))
        self.declare_parameter("camera_port", int(self._env_cfg("CAMERA_PORT", "80")))
        self.declare_parameter("camera_user", self._env_cfg("CAMERA_USER", "admin"))
        self.declare_parameter("camera_pass", self._env_cfg("CAMERA_PASS", "CHANGE_ME"))
        self.declare_parameter("camera_channel", int(self._env_cfg("CAMERA_CHANNEL", "1")))
        self.declare_parameter("camera_timeout_s", 2.0)

        self.declare_parameter("camera_az_min", 0.0)
        self.declare_parameter("camera_az_max", 355.0)
        self.declare_parameter("camera_el_min", 0.0)
        self.declare_parameter("camera_el_max", 90.0)
        self.declare_parameter("camera_zoom_min", 1.0)
        self.declare_parameter("camera_zoom_max", 4.0)
        self.declare_parameter("camera_zoom_fixed_level", 4.0)
        self.declare_parameter("camera_zoom_zero_level", 1.0)
        self.declare_parameter("camera_zoom_initial_in", False)
        self.declare_parameter("camera_preset_front_azimuth_deg", 0.0)
        self.declare_parameter("camera_preset_left_azimuth_deg", 90.0)
        self.declare_parameter("camera_preset_right_azimuth_deg", 270.0)
        self.declare_parameter("camera_preset_rear_azimuth_deg", 180.0)
        self.declare_parameter("camera_preset_neutral_elevation_deg", 0.0)
        self.declare_parameter("camera_preset_home_zoom_level", 1.0)

        self._host = str(self.get_parameter("camera_host").value)
        self._port = int(self.get_parameter("camera_port").value)
        self._user = str(self.get_parameter("camera_user").value)
        self._password = str(self.get_parameter("camera_pass").value)
        self._channel = max(1, int(self.get_parameter("camera_channel").value))
        self._timeout_s = max(0.2, float(self.get_parameter("camera_timeout_s").value))
        self._az_min = float(self.get_parameter("camera_az_min").value)
        self._az_max = float(self.get_parameter("camera_az_max").value)
        self._el_min = float(self.get_parameter("camera_el_min").value)
        self._el_max = float(self.get_parameter("camera_el_max").value)
        self._zoom_min = float(self.get_parameter("camera_zoom_min").value)
        self._zoom_max = float(self.get_parameter("camera_zoom_max").value)
        self._zoom_fixed_level = self._clamp(
            float(self.get_parameter("camera_zoom_fixed_level").value),
            self._zoom_min,
            self._zoom_max,
        )
        self._zoom_zero_level = self._clamp(
            float(self.get_parameter("camera_zoom_zero_level").value),
            self._zoom_min,
            self._zoom_max,
        )
        self._zoom_in = bool(self.get_parameter("camera_zoom_initial_in").value)
        self._preset_neutral_elevation = self._clamp(
            float(self.get_parameter("camera_preset_neutral_elevation_deg").value),
            self._el_min,
            self._el_max,
        )
        self._preset_home_zoom = self._clamp(
            float(self.get_parameter("camera_preset_home_zoom_level").value),
            self._zoom_min,
            self._zoom_max,
        )
        self._base_url = (
            f"http://{self._host}:{self._port}/ISAPI/PTZCtrl/channels/{self._channel}"
        )
        self._absolute_url = f"{self._base_url}/absoluteEx"
        self._session = requests.Session()
        self._session.auth = HTTPDigestAuth(self._user, self._password)
        self._ready = False
        self._ready_error = ""
        self._last_command = "none"

        self._base_presets = {
            "home": (
                self._normalize_azimuth(
                    float(self.get_parameter("camera_preset_front_azimuth_deg").value)
                ),
                self._preset_neutral_elevation,
                self._preset_home_zoom,
            ),
            "front": (
                self._normalize_azimuth(
                    float(self.get_parameter("camera_preset_front_azimuth_deg").value)
                ),
                self._preset_neutral_elevation,
                self._zoom_zero_level,
            ),
            "left": (
                self._normalize_azimuth(
                    float(self.get_parameter("camera_preset_left_azimuth_deg").value)
                ),
                self._preset_neutral_elevation,
                self._zoom_zero_level,
            ),
            "right": (
                self._normalize_azimuth(
                    float(self.get_parameter("camera_preset_right_azimuth_deg").value)
                ),
                self._preset_neutral_elevation,
                self._zoom_zero_level,
            ),
            "rear": (
                self._normalize_azimuth(
                    float(self.get_parameter("camera_preset_rear_azimuth_deg").value)
                ),
                self._preset_neutral_elevation,
                self._zoom_zero_level,
            ),
        }
        self._preset_overrides = {}
        self._presets = dict(self._base_presets)
        self._load_preset_overrides()

        self._connect_isapi()

        self.create_service(CameraPan, "/camara/camera_pan", self._on_camera_pan)
        self.create_service(
            Trigger,
            "/camara/camera_zoom_toggle",
            self._on_camera_zoom_toggle,
        )
        self.create_service(CameraStatus, "/camara/camera_status", self._on_camera_status)
        self.create_service(CameraPtz, "/camara/camera_ptz", self._on_camera_ptz)
        self.create_service(
            CameraPreset,
            "/camara/camera_preset",
            self._on_camera_preset,
        )
        self.create_service(
            CameraSavePreset,
            "/camara/camera_save_preset",
            self._on_camera_save_preset,
        )
        self.create_service(
            CameraPtzState,
            "/camara/camera_ptz_state",
            self._on_camera_ptz_state,
        )

        self.get_logger().info(
            "camara node ready "
            f"(env_file={self._env_file}, host={self._host}:{self._port}, "
            f"channel={self._channel}, absolute_url={self._absolute_url}, "
            f"presets_file={self._presets_file}, "
            f"isapi_ready={self._ready})"
        )

    def _env_cfg(self, key: str, default: str) -> str:
        value = self._env_data.get(key, "")
        if value:
            return value
        env_value = os.environ.get(key, "")
        if env_value:
            return env_value
        return default

    def _connect_isapi(self) -> None:
        if not self._host or not self._user or not self._password:
            self._ready = False
            self._ready_error = "missing CAMERA_HOST/CAMERA_USER/CAMERA_PASS in env config"
            self.get_logger().error(self._ready_error)
            return

        try:
            state, err = self._get_absolute_state()
            if state is None:
                raise RuntimeError(err or "ISAPI absoluteEx probe failed")
            self._zoom_in = abs(state[2] - self._zoom_zero_level) > 0.05
            self._ready = True
            self._ready_error = ""
        except Exception as exc:
            self._ready = False
            self._ready_error = f"ISAPI init failed: {exc}"
            self.get_logger().error(self._ready_error)

    def _normalize_preset_state(
        self, values: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        pan_deg, tilt_deg, zoom_level = values
        return (
            self._normalize_azimuth(float(pan_deg)),
            self._clamp(float(tilt_deg), self._el_min, self._el_max),
            self._clamp(float(zoom_level), self._zoom_min, self._zoom_max),
        )

    def _refresh_presets_from_sources(self) -> None:
        presets = dict(self._base_presets)
        for name, values in self._preset_overrides.items():
            if name not in presets:
                continue
            presets[name] = self._normalize_preset_state(values)
        self._presets = presets

    def _load_preset_overrides(self) -> None:
        overrides, err = _load_preset_overrides_file(self._presets_file)
        sanitized: Dict[str, Tuple[float, float, float]] = {}
        for name, values in overrides.items():
            if name not in self._base_presets:
                continue
            sanitized[name] = self._normalize_preset_state(values)
        self._preset_overrides = sanitized
        self._refresh_presets_from_sources()
        if err:
            self.get_logger().warning(err)

    def _write_preset_overrides(
        self, overrides: Dict[str, Tuple[float, float, float]]
    ) -> Tuple[bool, str]:
        try:
            self._presets_file.parent.mkdir(parents=True, exist_ok=True)
            self._presets_file.write_text(
                json.dumps(_serialize_preset_map(overrides), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            return False, f"cannot write preset overrides file '{self._presets_file}': {exc}"
        return True, ""

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(float(lo), min(float(hi), float(value)))

    def _normalize_azimuth(self, angle_deg: float) -> float:
        angle = math.fmod(float(angle_deg), 360.0)
        if angle < 0.0:
            angle += 360.0
        if angle > self._az_max:
            angle = self._az_min
        return self._clamp(angle, self._az_min, self._az_max)

    def _request_isapi(
        self, method: str, url: str, data: Optional[str] = None
    ) -> Tuple[Optional[str], str]:
        headers = {"Content-Type": "application/xml"} if data is not None else None
        try:
            res = self._session.request(
                method=method,
                url=url,
                data=data,
                headers=headers,
                timeout=self._timeout_s,
            )
        except RequestException as exc:
            return None, f"ISAPI {method} request failed: {exc}"

        if not res.ok:
            body = _compact_body(res.text)
            return (
                None,
                f"ISAPI {method} failed: HTTP {res.status_code} {res.reason}; body='{body}'",
            )
        return res.text, ""

    def _get_absolute_state(self) -> Tuple[Optional[Tuple[float, float, float]], str]:
        xml_text, err = self._request_isapi("GET", self._absolute_url)
        if xml_text is None:
            return None, err
        try:
            root = ET.fromstring(xml_text)
            el_raw = _local_xml_text(root, "elevation")
            az_raw = _local_xml_text(root, "azimuth")
            zm_raw = _local_xml_text(root, "absoluteZoom")
            if el_raw is None or az_raw is None or zm_raw is None:
                return (
                    None,
                    "ISAPI absoluteEx response missing elevation/azimuth/absoluteZoom",
                )
            return (_to_float(el_raw), _to_float(az_raw), _to_float(zm_raw)), ""
        except Exception as exc:
            body = _compact_body(xml_text)
            return None, f"invalid ISAPI absoluteEx XML: {exc}; body='{body}'"

    def _set_absolute_state(
        self, elevation: float, azimuth: float, zoom: float
    ) -> Tuple[bool, str]:
        el = int(round(self._clamp(elevation, self._el_min, self._el_max)))
        az = int(round(self._normalize_azimuth(azimuth)))
        zm = int(round(self._clamp(zoom, self._zoom_min, self._zoom_max)))
        payload = (
            '<PTZAbsoluteEx version="2.0" xmlns="http://www.hikvision.com/ver20/XMLSchema">'
            f"<elevation>{el}</elevation>"
            f"<azimuth>{az}</azimuth>"
            f"<absoluteZoom>{zm}</absoluteZoom>"
            "</PTZAbsoluteEx>"
        )
        _, err = self._request_isapi("PUT", self._absolute_url, data=payload)
        if err:
            return False, err
        return True, ""

    def _require_ready(self) -> Tuple[bool, str]:
        if self._ready:
            return True, ""
        return False, self._ready_error or "camera is not ready"

    def _state_to_payload(self, state: Tuple[float, float, float]) -> Dict[str, float | str | bool]:
        pan_deg = float(self._normalize_azimuth(state[1]))
        tilt_deg = float(self._clamp(state[0], self._el_min, self._el_max))
        zoom_level = float(self._clamp(state[2], self._zoom_min, self._zoom_max))
        active_preset = self._match_preset(pan_deg, tilt_deg, zoom_level)
        self._zoom_in = abs(zoom_level - self._zoom_zero_level) > 0.05
        return {
            "pan_deg": pan_deg,
            "tilt_deg": tilt_deg,
            "zoom_level": zoom_level,
            "zoom_in": bool(self._zoom_in),
            "last_command": self._last_command,
            "active_preset": active_preset,
        }

    def _read_state_payload(self) -> Tuple[Optional[Dict[str, float | str | bool]], str]:
        state, err = self._get_absolute_state()
        if state is None:
            return None, err
        return self._state_to_payload(state), ""

    def _match_preset(self, pan_deg: float, tilt_deg: float, zoom_level: float) -> str:
        tolerance_deg = 1.5
        tolerance_zoom = 0.2
        for name, (preset_pan, preset_tilt, preset_zoom) in self._presets.items():
            pan_error = abs(self._normalize_azimuth(pan_deg - preset_pan))
            pan_error = min(pan_error, abs(360.0 - pan_error))
            if (
                pan_error <= tolerance_deg
                and abs(tilt_deg - preset_tilt) <= tolerance_deg
                and abs(zoom_level - preset_zoom) <= tolerance_zoom
            ):
                return name
        return ""

    def _apply_ptz_move(
        self,
        *,
        relative: bool,
        apply_pan: bool,
        pan_deg: float,
        apply_tilt: bool,
        tilt_deg: float,
        apply_zoom: bool,
        zoom_level: float,
        command_label: str,
    ) -> Tuple[bool, str, Optional[Dict[str, float | str | bool]]]:
        state, err = self._get_absolute_state()
        if state is None:
            return False, f"cannot read current PTZ state: {err}", None

        current_tilt, current_pan, current_zoom = state
        target_pan = current_pan
        target_tilt = current_tilt
        target_zoom = current_zoom

        if apply_pan:
            if relative:
                target_pan = self._normalize_azimuth(current_pan + float(pan_deg))
            else:
                target_pan = self._normalize_azimuth(float(pan_deg))
        if apply_tilt:
            if relative:
                target_tilt = self._clamp(current_tilt + float(tilt_deg), self._el_min, self._el_max)
            else:
                target_tilt = self._clamp(float(tilt_deg), self._el_min, self._el_max)
        if apply_zoom:
            if relative:
                target_zoom = self._clamp(current_zoom + float(zoom_level), self._zoom_min, self._zoom_max)
            else:
                target_zoom = self._clamp(float(zoom_level), self._zoom_min, self._zoom_max)

        ok, set_err = self._set_absolute_state(target_tilt, target_pan, target_zoom)
        if not ok:
            return False, set_err, None

        updated_state, read_err = self._read_state_payload()
        if updated_state is None:
            return False, f"PTZ updated but state refresh failed: {read_err}", None
        self._last_command = command_label
        updated_state["last_command"] = self._last_command
        return True, "", updated_state

    def _resolve_preset(self, preset_raw: str) -> Tuple[Optional[str], str]:
        normalized = str(preset_raw or "").strip().lower()
        if not normalized:
            return None, "preset is required"
        preset = self._PRESET_ALIASES.get(normalized)
        if preset is None:
            return None, f"unsupported preset '{preset_raw}'"
        if preset not in self._presets:
            return None, f"preset '{preset}' is not configured"
        return preset, ""

    def _save_preset_from_current_state(
        self, preset_raw: str, *, save_zoom: bool
    ) -> Tuple[bool, str, Optional[Dict[str, float | str | bool]]]:
        preset, preset_err = self._resolve_preset(preset_raw)
        if preset is None:
            return False, preset_err, None
        if preset not in self._SAVEABLE_PRESETS:
            return False, f"preset '{preset}' cannot be overwritten from the UI", None

        state, err = self._get_absolute_state()
        if state is None:
            return False, f"cannot read current PTZ state: {err}", None

        current_tilt, current_pan, current_zoom = state
        target_pan = self._normalize_azimuth(current_pan)
        target_tilt = self._clamp(current_tilt, self._el_min, self._el_max)
        preserved_zoom = self._presets[preset][2]
        target_zoom = (
            self._clamp(current_zoom, self._zoom_min, self._zoom_max)
            if save_zoom
            else preserved_zoom
        )

        next_overrides = dict(self._preset_overrides)
        next_overrides[preset] = (target_pan, target_tilt, target_zoom)
        ok, write_err = self._write_preset_overrides(next_overrides)
        if not ok:
            return False, write_err, None

        self._preset_overrides = next_overrides
        self._refresh_presets_from_sources()
        self._last_command = f"save_preset:{preset}"
        payload = self._state_to_payload(state)
        payload["saved_preset"] = preset
        payload["saved_zoom"] = bool(save_zoom)
        return True, "", payload

    def _fill_status_response(
        self,
        response: CameraStatus.Response | CameraPtzState.Response,
        *,
        ok: bool,
        error: str,
        payload: Optional[Dict[str, float | str | bool]],
    ) -> None:
        response.ok = bool(ok)
        response.error = str(error)
        response.last_command = str(
            (payload or {}).get("last_command", self._last_command)
        )
        response.zoom_in = bool((payload or {}).get("zoom_in", self._zoom_in))
        response.pan_deg = float((payload or {}).get("pan_deg", 0.0))
        response.tilt_deg = float((payload or {}).get("tilt_deg", 0.0))
        response.zoom_level = float((payload or {}).get("zoom_level", 0.0))
        response.active_preset = str((payload or {}).get("active_preset", ""))

    def _on_camera_pan(
        self, request: CameraPan.Request, response: CameraPan.Response
    ) -> CameraPan.Response:
        ready, err = self._require_ready()
        if not ready:
            response.ok = False
            response.error = err
            response.applied_angle_deg = 0.0
            return response

        input_angle = float(request.angle_deg)
        if not math.isfinite(input_angle):
            response.ok = False
            response.error = "angle_deg must be finite"
            response.applied_angle_deg = 0.0
            return response

        applied_angle = self._normalize_azimuth(input_angle)
        ok, err, _ = self._apply_ptz_move(
            relative=False,
            apply_pan=True,
            pan_deg=applied_angle,
            apply_tilt=False,
            tilt_deg=0.0,
            apply_zoom=False,
            zoom_level=0.0,
            command_label=f"angle:{applied_angle:.1f}",
        )
        response.ok = bool(ok)
        response.error = "" if ok else err
        response.applied_angle_deg = float(applied_angle)
        return response

    def _on_camera_zoom_toggle(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ready, err = self._require_ready()
        if not ready:
            response.success = False
            response.message = err
            return response

        state, state_err = self._get_absolute_state()
        if state is None:
            response.success = False
            response.message = state_err
            return response

        current_zoom = float(state[2])
        epsilon = 0.05
        is_zero = abs(current_zoom - self._zoom_zero_level) <= epsilon
        target = self._zoom_fixed_level if is_zero else self._zoom_zero_level
        ok, err, _ = self._apply_ptz_move(
            relative=False,
            apply_pan=False,
            pan_deg=0.0,
            apply_tilt=False,
            tilt_deg=0.0,
            apply_zoom=True,
            zoom_level=target,
            command_label="zoom_toggle",
        )
        response.success = bool(ok)
        response.message = "" if ok else err
        return response

    def _on_camera_status(
        self, _request: CameraStatus.Request, response: CameraStatus.Response
    ) -> CameraStatus.Response:
        ready, err = self._require_ready()
        if not ready:
            self._fill_status_response(response, ok=False, error=err, payload=None)
            return response

        payload, state_err = self._read_state_payload()
        if payload is None:
            self._fill_status_response(response, ok=False, error=state_err, payload=None)
            return response

        self._fill_status_response(response, ok=True, error="", payload=payload)
        return response

    def _on_camera_ptz(
        self, request: CameraPtz.Request, response: CameraPtz.Response
    ) -> CameraPtz.Response:
        ready, err = self._require_ready()
        if not ready:
            response.ok = False
            response.error = err
            return response

        if not (request.apply_pan or request.apply_tilt or request.apply_zoom):
            response.ok = False
            response.error = "at least one PTZ axis must be requested"
            return response

        for value, label, enabled in (
            (request.pan_deg, "pan_deg", request.apply_pan),
            (request.tilt_deg, "tilt_deg", request.apply_tilt),
            (request.zoom_level, "zoom_level", request.apply_zoom),
        ):
            if enabled and not math.isfinite(float(value)):
                response.ok = False
                response.error = f"{label} must be finite"
                return response

        ok, err, payload = self._apply_ptz_move(
            relative=bool(request.relative),
            apply_pan=bool(request.apply_pan),
            pan_deg=float(request.pan_deg),
            apply_tilt=bool(request.apply_tilt),
            tilt_deg=float(request.tilt_deg),
            apply_zoom=bool(request.apply_zoom),
            zoom_level=float(request.zoom_level),
            command_label=(
                "ptz:relative"
                if bool(request.relative)
                else "ptz:absolute"
            ),
        )
        response.ok = bool(ok)
        response.error = "" if ok else err
        if payload is not None:
            response.pan_deg = float(payload["pan_deg"])
            response.tilt_deg = float(payload["tilt_deg"])
            response.zoom_level = float(payload["zoom_level"])
        return response

    def _on_camera_preset(
        self, request: CameraPreset.Request, response: CameraPreset.Response
    ) -> CameraPreset.Response:
        ready, err = self._require_ready()
        if not ready:
            response.ok = False
            response.error = err
            return response

        preset, preset_err = self._resolve_preset(request.preset)
        if preset is None:
            response.ok = False
            response.error = preset_err
            return response

        target_pan, target_tilt, target_zoom = self._presets[preset]
        ok, err, payload = self._apply_ptz_move(
            relative=False,
            apply_pan=True,
            pan_deg=target_pan,
            apply_tilt=True,
            tilt_deg=target_tilt,
            apply_zoom=True,
            zoom_level=target_zoom,
            command_label=f"preset:{preset}",
        )
        response.ok = bool(ok)
        response.error = "" if ok else err
        response.applied_preset = preset if ok else ""
        if payload is not None:
            response.pan_deg = float(payload["pan_deg"])
            response.tilt_deg = float(payload["tilt_deg"])
            response.zoom_level = float(payload["zoom_level"])
        return response

    def _on_camera_ptz_state(
        self, _request: CameraPtzState.Request, response: CameraPtzState.Response
    ) -> CameraPtzState.Response:
        ready, err = self._require_ready()
        if not ready:
            self._fill_status_response(response, ok=False, error=err, payload=None)
            return response

        payload, state_err = self._read_state_payload()
        if payload is None:
            self._fill_status_response(response, ok=False, error=state_err, payload=None)
            return response

        self._fill_status_response(response, ok=True, error="", payload=payload)
        return response

    def _on_camera_save_preset(
        self, request: CameraSavePreset.Request, response: CameraSavePreset.Response
    ) -> CameraSavePreset.Response:
        ready, err = self._require_ready()
        if not ready:
            response.ok = False
            response.error = err
            return response

        ok, err, payload = self._save_preset_from_current_state(
            request.preset,
            save_zoom=bool(request.save_zoom),
        )
        response.ok = bool(ok)
        response.error = "" if ok else err
        response.saved_preset = str((payload or {}).get("saved_preset", "")) if ok else ""
        if payload is not None:
            saved_preset = str(payload.get("saved_preset", ""))
            response.saved_preset = saved_preset if ok else ""
            target_pan, target_tilt, target_zoom = self._presets.get(
                saved_preset,
                (
                    float(payload.get("pan_deg", 0.0)),
                    float(payload.get("tilt_deg", 0.0)),
                    float(payload.get("zoom_level", 0.0)),
                ),
            )
            response.pan_deg = float(target_pan)
            response.tilt_deg = float(target_tilt)
            response.zoom_level = float(target_zoom)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamaraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
