from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping


_ENV_OVERRIDE = "SALUS_CONTROLLER_SERIAL_PORT"
_BY_ID_ROOT = "/dev/serial/by-id"
_CP2102_PATTERNS = (
    f"{_BY_ID_ROOT}/*Silicon_Labs_CP2102*",
    f"{_BY_ID_ROOT}/*CP2102*",
)
_GENERIC_USB_HINTS = (
    "CP210",
    "FTDI",
    "CH340",
    "PL2303",
    "USB_UART",
    "USB-TO-UART",
    "USB_TO_UART",
    "USB SERIAL",
    "USB-SERIAL",
    "USBSERIAL",
    "UART",
)


@dataclass(frozen=True)
class SerialPortSelection:
    port: str
    reason: str


class SerialPortResolutionError(RuntimeError):
    pass


def _normalize(value: object) -> str:
    return str(value).strip()


def _is_auto_request(value: object) -> bool:
    normalized = _normalize(value)
    return normalized == "" or normalized.lower() == "auto"


def _existing_unique(
    patterns: Iterable[str],
    *,
    glob_fn: Callable[[str], list[str]],
    exists_fn: Callable[[str], bool],
) -> list[str]:
    matches: list[str] = []
    for pattern in patterns:
        matches.extend(glob_fn(pattern))
    unique_matches = sorted({_normalize(match) for match in matches if exists_fn(match)})
    return [match for match in unique_matches if match]


def _generic_by_id_candidates(
    *,
    glob_fn: Callable[[str], list[str]],
    exists_fn: Callable[[str], bool],
) -> list[str]:
    candidates = _existing_unique(
        [f"{_BY_ID_ROOT}/*"],
        glob_fn=glob_fn,
        exists_fn=exists_fn,
    )
    filtered: list[str] = []
    for candidate in candidates:
        label = os.path.basename(candidate).upper().replace("-", " ").replace("_", " ")
        if any(hint in label for hint in _GENERIC_USB_HINTS):
            filtered.append(candidate)
    return filtered


def _build_error_message(
    *,
    requested_port: str,
    cp2102_candidates: list[str],
    generic_by_id_candidates: list[str],
    ttyusb_candidates: list[str],
    serial0_exists: bool,
) -> str:
    lines = [
        "Unable to resolve controller serial port.",
        f"requested={requested_port or '<empty>'}",
        f"env_override={_normalize(os.environ.get(_ENV_OVERRIDE, '')) or '<unset>'}",
        f"cp2102_candidates={cp2102_candidates or ['<none>']}",
        f"usb_by_id_candidates={generic_by_id_candidates or ['<none>']}",
        f"ttyusb_candidates={ttyusb_candidates or ['<none>']}",
        f"serial0_exists={serial0_exists}",
        (
            "Set serial_port explicitly or export "
            f"{_ENV_OVERRIDE}=<device> to disambiguate."
        ),
    ]
    return " ".join(lines)


def resolve_serial_port(
    requested_port: object,
    *,
    env: Mapping[str, str] | None = None,
    glob_fn: Callable[[str], list[str]] | None = None,
    exists_fn: Callable[[str], bool] | None = None,
) -> SerialPortSelection:
    normalized_requested = _normalize(requested_port)
    if not _is_auto_request(normalized_requested):
        return SerialPortSelection(port=normalized_requested, reason="explicit")

    env_mapping = os.environ if env is None else env
    glob_impl = glob.glob if glob_fn is None else glob_fn
    exists_impl = os.path.exists if exists_fn is None else exists_fn

    env_override = _normalize(env_mapping.get(_ENV_OVERRIDE, ""))
    if env_override:
        return SerialPortSelection(port=env_override, reason="env")

    cp2102_candidates = _existing_unique(
        _CP2102_PATTERNS,
        glob_fn=glob_impl,
        exists_fn=exists_impl,
    )
    if len(cp2102_candidates) == 1:
        return SerialPortSelection(port=cp2102_candidates[0], reason="auto-cp2102")
    if len(cp2102_candidates) > 1:
        raise SerialPortResolutionError(
            _build_error_message(
                requested_port=normalized_requested,
                cp2102_candidates=cp2102_candidates,
                generic_by_id_candidates=[],
                ttyusb_candidates=[],
                serial0_exists=exists_impl("/dev/serial0"),
            )
        )

    generic_by_id_candidates = _generic_by_id_candidates(
        glob_fn=glob_impl,
        exists_fn=exists_impl,
    )
    if len(generic_by_id_candidates) == 1:
        return SerialPortSelection(
            port=generic_by_id_candidates[0],
            reason="auto-usb",
        )
    if len(generic_by_id_candidates) > 1:
        raise SerialPortResolutionError(
            _build_error_message(
                requested_port=normalized_requested,
                cp2102_candidates=cp2102_candidates,
                generic_by_id_candidates=generic_by_id_candidates,
                ttyusb_candidates=[],
                serial0_exists=exists_impl("/dev/serial0"),
            )
        )

    ttyusb_candidates = _existing_unique(
        ["/dev/ttyUSB*"],
        glob_fn=glob_impl,
        exists_fn=exists_impl,
    )
    if len(ttyusb_candidates) == 1:
        return SerialPortSelection(port=ttyusb_candidates[0], reason="auto-usb")
    if len(ttyusb_candidates) > 1:
        raise SerialPortResolutionError(
            _build_error_message(
                requested_port=normalized_requested,
                cp2102_candidates=cp2102_candidates,
                generic_by_id_candidates=generic_by_id_candidates,
                ttyusb_candidates=ttyusb_candidates,
                serial0_exists=exists_impl("/dev/serial0"),
            )
        )

    if exists_impl("/dev/serial0"):
        return SerialPortSelection(port="/dev/serial0", reason="fallback-serial0")

    raise SerialPortResolutionError(
        _build_error_message(
            requested_port=normalized_requested,
            cp2102_candidates=cp2102_candidates,
            generic_by_id_candidates=generic_by_id_candidates,
            ttyusb_candidates=ttyusb_candidates,
            serial0_exists=False,
        )
    )
