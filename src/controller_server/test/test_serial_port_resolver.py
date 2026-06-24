from controller_server.serial_port_resolver import (
    SerialPortResolutionError,
    resolve_serial_port,
)


def _make_glob_fn(mapping):
    def _glob(pattern):
        return list(mapping.get(pattern, []))

    return _glob


def _make_exists_fn(existing_paths):
    existing = set(existing_paths)

    def _exists(path):
        return path in existing

    return _exists


def test_resolve_serial_port_uses_explicit_path() -> None:
    selection = resolve_serial_port(
        "/dev/ttyUSB9",
        env={},
        glob_fn=_make_glob_fn({}),
        exists_fn=_make_exists_fn(set()),
    )

    assert selection.port == "/dev/ttyUSB9"
    assert selection.reason == "explicit"


def test_resolve_serial_port_uses_env_override() -> None:
    selection = resolve_serial_port(
        "auto",
        env={"SALUS_CONTROLLER_SERIAL_PORT": "/dev/serial/by-id/usb-custom"},
        glob_fn=_make_glob_fn({}),
        exists_fn=_make_exists_fn(set()),
    )

    assert selection.port == "/dev/serial/by-id/usb-custom"
    assert selection.reason == "env"


def test_resolve_serial_port_prefers_cp2102_by_id() -> None:
    cp2102 = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller"
    selection = resolve_serial_port(
        "auto",
        env={},
        glob_fn=_make_glob_fn(
            {
                "/dev/serial/by-id/*Silicon_Labs_CP2102*": [cp2102],
                "/dev/serial/by-id/*CP2102*": [cp2102],
                "/dev/serial/by-id/*": [cp2102],
            }
        ),
        exists_fn=_make_exists_fn({cp2102}),
    )

    assert selection.port == cp2102
    assert selection.reason == "auto-cp2102"


def test_resolve_serial_port_uses_single_ttyusb_candidate() -> None:
    ttyusb = "/dev/ttyUSB0"
    selection = resolve_serial_port(
        "auto",
        env={},
        glob_fn=_make_glob_fn(
            {
                "/dev/serial/by-id/*Silicon_Labs_CP2102*": [],
                "/dev/serial/by-id/*CP2102*": [],
                "/dev/serial/by-id/*": [],
                "/dev/ttyUSB*": [ttyusb],
            }
        ),
        exists_fn=_make_exists_fn({ttyusb}),
    )

    assert selection.port == ttyusb
    assert selection.reason == "auto-usb"


def test_resolve_serial_port_falls_back_to_serial0() -> None:
    selection = resolve_serial_port(
        "auto",
        env={},
        glob_fn=_make_glob_fn(
            {
                "/dev/serial/by-id/*Silicon_Labs_CP2102*": [],
                "/dev/serial/by-id/*CP2102*": [],
                "/dev/serial/by-id/*": [],
                "/dev/ttyUSB*": [],
            }
        ),
        exists_fn=_make_exists_fn({"/dev/serial0"}),
    )

    assert selection.port == "/dev/serial0"
    assert selection.reason == "fallback-serial0"


def test_resolve_serial_port_fails_on_ambiguous_usb_candidates() -> None:
    usb0 = "/dev/ttyUSB0"
    usb1 = "/dev/ttyUSB1"

    try:
        resolve_serial_port(
            "auto",
            env={},
            glob_fn=_make_glob_fn(
                {
                    "/dev/serial/by-id/*Silicon_Labs_CP2102*": [],
                    "/dev/serial/by-id/*CP2102*": [],
                    "/dev/serial/by-id/*": [],
                    "/dev/ttyUSB*": [usb0, usb1],
                }
            ),
            exists_fn=_make_exists_fn({usb0, usb1}),
        )
    except SerialPortResolutionError as exc:
        message = str(exc)
        assert "ttyusb_candidates=" in message
        assert usb0 in message
        assert usb1 in message
    else:
        raise AssertionError("Expected SerialPortResolutionError for ambiguous ttyUSB")
