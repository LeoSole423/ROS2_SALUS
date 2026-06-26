import struct

import pytest

from controller_server.rpy_esp32_comms.controller import CommandState
from controller_server.rpy_esp32_comms.protocol import (
    EspFrameParser,
    crc8_maxim,
    decode_battery_frame,
    decode_esp_frame,
    encode_pi_frame,
)
from controller_server.rpy_esp32_comms.telemetry import ControlSource


def make_esp_frame(status: int, speed_raw: int, steer_raw: int, brake: int) -> bytes:
    frame = bytearray(8)
    frame[0] = 0x55
    frame[1] = status & 0xFF
    frame[2] = speed_raw & 0xFF
    frame[3] = (speed_raw >> 8) & 0xFF
    frame[4:6] = struct.pack("<h", steer_raw)
    frame[6] = brake & 0xFF
    frame[7] = crc8_maxim(frame[:-1])
    return bytes(frame)


def make_battery_frame(
    status: int, battery_centi_volts: int, adc_mv: int, sample_age_ds: int
) -> bytes:
    frame = bytearray(8)
    frame[0] = 0x56
    frame[1] = status & 0xFF
    frame[2] = battery_centi_volts & 0xFF
    frame[3] = (battery_centi_volts >> 8) & 0xFF
    frame[4] = adc_mv & 0xFF
    frame[5] = (adc_mv >> 8) & 0xFF
    frame[6] = sample_age_ds & 0xFF
    frame[7] = crc8_maxim(frame[:-1])
    return bytes(frame)


def test_encode_pi_frame_fields_and_crc() -> None:
    state = CommandState(
        drive_enabled=True,
        estop=False,
        steer_pct=-25,
        speed_mps=1.23,
        brake_pct=55,
    )
    frame = encode_pi_frame(state)

    assert len(frame) == 7
    assert frame[0] == 0xAA
    assert frame[1] == 0x22  # version=2, DRIVE_EN=1
    assert frame[2] == 0xE7  # -25 in int8
    assert frame[3] == 0x7B
    assert frame[4] == 0x00
    assert frame[5] == 55
    assert frame[6] == crc8_maxim(frame[:-1])


def test_decode_esp_frame_ok() -> None:
    status = 0x59  # READY + PI_FRESH + SOURCE=PI + OVERSPEED
    frame = make_esp_frame(status=status, speed_raw=250, steer_raw=-1234, brake=42)
    telemetry = decode_esp_frame(frame, rx_monotonic_s=123.0)

    assert telemetry.ready is True
    assert telemetry.pi_fresh is True
    assert telemetry.control_source == ControlSource.PI
    assert telemetry.overspeed_active is True
    assert telemetry.speed_mps == 2.5
    assert telemetry.steer_deg == -12.34
    assert telemetry.brake_applied_pct == 42
    assert telemetry.rx_monotonic_s == 123.0


def test_decode_esp_frame_sentinels() -> None:
    frame = make_esp_frame(status=0x01, speed_raw=0xFFFF, steer_raw=-32768, brake=0)
    telemetry = decode_esp_frame(frame)

    assert telemetry.speed_mps is None
    assert telemetry.steer_deg is None


def test_decode_esp_frame_bad_crc_raises() -> None:
    frame = bytearray(make_esp_frame(status=0x01, speed_raw=10, steer_raw=20, brake=0))
    frame[-1] ^= 0xFF
    with pytest.raises(ValueError, match="CRC"):
        decode_esp_frame(bytes(frame))


def test_decode_battery_frame_ok() -> None:
    frame = make_battery_frame(status=0x0B, battery_centi_volts=6870, adc_mv=3075, sample_age_ds=12)
    telemetry = decode_battery_frame(frame, rx_monotonic_s=321.0)

    assert telemetry.ready is True
    assert telemetry.fresh is True
    assert telemetry.calibrated is True
    assert telemetry.suspect is False
    assert telemetry.battery_voltage_v == pytest.approx(68.70)
    assert telemetry.adc_pin_voltage_v == pytest.approx(3.075)
    assert telemetry.sample_age_s == pytest.approx(1.2)
    assert telemetry.rx_monotonic_s == pytest.approx(321.0)


def test_decode_battery_frame_bad_crc_raises() -> None:
    frame = bytearray(make_battery_frame(status=0x01, battery_centi_volts=5500, adc_mv=2500, sample_age_ds=5))
    frame[-1] ^= 0xFF
    with pytest.raises(ValueError, match="CRC"):
        decode_battery_frame(bytes(frame))


def test_parser_resync_after_corrupt_bytes() -> None:
    parser = EspFrameParser()

    frame1 = make_esp_frame(status=0x01, speed_raw=100, steer_raw=5, brake=1)
    frame2 = make_esp_frame(status=0x11, speed_raw=200, steer_raw=-5, brake=2)

    stream = b"\x00\x99" + frame1[:4] + b"\x55" + frame2[1:]
    out = parser.feed(stream)

    assert len(out) == 1
    assert out[0] == frame2
    assert parser.dropped_partial_frames >= 1
    assert parser.crc_error_frames >= 1


def test_parser_does_not_drop_on_payload_0x55() -> None:
    parser = EspFrameParser()

    # status=0x55 simula el byte de header dentro del payload.
    frame = make_esp_frame(status=0x55, speed_raw=333, steer_raw=444, brake=12)
    out = parser.feed(frame)

    assert out == [frame]
    assert parser.dropped_partial_frames == 0
    assert parser.crc_error_frames == 0


def test_parser_accepts_mixed_control_and_battery_frames() -> None:
    parser = EspFrameParser()

    control_frame = make_esp_frame(status=0x09, speed_raw=123, steer_raw=456, brake=7)
    battery_frame = make_battery_frame(
        status=0x0B, battery_centi_volts=6870, adc_mv=3075, sample_age_ds=8
    )

    out = parser.feed(control_frame + battery_frame)

    assert out == [control_frame, battery_frame]
    assert parser.dropped_partial_frames == 0
    assert parser.crc_error_frames == 0


def test_encode_pi_frame_negative_speed_sets_rev_req() -> None:
    state = CommandState(
        drive_enabled=True,
        estop=False,
        steer_pct=10,
        speed_mps=-1.23,
        brake_pct=5,
    )
    frame = encode_pi_frame(state)

    assert len(frame) == 7
    assert frame[0] == 0xAA
    assert frame[1] == 0x26  # version=2, DRIVE_EN=1, REV_REQ=1
    assert frame[2] == 10
    assert frame[3] == 0x7B
    assert frame[4] == 0x00
    assert frame[5] == 5
    assert frame[6] == crc8_maxim(frame[:-1])


def test_encode_pi_frame_zero_speed_clears_rev_req() -> None:
    state = CommandState(
        drive_enabled=True,
        estop=False,
        steer_pct=0,
        speed_mps=0.0,
        brake_pct=0,
    )
    frame = encode_pi_frame(state)

    assert len(frame) == 7
    assert frame[0] == 0xAA
    assert frame[1] == 0x22  # version=2, DRIVE_EN=1
    assert frame[3] == 0x00
    assert frame[4] == 0x00
    assert frame[6] == crc8_maxim(frame[:-1])
