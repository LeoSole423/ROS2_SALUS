from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from jetson_power_lib import (
    AGGRESSIVE_PROFILE,
    FAST_PREFIX,
    JsonlLoadResult,
    SegmentedJsonlWriter,
    Thresholds,
    classify_boot,
    discover_ina3221_channels,
    load_boot_data,
    load_jsonl,
    segment_file_path,
)


def _fast_sample(
    monotonic_s: float,
    vdd_in_mv: float,
    vdd_in_ma: float,
    dv_dt_mv_per_s: float = 0.0,
) -> dict[str, float]:
    return {
        "monotonic_s": monotonic_s,
        "vdd_in_mv": vdd_in_mv,
        "vdd_in_ma": vdd_in_ma,
        "dv_dt_mv_per_s": dv_dt_mv_per_s,
    }


def _event(event: str, monotonic_s: float, **extra: object) -> dict[str, object]:
    return {"event": event, "monotonic_s": monotonic_s, **extra}


class JetsonPowerLibTests(unittest.TestCase):
    def test_discover_ina3221_channels_finds_vdd_in(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            tmp_path = Path(temp_dir)
            hwmon = tmp_path / "hwmon0"
            hwmon.mkdir(parents=True)
            (hwmon / "name").write_text("ina3221", encoding="utf-8")
            (hwmon / "in1_label").write_text("VDD_IN", encoding="utf-8")
            (hwmon / "in1_input").write_text("5008", encoding="utf-8")
            (hwmon / "curr1_input").write_text("1504", encoding="utf-8")

            channels = discover_ina3221_channels(hwmon_root=tmp_path)

            self.assertIn("VDD_IN", channels)
            self.assertEqual(channels["VDD_IN"].voltage_path, hwmon / "in1_input")
            self.assertEqual(channels["VDD_IN"].current_path, hwmon / "curr1_input")

    def test_load_jsonl_tolerates_nul_tail(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "fast-test.jsonl"
            with path.open("wb") as handle:
                handle.write(json.dumps({"seq": 1}).encode("utf-8") + b"\n")
                handle.write(b"\x00\x00\x00\x00")

            result = load_jsonl(path)

            self.assertIsInstance(result, JsonlLoadResult)
            self.assertEqual(len(result.records), 1)
            self.assertTrue(result.corrupt_tail_detected)

    def test_segment_rotation_preserves_temporal_order(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            log_dir = Path(temp_dir)
            writer = SegmentedJsonlWriter(
                log_dir=log_dir,
                prefix=FAST_PREFIX,
                boot_id="boot123",
                segment_seconds=1.0,
                sync_interval_ms=0.0,
            )
            writer.write({"seq": 0, "monotonic_s": 0.0}, monotonic_s=0.0)
            writer.write({"seq": 1, "monotonic_s": 0.4}, monotonic_s=0.4)
            writer.write({"seq": 2, "monotonic_s": 1.2}, monotonic_s=1.2)
            writer.close()

            data = load_boot_data(log_dir, "boot123")
            seqs = [int(sample["seq"]) for sample in data.fast_samples]

            self.assertEqual(
                segment_file_path(log_dir, FAST_PREFIX, "boot123", 1).exists(),
                True,
            )
            self.assertEqual(
                segment_file_path(log_dir, FAST_PREFIX, "boot123", 2).exists(),
                True,
            )
            self.assertEqual(seqs, [0, 1, 2])

    def test_classify_boot_internal_rail_drop_suspected(self) -> None:
        summary = classify_boot(
            load_boot_data_from_memory(
                fast_samples=[
                    _fast_sample(8.0, 5000.0, 1200.0),
                    _fast_sample(9.8, 4620.0, 2000.0, dv_dt_mv_per_s=-950.0),
                ],
                events=[_event("vdd_in_critical", 9.8)],
            ),
            profile=AGGRESSIVE_PROFILE,
        )
        self.assertEqual(summary["classification"], "internal_rail_drop_suspected")

    def test_classify_boot_abrupt_reset_internal_rail_stable(self) -> None:
        summary = classify_boot(
            load_boot_data_from_memory(
                fast_samples=[
                    _fast_sample(8.0, 5008.0, 1200.0),
                    _fast_sample(9.8, 5008.0, 1400.0, dv_dt_mv_per_s=0.0),
                ],
                events=[_event("monitor_started", 0.0)],
            ),
            profile=AGGRESSIVE_PROFILE,
        )
        self.assertEqual(summary["classification"], "abrupt_reset_internal_rail_stable")

    def test_classify_boot_monitor_gap_unknown(self) -> None:
        summary = classify_boot(
            load_boot_data_from_memory(
                fast_samples=[_fast_sample(0.1, 5008.0, 1200.0)],
                events=[],
            ),
            profile=AGGRESSIVE_PROFILE,
        )
        self.assertEqual(summary["classification"], "monitor_gap_unknown")


def load_boot_data_from_memory(
    *,
    fast_samples: list[dict[str, object]],
    events: list[dict[str, object]],
) -> object:
    class _BootData:
        def __init__(self) -> None:
            self.fast_samples = tuple(fast_samples)
            self.context_samples = ()
            self.events = tuple(events)
            self.corrupt_tail_detected = False
            self.corrupt_paths = ()

    return _BootData()


if __name__ == "__main__":
    unittest.main()
