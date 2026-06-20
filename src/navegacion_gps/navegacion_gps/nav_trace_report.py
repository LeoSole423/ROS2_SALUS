import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

from navegacion_gps.nav_trace_recorder import render_trace_summary


def main(args: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Regenerate a SALUS navigation trace report")
    parser.add_argument("trace_dir", help="Directory containing metadata.json and timeline.jsonl")
    parsed = parser.parse_args(args)
    trace_dir = Path(parsed.trace_dir)
    metadata = json.loads((trace_dir / "metadata.json").read_text(encoding="utf-8"))
    records = [
        json.loads(line)
        for line in (trace_dir / "timeline.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    output = trace_dir / "summary.md"
    output.write_text(render_trace_summary(metadata, records), encoding="utf-8")
    print(output)

