#!/usr/bin/env python3
"""对离线通用视觉 replay corpus 记录确定性结果哈希和延迟。"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("corpus", type=Path)
    parser.add_argument("--px-per-mm", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    records: list[dict[str, object]] = []
    for image_path in sorted(arguments.corpus.glob("*")):
        if not image_path.is_file():
            continue
        output = arguments.output.parent / f"{image_path.stem}.json"
        started = time.monotonic_ns()
        completed = subprocess.run(
            [
                sys.executable,
                "workspace/tools/analyze_puzzle_frame.py",
                str(image_path),
                "--px-per-mm",
                str(arguments.px_per_mm),
                "--output",
                str(output),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        payload = output.read_bytes() if output.exists() else b""
        records.append(
            {
                "path": str(image_path),
                "returncode": completed.returncode,
                "latency_ms": (time.monotonic_ns() - started) / 1_000_000,
                "result_sha256": hashlib.sha256(payload).hexdigest(),
            }
        )
    arguments.output.write_text(
        json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
