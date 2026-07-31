#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""离线分析已校正碎片图；不连接控制或机械臂。"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

from vision import RectifiedFrame, VisionProfile, observe_pieces


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--px-per-mm", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    raw = arguments.image.read_bytes()
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("无法解码输入图像")
    frame = RectifiedFrame(
        image,
        arguments.px_per_mm,
        (210.0, 297.0),
        hashlib.sha256(raw).hexdigest(),
        arguments.image.name,
        time.monotonic_ns(),
        "offline",
        "UNKNOWN",
        None,
    )
    result = observe_pieces(
        frame, VisionProfile("white-galvanized-v1", "white_paper_on_metal", False)
    )
    payload = {
        "status": result.status,
        "error_code": result.error_code,
        "observations": [item.as_mapping() for item in result.observations],
        "processing_time_ms": result.processing_time_ms,
    }
    arguments.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
