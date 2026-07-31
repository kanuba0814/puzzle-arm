#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""从固定 1280×720 UVC 相机抓帧并输出红色碎片观察。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from vision import detect_red_pieces, load_homography


def camera_source(value: str) -> int | str:
    """Keep numeric commissioning indices compatible; prefer stable device paths."""
    if not value.isdecimal():
        return value
    try:
        return int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("相机索引不是有效整数") from error


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--homography",
        type=Path,
        default=Path("workspace/calibration/table_homography_gui_20260731.npy"),
    )
    parser.add_argument(
        "--camera",
        type=camera_source,
        default="/dev/epuzzle-camera",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("workspace/runtime/red_piece_detection"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 30)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"无法打开相机 {args.camera}")
        frame = None
        for _ in range(12):
            ok, current = capture.read()
            if ok and current is not None:
                frame = current
        if frame is None:
            raise RuntimeError("相机未返回有效画面")
    finally:
        capture.release()

    observations, mask, annotated = detect_red_pieces(
        frame, load_homography(args.homography)
    )
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "frame.jpg"), frame)
    cv2.imwrite(str(output_dir / "mask.png"), mask)
    cv2.imwrite(str(output_dir / "annotated.jpg"), annotated)
    payload = {
        "schema_version": 1,
        "frame_size": [1280, 720],
        "homography_file": str(args.homography),
        "pieces": [observation.as_mapping() for observation in observations],
    }
    (output_dir / "pieces.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"count": len(observations), "pieces": payload["pieces"]}))
    return 0 if observations else 2


if __name__ == "__main__":
    raise SystemExit(main())
