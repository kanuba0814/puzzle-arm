#!/usr/bin/env python3
"""用固定 ChArUco 板重新计算工作区 image_px -> table_mm 单应矩阵。

仅访问固定相机，不连接机械臂、不控制电磁铁。标定板必须平放在固定 A4 工作区。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np

A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0
BOARD_SQUARES = (8, 11)
SQUARE_MM = 21.12
MARKER_MM = SQUARE_MM * 16.0 / 22.0
PAPER_MARGIN_LEFT_MM = 19.0
PAPER_MARGIN_TOP_MM = 18.3
MIN_CORNERS = 50


def detect_board(frame: np.ndarray, board: cv2.aruco.CharucoBoard):
    dictionary = board.getDictionary()
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(board, detectorParams=params)
    return detector.detectBoard(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[:2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/dev/epuzzle-camera")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.samples < 1:
        raise SystemExit("--samples 必须大于零")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(
        BOARD_SQUARES, SQUARE_MM, MARKER_MM, dictionary
    )
    board_points = np.asarray(board.getChessboardCorners(), dtype=np.float64)[:, :2]

    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"无法打开相机: {args.camera}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    observations: list[tuple[np.ndarray, np.ndarray]] = []
    frame_size: tuple[int, int] | None = None
    try:
        for _ in range(8):
            capture.grab()
        for _ in range(args.samples * 4):
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            frame_size = (int(frame.shape[1]), int(frame.shape[0]))
            corners, ids = detect_board(frame, board)
            if ids is None or corners is None or len(ids) < MIN_CORNERS:
                continue
            observations.append((corners.reshape(-1, 2).astype(np.float64), ids.ravel()))
            if len(observations) >= args.samples:
                break
    finally:
        capture.release()

    if not observations:
        raise SystemExit(f"ChArUco 角点不足：至少需要 {MIN_CORNERS} 个")

    by_id: dict[int, list[np.ndarray]] = {}
    for corners, ids in observations:
        for corner, corner_id in zip(corners, ids, strict=True):
            by_id.setdefault(int(corner_id), []).append(corner)
    common_ids = sorted(corner_id for corner_id, values in by_id.items() if len(values) >= 1)
    if len(common_ids) < MIN_CORNERS:
        raise SystemExit(f"有效公共角点不足：{len(common_ids)} < {MIN_CORNERS}")

    image_points = np.asarray(
        [np.mean(by_id[corner_id], axis=0) for corner_id in common_ids], dtype=np.float64
    )
    object_points = board_points[common_ids]
    world_points = np.column_stack(
        (
            PAPER_MARGIN_LEFT_MM + object_points[:, 0],
            (A4_HEIGHT_MM - PAPER_MARGIN_TOP_MM) - object_points[:, 1],
        )
    )
    homography, _ = cv2.findHomography(image_points, world_points, method=0)
    if homography is None:
        raise SystemExit("单应矩阵计算失败")

    projected = cv2.perspectiveTransform(
        image_points.reshape(-1, 1, 2).astype(np.float32), homography.astype(np.float32)
    ).reshape(-1, 2)
    residual_mm = np.linalg.norm(projected - world_points, axis=1)
    rms_mm = float(np.sqrt(np.mean(residual_mm**2)))
    max_mm = float(np.max(residual_mm))
    if rms_mm > 1.0 or max_mm > 3.0:
        raise SystemExit(f"标定残差超限：RMS={rms_mm:.3f}mm max={max_mm:.3f}mm")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    np.save(args.output, homography.astype(np.float64))
    summary = {
        "schema_version": 1,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "mode": "workspace-charuco-single-plane",
        "camera": args.camera,
        "image_size": list(frame_size or (1280, 720)),
        "samples": len(observations),
        "corners": len(common_ids),
        "homography_file": str(args.output),
        "homography_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "world_frame": "table_mm: A4 左下角原点，x 右，y 上",
        "homography_residual_mm": {"rms": rms_mm, "max": max_mm},
    }
    args.summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
