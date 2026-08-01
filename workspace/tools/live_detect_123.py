#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""实机相机识别 + 求解验证（只读相机，不下发任何舵机目标、不通电磁铁）。

从 Pi 运行：
    cd /opt/epuzzle/workspace
    PYTHONPATH=/opt/epuzzle/workspace/src /opt/epuzzle/venv/bin/python \
        tools/live_detect_123.py --out out/live_123
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from integration._adapter_123 import (  # noqa: E402
    _find_homography_file,
    solve_puzzle_123,
)
from puzzle.solver import (  # noqa: E402
    PuzzleOptions,
    PuzzleSolveStatus,
    TargetRegion,
    WorkspaceState,
)
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

FLAT_TH_SWEEP = (0.82, 0.78, 0.86, 0.74, 0.90)


@dataclass(frozen=True)
class LivePiece:
    """满足 solver._PieceLike 协议的实机观察。"""

    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float
    timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]
    grip_point_table_mm: tuple[float, float] | None = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="/dev/epuzzle-camera")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--warmup", type=int, default=8)
    parser.add_argument("--out", default="out/live_123")
    parser.add_argument(
        "--homography",
        default=None,
        help="显式指定标定文件；缺省走适配器自动匹配",
    )
    return parser


def grab_frame(args: argparse.Namespace) -> np.ndarray | None:
    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        return None
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    frame = None
    try:
        for _ in range(max(1, args.warmup)):
            ok, candidate = capture.read()
            if ok and candidate is not None:
                frame = candidate
    finally:
        capture.release()
    return frame


def to_live_pieces(pieces: list[dict]) -> tuple[LivePiece, ...]:
    now_ns = time.monotonic_ns()
    result = []
    for piece in pieces:
        polygon = tuple(
            (float(v[0]), float(v[1])) for v in np.asarray(piece["vertices"])
        )
        center = piece["centroid"]
        grip = piece.get("grip_point")
        result.append(
            LivePiece(
                piece_id=f"piece_{piece['id']:02d}",
                polygon_table_mm=polygon,
                center_table_mm=(float(center[0]), float(center[1])),
                yaw_deg=float(piece.get("angle_deg", 0.0)),
                timestamp_ns=now_ns,
                equivalent_yaws_deg=(float(piece.get("angle_deg", 0.0)),),
                grip_point_table_mm=(
                    (float(grip[0]), float(grip[1])) if grip is not None else None
                ),
            )
        )
    return tuple(result)


def main() -> int:
    args = build_parser().parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    homography_path = (
        Path(args.homography) if args.homography else _find_homography_file()
    )
    if homography_path is None or not homography_path.exists():
        print("FAIL 未找到可用标定文件")
        return 2
    print(f"标定文件（自动匹配）: {homography_path}")

    frame = grab_frame(args)
    if frame is None:
        print("FAIL 相机抓帧失败")
        return 2
    print(f"抓帧: {frame.shape[1]}x{frame.shape[0]}")
    cv2.imwrite(str(out_dir / "frame.jpg"), frame)

    vision = PuzzleVision(backend=None)
    vision.load_calibration(str(homography_path))
    quad = vision.quad_from_homography()
    if quad is None:
        print("FAIL quad_from_homography 返回 None（标定与当前帧尺寸不匹配？）")
        return 2

    # flat_th 扫描：取检测数量落在 1..4 且面积最合理的一组。
    # 若检测到 5 片（常见为纸边/杂片误检），按面积降序取前 4 片。
    best: tuple[float, list[dict]] | None = None
    for flat_th in FLAT_TH_SWEEP:
        pieces = vision.detect_pieces_arbitrary(frame, quad, flat_th=flat_th)
        print(f"  flat_th={flat_th}: 检测 {len(pieces)} 片")
        if not pieces:
            continue
        # 显式按面积降序，取最大 4 片（排除纸边/杂片误检）
        pieces.sort(key=lambda p: p["area_mm2"], reverse=True)
        selected = pieces[:4]
        if 1 <= len(selected) <= 4 and (best is None or len(selected) > len(best[1])):
            best = (flat_th, selected)

    if best is None:
        print("FAIL 所有 flat_th 都未检出 1--4 片")
        return 1
    flat_th, pieces = best
    if len(pieces) < 4:
        print(f"WARNING 仅检出 {len(pieces)} 片（期望 4）")
    print(f"采用 flat_th={flat_th}，{len(pieces)} 片：")
    for piece in pieces:
        centroid = piece["centroid"]
        print(
            f"  {piece['id']:02d}: {len(piece['vertices'])}边 "
            f"{piece['area_mm2']:.0f}mm² "
            f"中心({centroid[0]:.1f}, {centroid[1]:.1f})mm "
            f"yaw={piece.get('angle_deg', 0.0):.1f}deg"
        )

    # 目标区：A4 竖放 210×297，分界线 y=148.5。
    # 碎片在上半区（y>148.5），矩形拼在下半区中心 (105, 74.25)。
    state = WorkspaceState(pieces=to_live_pieces(pieces), locked_placements=())
    target = TargetRegion(
        frame="table_mm", min_x_mm=0.0, min_y_mm=0.0, max_x_mm=210.0, max_y_mm=148.5
    )
    result = solve_puzzle_123(state, target, PuzzleOptions())
    print(f"求解: {result.status.value} error={result.error_code}")
    if result.inferred_target is not None:
        width, height = result.inferred_target
        print(f"  反推矩形: {width:.1f} x {height:.1f} mm")
    for plan in result.placements:
        print(
            f"  {plan.piece_id}: "
            f"({plan.source.center_mm[0]:.1f}, {plan.source.center_mm[1]:.1f}) "
            f"yaw {plan.source.yaw_deg:.1f} -> "
            f"({plan.target.center_mm[0]:.1f}, {plan.target.center_mm[1]:.1f}) "
            f"yaw {plan.target.yaw_deg:.1f}"
        )

    payload = {
        "homography": str(homography_path),
        "flat_th": flat_th,
        "frame_size": [int(frame.shape[1]), int(frame.shape[0])],
        "pieces": [
            {
                "id": piece["id"],
                "vertices_table_mm": np.asarray(piece["vertices"]).tolist(),
                "centroid_table_mm": list(piece["centroid"]),
                "area_mm2": piece["area_mm2"],
                "angle_deg": piece.get("angle_deg"),
                "grip_point_table_mm": (
                    list(piece["grip_point"]) if piece.get("grip_point") else None
                ),
            }
            for piece in pieces
        ],
        "solve": {
            "status": result.status.value,
            "error_code": result.error_code,
            "inferred_target_mm": (
                list(result.inferred_target) if result.inferred_target else None
            ),
            "placements": [plan.as_mapping() for plan in result.placements],
            "timing_ms": result.timing_ms,
        },
        "note": "vision+solve only; no servo goal, no magnet",
    }
    (out_dir / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"已写出 {out_dir}/frame.jpg 和 {out_dir}/result.json")
    return 0 if result.status is not PuzzleSolveStatus.NO_SOLUTION else 1


if __name__ == "__main__":
    raise SystemExit(main())
