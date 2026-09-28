#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""通用碎片检测 CLI：A4 校正 + 材料 profile 分割，输出 PieceObservation JSON。

与 `detect_red_pieces.py` 保持相同的输出契约（`pieces.json` 顶层含 `pieces` 数组），
因此 Supervisor 的 `JsonVision` 可以直接消费。区别在于本工具走通用观察路径，
支持白色竞赛材料和开发期彩色材料，并输出实测轮廓、抓取净空与诊断。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from vision import (
    VisionProfile,
    load_a4_homography,
    observe_pieces,
    rectify_a4_frame,
)

DEFAULT_HOMOGRAPHY = Path(
    "workspace/calibration/table_homography_charuco_recalibrated_20260801.npy"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--material",
        default="white",
        help="材料 profile：white（竞赛）或 red/green/blue（开发验证）",
    )
    parser.add_argument("--homography", type=Path, default=DEFAULT_HOMOGRAPHY)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--px-per-mm", type=float, default=3.0)
    parser.add_argument(
        "--rounded-corners",
        action="store_true",
        help="启用圆角虚拟化（仅印刷牌 profile）",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("workspace/runtime/generic_piece_detection"),
    )
    return parser


def _open_camera(device: str) -> cv2.VideoCapture:
    """数字索引和 /dev 路径都接受；冻结分辨率与 MJPG 与既有链路一致。"""
    source: int | str = int(device) if device.isdigit() else device
    capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_FPS, 30)
    return capture


def main() -> int:
    args = build_parser().parse_args()
    capture = _open_camera(args.camera)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"无法打开相机: {args.camera}")
        frame_bgr = None
        # 丢弃前若干帧，等自动曝光稳定后再取用。
        for _ in range(12):
            ok, current = capture.read()
            if ok and current is not None:
                frame_bgr = current
        if frame_bgr is None:
            raise RuntimeError("相机未返回有效画面")
    finally:
        capture.release()

    homography = load_a4_homography(args.homography)
    rectified = rectify_a4_frame(
        frame_bgr,
        homography,
        px_per_mm=args.px_per_mm,
        source_image_id=f"generic-{args.material}",
    )
    # 彩色 profile 仅用于当前开发期黑底彩片验证；实拍片的一条短边约 9 mm，
    # 默认 20 mm 会把合法轮廓拒绝为 VISION_MERGED_BLOBS。正式白色竞赛材料
    # 继续使用题面/工程默认 20 mm，不因开发 profile 放宽。
    min_edge_length_mm = (
        10.0 if args.material.strip().lower() in {"red", "green", "blue"} else 20.0
    )
    profile = VisionProfile(
        profile_id=f"{args.material}-{args.px_per_mm:g}ppmm-v2",
        material=args.material,
        enable_rounded_corner_recovery=args.rounded_corners,
        min_edge_length_mm=min_edge_length_mm,
    )
    result = observe_pieces(rectified, profile)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "raw.jpg"), frame_bgr)
    cv2.imwrite(str(output_dir / "rectified.jpg"), rectified.image)

    payload = {
        "schema_version": 1,
        "frame_size": [1280, 720],
        "material": args.material,
        "px_per_mm": args.px_per_mm,
        "homography_file": str(args.homography),
        "homography_sha256": rectified.homography_sha256,
        "source_image_sha256": rectified.source_image_sha256,
        "status": result.status,
        "error_code": result.error_code,
        "processing_time_ms": result.processing_time_ms,
        "pieces": [observation.as_mapping() for observation in result.observations],
    }
    (output_dir / "pieces.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": result.status,
                "error_code": result.error_code,
                "count": len(result.observations),
                "processing_time_ms": round(result.processing_time_ms, 1),
            },
            ensure_ascii=False,
        )
    )
    # 退出码 2 表示"运行正常但无可用碎片"，与 detect_red_pieces.py 一致。
    return 0 if result.observations else 2


if __name__ == "__main__":
    raise SystemExit(main())
