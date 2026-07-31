#!/usr/bin/env python3
"""本轮 I/J/K/L 彩色块顶面接触标定的安全采集工具。

本工具只管理目标和采集舵机读数：不拟合、不使能、不运动，也不修改
config.py、arm_base.json 或 homography.npy。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
DEFAULT_SESSION = ROOT / "out" / "contact_calibration_IJKL.json"
SOURCE_IMAGE = "out/contact_calib_IJKL_ready_clean.jpg"
BLOCK_TOP_Z_MM = 1.0
MANUAL_XY_UNCERTAINTY_MM = 5.0
DEFAULT_TARGETS: dict[str, tuple[float, float, float]] = {
    "I": (127.64, 252.30, BLOCK_TOP_Z_MM),
    "J": (49.81, 212.15, BLOCK_TOP_Z_MM),
    "K": (146.80, 103.74, BLOCK_TOP_Z_MM),
    "L": (46.86, 101.72, BLOCK_TOP_Z_MM),
}


def empty_session() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "protocol": "magnet working face flat on colored block top",
        "block_top_z_mm": BLOCK_TOP_Z_MM,
        "manual_xy_uncertainty_mm": MANUAL_XY_UNCERTAINTY_MM,
        "orientation_observation": "working face approximately flat; not exact",
        "source_image": SOURCE_IMAGE,
        "torn_frame_coordinates_invalid": True,
        "targets": {label: list(world) for label, world in DEFAULT_TARGETS.items()},
        "points": [],
    }


def load_session(path: Path) -> dict[str, Any]:
    if not path.exists():
        return empty_session()
    try:
        text = path.read_text(encoding="utf-8")
        data = json.loads(text)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取会话文件 {path}: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("points", []), list):
        raise ValueError(f"无效会话文件: {path}")
    return data


def atomic_save(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def session_targets(data: dict[str, Any]) -> dict[str, tuple[float, float, float]]:
    raw_targets = data.get("targets", {})
    if not isinstance(raw_targets, dict):
        raise ValueError("会话 targets 必须是对象")
    targets: dict[str, tuple[float, float, float]] = {}
    for label, world in raw_targets.items():
        if not isinstance(label, str) or not isinstance(world, list) or len(world) != 3:
            raise ValueError(f"无效目标: {label!r}={world!r}")
        try:
            targets[label] = (
                float(world[0]),
                float(world[1]),
                float(world[2]),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"目标坐标不是有限数值: {label!r}={world!r}") from exc
    return targets


def numeric_float(value: Any, context: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{context} 不是数值: {value!r}") from exc
    if not np.isfinite(result):
        raise ValueError(f"{context} 不是有限数值: {value!r}")
    return result


def numeric_int(value: Any, context: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{context} 不是整数: {value!r}") from exc


def piece_measurement(piece: dict[str, Any]) -> tuple[int, float, float, float]:
    grip = piece.get("grip_point")
    if not isinstance(grip, tuple) or len(grip) != 2:
        raise ValueError(f"碎片 grip_point 无效: {grip!r}")
    return (
        numeric_int(piece.get("id"), "碎片 id"),
        numeric_float(grip[0], "碎片 x"),
        numeric_float(grip[1], "碎片 y"),
        numeric_float(piece.get("area_mm2"), "碎片面积"),
    )


def capture_stable_frame(frame_count: int = 50) -> np.ndarray:
    """从静态场景选择最接近时间中值且行撕裂指标较低的原始帧。"""
    capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
    fourcc = getattr(cv2, "VideoWriter_fourcc", cv2.VideoWriter.fourcc)(*"MJPG")
    capture.set(cv2.CAP_PROP_FOURCC, fourcc)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError("无法打开 /dev/video0")
    frames: list[np.ndarray] = []
    try:
        for index in range(frame_count + 15):
            ok, frame = capture.read()
            if ok and frame is not None and frame.shape[:2] == (720, 1280) and index >= 15:
                frames.append(frame.copy())
            time.sleep(0.05)
    finally:
        capture.release()
    if len(frames) < 15:
        raise RuntimeError(f"有效相机帧不足: {len(frames)}")

    small = np.stack(
        [cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA) for frame in frames]
    ).astype(np.float32)
    reference = np.median(small, axis=0)
    temporal_scores = np.mean(np.abs(small - reference), axis=(1, 2, 3))
    candidate_indices = np.argsort(temporal_scores)[:15]
    candidates: list[tuple[float, int]] = []
    for raw_index in candidate_indices:
        index = numeric_int(raw_index, "候选帧索引")
        gray = cv2.cvtColor(frames[index], cv2.COLOR_BGR2GRAY).astype(np.float32)
        metric_value = np.percentile(
            np.mean(np.abs(np.diff(gray, axis=0)), axis=1), 99.5
        )
        row_metric = numeric_float(metric_value, "行撕裂指标")
        candidates.append((row_metric, index))
    _, best_index = min(candidates)
    return frames[best_index]


def prepare_round(path: Path, labels: list[str]) -> None:
    """相机只读准备一轮动态目标；不初始化舵机、GPIO或运动学。"""
    if len(labels) != 4 or len(set(labels)) != 4:
        raise ValueError("prepare 当前要求四个互不重复的标签")
    if any(not label.isalpha() or len(label) != 1 for label in labels):
        raise ValueError("标签必须是单个字母")

    from vision import PuzzleVision

    frame = capture_stable_frame()
    vision = PuzzleVision()
    homography_path = ROOT / "homography.npy"
    if not vision.load_calibration(str(homography_path)):
        raise RuntimeError(f"无法加载固定单应: {homography_path}")
    quad = vision.quad_from_homography()
    if quad is None:
        raise RuntimeError("无法从固定单应反算工作区四角")
    pieces = vision.detect_pieces_arbitrary(frame=frame, quad=quad)
    valid: list[dict[str, Any]] = []
    rejected: list[tuple[int, float, float, float]] = []
    measurements: dict[int, tuple[int, float, float, float]] = {}
    for piece in pieces:
        measurement = piece_measurement(piece)
        piece_id, x, y, area = measurement
        measurements[id(piece)] = measurement
        if area >= 450.0 and 15.0 <= x <= 195.0 and 15.0 <= y <= 282.0:
            valid.append(piece)
        else:
            rejected.append((piece_id, x, y, area))
    if len(valid) != len(labels):
        raise RuntimeError(
            f"要求 {len(labels)} 块，过滤后得到 {len(valid)} 块；"
            f"原始={len(pieces)}，拒绝={rejected}。会话未写入"
        )

    ordered = sorted(
        valid,
        key=lambda piece: (
            -measurements[id(piece)][2],
            measurements[id(piece)][1],
        ),
    )
    round_name = "".join(labels)
    image_relative = f"out/contact_calib_{round_name}_ready_clean.jpg"
    annotated_relative = f"out/contact_calib_{round_name}_annotated.jpg"
    image_path = ROOT / image_relative
    annotated_path = ROOT / annotated_relative
    image_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(image_path), frame):
        raise RuntimeError(f"无法保存可信源图: {image_path}")

    if vision.homography is None:
        raise RuntimeError("单应加载成功但矩阵为空")
    homography = np.asarray(vision.homography, dtype=np.float64)
    inverse_h = np.linalg.inv(homography)
    annotated = frame.copy()
    targets: dict[str, list[float]] = {}
    colors = [(0, 255, 255), (0, 255, 0), (255, 255, 0), (255, 0, 255)]
    for label, piece, color in zip(labels, ordered, colors, strict=True):
        _, x, y, _ = measurements[id(piece)]
        targets[label] = [x, y, BLOCK_TOP_Z_MM]
        world = np.array([[[x, y]]], dtype=np.float32)
        pixel = cv2.perspectiveTransform(world, inverse_h)[0, 0]
        center = (
            numeric_int(round(numeric_float(pixel[0], "标注像素 x")), "标注像素 x"),
            numeric_int(round(numeric_float(pixel[1], "标注像素 y")), "标注像素 y"),
        )
        cv2.circle(annotated, center, 28, color, 5)
        cv2.drawMarker(annotated, center, color, cv2.MARKER_CROSS, 46, 5)
        cv2.putText(
            annotated,
            label,
            (center[0] - 18, center[1] - 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            color,
            4,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(str(annotated_path), annotated):
        raise RuntimeError(f"无法保存标注图: {annotated_path}")

    data = {
        "schema_version": 1,
        "protocol": "magnet working face flat on colored block top",
        "block_top_z_mm": BLOCK_TOP_Z_MM,
        "manual_xy_uncertainty_mm": MANUAL_XY_UNCERTAINTY_MM,
        "orientation_observation": "working face approximately flat; not exact",
        "source_image": image_relative,
        "annotated_image": annotated_relative,
        "torn_frame_coordinates_invalid": True,
        "labels": labels,
        "targets": targets,
        "rejected_contours": rejected,
        "points": [],
    }
    atomic_save(path, data)
    print(f"新一轮会话已保存: {path}")
    print(f"可信源图: {image_path}")
    print(f"标注图: {annotated_path}")
    for label, world in session_targets(data).items():
        print(f"  {label}: ({world[0]:.2f}, {world[1]:.2f}, {world[2]:.2f}) mm")
    print("prepare 仅访问相机；未初始化舵机串口或 GPIO。")


def print_status(path: Path) -> None:
    data = load_session(path)
    targets = session_targets(data)
    recorded = {str(point.get("label")): point for point in data.get("points", [])}
    print(f"会话文件: {path}")
    print(f"协议: {data.get('protocol')}")
    print(f"彩色块顶面 Z: {data.get('block_top_z_mm')} mm")
    print(
        f"人工 XY 不确定度: ±{data.get('manual_xy_uncertainty_mm', MANUAL_XY_UNCERTAINTY_MM)} mm"
    )
    print(f"可信源图: {data.get('source_image')}")
    for label, world in targets.items():
        point = recorded.get(label)
        state = "已记录" if point is not None else "待记录"
        suffix = ""
        if point is not None:
            pitch = point.get("j2_j3_j4_sum_deg")
            suffix = (
                f"，俯仰角和={pitch:.3f}°" if isinstance(pitch, (int, float)) else ""
            )
        print(
            f"  {label}: ({world[0]:.2f}, {world[1]:.2f}, {world[2]:.2f}) mm — {state}{suffix}"
        )
    print("注意: status 不访问相机、串口或 GPIO。")


def open_arm_disabled():
    # 延迟导入，确保 status 命令完全不初始化硬件。
    from arm_controller import ArmController

    arm = ArmController()
    if arm.simulation:
        arm.close()
        raise RuntimeError("机械臂串口不可用；拒绝在 simulation 模式伪造标定")
    arm.open_gripper()
    arm.disable_torque()
    time.sleep(0.3)
    return arm


def disable_hardware() -> None:
    arm = open_arm_disabled()
    try:
        print("电磁铁已断电，全部舵机已失能。请持续用手托住机械臂。")
    finally:
        arm.close()


def save_park_in_place(session_path: Path, samples_count: int) -> None:
    """记录当前 raw，预写同一目标后原位使能，并保存本轮停车证据。"""
    from config import SERVO_IDS
    from kinematics import servo_to_deg

    arm = open_arm_disabled()
    try:
        samples: list[list[int]] = []
        for _ in range(samples_count):
            raw = arm.get_current_position()
            missing = [servo_id for servo_id in SERVO_IDS if servo_id not in raw]
            if missing:
                raise RuntimeError(f"舵机读回不完整，缺少 ID: {missing}")
            samples.append([int(raw[servo_id]) for servo_id in SERVO_IDS])
            time.sleep(0.12)
        median = np.median(np.asarray(samples, dtype=float), axis=0)
        park_raw = {
            servo_id: int(round(median[index]))
            for index, servo_id in enumerate(SERVO_IDS)
        }
        angles = [
            float(servo_to_deg(park_raw[servo_id], servo_id)) for servo_id in SERVO_IDS
        ]

        # Torque is disabled: preload the exact current raw target before enabling.
        if not arm.write_positions_sync(park_raw, speed=180):
            raise RuntimeError("停车目标预写失败；舵机保持失能")
        arm.enable_torque()
        time.sleep(1.5)
        readback = arm.get_current_position()
        missing = [servo_id for servo_id in SERVO_IDS if servo_id not in readback]
        if missing:
            raise RuntimeError(f"使能后读回不完整，缺少 ID: {missing}")
        delta = {
            str(servo_id): int(readback[servo_id]) - park_raw[servo_id]
            for servo_id in SERVO_IDS
        }
        evidence = {
            "schema_version": 1,
            "source": "manual park after IJKL contact calibration",
            "servo_raw": {str(key): value for key, value in park_raw.items()},
            "joint_deg_diagnostic": angles,
            "enabled_readback_raw": {
                str(servo_id): int(readback[servo_id]) for servo_id in SERVO_IDS
            },
            "enabled_delta_raw": delta,
            "speed": 180,
        }
        output = ROOT / "out" / "contact_calibration_park.json"
        atomic_save(output, evidence)
        session = load_session(session_path)
        session.update(
            {
                "manual_xy_uncertainty_mm": MANUAL_XY_UNCERTAINTY_MM,
                "orientation_observation": "working face approximately flat; not exact",
                "block_top_z_mm": BLOCK_TOP_Z_MM,
            }
        )
        atomic_save(session_path, session)
        print(f"停车证据已保存: {output}")
        print(f"会话不确定度元数据已更新: {session_path}")
        print(f"停车 raw: {park_raw}")
        print(f"原位使能读回: {readback}")
        print(f"读回差值: {delta}")
    finally:
        arm.close()


def record_point(path: Path, label: str, samples_count: int) -> None:
    data = load_session(path)
    targets = session_targets(data)
    if label not in targets:
        raise ValueError(f"未知目标 {label!r}；只允许 {', '.join(targets)}")
    if samples_count < 3 or samples_count % 2 == 0:
        raise ValueError("samples 必须是至少 3 的奇数")

    # 延迟导入，避免 status 触碰硬件依赖。
    from config import SERVO_IDS
    from kinematics import servo_to_deg

    arm = open_arm_disabled()
    try:
        samples: list[list[int]] = []
        for _ in range(samples_count):
            raw = arm.get_current_position()
            missing = [servo_id for servo_id in SERVO_IDS if servo_id not in raw]
            if missing:
                raise RuntimeError(f"舵机读回不完整，缺少 ID: {missing}")
            samples.append([int(raw[servo_id]) for servo_id in SERVO_IDS])
            time.sleep(0.12)

        median = np.median(np.asarray(samples, dtype=float), axis=0)
        raw_median = {
            str(servo_id): int(round(median[index]))
            for index, servo_id in enumerate(SERVO_IDS)
        }
        angles = [
            float(servo_to_deg(raw_median[str(servo_id)], servo_id))
            for servo_id in SERVO_IDS
        ]
        pitch_sum = float(sum(angles[1:4]))

        data.update(
            {
                "schema_version": 1,
                "protocol": "magnet working face flat on colored block top",
                "block_top_z_mm": BLOCK_TOP_Z_MM,
                "manual_xy_uncertainty_mm": MANUAL_XY_UNCERTAINTY_MM,
                "orientation_observation": "working face approximately flat; not exact",
                "source_image": data.get("source_image", SOURCE_IMAGE),
                "torn_frame_coordinates_invalid": True,
                "targets": {
                    target_label: list(world) for target_label, world in targets.items()
                },
            }
        )
        data["points"] = [
            point for point in data.get("points", []) if point.get("label") != label
        ]
        data["points"].append(
            {
                "label": label,
                "world_mm": list(targets[label]),
                "servo_raw": raw_median,
                "joint_deg": angles,
                "j2_j3_j4_sum_deg": pitch_sum,
                "samples_raw": samples,
                "torque_enabled": False,
                "magnet_on": False,
            }
        )
        order = {target_label: index for index, target_label in enumerate(targets)}
        data["points"].sort(key=lambda point: order[point["label"]])
        atomic_save(path, data)

        print(f"{label} 已原子保存到 {path}")
        print(f"世界坐标: {targets[label]}")
        print(f"raw 中位数: {raw_median}")
        print(f"软件角度: {[round(value, 3) for value in angles]}")
        print(f"J2+J3+J4: {pitch_sum:.3f}°")
        print("舵机保持失能，电磁铁保持断电。")
    finally:
        arm.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--session",
        type=Path,
        default=DEFAULT_SESSION,
        help=f"会话 JSON（默认 {DEFAULT_SESSION}）",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="只读显示目标和记录状态，不访问硬件")
    subparsers.add_parser("disable", help="电磁铁断电并失能舵机，不运动")
    park = subparsers.add_parser("park", help="记录当前位置并预写同一raw后原位使能")
    park.add_argument("--samples", type=int, default=11)
    prepare = subparsers.add_parser("prepare", help="相机多帧筛选并创建新一轮目标")
    prepare.add_argument("labels", nargs="+", help="按世界Y从大到小、X从小到大分配标签")
    record = subparsers.add_parser("record", help="失能后读取并保存一个接触点")
    record.add_argument("label")
    record.add_argument("--samples", type=int, default=11)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    session_path = args.session.resolve()
    if args.command == "status":
        print_status(session_path)
    elif args.command == "disable":
        disable_hardware()
    elif args.command == "park":
        save_park_in_place(session_path, args.samples)
    elif args.command == "prepare":
        prepare_round(session_path, args.labels)
    elif args.command == "record":
        record_point(session_path, args.label, args.samples)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
