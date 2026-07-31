#!/usr/bin/env python3
"""只用 fit 触点拟合单臂平面运动学，并原样评估独立 holdout。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import numbers
from pathlib import Path
from typing import Any, TypedDict

import numpy as np
from scipy.optimize import least_squares

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
JOINT_FEATURES = tuple(f"{name}.pos" for name in JOINT_NAMES)
JOINT_DIRECTIONS = (-1, -1, -1, -1, -1)
HOLDOUT_XY_RMS_LIMIT_MM = 5.0
HOLDOUT_XY_MAX_LIMIT_MM = 8.0
HOLDOUT_Z_MAX_ABS_LIMIT_MM = 3.0
# 同机机械端点标定内缩后的竞赛实用范围；J5 额外限制线缆扭转。
PRACTICAL_SOFT_LIMITS_DEG = (
    (-110.0, 110.0),
    (-100.0, 100.0),
    (-90.0, 90.0),
    (-95.0, 95.0),
    (-145.0, -20.0),
)


class ContactPoint(TypedDict):
    label: str
    joints_deg: np.ndarray
    target_mm: np.ndarray


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="拟合单臂五轴运动学")
    parser.add_argument("contacts", type=Path)
    parser.add_argument("tcp_measurement", type=Path)
    parser.add_argument("output_calibration", type=Path)
    parser.add_argument("output_transform", type=Path)
    parser.add_argument("output_report", type=Path)
    parser.add_argument(
        "--status",
        choices=("provisional", "accepted", "rejected"),
        default="provisional",
    )
    return parser


def finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise RuntimeError(f"{field} 必须是实数")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"{field} 无法转换为有限数值") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"{field} 必须是有限数值")
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source_file:
            for block in iter(lambda: source_file.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise RuntimeError(f"无法读取 {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"{path} 不是有效 JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"{path} 顶层必须是对象")
    return payload


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    except OSError as exc:
        raise RuntimeError(f"无法写入 {path}: {exc}") from exc


def load_points(
    contacts: dict[str, Any],
) -> tuple[list[ContactPoint], list[ContactPoint]]:
    if contacts.get("capture_status") != "complete":
        raise RuntimeError("触点证据 capture_status 必须为 complete")
    if tuple(contacts.get("joint_features", ())) != JOINT_FEATURES:
        raise RuntimeError("触点证据不是固定显式五轴顺序")
    raw_points = contacts.get("points")
    if not isinstance(raw_points, list):
        raise RuntimeError("触点证据 points 必须是数组")

    result: dict[str, list[ContactPoint]] = {"fit": [], "holdout": []}
    labels: set[str] = set()
    for index, raw_point in enumerate(raw_points):
        if not isinstance(raw_point, dict):
            raise RuntimeError(f"points[{index}] 必须是对象")
        label = raw_point.get("label")
        role = raw_point.get("role")
        target = raw_point.get("target")
        joint_deg = raw_point.get("joint_deg")
        if not isinstance(label, str) or not label or label in labels:
            raise RuntimeError(f"points[{index}].label 必须唯一")
        if role not in result:
            raise RuntimeError(f"points[{index}].role 必须是 fit/holdout")
        if (
            not isinstance(target, dict)
            or target.get("frame") != "calibration_sheet_mm"
        ):
            raise RuntimeError(f"points[{index}].target frame 错误")
        position = target.get("position_mm")
        if not isinstance(position, list) or len(position) != 3:
            raise RuntimeError(f"points[{index}] target position 必须有三个值")
        if not isinstance(joint_deg, dict) or set(joint_deg) != set(JOINT_NAMES):
            raise RuntimeError(f"points[{index}] joint_deg 必须精确五轴")
        try:
            position_array = np.asarray(position, dtype=float)
            joint_array = np.asarray(
                [joint_deg[name] for name in JOINT_NAMES], dtype=float
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(f"points[{index}] 含无效数值") from exc
        if not np.all(np.isfinite(position_array)) or not np.all(
            np.isfinite(joint_array)
        ):
            raise RuntimeError(f"points[{index}] 含非有限数值")
        labels.add(label)
        result[str(role)].append(
            {
                "label": label,
                "joints_deg": joint_array,
                "target_mm": position_array,
            }
        )
    if len(result["fit"]) < 3 or len(result["holdout"]) < 4:
        raise RuntimeError("至少需要 3 个 fit 和 4 个 holdout")
    return result["fit"], result["holdout"]


def geometry_from_evidence(tcp_measurement: dict[str, Any]) -> dict[str, float]:
    extension = tcp_measurement.get("magnet_extension")
    if not isinstance(extension, dict):
        raise RuntimeError("TCP 证据缺少 magnet_extension")
    readings = extension.get("readings")
    if not isinstance(readings, list) or not readings:
        raise RuntimeError("TCP 证据缺少轴向读数")
    extension_mm = finite_float(readings[0], "TCP 轴向读数")
    if extension_mm <= 0.0:
        raise RuntimeError("TCP 轴向读数必须为正有限值")
    return {
        "shoulder_height": 116.6,
        "shoulder_radial_offset": 0.0,
        "shoulder_lateral_offset": 0.0,
        "upper_arm": 116.0,
        "forearm": 135.0,
        "tool_forward": 61.1 + extension_mm,
        "tool_normal": 18.1,
    }


def forward_robot_mm(
    joints_deg: np.ndarray,
    zero_offsets_deg: np.ndarray,
    geometry: dict[str, float],
) -> np.ndarray:
    physical = np.radians(
        joints_deg[:, :4] * np.asarray(JOINT_DIRECTIONS[:4]) + zero_offsets_deg[:4]
    )
    pan, shoulder, elbow, wrist = physical.T
    forearm_pitch = shoulder + elbow
    tool_pitch = forearm_pitch + wrist
    radial = (
        geometry["shoulder_radial_offset"]
        + geometry["upper_arm"] * np.cos(shoulder)
        + geometry["forearm"] * np.cos(forearm_pitch)
        + geometry["tool_forward"] * np.cos(tool_pitch)
        - geometry["tool_normal"] * np.sin(tool_pitch)
    )
    height = (
        geometry["shoulder_height"]
        + geometry["upper_arm"] * np.sin(shoulder)
        + geometry["forearm"] * np.sin(forearm_pitch)
        + geometry["tool_forward"] * np.sin(tool_pitch)
        + geometry["tool_normal"] * np.cos(tool_pitch)
    )
    lateral = geometry["shoulder_lateral_offset"]
    x = np.cos(pan) * radial - np.sin(pan) * lateral
    y = np.sin(pan) * radial + np.cos(pan) * lateral
    return np.column_stack((x, y, height))


def fit_sheet_from_robot(
    robot_points: np.ndarray, sheet_points: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    robot_xy = robot_points[:, :2]
    sheet_xy = sheet_points[:, :2]
    robot_center = robot_xy.mean(axis=0)
    sheet_center = sheet_xy.mean(axis=0)
    cross = (robot_xy - robot_center).T @ (sheet_xy - sheet_center)
    left, _, right_t = np.linalg.svd(cross)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_t[-1] *= -1.0
        rotation = right_t.T @ left.T
    translation_xy = sheet_center - rotation @ robot_center
    translation_z = finite_float(
        np.mean(sheet_points[:, 2] - robot_points[:, 2]), "Z 平移"
    )
    aligned = np.column_stack(
        (
            (rotation @ robot_xy.T).T + translation_xy,
            robot_points[:, 2] + translation_z,
        )
    )
    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return aligned, rotation, translation_xy, translation_z, yaw_deg


def metrics(errors: np.ndarray) -> dict[str, float]:
    xy = np.linalg.norm(errors[:, :2], axis=1)
    xy_mean_square = finite_float(np.mean(xy**2), "XY 均方")
    z_mean_square = finite_float(np.mean(errors[:, 2] ** 2), "Z 均方")
    return {
        "xy_rms_mm": math.sqrt(xy_mean_square),
        "xy_max_mm": finite_float(np.max(xy), "XY 最大误差"),
        "z_rms_mm": math.sqrt(z_mean_square),
        "z_max_abs_mm": finite_float(np.max(np.abs(errors[:, 2])), "Z 最大绝对误差"),
    }


def fit_model(
    fit_points: list[ContactPoint],
    geometry: dict[str, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    joints = np.asarray([point["joints_deg"] for point in fit_points], dtype=float)
    targets = np.asarray([point["target_mm"] for point in fit_points], dtype=float)

    def residual(candidate: np.ndarray) -> np.ndarray:
        zero_offsets = np.asarray((0.0, candidate[0], candidate[1], candidate[2], 0.0))
        predicted = forward_robot_mm(joints, zero_offsets, geometry)
        aligned, *_ = fit_sheet_from_robot(predicted, targets)
        return (aligned - targets).reshape(-1)

    starts = (
        (0.0, 0.0, 0.0),
        (60.0, -60.0, 0.0),
        (70.0, -55.0, -5.0),
        (90.0, -90.0, 0.0),
        (-60.0, 60.0, 0.0),
    )
    best = None
    for start in starts:
        solution = least_squares(
            residual,
            np.asarray(start, dtype=float),
            bounds=(np.full(3, -180.0), np.full(3, 180.0)),
            max_nfev=5000,
        )
        mean_square = finite_float(np.mean(residual(solution.x) ** 2), "优化残差均方")
        score = math.sqrt(mean_square)
        if best is None or score < best[0]:
            best = (score, solution.x)
    if best is None:
        raise RuntimeError("运动学拟合没有生成候选解")
    score, fitted = best
    zero_offsets = np.asarray((0.0, fitted[0], fitted[1], fitted[2], 0.0))
    predicted = forward_robot_mm(joints, zero_offsets, geometry)
    _, rotation, translation_xy, translation_z, yaw_deg = fit_sheet_from_robot(
        predicted, targets
    )
    return zero_offsets, rotation, translation_xy, translation_z, score


def transform_to_sheet(
    robot_points: np.ndarray,
    rotation: np.ndarray,
    translation_xy: np.ndarray,
    translation_z: float,
) -> np.ndarray:
    return np.column_stack(
        (
            (rotation @ robot_points[:, :2].T).T + translation_xy,
            robot_points[:, 2] + translation_z,
        )
    )


def rounded_list(values: np.ndarray, digits: int = 6) -> list[float]:
    return [
        round(finite_float(value, "输出数组元素"), digits) for value in values.tolist()
    ]


def main() -> int:
    args = build_parser().parse_args()
    contacts_path = args.contacts.expanduser().resolve(strict=True)
    tcp_path = args.tcp_measurement.expanduser().resolve(strict=True)
    contacts = load_json(contacts_path)
    tcp_measurement = load_json(tcp_path)
    fit_points, holdout_points = load_points(contacts)
    geometry = geometry_from_evidence(tcp_measurement)

    zero_offsets, rotation, translation_xy, translation_z, fit_score = fit_model(
        fit_points, geometry
    )
    fit_joints = np.asarray([point["joints_deg"] for point in fit_points], dtype=float)
    fit_targets = np.asarray([point["target_mm"] for point in fit_points], dtype=float)
    holdout_joints = np.asarray(
        [point["joints_deg"] for point in holdout_points], dtype=float
    )
    holdout_targets = np.asarray(
        [point["target_mm"] for point in holdout_points], dtype=float
    )

    fit_robot = forward_robot_mm(fit_joints, zero_offsets, geometry)
    holdout_robot = forward_robot_mm(holdout_joints, zero_offsets, geometry)
    fit_sheet = transform_to_sheet(fit_robot, rotation, translation_xy, translation_z)
    holdout_sheet = transform_to_sheet(
        holdout_robot, rotation, translation_xy, translation_z
    )
    fit_errors = fit_sheet - fit_targets
    holdout_errors = holdout_sheet - holdout_targets
    fit_metrics = metrics(fit_errors)
    holdout_metrics = metrics(holdout_errors)
    metrics_pass = (
        holdout_metrics["xy_rms_mm"] <= HOLDOUT_XY_RMS_LIMIT_MM
        and holdout_metrics["xy_max_mm"] <= HOLDOUT_XY_MAX_LIMIT_MM
        and holdout_metrics["z_max_abs_mm"] <= HOLDOUT_Z_MAX_ABS_LIMIT_MM
    )
    if args.status == "accepted" and not metrics_pass:
        raise RuntimeError("holdout 指标未通过，禁止输出 accepted")

    # 软限位来自同机机械端点而非触点包络，holdout 不参与任何输出参数。
    soft_limits = [list(limit_pair) for limit_pair in PRACTICAL_SOFT_LIMITS_DEG]
    neutral_roll = finite_float(np.median(fit_joints[:, 4]), "J5 中性角")
    zero_offsets[4] = neutral_roll
    evidence_hashes = [
        sha256_file(contacts_path),
        sha256_file(tcp_path),
        str(contacts.get("motor_calibration_sha256", "")),
        str(contacts.get("plan_sha256", "")),
    ]
    if any(len(value) != 64 for value in evidence_hashes):
        raise RuntimeError("证据 SHA256 不完整")

    quality = {
        "status": args.status,
        "fit_samples": len(fit_points),
        "holdout_samples": len(holdout_points),
        "holdout_xy_rms_mm": round(holdout_metrics["xy_rms_mm"], 6),
        "holdout_xy_max_mm": round(holdout_metrics["xy_max_mm"], 6),
        "holdout_z_max_abs_mm": round(holdout_metrics["z_max_abs_mm"], 6),
    }
    calibration = {
        "schema_version": 1,
        "calibration_id": "single-arm-contact-sheet-v2-20260731",
        "robot_type": "so101_follower_5d",
        "source_arm": "so101_follower_modified_single_arm",
        "frame": "robot_base_mm",
        "joint_features": list(JOINT_FEATURES),
        "geometry_mm": {key: round(value, 6) for key, value in geometry.items()},
        "joint_mapping": {
            "directions": list(JOINT_DIRECTIONS),
            "zero_offsets_deg": rounded_list(zero_offsets),
            "soft_limits_deg": soft_limits,
        },
        "quality": quality,
        "evidence_sha256": evidence_hashes,
    }

    robot_from_sheet_rotation = rotation.T
    robot_from_sheet_translation_xy = -robot_from_sheet_rotation @ translation_xy
    transform = {
        "schema_version": 1,
        "transform_id": "contact-sheet-to-robot-base-20260731",
        "from_frame": "calibration_sheet_mm",
        "to_frame": "robot_base_mm",
        "translation_unit": "mm",
        "rotation_unit": "degree",
        "yaw_deg": round(
            math.degrees(
                math.atan2(
                    robot_from_sheet_rotation[1, 0],
                    robot_from_sheet_rotation[0, 0],
                )
            ),
            6,
        ),
        "translation_mm": [
            *rounded_list(robot_from_sheet_translation_xy),
            round(-translation_z, 6),
        ],
        "inverse_sheet_from_robot": {
            "yaw_deg": round(
                math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])), 6
            ),
            "translation_mm": [
                *rounded_list(translation_xy),
                round(translation_z, 6),
            ],
        },
        "evidence_sha256": evidence_hashes,
    }

    point_report = []
    for role, points, predicted, errors in (
        ("fit", fit_points, fit_sheet, fit_errors),
        ("holdout", holdout_points, holdout_sheet, holdout_errors),
    ):
        for point, prediction, error in zip(points, predicted, errors, strict=True):
            point_report.append(
                {
                    "label": point["label"],
                    "role": role,
                    "target_sheet_mm": rounded_list(point["target_mm"]),
                    "predicted_sheet_mm": rounded_list(prediction),
                    "error_mm": rounded_list(error),
                    "xy_error_mm": round(
                        finite_float(np.linalg.norm(error[:2]), "逐点 XY 误差"), 6
                    ),
                }
            )
    physical_fit = (
        fit_joints[:, :4] * np.asarray(JOINT_DIRECTIONS[:4]) + zero_offsets[:4]
    )
    tool_pitch = physical_fit[:, 1] + physical_fit[:, 2] + physical_fit[:, 3]
    report = {
        "schema_version": 1,
        "model": "fixed-geometry planar fit; optimize J2-J4 zero offsets and sheet rigid transform",
        "holdout_used_during_fit": False,
        "fit_optimizer_xyz_rmse_mm": round(fit_score, 6),
        "fit_metrics": {key: round(value, 6) for key, value in fit_metrics.items()},
        "holdout_metrics": {
            key: round(value, 6) for key, value in holdout_metrics.items()
        },
        "metric_gate_pass": metrics_pass,
        "metric_limits": {
            "holdout_xy_rms_mm": HOLDOUT_XY_RMS_LIMIT_MM,
            "holdout_xy_max_mm": HOLDOUT_XY_MAX_LIMIT_MM,
            "holdout_z_max_abs_mm": HOLDOUT_Z_MAX_ABS_LIMIT_MM,
        },
        "tool_pitch_deg_on_fit_contacts": {
            "mean": round(finite_float(np.mean(tool_pitch), "工具俯仰均值"), 6),
            "min": round(finite_float(np.min(tool_pitch), "工具俯仰最小值"), 6),
            "max": round(finite_float(np.max(tool_pitch), "工具俯仰最大值"), 6),
        },
        "requested_quality_status": args.status,
        "acceptance_decision": (
            "operator explicitly accepted the metric-passing calibration while "
            "retaining the recorded non-metric risks"
            if args.status == "accepted"
            else "no accepted status requested"
        ),
        "recorded_non_metric_risks": [
            "hand-drawn target placement uncertainty is not quantified",
            "magnet extension has one caliper reading; repeatability is UNKNOWN",
            "historical URDF source SHA256 is UNKNOWN",
            "practical soft limits do not constitute a complete self-collision map",
        ],
        "soft_limit_basis": (
            "same-arm calibrated motor endpoints with conservative margins; "
            "J5 additionally limited for cable twist"
        ),
        "points": point_report,
    }

    atomic_write_json(args.output_calibration.expanduser(), calibration)
    atomic_write_json(args.output_transform.expanduser(), transform)
    atomic_write_json(args.output_report.expanduser(), report)
    print(
        json.dumps(
            {"quality": quality, "metric_gate_pass": metrics_pass}, ensure_ascii=False
        )
    )
    print(f"calibration: {args.output_calibration}")
    print(f"transform: {args.output_transform}")
    print(f"report: {args.output_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
