#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""正式页面调用的红色碎片中心控制工具；磁铁始终关闭。"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import time
from pathlib import Path
from typing import Any, TypedDict

from control import (
    ArmCalibration,
    JointVector,
    MotionConfig,
    MotionController,
    MotionGoal,
    PlanarArmKinematics,
    ToolPose,
)
from lerobot_robot_so101_5d import (
    JOINT_NAMES,
    SO101Follower5D,
    SO101Follower5DConfig,
)

CALIBRATION_PATH = Path(
    "workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"
)
TRANSFORM_PATH = Path("workspace/calibration/contact_sheet_to_robot_base_20260731.json")
PARK_PATH = Path("workspace/calibration/park_pose_single_arm_20260731.json")
DEFAULT_PIECES_PATH = Path("workspace/runtime/red_piece_detection/pieces.json")
HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -22.202025,
    "elbow_flex": 38.197802,
    "wrist_flex": 84.659341,
    "wrist_roll": -82.505495,
}


class Piece(TypedDict):
    id: int
    piece_id: str
    x: float
    y: float
    yaw_deg: float


def finite_float(value: object, field: str) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeError(f"{field} 必须是有限数值") from error
    if not math.isfinite(number):
        raise RuntimeError(f"{field} 必须是有限数值")
    return number


def load_json_object(path: Path, field: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"无法读取 {field}: {path}") from error
    if not isinstance(payload, dict):
        raise RuntimeError(f"{field} 必须是 JSON 对象")
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("repeat", "sweep", "pregrasp", "park"))
    parser.add_argument("--pieces", type=Path, default=DEFAULT_PIECES_PATH)
    parser.add_argument("--piece-index", type=int, default=0)
    parser.add_argument("--output", type=Path)
    return parser


def load_pieces(path: Path) -> list[Piece]:
    payload = load_json_object(path, "碎片观察")
    pieces = payload.get("pieces")
    if not isinstance(pieces, list) or not pieces:
        raise RuntimeError("没有可执行的碎片观察")
    result = []
    for index, piece in enumerate(pieces):
        if not isinstance(piece, dict):
            raise RuntimeError(f"pieces[{index}] 必须是对象")
        center = piece.get("center_table_mm")
        if not isinstance(center, list) or len(center) != 2:
            raise RuntimeError(f"pieces[{index}].center_table_mm 无效")
        result.append(
            Piece(
                id=index,
                piece_id=str(piece.get("piece_id", f"red-{index}")),
                x=finite_float(center[0], f"pieces[{index}].center_table_mm[0]"),
                y=finite_float(center[1], f"pieces[{index}].center_table_mm[1]"),
                yaw_deg=finite_float(
                    piece.get("yaw_deg", 0.0), f"pieces[{index}].yaw_deg"
                ),
            )
        )
    return result


def main() -> int:
    args = build_parser().parse_args()
    calibration = ArmCalibration.load(CALIBRATION_PATH)
    geometry = calibration.require_accepted_geometry()
    model = PlanarArmKinematics(geometry)
    transform = load_json_object(TRANSFORM_PATH, "sheet-to-robot 变换")
    park_payload = load_json_object(PARK_PATH, "PARK 标定")
    park_deg = {
        name: finite_float(park_payload["joint_deg"][name], f"PARK {name}")
        for name in JOINT_NAMES
    }
    pieces = load_pieces(args.pieces) if args.mode != "park" else []
    if pieces and not 0 <= args.piece_index < len(pieces):
        raise RuntimeError("piece-index 超出检测结果范围")

    yaw = math.radians(transform["yaw_deg"])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation_x, translation_y, translation_z = transform["translation_mm"]
    origin_x, origin_y = transform["inverse_sheet_from_robot"]["translation_mm"][:2]

    robot = SO101Follower5D(
        SO101Follower5DConfig(
            port=("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00"),
            id="epuzzle-single-arm",
            calibration_dir=(
                Path.home()
                / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
            ),
            disable_torque_on_disconnect=True,
            use_degrees=True,
        )
    )
    controller = MotionController.from_calibration_file(
        robot,
        CALIBRATION_PATH,
        MotionConfig(
            10.0,
            5.0,
            10.0,
            5.0,
            1,
            0.05,
            3.0,
            -20.0,
            -30.0,
            180.0,
            waypoint_min_dwell_s_by_profile={
                "EMPTY_NORMAL": 0.15,
                "EMPTY_SLOW": 0.20,
                "HOLDING_SLOW": 0.30,
                "LOCAL_POLICY": 0.15,
            },
        ),
    )

    def read_deg() -> dict[str, float]:
        return {
            name: finite_float(value, f"Present_Position.{name}")
            for name, value in robot.bus.sync_read(
                "Present_Position", normalize=True, num_retry=2
            ).items()
        }

    def direct_move(target: dict[str, float]) -> None:
        start = read_deg()
        steps = max(
            1,
            math.ceil(max(abs(target[name] - start[name]) for name in JOINT_NAMES)),
        )
        for step in range(1, steps + 1):
            fraction = step / steps
            command = {
                name: start[name] + (target[name] - start[name]) * fraction
                for name in JOINT_NAMES
            }
            robot.send_action({f"{name}.pos": command[name] for name in JOINT_NAMES})
            time.sleep(0.06)
            actual = read_deg()
            if max(abs(actual[name] - command[name]) for name in JOINT_NAMES) > 8.0:
                raise RuntimeError("PARK/HOME 恢复跟随误差超限")
        time.sleep(0.3)

    def park_to_home() -> None:
        upright = dict(HOME)
        upright["shoulder_pan"] = park_deg["shoulder_pan"]
        direct_move(upright)
        direct_move(HOME)

    def home_to_park() -> None:
        direct_move(HOME)
        upright = dict(HOME)
        upright["shoulder_pan"] = park_deg["shoulder_pan"]
        direct_move(upright)
        direct_move(park_deg)

    def target_pose(piece: Piece, height_mm: float) -> ToolPose:
        table_x = piece["x"]
        table_y = piece["y"]
        delta_x = table_x - origin_x
        delta_y = table_y - origin_y
        radius = math.hypot(delta_x, delta_y)
        table_x += 3.0 * delta_x / radius
        table_y += 3.0 * delta_y / radius
        robot_x = cosine * table_x - sine * table_y + translation_x
        robot_y = sine * table_x + cosine * table_y + translation_y
        robot_z = translation_z + height_mm
        lateral = geometry.shoulder_lateral_offset_mm
        radial = math.sqrt(robot_x**2 + robot_y**2 - lateral**2)
        physical_pan = math.degrees(
            math.atan2(robot_y, robot_x) - math.atan2(lateral, radial)
        )
        pose = ToolPose(
            "robot_base_mm",
            (robot_x, robot_y, robot_z),
            -87.5,
            physical_pan,
        )
        model.inverse(pose)
        return pose

    def move(goal_id: str, pose: ToolPose) -> None:
        result = controller.move(
            MotionGoal(
                goal_id,
                time.monotonic_ns(),
                "robot_base_mm",
                pose.position_mm,
                pose.pitch_deg,
                pose.yaw_deg,
                "PREGRASP",
                "EMPTY_NORMAL",
            )
        )
        if not result.success:
            raise RuntimeError(
                f"{goal_id}: {result.error_code.value}: {result.message}"
            )

    events: list[dict[str, object]] = []
    records: dict[str, object] = {
        "schema_version": 1,
        "mode": args.mode,
        "park_id": park_payload["park_id"],
        "magnet_enabled": False,
        "events": events,
    }
    at_park = False
    try:
        observation = robot.connect_bumpless(
            # 与官方 SOFollower.configure() 一致，P=16 用于降低负载下抖动。
            p_coefficient=16,
            acceleration=30,
        )
        start = {
            name: finite_float(observation[f"{name}.pos"], f"start {name}")
            for name in JOINT_NAMES
        }
        if max(abs(start[name] - park_deg[name]) for name in JOINT_NAMES) > 4.0:
            raise RuntimeError("机械臂不在正式 PARK，拒绝页面动作")
        if args.mode == "park":
            direct_move(park_deg)
        else:
            park_to_home()
            selected = pieces[args.piece_index]
            if args.mode == "repeat":
                high = target_pose(selected, 60.0)
                low = target_pose(selected, 20.0)
                move("page-repeat-high", high)
                for repetition in (1, 2):
                    move(f"page-repeat-{repetition}", low)
                    robot.send_action(model.inverse(low).as_mapping())
                    time.sleep(3.0)
                    events.append(
                        {
                            "piece_id": selected["piece_id"],
                            "repetition": repetition,
                            "joint_deg": read_deg(),
                        }
                    )
                    move(f"page-repeat-up-{repetition}", high)
            elif args.mode == "pregrasp":
                high = target_pose(selected, 60.0)
                low = target_pose(selected, 20.0)
                move("page-pregrasp-high", high)
                move("page-pregrasp-low", low)
                time.sleep(5.0)
                move("page-pregrasp-up", high)
            else:
                ordered = sorted(pieces, key=lambda piece: (-piece["y"], piece["x"]))
                for index, piece in enumerate(ordered):
                    high = target_pose(piece, 60.0)
                    low = target_pose(piece, 20.0)
                    move(f"page-sweep-{index}-high", high)
                    move(f"page-sweep-{index}-low", low)
                    robot.send_action(model.inverse(low).as_mapping())
                    time.sleep(1.0)
                    events.append(
                        {"piece_id": piece["piece_id"], "joint_deg": read_deg()}
                    )
                    move(f"page-sweep-{index}-up", high)
            home_pose = model.forward(
                JointVector(tuple(HOME[name] for name in JOINT_NAMES))
            )
            move("page-return-home", home_pose)
            home_to_park()
        at_park = True
        records["result"] = "PASS"
        records["final_joint_deg"] = read_deg()
    finally:
        if robot.bus.is_connected:
            if not at_park:
                with contextlib.suppress(BaseException):
                    home_to_park()
            robot.disconnect()
    records["torque_disabled_after_test"] = True
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(records, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(records, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
