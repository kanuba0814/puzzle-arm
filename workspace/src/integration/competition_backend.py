#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""赛场 GUI 后端接口：把复杂参数验证集中在 Adapter，GUI 只处理统一请求。

设计要点：
- ``CompetitionBackend`` 是 GUI 唯一依赖的接口。
- ``BackendCapability`` 报告每个命令 ID 的可用性、原因和安全属性。
- ``BackendRequest`` 包含子进程 argv、超时和结构化 status/result 路径。
- ``SupervisorRunBackend`` 把请求转发到 ``workspace/tools/supervisor_run.py``。
- 未实现或门禁未通过的比赛/手动命令显式返回 ``LOCKED``/``UNAVAILABLE``，
  由 GUI 渲染为禁用按钮和具体原因。
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

WORKSPACE = Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE.parent
DEFAULT_PYTHON = Path("/opt/epuzzle/venv/bin/python")
DEFAULT_SUPERVISOR_TOOL = WORKSPACE / "tools" / "supervisor_run.py"
DEFAULT_ROBOT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_CAMERA_DEVICE = Path("/dev/epuzzle-camera")
DEFAULT_ROBOT_MOTION_LOCK = Path("/etc/epuzzle/robot-motion.lock.json")
DEFAULT_LEROBOT_CALIBRATION = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)

# 比赛计时仅用于显示/记录；后端不再因 120 秒自动终止运行。
COMPETITION_TIMEOUT_S = None
MAGNET_PULSE_TIMEOUT_MARGIN_S = 15.0
MAGNET_OFF_TIMEOUT_S = 10.0

# 五轴固定顺序；禁止依赖字典遍历顺序。
JOINT_ORDER = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


class CapabilityAvailability(str, Enum):
    READY = "READY"
    LOCKED = "LOCKED"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class BackendCapability:
    command_id: str
    availability: CapabilityAvailability
    reason: str
    requires_robot: bool
    requires_manual_unlock: bool


@dataclass(frozen=True)
class BackendRequest:
    command_id: str
    argv: list[str]
    timeout_s: float
    status_path: Path
    result_path: Path


class CompetitionBackend(Protocol):
    def capabilities(self) -> dict[str, BackendCapability]:
        """返回命令 ID 到能力的映射。"""
        ...

    def build_request(
        self,
        command_id: str,
        parameters: dict[str, Any],
        *,
        status_path: Path,
        result_path: Path,
    ) -> BackendRequest:
        """构造子进程请求；参数非法时抛出 ``ValueError``，门禁不满足返回明确异常。"""
        ...

    def robot_motion_lock_reason(self) -> str | None:
        """返回运动安全锁原因；None 表示未锁定。"""
        ...

    def robot_available(self) -> bool:
        """机械臂设备节点是否存在。"""
        ...


def _read_json_file(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def robot_motion_lock_reason(motion_lock_path: Path) -> str | None:
    """读取持久运动锁；文件存在即锁定。"""
    if not motion_lock_path.exists():
        return None
    payload = _read_json_file(motion_lock_path)
    if payload is None:
        return f"运动安全锁无法解析: {motion_lock_path}"
    reason = payload.get("reason")
    return str(reason) if reason else "存在未解除的运动安全锁"


def _finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (str, int, float)):
        raise ValueError(f"{field} 必须是有限数值")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须是有限数值") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} 必须是有限数值")
    return number


def _integer(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError(f"{field} 必须是整数")
    try:
        number = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须是整数") from error
    return number


def pick_place_lock_reason(calibration_path: Path | None = None) -> str | None:
    """PICK/PLACE 自动抓放门禁；返回 None 表示已解锁。"""
    if calibration_path is None:
        calibration_path = WORKSPACE / "calibration" / "pick_place_contact_height.json"
    payload = _read_json_file(calibration_path)
    if payload is None:
        return "PICK/PLACE 接触高度 UNKNOWN：未找到已验收的接触高度标定"
    quality = payload.get("quality")
    if not isinstance(quality, dict) or quality.get("status") != "accepted":
        return "接触高度标定未通过验收（quality.status != accepted）"
    success_rate = _finite_float(
        payload.get("single_piece_success_rate", 0.0),
        "single_piece_success_rate",
    )
    if success_rate < 0.8:
        return f"单片成功率 {success_rate:.0%} 低于 80% 解锁门"
    return None


class SupervisorRunBackend:
    """基于 ``supervisor_run.py`` 的 Adapter；所有复杂参数验证集中于此。"""

    # 命令 ID 到 CLI mode 的映射；未来新增命令必须同时更新此处。
    COMMAND_TO_MODE = {
        "detect": "detect",
        "red-baseline": "red-baseline",
        "sweep-shortest": "sweep-shortest",
        "pregrasp": "pregrasp",
        "park": "park",
        "puzzle-shadow": "puzzle-shadow",
        "pick-place": "pick-place",
        "competition-self-color": "competition-self-color",
        "competition-white": "competition-white",
        "competition-card": "competition-card",
        "camera-calibration": "camera-calibration",
        "manual-clear-motion-lock": "manual-clear-motion-lock",
        "manual-read-state": "manual-read-state",
        "manual-jog-joint": "manual-jog-joint",
        "manual-jog-cartesian": "manual-jog-cartesian",
        "manual-magnet-pulse": "manual-magnet-pulse",
        "manual-magnet-off": "manual-magnet-off",
        "commission-center": "commission-center",
    }

    # 命令 ID 是否需要真实机械臂硬件。
    REQUIRES_ROBOT = frozenset(
        {
            "red-baseline",
            "sweep-shortest",
            "pregrasp",
            "park",
            "pick-place",
            "competition-self-color",
            "competition-white",
            "competition-card",
            "manual-read-state",
            "manual-jog-joint",
            "manual-jog-cartesian",
            "manual-magnet-pulse",
            "manual-magnet-off",
            "commission-center",
        }
    )

    # 需要手动维护解锁的命令。
    REQUIRES_MANUAL_UNLOCK = frozenset(
        {
            "manual-jog-joint",
            "manual-jog-cartesian",
            "manual-magnet-pulse",
        }
    )

    def __init__(
        self,
        *,
        python_path: str | Path | None = None,
        supervisor_tool: str | Path | None = None,
        robot_port: str | Path | None = None,
        camera_device: str | Path | None = None,
        robot_motion_lock: str | Path | None = None,
        pieces_path: str | Path | None = None,
        pick_place_calibration: str | Path | None = None,
        unlock_confirmation_path: str | Path | None = None,
    ) -> None:
        self.python_path = Path(
            python_path
            or os.environ.get("EP_GUI_PYTHON")
            or (str(DEFAULT_PYTHON) if DEFAULT_PYTHON.is_file() else sys.executable)
        )
        self.supervisor_tool = Path(
            supervisor_tool
            or os.environ.get("EP_GUI_SUPERVISOR_TOOL", str(DEFAULT_SUPERVISOR_TOOL))
        )
        self.robot_port = Path(
            robot_port or os.environ.get("EPUZZLE_ROBOT_PORT", str(DEFAULT_ROBOT_PORT))
        )
        self.camera_device = Path(
            camera_device
            or os.environ.get("EPUZZLE_CAMERA_DEVICE", str(DEFAULT_CAMERA_DEVICE))
        )
        self.robot_motion_lock = Path(
            robot_motion_lock
            or os.environ.get(
                "EPUZZLE_ROBOT_MOTION_LOCK", str(DEFAULT_ROBOT_MOTION_LOCK)
            )
        )
        self.pieces_path = Path(
            pieces_path
            or os.environ.get(
                "EPUZZLE_PIECES_PATH",
                str(WORKSPACE / "runtime" / "red_piece_detection" / "pieces.json"),
            )
        )
        self.pick_place_calibration = Path(
            pick_place_calibration
            or os.environ.get(
                "EPUZZLE_PICK_PLACE_CALIBRATION",
                str(WORKSPACE / "calibration" / "pick_place_contact_height.json"),
            )
        )
        self.unlock_confirmation_path = Path(
            unlock_confirmation_path
            or os.environ.get(
                "EPUZZLE_UNLOCK_CONFIRMATION_PATH",
                str(WORKSPACE / "runtime/gui/motion_lock_confirmation.json"),
            )
        )
        # 运行时覆盖，仅用于测试。
        self._capability_overrides: dict[str, BackendCapability] | None = None

    def override_capabilities(
        self, capabilities: dict[str, BackendCapability] | None
    ) -> None:
        """测试钩子：临时替换能力表。"""
        self._capability_overrides = capabilities

    def capabilities(self) -> dict[str, BackendCapability]:
        if self._capability_overrides is not None:
            return self._capability_overrides
        return {
            command_id: self._capability(command_id)
            for command_id in self.COMMAND_TO_MODE
        }

    def _capability(self, command_id: str) -> BackendCapability:
        robot_ready = self.robot_available() and self.robot_motion_lock_reason() is None

        # 既有巡检/观察类命令保持可用；PARK 受运动锁限制由运行时按钮状态处理。
        if command_id in {"detect", "puzzle-shadow"}:
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY,
                reason="仅依赖视觉数据，不操作机械臂",
                requires_robot=False,
                requires_manual_unlock=False,
            )
        if command_id == "camera-calibration":
            camera_ready = self.camera_device.exists()
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if camera_ready
                else CapabilityAvailability.LOCKED,
                reason=(
                    "固定相机可用；仅生成候选标定，不改写正式标定"
                    if camera_ready
                    else f"固定相机离线: {self.camera_device}"
                ),
                requires_robot=False,
                requires_manual_unlock=False,
            )
        if command_id in {
            "red-baseline",
            "sweep-shortest",
            "pregrasp",
            "park",
            "commission-center",
        }:
            reason = "机械臂已就绪" if robot_ready else "机械臂离线或运动安全锁锁定"
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if robot_ready
                else CapabilityAvailability.LOCKED,
                reason=reason,
                requires_robot=True,
                requires_manual_unlock=False,
            )
        if command_id == "pick-place":
            contact_lock = pick_place_lock_reason(self.pick_place_calibration)
            if contact_lock is not None:
                return BackendCapability(
                    command_id="pick-place",
                    availability=CapabilityAvailability.LOCKED,
                    reason=contact_lock,
                    requires_robot=True,
                    requires_manual_unlock=False,
                )
            return BackendCapability(
                command_id="pick-place",
                availability=CapabilityAvailability.READY
                if robot_ready
                else CapabilityAvailability.LOCKED,
                reason="接触高度已验收，自动抓放可用"
                if robot_ready
                else "机械臂离线或运动安全锁锁定",
                requires_robot=True,
                requires_manual_unlock=False,
            )

        # 比赛流程复用已验收的 live_detect_123 → 123 solver → 四片抓放链路；
        # 每次仍由 CLI 重新检查接触高度、设备节点和持久运动锁。
        if command_id in {
            "competition-self-color",
            "competition-white",
            "competition-card",
        }:
            contact_lock = pick_place_lock_reason(self.pick_place_calibration)
            if contact_lock is not None:
                return BackendCapability(
                    command_id=command_id,
                    availability=CapabilityAvailability.LOCKED,
                    reason=contact_lock,
                    requires_robot=True,
                    requires_manual_unlock=False,
                )
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if robot_ready
                else CapabilityAvailability.LOCKED,
                reason="123 视觉、求解与四片抓放链路已接通"
                if robot_ready
                else "机械臂离线或运动安全锁锁定",
                requires_robot=True,
                requires_manual_unlock=False,
            )

        if command_id == "manual-clear-motion-lock":
            lock_reason = self.robot_motion_lock_reason()
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if lock_reason is not None
                else CapabilityAvailability.LOCKED,
                reason=(
                    "须在本地逐项确认 clear_requires 并长按解除"
                    if lock_reason is not None
                    else "当前不存在运动安全锁"
                ),
                requires_robot=False,
                requires_manual_unlock=False,
            )

        # 手动读回是只读动作，允许在运动锁存在时用于诊断；点动仍须机械臂
        # 可动作且需 GUI 会话维护解锁。
        if command_id == "manual-read-state":
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if self.robot_available()
                else CapabilityAvailability.LOCKED,
                reason="只读五轴/TCP 状态可查询"
                if self.robot_available()
                else "机械臂设备离线",
                requires_robot=True,
                requires_manual_unlock=False,
            )
        if command_id in {"manual-jog-joint", "manual-jog-cartesian"}:
            return BackendCapability(
                command_id=command_id,
                availability=CapabilityAvailability.READY
                if robot_ready
                else CapabilityAvailability.LOCKED,
                reason="维护解锁后可执行受限安全点动"
                if robot_ready
                else "机械臂离线或运动安全锁锁定",
                requires_robot=True,
                requires_manual_unlock=True,
            )

        # 电磁铁：立即关闭尽量可用；定时脉冲需要维护解锁。
        if command_id == "manual-magnet-off":
            return BackendCapability(
                command_id="manual-magnet-off",
                availability=CapabilityAvailability.READY,
                reason="可直接请求继电器释放",
                requires_robot=False,
                requires_manual_unlock=False,
            )
        if command_id == "manual-magnet-pulse":
            return BackendCapability(
                command_id="manual-magnet-pulse",
                availability=CapabilityAvailability.READY
                if robot_ready
                else CapabilityAvailability.LOCKED,
                reason="维护解锁后可启动有界脉冲"
                if robot_ready
                else "机械臂离线或运动安全锁锁定",
                requires_robot=True,
                requires_manual_unlock=True,
            )

        raise ValueError(f"未知命令 ID: {command_id}")

    def build_request(
        self,
        command_id: str,
        parameters: dict[str, Any],
        *,
        status_path: Path,
        result_path: Path,
    ) -> BackendRequest:
        cap = self.capabilities().get(command_id)
        if cap is None:
            raise ValueError(f"未知命令 ID: {command_id}")
        if cap.availability is not CapabilityAvailability.READY:
            raise RuntimeError(f"{command_id} 未就绪: {cap.reason}")
        mode = self.COMMAND_TO_MODE.get(command_id)
        if mode is None:
            raise ValueError(f"未知命令 ID: {command_id}")

        argv: list[str] = [str(self.python_path), str(self.supervisor_tool), mode]
        extra = self._build_extra_args(command_id, parameters)
        argv.extend(extra)
        argv.extend(
            ["--status-file", str(status_path), "--result-file", str(result_path)]
        )

        timeout_s = self._timeout_for(command_id, parameters)
        return BackendRequest(
            command_id=command_id,
            argv=argv,
            timeout_s=timeout_s,
            status_path=status_path,
            result_path=result_path,
        )

    def _build_extra_args(
        self, command_id: str, parameters: dict[str, Any]
    ) -> list[str]:
        if command_id == "camera-calibration":
            return ["--camera", str(self.camera_device)]
        if command_id == "manual-clear-motion-lock":
            return ["--lock-confirmation", str(self.unlock_confirmation_path)]
        if command_id in {
            "detect",
            "red-baseline",
            "sweep-shortest",
            "pregrasp",
            "park",
            "puzzle-shadow",
            "pick-place",
            "commission-center",
        }:
            if command_id == "commission-center":
                hold_seconds = _finite_float(
                    parameters.get("hold_seconds", 300.0), "hold_seconds"
                )
                if not 0.0 < hold_seconds <= 300.0:
                    raise ValueError("hold_seconds 必须在 (0, 300] 范围内")
                return ["--hold-seconds", f"{hold_seconds:.1f}"]
            if command_id == "puzzle-shadow":
                angle = _finite_float(
                    parameters.get("target_angle_deg", 0.0), "target_angle_deg"
                )
                return ["--target-angle", f"{angle:.1f}"]
            if command_id == "pregrasp":
                index = _integer(parameters.get("piece_index", 0), "piece_index")
                return ["--piece-index", str(index), "--pieces", str(self.pieces_path)]
            if command_id in {"red-baseline", "sweep-shortest"}:
                return ["--pieces", str(self.pieces_path)]
            return []

        if command_id == "manual-jog-joint":
            joint = str(parameters.get("joint_feature"))
            if joint not in JOINT_ORDER:
                raise ValueError(f"joint_feature 必须是 {JOINT_ORDER} 之一")
            delta = _finite_float(parameters.get("delta_deg", 0.0), "delta_deg")
            if not -10.0 <= delta <= 10.0:
                raise ValueError("delta_deg 必须在 ±10° 安全步长内")
            return ["--joint-feature", joint, "--delta-deg", f"{delta:.2f}"]

        if command_id == "manual-jog-cartesian":
            axis = str(parameters.get("axis"))
            if axis not in {"x", "y", "z", "pitch", "yaw"}:
                raise ValueError("axis 必须是 x|y|z|pitch|yaw")
            delta = _finite_float(parameters.get("delta", 0.0), "delta")
            maximum_delta = 10.0 if axis in {"x", "y", "z"} else 15.0
            if not -maximum_delta <= delta <= maximum_delta:
                raise ValueError(f"{axis} 点动超过 ±{maximum_delta:g} 安全步长")
            frame = str(parameters.get("frame", "robot_base_mm"))
            if frame != "robot_base_mm":
                raise ValueError("frame 当前只支持 robot_base_mm")
            return ["--axis", axis, "--delta", f"{delta:.3f}", "--frame", frame]

        if command_id == "manual-magnet-pulse":
            duration = _finite_float(parameters.get("duration_s", 3.0), "duration_s")
            if duration <= 0.0:
                raise ValueError("duration_s 必须是大于 0 的有限秒数")
            return ["--duration-s", f"{duration:.3f}"]

        if command_id == "manual-magnet-off":
            return []

        if command_id in {
            "competition-self-color",
            "competition-white",
            "competition-card",
        }:
            return []

        if command_id == "manual-read-state":
            return []

        raise ValueError(f"未实现参数构造: {command_id}")

    def _timeout_for(self, command_id: str, parameters: dict[str, Any]) -> float:
        if command_id == "camera-calibration":
            return 55.0
        if command_id == "manual-clear-motion-lock":
            return 30.0
        if command_id in {
            "competition-self-color",
            "competition-white",
            "competition-card",
        }:
            return None
        if command_id == "manual-magnet-pulse":
            duration = _finite_float(parameters.get("duration_s", 3.0), "duration_s")
            return duration + MAGNET_PULSE_TIMEOUT_MARGIN_S
        if command_id == "manual-magnet-off":
            return MAGNET_OFF_TIMEOUT_S
        if command_id == "commission-center":
            hold_seconds = _finite_float(
                parameters.get("hold_seconds", 300.0), "hold_seconds"
            )
            return hold_seconds + 60.0
        if command_id in {
            "detect",
            "red-baseline",
            "sweep-shortest",
            "pregrasp",
            "park",
            "puzzle-shadow",
        }:
            return _finite_float(parameters.get("timeout_s", 120.0), "timeout_s")
        return 60.0

    def robot_motion_lock_reason(self) -> str | None:
        return robot_motion_lock_reason(self.robot_motion_lock)

    def robot_available(self) -> bool:
        return self.robot_port.exists()


def create_default_backend(**kwargs: Any) -> SupervisorRunBackend:
    """工厂函数；GUI 启动时调用。"""
    return SupervisorRunBackend(**kwargs)
