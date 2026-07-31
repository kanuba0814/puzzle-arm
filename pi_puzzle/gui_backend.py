"""
GUI 后端接口定义 (Mock + 树莓派本地真实后端)

分层结构:
    gui_app.py      — Tkinter 界面 (只跟 RobotBackend 打交道, 不 import 任何硬件模块)
    gui_fb.py       — pygame/kmsdrm 比赛界面
    gui_backend.py  — 本文件: 抽象接口 + MockBackend + RealBackend
                      RealBackend 将调用转发到 ArmController 和 PuzzleRobot。

接口约定:
1. 所有控制方法都是 *非阻塞* 的: 立即返回, 在后台线程执行,
   进度通过回调通知 GUI (GUI 线程安全地刷界面)。
2. 返回值约定: 同步可判断的错误 (如参数非法) 返回 (False, msg);
   异步执行结果通过 on_task_done / on_log 回调汇报。
3. 回调由后端在工作线程触发, GUI 内部用 queue + after() 转到主线程,
   后端 *不需要* 关心线程安全。

后续扩展硬件功能时, 优先保持本接口稳定,
GUI 代码零改动。
"""

from __future__ import annotations

import os
import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

# ==================== 数据结构 ====================


@dataclass
class RobotState:
    """GUI 状态栏展示用的快照, 后端每次状态变化时通过 on_state_change 推送"""

    connected: bool = False  # 舵机串口是否已连接 (模拟模式也算 connected)
    simulation: bool = True  # 是否模拟模式 (无硬件)
    torque_enabled: bool = False  # 力矩使能
    magnet_on: bool = False  # 电磁铁通电
    calibrated: bool = False  # 视觉标定是否完成
    task_state: str = "IDLE"  # IDLE/CALIBRATING/DETECTING/SOLVING/EXECUTING/DONE/ERROR
    joints_deg: list[float] = field(default_factory=lambda: [0.0] * 5)  # J1-J5 当前角度
    servo_raw: dict[int, int] = field(default_factory=dict)  # 舵机原始值 0-4095


@dataclass
class RunMetrics:
    """比赛运行页展示用的实时指标。"""

    elapsed_s: float = 0.0
    limit_s: float = 120.0
    stage: str = "IDLE"
    detail: str = "-"
    progress: float = 0.0
    detected_count: int = 0
    expected_count: int = 4
    placed_count: int = 0
    current_piece: int = 0
    success: bool | None = None
    message: str = ""


TASK_PROFILES = {
    "task1": {
        "label": "任务1 自备纯色",
        "solver_mode": "task1",
        "expected_pieces": 4,
    },
    "task2_white": {
        "label": "任务2A 现场白色",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
    "task2_card": {
        "label": "任务2B 扑克牌",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
    # 兼容旧版 Tkinter GUI / 命令行中的 task2。
    "task2": {
        "label": "任务2 扑克牌",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
}


def normalize_task_mode(task_mode: str) -> str:
    """返回 TASK_PROFILES 中的合法键；未知模式原样返回以便调用方报错。"""
    return task_mode if task_mode in TASK_PROFILES else task_mode


def task_profile(task_mode: str) -> dict:
    return TASK_PROFILES.get(task_mode, TASK_PROFILES["task1"])


# 回调类型 (GUI 注册, 后端调用)
LogCallback = Callable[[str, str], None]  # (level, msg) level: INFO/WARN/ERROR
StateCallback = Callable[[RobotState], None]
ProgressCallback = Callable[[str, float], None]  # (阶段名, 0~1 进度)
TaskDoneCallback = Callable[[bool, str], None]  # (成功, 结果描述)
MetricsCallback = Callable[[RunMetrics], None]
FrameCallback = Callable[[object], None]  # BGR相机帧, 由GUI负责缩放显示


# ==================== 抽象接口 ====================


class RobotBackend(ABC):
    """
    机械臂 GUI 后端抽象接口。
    GUI 只依赖本类; 每个方法对应界面上的一组按钮/动作。
    """

    def __init__(self):
        # GUI 启动时通过 set_callbacks() 注入
        self.on_log: LogCallback = lambda level, msg: None
        self.on_state: StateCallback = lambda state: None
        self.on_progress: ProgressCallback = lambda stage, frac: None
        self.on_task_done: TaskDoneCallback = lambda ok, msg: None
        self.on_metrics: MetricsCallback = lambda metrics: None
        self.on_frame: FrameCallback = lambda frame: None

    def set_callbacks(
        self,
        on_log: LogCallback,
        on_state: StateCallback,
        on_progress: ProgressCallback,
        on_task_done: TaskDoneCallback,
    ):
        self.on_log = on_log
        self.on_state = on_state
        self.on_progress = on_progress
        self.on_task_done = on_task_done

    def set_metrics_callback(self, on_metrics: MetricsCallback) -> None:
        """注册比赛指标回调；旧GUI不调用时保持默认no-op。"""
        self.on_metrics = on_metrics

    def set_frame_callback(self, on_frame: FrameCallback) -> None:
        """注册相机帧回调；旧GUI不调用时保持默认no-op。"""
        self.on_frame = on_frame

    # ---------- 连接 ----------
    @abstractmethod
    def connect(self) -> tuple[bool, str]:
        """连接舵机串口 + 电磁铁 GPIO。返回 (是否成功, 状态描述)"""

    @abstractmethod
    def disconnect(self) -> None:
        """断开连接, 释放资源"""

    @abstractmethod
    def get_state(self) -> RobotState:
        """获取当前状态快照 (GUI 定时轮询用)"""

    # ---------- 使能 / 失能 ----------
    @abstractmethod
    def set_torque(self, enable: bool, servo_id: int | None = None) -> tuple[bool, str]:
        """力矩使能/失能。servo_id=None 表示全部 5 个关节"""

    @abstractmethod
    def set_magnet(self, on: bool) -> tuple[bool, str]:
        """电磁铁通电(吸附)/断电(释放)"""

    # ---------- 手动控制 ----------
    @abstractmethod
    def home(self) -> None:
        """回初始位置 (异步)"""

    @abstractmethod
    def jog_joint(self, joint_idx: int, delta_deg: float) -> tuple[bool, str]:
        """
        单关节步进 (相对当前位置)。
        joint_idx: 0-4 对应 J1-J5; delta_deg: 正负步进角度
        """

    @abstractmethod
    def move_joints(self, joints_deg: list[float]) -> None:
        """给定 5 个关节绝对角度运动 (异步)"""

    @abstractmethod
    def jog_xyz(self, dx: float, dy: float, dz: float) -> tuple[bool, str]:
        """末端笛卡尔步进 (mm, 相对当前末端位置)"""

    @abstractmethod
    def move_to_xyz(
        self, x: float, y: float, z: float, wrist_pitch: float = -90
    ) -> None:
        """末端绝对坐标运动 (异步)"""

    @abstractmethod
    def read_positions(self) -> dict[int, int]:
        """读全部舵机当前原始值 {id: 0-4095}"""

    # ---------- 自动流程 ----------
    @abstractmethod
    def calibrate(self) -> None:
        """一键标定 (相机内参 + 单应矩阵, 异步, 进度走 on_progress)"""

    def mark_calibrated(self) -> None:
        """重新检查持久化标定文件；不支持的后端可忽略。"""
        return

    @abstractmethod
    def start_task(self, task_mode: str) -> tuple[bool, str]:
        """
        开始自动检测搬运全流程 (异步):
        标定检查 -> 碎片检测 -> 拼图求解 -> 拾取放置执行 -> 声光提示
        task_mode: 'task1' (纯色碎片) | 'task2' (扑克牌碎片)
        """

    @abstractmethod
    def abort(self) -> tuple[bool, str]:
        """中止当前自动任务/运动 (尽快停, 保持力矩使能)"""

    @abstractmethod
    def emergency_stop(self) -> None:
        """急停: 立即停止运动并失能力矩"""


# ==================== Mock 实现 (无硬件, 用于 GUI 联调) ====================


class MockBackend(RobotBackend):
    """纯软件模拟后端: 打印日志 + 假进度, 让 GUI 可以在开发机上直接调试"""

    def __init__(self):
        super().__init__()
        self._state = RobotState(connected=False, simulation=True)
        self._abort_flag = threading.Event()
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None

    # ---- 内部工具 ----
    def _log(self, msg, level="INFO"):
        self.on_log(level, f"[Mock] {msg}")

    def _push_state(self):
        self.on_state(self._state)

    def _run_async(self, fn, *args):
        """后台线程执行; 自动检查上一个任务"""
        if self._worker and self._worker.is_alive():
            self._log("上一个任务仍在执行, 请先中止", "WARN")
            return False
        self._abort_flag.clear()
        self._worker = threading.Thread(target=fn, args=args, daemon=True)
        self._worker.start()
        return True

    def _fake_steps(self, steps: list[tuple[str, float]], stage: str):
        """按步骤模拟进度; 每步检查中止标志, 被中止返回 False"""
        for name, dur in steps:
            if self._abort_flag.is_set():
                self._log(f"{stage}已中止", "WARN")
                self._state.task_state = "IDLE"
                self._push_state()
                return False
            self._log(f"{stage}: {name}")
            time.sleep(dur)
        return True

    # ---- 连接 ----
    def connect(self):
        self._state.connected = True
        self._state.simulation = True
        self._log("已连接 (模拟模式)")
        self._push_state()
        return True, "模拟模式已连接"

    def disconnect(self):
        self.abort()
        self._state.connected = False
        self._state.torque_enabled = False
        self._push_state()
        self._log("已断开")

    def get_state(self):
        return self._state

    # ---- 使能 ----
    def set_torque(self, enable, servo_id=None):
        self._state.torque_enabled = enable
        target = f"舵机{servo_id}" if servo_id else "全部舵机"
        self._log(f"{target}力矩{'使能' if enable else '失能'}")
        self._push_state()
        return True, "ok"

    def set_magnet(self, on):
        self._state.magnet_on = on
        self._log(f"电磁铁{'通电吸附' if on else '断电释放'}")
        self._push_state()
        return True, "ok"

    # ---- 手动 ----
    def home(self):
        def _go():
            self._state.task_state = "EXECUTING"
            self._push_state()
            if self._fake_steps([("回到初始位置", 1.0)], "Home"):
                self._state.joints_deg = [0, 60, -75, -75, 0]
                self._state.task_state = "IDLE"
                self._push_state()

        self._run_async(_go)

    def jog_joint(self, joint_idx, delta_deg):
        if not (0 <= joint_idx < 5):
            return False, f"关节编号非法: {joint_idx}"
        self._state.joints_deg[joint_idx] += delta_deg
        self._log(
            f"J{joint_idx + 1} 步进 {delta_deg:+.1f}° -> "
            f"{self._state.joints_deg[joint_idx]:.1f}°"
        )
        self._push_state()
        return True, "ok"

    def move_joints(self, joints_deg):
        def _go():
            self._state.joints_deg = list(joints_deg)
            self._log(f"关节运动 -> {joints_deg}")
            time.sleep(0.5)
            self._push_state()

        self._run_async(_go)

    def jog_xyz(self, dx, dy, dz):
        self._log(f"末端步进 ({dx:+.0f},{dy:+.0f},{dz:+.0f}) mm")
        return True, "ok"

    def move_to_xyz(self, x, y, z, wrist_pitch=-90):
        def _go():
            self._log(f"末端运动 -> ({x:.0f},{y:.0f},{z:.0f}), 俯仰{wrist_pitch:.0f}°")
            time.sleep(0.5)

        self._run_async(_go)

    def read_positions(self):
        return dict.fromkeys(range(1, 6), 2048)

    # ---- 自动 ----
    def calibrate(self):
        def _calib():
            self._state.task_state = "CALIBRATING"
            self._push_state()
            steps = [
                ("打开相机", 0.5),
                ("采集标定帧 1/5", 0.6),
                ("采集标定帧 3/5", 0.6),
                ("采集标定帧 5/5", 0.6),
                ("求解单应矩阵", 0.8),
                ("保存标定文件", 0.3),
            ]
            total = sum(d for _, d in steps)
            done = 0.0
            for name, dur in steps:
                if self._abort_flag.is_set():
                    self._log("标定已中止", "WARN")
                    self._state.task_state = "IDLE"
                    self._push_state()
                    return
                self._log(f"标定: {name}")
                time.sleep(dur)
                done += dur
                self.on_progress("标定", done / total)
            self._state.calibrated = True
            self._state.task_state = "IDLE"
            self._push_state()
            self.on_task_done(True, "标定完成, 重投影误差 0.42px")

        self._run_async(_calib)

    def _emit_metrics(
        self,
        started_at: float,
        stage: str,
        detail: str,
        progress: float,
        expected: int,
        detected: int = 0,
        placed: int = 0,
        current: int = 0,
        success: bool | None = None,
        message: str = "",
    ) -> None:
        self.on_metrics(
            RunMetrics(
                elapsed_s=time.monotonic() - started_at,
                stage=stage,
                detail=detail,
                progress=max(0.0, min(1.0, progress)),
                detected_count=detected,
                expected_count=expected,
                placed_count=placed,
                current_piece=current,
                success=success,
                message=message,
            )
        )

    def start_task(self, task_mode):
        profile = task_profile(normalize_task_mode(task_mode))
        if normalize_task_mode(task_mode) not in TASK_PROFILES:
            return False, f"未知任务模式: {task_mode}"
        expected = profile["expected_pieces"]
        label = profile["label"]

        def _task():
            started_at = time.monotonic()
            stages = [
                ("DETECTING", [("采集图像", 0.8), (f"检测碎片 x{expected}", 1.0)]),
                ("SOLVING", [("求解拼图布局", 1.0), ("规划路径", 0.5)]),
                (
                    "EXECUTING",
                    [
                        ("拾取碎片 1/4", 1.0),
                        ("放置碎片 1/4", 0.8),
                        ("拾取碎片 2/4", 1.0),
                        ("放置碎片 2/4", 0.8),
                        ("完成剩余碎片", 1.5),
                    ],
                ),
                ("DONE", [("声光提示", 0.5)]),
            ]
            total = sum(d for _, steps in stages for _, d in steps)
            done_time = 0.0
            detected = 0
            placed = 0
            current = 0
            for state, steps in stages:
                self._state.task_state = state
                self._push_state()
                for name, dur in steps:
                    if self._abort_flag.is_set():
                        msg = "任务被中止"
                        self._log(msg, "WARN")
                        self._state.task_state = "IDLE"
                        self._push_state()
                        self._emit_metrics(
                            started_at,
                            "IDLE",
                            msg,
                            done_time / total,
                            expected,
                            detected,
                            placed,
                            current,
                            success=False,
                            message=msg,
                        )
                        self.on_task_done(False, msg)
                        return
                    self._log(f"{state}: {name}")
                    time.sleep(dur)
                    done_time += dur
                    if state == "DETECTING":
                        detected = expected
                    elif state == "EXECUTING":
                        if name.startswith("拾取"):
                            current = min(expected, current + 1)
                        elif name.startswith("放置"):
                            placed = min(expected, placed + 1)
                        elif name.startswith("完成"):
                            placed = expected
                    self.on_progress(state, done_time / total)
                    self._emit_metrics(
                        started_at,
                        state,
                        name,
                        done_time / total,
                        expected,
                        detected,
                        placed,
                        current,
                    )
            self._state.task_state = "DONE"
            self._push_state()
            elapsed = time.monotonic() - started_at
            msg = f"{label} 完成, 用时 {elapsed:.1f}s"
            self._emit_metrics(
                started_at,
                "DONE",
                "完成",
                1.0,
                expected,
                detected,
                placed,
                current,
                success=True,
                message=msg,
            )
            self.on_task_done(True, msg)

        if not self._run_async(_task):
            return False, "已有任务在运行"
        return True, "比赛任务已启动"

    def abort(self):
        self._abort_flag.set()
        return True, "中止信号已发送"

    def emergency_stop(self):
        self._abort_flag.set()
        self._state.torque_enabled = False
        self._state.task_state = "IDLE"
        self._push_state()
        self._log("急停! 力矩已失能", "ERROR")


# ==================== 真实后端 (预留, 本次会话不实现) ====================


class RealBackend(RobotBackend):
    """树莓派本地硬件后端；舵机/GPIO调用均不经过SSH。"""

    def __init__(self):
        super().__init__()
        self._arm = None
        self._state = RobotState(simulation=False)
        self._abort_flag = threading.Event()
        self._worker: threading.Thread | None = None

    def _push_state(self):
        self.on_state(self._state)

    def _run_async(self, fn, *args):
        if self._worker and self._worker.is_alive():
            return False
        self._abort_flag.clear()
        self._worker = threading.Thread(target=fn, args=args, daemon=True)
        self._worker.start()
        return True

    def _refresh_joints(self):
        if self._arm is None:
            return
        from config import SERVO_IDS
        from kinematics import servo_to_deg

        raw = self._arm.get_current_position()
        self._state.servo_raw = raw
        try:
            self._state.joints_deg = [
                float(servo_to_deg(raw[sid], sid)) if sid in raw else 0.0
                for sid in SERVO_IDS
            ]
        except (TypeError, ValueError, KeyError) as exc:
            self.on_log("ERROR", f"舵机角度换算失败: {exc}")

    def connect(self):
        from arm_controller import ArmController

        self._arm = ArmController()
        if self._arm.simulation:
            self._arm.close()
            self._arm = None
            return False, "未发现本地舵机串口，真实后端不可用"
        self._state.connected = True
        self._state.simulation = False
        self._state.torque_enabled = True
        self._state.calibrated = (
            Path("homography.npy").exists() and Path("arm_base.json").exists()
        )
        self._refresh_joints()
        self._push_state()
        return True, "本地舵机/GPIO已连接（无SSH）"

    def disconnect(self):
        self._abort_flag.set()
        if self._arm is not None:
            self._arm.magnet_off()
            self._arm.close()
        self._state.connected = False
        self._push_state()

    def get_state(self):
        return self._state

    def set_torque(self, enable, servo_id=None):
        if self._arm is None:
            return False, "机械臂未连接"
        try:
            if enable:
                self._arm.enable_torque(servo_id)
            else:
                self._arm.disable_torque(servo_id)
        except Exception as exc:
            return False, f"力矩控制失败: {exc}"
        self._state.torque_enabled = bool(enable)
        self._push_state()
        return True, "力矩已使能" if enable else "力矩已释放"

    def set_magnet(self, on):
        if self._arm is None:
            return False, "机械臂未连接"
        ok = self._arm.magnet_on() if on else self._arm.magnet_off()
        succeeded = bool(ok)
        self._state.magnet_on = bool(on and succeeded)
        self._push_state()
        return succeeded, "电磁铁已吸合" if on else "电磁铁已释放"

    def home(self):
        arm = self._arm
        if arm is None:
            return

        def _go():
            self._state.task_state = "EXECUTING"
            self._push_state()
            arm.go_home()
            arm.wait_for_stop()
            self._refresh_joints()
            self._state.task_state = "IDLE"
            self._push_state()

        self._run_async(_go)

    def jog_joint(self, joint_idx, delta_deg):
        if self._arm is None:
            return False, "机械臂未连接"
        if not 0 <= joint_idx < 5:
            return False, "关节编号非法"
        self._refresh_joints()
        joints = list(self._state.joints_deg)
        joints[joint_idx] += delta_deg
        self.move_joints(joints)
        return True, "关节步进已启动"

    def move_joints(self, joints_deg):
        arm = self._arm
        if arm is None:
            return

        def _go():
            arm.move_joints(joints_deg)
            arm.wait_for_stop()
            self._refresh_joints()
            self._push_state()

        self._run_async(_go)

    def jog_xyz(self, dx, dy, dz):
        if self._arm is None:
            return False, "机械臂未连接"
        self._refresh_joints()
        from kinematics import forward_kinematics

        xyz = forward_kinematics(self._state.joints_deg[:4])
        try:
            target = (float(xyz[0] + dx), float(xyz[1] + dy), float(xyz[2] + dz))
        except (TypeError, ValueError, IndexError) as exc:
            return False, f"末端坐标计算失败: {exc}"
        self.move_to_xyz(*target)
        return True, "末端步进已启动"

    def move_to_xyz(self, x, y, z, wrist_pitch=-90):
        arm = self._arm
        if arm is None:
            return
        try:
            pitch = int(wrist_pitch)
        except (TypeError, ValueError) as exc:
            self.on_log("ERROR", f"腕部角度无效: {exc}")
            return

        def _go():
            arm.move_to_xyz(x, y, z, wrist_pitch=pitch)
            arm.wait_for_stop()
            self._refresh_joints()
            self._push_state()

        self._run_async(_go)

    def read_positions(self):
        return {} if self._arm is None else self._arm.get_current_position()

    def calibrate(self):
        self.on_log("INFO", "请进入“系统标定”页面，使用本地相机完成标定")

    def mark_calibrated(self):
        self._state.calibrated = (
            Path("homography.npy").exists() and Path("arm_base.json").exists()
        )
        self._push_state()

    def _emit_metrics(
        self,
        started_at: float,
        stage: str,
        detail: str,
        progress: float,
        expected: int,
        detected: int = 0,
        placed: int = 0,
        current: int = 0,
        success: bool | None = None,
        message: str = "",
    ) -> None:
        self.on_metrics(
            RunMetrics(
                elapsed_s=time.monotonic() - started_at,
                stage=stage,
                detail=detail,
                progress=max(0.0, min(1.0, progress)),
                detected_count=detected,
                expected_count=expected,
                placed_count=placed,
                current_piece=current,
                success=success,
                message=message,
            )
        )

    def start_task(self, task_mode):
        mode = normalize_task_mode(task_mode)
        if mode not in TASK_PROFILES:
            return False, f"未知任务模式: {task_mode}"
        profile = task_profile(mode)
        if self._arm is None or not self._state.connected:
            return False, "机械臂未连接"
        if not self._state.calibrated:
            return False, "请先完成工作区和机械臂基座标定"

        expected = profile["expected_pieces"]
        solver_mode = profile["solver_mode"]
        label = profile["label"]

        def _task():
            started_at = time.monotonic()
            counters = {"detected": 0, "placed": 0, "current": 0}

            def progress(stage, frac, detail="-", detected=0, placed=0, current=0):
                counters["detected"] = max(counters["detected"], detected)
                counters["placed"] = max(counters["placed"], placed)
                counters["current"] = max(counters["current"], current)
                self._state.task_state = stage
                self._push_state()
                self.on_progress(stage, frac)
                self._emit_metrics(
                    started_at,
                    stage,
                    detail,
                    frac,
                    expected,
                    counters["detected"],
                    counters["placed"],
                    counters["current"],
                )

            def robot_log(level, message):
                self.on_log(level, message)

            def robot_frame(frame):
                self.on_frame(frame.copy())

            ok = False
            msg = "任务未执行"
            robot = None
            try:
                from config import MAIXCAM_UVC_INDEX, MAIXCAM_UVC_SIZE
                from main import PuzzleRobot

                width, height = MAIXCAM_UVC_SIZE
                robot = PuzzleRobot(
                    arm=self._arm,
                    abort_event=self._abort_flag,
                    progress_cb=progress,
                    log_cb=robot_log,
                    frame_cb=robot_frame,
                    allow_interactive=False,
                    camera_type="usb",
                    camera_kwargs={
                        "index": MAIXCAM_UVC_INDEX,
                        "width": width,
                        "height": height,
                        "mjpg": True,
                    },
                )
                self.on_log("INFO", f"比赛开始：{label}，限时120秒")
                ok = robot.run(task_mode=solver_mode)
                msg = robot.result_message
            except Exception as exc:  # GUI后台线程必须把硬件/视觉异常转为结果
                msg = f"自动任务异常: {exc}"
                self.on_log("ERROR", msg)
            finally:
                if robot is not None:
                    try:
                        robot.cleanup()
                    except Exception as exc:
                        self.on_log("WARN", f"自动任务资源清理失败: {exc}")
                self._refresh_joints()
                elapsed = time.monotonic() - started_at
                final_state = "DONE" if ok else "ERROR"
                if self._abort_flag.is_set() and not ok:
                    final_state = "IDLE"
                    if "中止" not in msg and "急停" not in msg:
                        msg = f"{msg}（已中止）"
                self._state.task_state = final_state
                self._push_state()
                self._emit_metrics(
                    started_at,
                    final_state,
                    msg,
                    1.0 if ok else 0.0,
                    expected,
                    counters["detected"],
                    counters["placed"],
                    counters["current"],
                    success=ok,
                    message=msg,
                )
                self.on_progress(final_state, 1.0 if ok else 0.0)
                self.on_task_done(ok, f"{msg}；用时 {elapsed:.1f}s")

        if not self._run_async(_task):
            return False, "已有任务在运行"
        return True, "比赛任务已启动，请立即移除摄像头遮挡"

    def abort(self):
        self._abort_flag.set()
        return True, "中止信号已发送"

    def emergency_stop(self):
        self._abort_flag.set()
        if self._arm is not None:
            self._arm.magnet_off()
            self._arm.disable_torque()
        self._state.magnet_on = False
        self._state.torque_enabled = False
        self._state.task_state = "IDLE"
        self._push_state()
        self.on_log("ERROR", "急停：电磁铁关闭，全部力矩释放")


def create_backend(prefer_real: bool = False) -> RobotBackend:
    """Pi启动脚本设置PUZZLE_REAL=1；开发机默认使用Mock。"""
    if prefer_real or os.environ.get("PUZZLE_REAL") == "1":
        return RealBackend()
    return MockBackend()
