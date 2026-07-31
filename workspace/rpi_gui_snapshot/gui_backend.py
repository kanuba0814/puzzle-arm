# pyright: reportMissingImports=false
"""
GUI 后端接口定义 (本次会话只定义接口 + Mock 实现, 硬件接线后实现 RealBackend)

分层结构:
    gui_app.py      — Tkinter 界面 (只跟 RobotBackend 打交道, 不 import 任何硬件模块)
    gui_backend.py  — 本文件: 抽象接口 + MockBackend (纯软件模拟, 可直接运行调试 GUI)
                      RealBackend (预留, 后续接线时把调用转发到:
                        ArmController / MotionPlanner / PuzzleRobot / calibrate_camera)

接口约定:
1. 所有控制方法都是 *非阻塞* 的: 立即返回, 在后台线程执行,
   进度通过回调通知 GUI (GUI 线程安全地刷界面)。
2. 返回值约定: 同步可判断的错误 (如参数非法) 返回 (False, msg);
   异步执行结果通过 on_task_done / on_log 回调汇报。
3. 回调由后端在工作线程触发, GUI 内部用 queue + after() 转到主线程,
   后端 *不需要* 关心线程安全。

后续 RealBackend 实现时, 逐个方法替换 TODO 注释处的转发逻辑即可,
GUI 代码零改动。
"""

from __future__ import annotations

import os
import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field

from control.magnet_controller import MagnetConfig, MagnetController, MagnetStatus

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


# 回调类型 (GUI 注册, 后端调用)
LogCallback = Callable[[str, str], None]  # (level, msg) level: INFO/WARN/ERROR
StateCallback = Callable[[RobotState], None]
ProgressCallback = Callable[[str, float], None]  # (阶段名, 0~1 进度)
TaskDoneCallback = Callable[[bool, str], None]  # (成功, 结果描述)


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

    def __init__(self, magnet_config: MagnetConfig | None = None):
        super().__init__()
        self._state = RobotState(connected=False, simulation=True)
        self._abort_flag = threading.Event()
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._magnet = (
            MagnetController(magnet_config, on_status=self._on_magnet_status)
            if magnet_config is not None
            else None
        )

    # ---- 内部工具 ----
    def _log(self, msg, level="INFO"):
        self.on_log(level, f"[Mock] {msg}")

    def _push_state(self):
        self.on_state(self._state)

    def _on_magnet_status(self, status: MagnetStatus):
        self._state.magnet_on = status.enabled
        self._log(
            f"电磁铁状态={'吸合' if status.enabled else '释放'}, 原因={status.reason}"
        )
        self._push_state()

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
        if self._magnet is not None:
            try:
                self._magnet.connect()
            except Exception as error:
                self._state.connected = False
                self._log(f"GPIO17 电磁铁连接失败: {error}", "ERROR")
                self._push_state()
                return False, f"电磁铁连接失败: {error}"
        self._state.connected = True
        self._state.simulation = True
        mode = "机械臂模拟 + GPIO17 电磁铁硬件" if self._magnet else "模拟模式"
        self._log(f"已连接 ({mode})")
        self._push_state()
        return True, f"{mode}已连接"

    def disconnect(self):
        self.abort()
        if self._magnet is not None:
            self._magnet.close()
        self._state.connected = False
        self._state.torque_enabled = False
        self._state.magnet_on = False
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
        if self._magnet is None:
            self._state.magnet_on = on
            self._log(f"电磁铁{'通电吸附' if on else '断电释放'}")
            self._push_state()
            return True, "ok (模拟)"
        try:
            status = self._magnet.set_enabled(bool(on))
        except Exception as error:
            self._log(f"电磁铁控制失败: {error}", "ERROR")
            return False, str(error)
        return True, status.reason

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

    def start_task(self, task_mode):
        if task_mode not in ("task1", "task2"):
            return False, f"未知任务模式: {task_mode}"

        def _task():
            stages = [
                ("DETECTING", [("采集图像", 0.8), ("检测碎片 x4", 1.0)]),
                ("SOLVING", [("求解拼图布局", 1.0), ("规划路径", 0.5)]),
                (
                    "EXECUTING",
                    [
                        ("拾取碎片 1/4", 1.0),
                        ("放置碎片 1/4", 0.8),
                        ("拾取碎片 2/4", 1.0),
                        ("放置碎片 2/4", 0.8),
                        ("...", 1.5),
                    ],
                ),
                ("DONE", [("声光提示", 0.5)]),
            ]
            for state, steps in stages:
                self._state.task_state = state
                self._push_state()
                if not self._fake_steps(steps, state):
                    self.on_task_done(False, "任务被中止")
                    return
            self._push_state()
            self.on_task_done(True, f"{task_mode} 完成, 用时 42s")

        if not self._run_async(_task):
            return False, "已有任务在运行"
        return True, "任务已启动"

    def abort(self):
        self._abort_flag.set()
        return True, "中止信号已发送"

    def emergency_stop(self):
        self._abort_flag.set()
        if self._magnet is not None:
            try:
                self._magnet.emergency_off()
            except Exception as error:
                self._log(f"急停断开电磁铁失败: {error}", "ERROR")
        self._state.torque_enabled = False
        self._state.task_state = "IDLE"
        self._push_state()
        self._log("急停! 力矩已失能, 电磁铁已请求断电", "ERROR")


def _magnet_config_from_environment() -> MagnetConfig | None:
    if os.environ.get("EPUZZLE_MAGNET_ENABLED", "0") != "1":
        return None
    try:
        line_offset = int(os.environ["EPUZZLE_MAGNET_GPIO_LINE"])
        max_on_seconds = float(os.environ["EPUZZLE_MAGNET_MAX_ON_SECONDS"])
    except (KeyError, ValueError) as error:
        raise RuntimeError("电磁铁 GPIO17 配置不完整") from error
    active_high = os.environ.get("EPUZZLE_MAGNET_ACTIVE_HIGH")
    if active_high not in {"0", "1"}:
        raise RuntimeError("EPUZZLE_MAGNET_ACTIVE_HIGH 必须为 0 或 1")
    return MagnetConfig(
        chip_path=os.environ.get("EPUZZLE_MAGNET_GPIO_CHIP", "/dev/gpiochip0"),
        line_offset=line_offset,
        active_high=active_high == "1",
        max_on_seconds=max_on_seconds,
    )


def create_backend(prefer_real: bool = False) -> RobotBackend:
    """当前机械臂保持模拟；可按环境配置启用真实 GPIO17 电磁铁。"""
    if prefer_real:
        raise RuntimeError("真实机械臂 Backend 尚未实现")
    return MockBackend(magnet_config=_magnet_config_from_environment())
