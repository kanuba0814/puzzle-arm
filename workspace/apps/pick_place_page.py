#!/usr/bin/env python3
# pyright: reportMissingImports=false, reportAttributeAccessIssue=false
# ruff: noqa: I001

"""正式赛场 7 英寸触屏 GUI（1024×600，中文，fb0 + evdev）。

设计要点：
- 四页结构：比赛页、运行监视页、手动页、Debug 页；
  常驻顶栏（状态色块 + 全局急停）与常驻底栏（大字状态 + 耗时/进度）。
- 显式页面状态机 PageState：IDLE / RUNNING / FAULT / ESTOP；
  按钮可用性由状态 + 后端能力表唯一决定。
- 动作全部通过 ``workspace/tools/supervisor_run.py`` 执行，合同见
  ``docs/10_接口与数据契约.md`` 与 ``data_contracts/supervisor_run_*.schema.json``：
  页面只读结构化 status/result JSON，绝不解析 stdout 最后一行。
- 后端复杂度集中在 ``integration.competition_backend``；GUI 只处理统一
  ``BackendRequest``（argv + 超时 + status/result 路径）。
- 急停顺序：终止子进程 → 请求磁铁 OFF → 请求五轴断力矩 → 锁存 ESTOP；
  任何一步未确认都保持 ESTOP 并提示人工断电。
- 页面自身不常驻串口/GPIO；串口/GPIO 只由短命子进程持有。
- 中文字体缺失时在屏幕上显式报错，而不是静默显示方块。
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from enum import Enum
from pathlib import Path
from typing import Any, TypedDict

WORKSPACE = Path(__file__).resolve().parents[1]
PROJECT_ROOT = WORKSPACE.parent
GUI_HELPERS = WORKSPACE / "rpi_gui_snapshot"
sys.path.insert(0, str(GUI_HELPERS))
sys.path.insert(0, str(WORKSPACE / "src"))

import pygame  # noqa: E402
from gui_fb import (  # noqa: E402
    C_BG,
    C_BLUE,
    C_BTN,
    C_DIM,
    C_GREEN,
    C_LINE,
    C_ORANGE,
    C_PANEL,
    C_RED,
    C_TEXT,
    Button,
    EvdevTouchReader,
    LinuxFramebuffer,
    find_font,
    load_font,
    touch_event_to_mouse_event,
)
from integration.competition_backend import (  # noqa: E402
    BackendCapability,
    CapabilityAvailability,
    CompetitionBackend,
    JOINT_ORDER,
    create_default_backend,
    pick_place_lock_reason,
    robot_motion_lock_reason,
)

TOOLS = WORKSPACE / "tools"
DEFAULT_PYTHON = Path("/opt/epuzzle/venv/bin/python")
CALIBRATION_DIR = WORKSPACE / "calibration"
HOMOGRAPHY_PATH = CALIBRATION_DIR / "table_homography_gui_20260731.npy"
PARK_PATH = CALIBRATION_DIR / "park_pose_single_arm_20260731.json"
DEFAULT_ROBOT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_ROBOT_MOTION_LOCK = Path("/etc/epuzzle/robot-motion.lock.json")

SCREEN_SIZE = (1024, 600)
TOP_BAR_HEIGHT = 56
BOTTOM_BAR_TOP = 536
PREVIEW_SOURCE_SIZE = (1280, 720)

COMPETITION_TIMEOUT_S = 120.0

# 手动页离散步长。
JOINT_STEP_OPTIONS = (0.1, 0.5, 1.0, 5.0, 10.0)
CARTESIAN_STEP_MM_OPTIONS = (0.1, 0.5, 1.0, 5.0, 10.0)
CARTESIAN_STEP_DEG_OPTIONS = (0.5, 1.0, 5.0, 10.0, 15.0)
MAGNET_DURATION_OPTIONS = (1.0, 3.0, 5.0)

STATE_LABELS = {
    "IDLE": "空闲",
    "RUNNING": "运行中",
    "FAULT": "故障",
    "ESTOP": "急停锁定",
}
STATE_COLORS = {
    "IDLE": C_GREEN,
    "RUNNING": C_BLUE,
    "FAULT": C_RED,
    "ESTOP": (150, 40, 40),
}
MESSAGE_COLORS = {"INFO": C_TEXT, "WARN": C_ORANGE, "ERROR": C_RED}
C_DISABLED = (44, 47, 56)
C_LOCK = (58, 50, 36)


class PageState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class PageEntry(TypedDict):
    buttons: list[PageButton]
    draw: Callable[[], None]


class ActionInfo(TypedDict):
    command: str
    label: str
    argv: list[str]
    status_path: Path
    result_path: Path
    timeout_s: float
    started_monotonic: float


class PageButton(Button):
    """带启用/禁用与可见性语义的按钮；禁用时置灰并忽略触摸。

    ``_tab`` 语义：
    - ``"joint"`` 或 ``"cartesian"``：仅在手动页对应页签下可见；
    - ``None``：两个页签共享，必须始终可见。
    """

    def __init__(
        self,
        rect,
        text,
        on_click,
        color=C_BTN,
        font_key="button",
        *,
        action=True,
        tab: str | None = None,
    ):
        super().__init__(rect, text, on_click, color=color, font_key=font_key)
        self.action = action
        self.enabled = True
        self.visible = True
        self.tab = tab

    def handle(self, event) -> bool:
        if not self.enabled or not self.visible:
            if event.type == pygame.MOUSEBUTTONUP:
                self.pressed = False
            return False
        return super().handle(event)

    def draw(self, surface, fonts) -> None:
        if not self.visible:
            return
        if self.enabled:
            super().draw(surface, fonts)
            return
        pygame.draw.rect(surface, C_DISABLED, self.rect, border_radius=8)
        pygame.draw.rect(surface, C_LINE, self.rect, width=1, border_radius=8)
        image = fonts[self.font_key].render(self.text, True, C_DIM)
        surface.blit(image, image.get_rect(center=self.rect.center))


def display_float(value: object) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError):
        return 0.0
    return number if math.isfinite(number) else 0.0


class PickPlacePage:
    def __init__(self, *, backend: CompetitionBackend | None = None) -> None:
        pygame.init()
        self.framebuffer: LinuxFramebuffer | None = None
        self.touch_reader: EvdevTouchReader | None = None
        framebuffer_path = os.environ.get("EP_GUI_FRAMEBUFFER")
        if framebuffer_path:
            size = LinuxFramebuffer.read_size(framebuffer_path)
            self.screen = pygame.display.set_mode(size, depth=32)
            self.framebuffer = LinuxFramebuffer(framebuffer_path, size)
            self.touch_reader = EvdevTouchReader(
                size, os.environ.get("EP_GUI_TOUCH_DEVICE")
            )
        else:
            self.screen = pygame.display.set_mode(SCREEN_SIZE)
        self.width, self.height = self.screen.get_size()

        font_override = os.environ.get("EP_GUI_FONT")
        font_path = font_override if font_override else find_font()
        self.font_missing = font_path is None
        self.fonts = {
            "title": load_font(font_path, 26),
            "big": load_font(font_path, 24),
            "button": load_font(font_path, 20),
            "small": load_font(font_path, 18),
            "tiny": load_font(font_path, 15),
        }

        self.python_path = os.environ.get("EP_GUI_PYTHON") or (
            str(DEFAULT_PYTHON) if DEFAULT_PYTHON.is_file() else sys.executable
        )
        self.runtime_dir = Path(
            os.environ.get("EP_GUI_RUNTIME_DIR", str(WORKSPACE / "runtime"))
        )
        self.detection_dir = self.runtime_dir / "red_piece_detection"
        self.pieces_path = self.detection_dir / "pieces.json"
        self.annotated_path = self.detection_dir / "annotated.jpg"
        self.gui_run_dir = self.runtime_dir / "gui"
        self.robot_port = Path(
            os.environ.get("EPUZZLE_ROBOT_PORT", str(DEFAULT_ROBOT_PORT))
        )
        self.robot_motion_lock = Path(
            os.environ.get("EPUZZLE_ROBOT_MOTION_LOCK", str(DEFAULT_ROBOT_MOTION_LOCK))
        )

        self.backend = backend or create_default_backend(
            python_path=self.python_path,
            robot_port=self.robot_port,
            robot_motion_lock=self.robot_motion_lock,
            pieces_path=self.pieces_path,
        )
        self.capabilities: dict[str, BackendCapability] = self.backend.capabilities()

        self.page = "competition"
        self.state = PageState.IDLE
        self.message_level = "INFO"
        self.message_text = "正式 PARK 已冻结；比赛/手动后端按能力表锁定"
        self.logs: list[tuple[str, str]] = []
        self.pieces: list[dict[str, object]] = []
        self.selected_index = 0
        self.target_rotation_deg = 0.0
        self.preview: pygame.Surface | None = None
        self.last_result: dict[str, object] | None = None
        self.self_checks: list[tuple[str, str, str]] = []

        self.worker: threading.Thread | None = None
        self.process: subprocess.Popen[str] | None = None
        self.messages: queue.Queue[tuple[object, ...]] = queue.Queue()
        self.current_action: ActionInfo | None = None
        self.heartbeat: dict[str, object] | None = None
        self._heartbeat_read_at = 0.0
        self._estop_reset_pending = False
        self.pointer_pos: tuple[int, int] | None = None
        self.quit_requested = False

        # 比赛页状态。
        self.competition_mode: str | None = None
        self.competition_started_monotonic: float | None = None
        # 动作超时覆盖（仅用于测试/调试）。
        self.timeout_overrides: dict[str, float] = {}

        # 手动页状态。
        self.manual_unlocked = False
        self.manual_tab = "joint"
        self.selected_joint_index = 0
        self.joint_step_deg = 1.0
        self.selected_cartesian_axis = "x"
        self.cartesian_step_value = 1.0
        self.cartesian_is_angle = False
        self.magnet_duration_s = 3.0
        self.joint_readbacks: dict[str, float] = {}
        self.tcp_readback: dict[str, float] = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        # Debug 页本次会话开关（仅内存，不修改环境或文件）。
        self.debug_overlays = False
        self.debug_verbose_logs = False
        self.debug_save_evidence = False
        self.debug_generic_vision = False
        self.debug_texture = False
        self.debug_act = False

        self.estop_button = PageButton(
            (self.width - 128, 4, 120, 48),
            "急停",
            self.emergency_stop,
            color=C_RED,
            font_key="button",
            action=False,
        )
        self.reset_button = PageButton(
            (688, 542, 320, 48),
            "回 PARK 并复位",
            self.reset_from_estop,
            color=C_RED,
            font_key="button",
            action=False,
        )
        self.pages = {
            "competition": self._build_competition_page(),
            "run": self._build_run_page(),
            "manual": self._build_manual_page(),
            "debug": self._build_debug_page(),
        }
        self.piece_list_rect = pygame.Rect(616, 108, 392, 276)
        self._reload_detection()
        self.run_self_checks()
        self._refresh_capabilities()
        self._refresh_button_states()

    # ---------------- 页面构建 ----------------

    def _build_competition_page(self) -> PageEntry:
        buttons: list[PageButton] = []
        cards = [
            ("competition-self-color", "自备纯色碎片", 50, C_GREEN),
            ("competition-white", "现场白色碎片", 20, C_BLUE),
            ("competition-card", "现场扑克牌碎片", 30, C_ORANGE),
        ]
        for index, (mode, title, score, color) in enumerate(cards):
            y = 72 + index * 110
            buttons.append(
                PageButton(
                    (16, y, 420, 96),
                    f"{title} ({score} 分)",
                    lambda m=mode: self._select_competition_mode(m),
                    color=color,
                    action=False,
                )
            )
            buttons.append(
                PageButton(
                    (444, y + 28, 80, 48),
                    "选择",
                    lambda m=mode: self._select_competition_mode(m),
                    color=color,
                    action=False,
                )
            )
        buttons.append(
            PageButton(
                (540, 392, 460, 96),
                "一键启动",
                self.start_competition,
                color=C_GREEN,
                font_key="big",
                action=True,
            )
        )
        buttons.extend(self._page_nav_buttons(y=494))
        return {"buttons": buttons, "draw": self._draw_competition}

    def _build_run_page(self) -> PageEntry:
        buttons: list[PageButton] = [
            PageButton(
                (16, 72, 140, 48),
                "安全中止",
                self.safe_abort,
                color=C_ORANGE,
                action=True,
            )
        ]
        buttons.extend(self._page_nav_buttons(y=494))
        return {"buttons": buttons, "draw": self._draw_run}

    def _build_manual_page(self) -> PageEntry:
        buttons: list[PageButton] = []
        buttons.append(
            PageButton(
                (16, 72, 120, 44),
                "关节",
                lambda: self._set_manual_tab("joint"),
                color=C_BTN,
                action=False,
            )
        )
        buttons.append(
            PageButton(
                (144, 72, 120, 44),
                "末端",
                lambda: self._set_manual_tab("cartesian"),
                color=C_BTN,
                action=False,
            )
        )
        buttons.append(
            PageButton(
                (280, 72, 180, 44),
                "解锁维护控制",
                self.manual_unlock,
                color=C_ORANGE,
                action=True,
            )
        )

        # 关节页签。
        row_height = 56
        for index, _feature in enumerate(JOINT_ORDER):
            y = 132 + index * row_height
            buttons.append(
                PageButton(
                    (16, y, 56, 44),
                    "-",
                    lambda i=index: self._jog_joint(i, -1),
                    color=C_BTN,
                    action=True,
                    tab="joint",
                )
            )
            buttons.append(
                PageButton(
                    (80, y, 56, 44),
                    "+",
                    lambda i=index: self._jog_joint(i, +1),
                    color=C_BTN,
                    action=True,
                    tab="joint",
                )
            )
        for index, step in enumerate(JOINT_STEP_OPTIONS):
            buttons.append(
                PageButton(
                    (160 + index * 66, 408, 60, 44),
                    f"{step:g}°",
                    lambda s=step: self._set_joint_step(s),
                    color=C_BTN,
                    action=False,
                    tab="joint",
                )
            )

        # 末端页签。
        axes = [("x", "X"), ("y", "Y"), ("z", "Z"), ("pitch", "Pitch"), ("yaw", "Yaw")]
        for index, (axis, label) in enumerate(axes):
            y = 132 + index * row_height
            buttons.append(
                PageButton(
                    (16, y, 56, 44),
                    "-",
                    lambda a=axis: self._jog_cartesian(a, -1),
                    color=C_BTN,
                    action=True,
                    tab="cartesian",
                )
            )
            buttons.append(
                PageButton(
                    (80, y, 56, 44),
                    "+",
                    lambda a=axis: self._jog_cartesian(a, +1),
                    color=C_BTN,
                    action=True,
                    tab="cartesian",
                )
            )
            buttons.append(
                PageButton(
                    (148, y, 90, 44),
                    label,
                    lambda a=axis: self._select_cartesian_axis(a),
                    color=C_BTN,
                    action=False,
                    tab="cartesian",
                )
            )
        for index, step in enumerate(CARTESIAN_STEP_MM_OPTIONS):
            buttons.append(
                PageButton(
                    (160 + index * 70, 408, 66, 44),
                    f"{step:g}mm",
                    lambda s=step: self._set_cartesian_step(s, is_angle=False),
                    color=C_BTN,
                    action=False,
                    tab="cartesian",
                )
            )
        for index, step in enumerate(CARTESIAN_STEP_DEG_OPTIONS):
            buttons.append(
                PageButton(
                    (160 + index * 70, 456, 66, 44),
                    f"{step:g}°",
                    lambda s=step: self._set_cartesian_step(s, is_angle=True),
                    color=C_BTN,
                    action=False,
                    tab="cartesian",
                )
            )

        # 公共控件。
        buttons.append(
            PageButton(
                (540, 72, 120, 44),
                "读取状态",
                self.manual_read_state,
                color=C_BTN,
                action=True,
            )
        )
        buttons.append(
            PageButton(
                (672, 72, 120, 44),
                "回 PARK",
                self.park,
                color=C_BTN,
                action=True,
            )
        )
        buttons.append(
            PageButton(
                (804, 72, 140, 44),
                "五轴断力矩",
                self.torque_off,
                color=C_RED,
                action=True,
            )
        )
        for index, duration in enumerate(MAGNET_DURATION_OPTIONS):
            buttons.append(
                PageButton(
                    (540 + index * 110, 132, 100, 44),
                    f"{duration:g}s",
                    lambda d=duration: self._set_magnet_duration(d),
                    color=C_BTN,
                    action=False,
                )
            )
        buttons.append(
            PageButton(
                (540, 188, 180, 52),
                "启动电磁铁",
                self.manual_magnet_pulse,
                color=C_ORANGE,
                action=True,
            )
        )
        buttons.append(
            PageButton(
                (740, 188, 180, 52),
                "立即断电",
                self.manual_magnet_off,
                color=C_RED,
                action=True,
            )
        )

        buttons.extend(self._page_nav_buttons(y=494))
        return {"buttons": buttons, "draw": self._draw_manual}

    def _build_debug_page(self) -> PageEntry:
        buttons: list[PageButton] = []
        tool_specs = [
            ("刷新检测", self.detect, C_BLUE),
            ("红色基线", self.red_baseline, C_GREEN),
            ("快速巡检", self.sweep_shortest, C_GREEN),
            ("通用 Shadow", self.puzzle_shadow, C_BLUE),
            ("到 20mm", self.pregrasp_selected, C_BLUE),
            ("回 PARK", self.park, C_BTN),
            ("重新自检", self.run_self_checks, C_BTN),
        ]
        for index, (text, callback, color) in enumerate(tool_specs):
            buttons.append(
                PageButton(
                    (16 + (index % 4) * 160, 72 + (index // 4) * 56, 150, 48),
                    text,
                    callback,
                    color=color,
                    action=True,
                )
            )
        switches = [
            ("轮廓覆盖", "debug_overlays"),
            ("详细日志", "debug_verbose_logs"),
            ("保存证据", "debug_save_evidence"),
            ("generic vision", "debug_generic_vision"),
            ("texture", "debug_texture"),
            ("ACT", "debug_act"),
        ]
        for index, (label, attr) in enumerate(switches):
            buttons.append(
                PageButton(
                    (16 + (index % 3) * 200, 200 + (index // 3) * 52, 190, 44),
                    label,
                    lambda a=attr: self._toggle_debug_switch(a),
                    color=C_BTN,
                    action=False,
                )
            )
        buttons.extend(self._page_nav_buttons(y=494))
        return {"buttons": buttons, "draw": self._draw_debug}

    def _page_nav_buttons(self, y: int) -> list[PageButton]:
        nav = [
            ("比赛", "competition"),
            ("运行", "run"),
            ("手动", "manual"),
            ("Debug", "debug"),
        ]
        buttons = []
        width = 120
        gap = 12
        total = len(nav) * width + (len(nav) - 1) * gap
        start_x = (self.width - total) // 2
        for index, (label, page) in enumerate(nav):
            buttons.append(
                PageButton(
                    (start_x + index * (width + gap), y, width, 44),
                    label,
                    lambda p=page: self._goto(p),
                    color=C_BLUE if page == "manual" else C_BTN,
                    action=False,
                )
            )
        return buttons

    def _goto(self, page: str) -> None:
        self.page = page
        for entry in self.pages.values():
            for button in entry["buttons"]:
                button.pressed = False
        if page != "manual":
            self.manual_unlocked = False

    def _set_manual_tab(self, tab: str) -> None:
        self.manual_tab = tab

    def _refresh_capabilities(self) -> None:
        self.capabilities = self.backend.capabilities()

    # ---------------- 消息与日志 ----------------

    def set_message(self, level: str, text: str) -> None:
        self.message_level = level
        self.message_text = text
        self._log(level, text)

    def _log(self, level: str, text: str) -> None:
        self.logs.append((level, text))
        if len(self.logs) > 80:
            self.logs = self.logs[-80:]

    # ---------------- 动作执行 ----------------

    def _start_backend_request(
        self,
        command_id: str,
        label: str,
        parameters: dict[str, Any],
        *,
        allow_in_estop: bool = False,
    ) -> None:
        if self.state == PageState.RUNNING or (
            self.state == PageState.ESTOP and not allow_in_estop
        ):
            self.set_message("WARN", "当前状态不允许新动作")
            return
        if self.worker is not None and self.worker.is_alive():
            self.set_message("WARN", "上一动作收尾中，请稍候")
            return
        self._refresh_capabilities()
        cap = self.capabilities.get(command_id)
        if cap is None:
            self.set_message("ERROR", f"未知命令: {command_id}")
            return
        if cap.availability is CapabilityAvailability.LOCKED:
            self.set_message("ERROR", f"{label} 锁定: {cap.reason}")
            return
        if cap.availability is CapabilityAvailability.UNAVAILABLE:
            self.set_message("ERROR", f"{label} 未接入后端: {cap.reason}")
            return
        self.gui_run_dir.mkdir(parents=True, exist_ok=True)
        status_path = self.gui_run_dir / "status.json"
        result_path = self.gui_run_dir / f"result_{command_id}.json"
        for stale in (status_path, result_path):
            with contextlib.suppress(OSError):
                stale.unlink()
        try:
            request = self.backend.build_request(
                command_id,
                parameters,
                status_path=status_path,
                result_path=result_path,
            )
        except (ValueError, RuntimeError) as error:
            self.set_message("ERROR", f"{label} 请求构造失败: {error}")
            return
        timeout_s = (
            float(self.timeout_overrides[command_id])
            if command_id in self.timeout_overrides
            else request.timeout_s
        )
        self.current_action = ActionInfo(
            command=request.command_id,
            label=label,
            argv=request.argv,
            status_path=request.status_path,
            result_path=request.result_path,
            timeout_s=timeout_s,
            started_monotonic=time.monotonic(),
        )
        self.heartbeat = None
        self.state = PageState.RUNNING
        # 自动比赛或手动动作开始时清除维护解锁。
        if command_id != "manual-magnet-off":
            self.manual_unlocked = False
        self.set_message("INFO", f"{label}：执行中")

        def worker() -> None:
            returncode: int | None = None
            timed_out = False
            error = ""
            try:
                self.process = subprocess.Popen(
                    request.argv,
                    cwd=PROJECT_ROOT,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                )
            except OSError as os_error:
                self.messages.put(("done", None, False, f"启动失败：{os_error}"))
                return
            deadline = time.monotonic() + timeout_s
            while self.process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if self.process.poll() is None:
                self.process.kill()
                timed_out = True
            output, _ = self.process.communicate()
            returncode = self.process.returncode
            self.process = None
            tail = (output or "").strip()[-300:]
            if tail:
                self.messages.put(("output", tail))
            self.messages.put(("done", returncode, timed_out, error))

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def detect(self) -> None:
        self._start_backend_request("detect", "刷新检测", {})

    def red_baseline(self) -> None:
        self._start_backend_request("red-baseline", "红色基线", {})

    def sweep_shortest(self) -> None:
        if not self.pieces:
            self.set_message("WARN", "没有检测结果：请先刷新检测")
            return
        self._start_backend_request("sweep-shortest", "快速最短巡检", {})

    def puzzle_shadow(self) -> None:
        if not self.pieces:
            self.set_message("WARN", "没有检测结果：请先刷新检测")
            return
        self._start_backend_request(
            "puzzle-shadow",
            "通用拼图 Shadow",
            {"target_angle_deg": self.target_rotation_deg},
        )

    def pregrasp_selected(self) -> None:
        if not self.pieces:
            self.set_message("WARN", "没有检测结果：请先刷新检测")
            return
        self._start_backend_request(
            "pregrasp",
            "到选中碎片 20mm",
            {"piece_index": self.selected_index},
        )

    def park(self) -> None:
        self._start_backend_request("park", "回正式 PARK", {})

    def torque_off(self) -> None:
        if self.state in (PageState.RUNNING, PageState.ESTOP):
            self.set_message("WARN", "当前状态不允许断力矩操作")
            return
        cap = self.backend.capabilities().get("park")
        if cap is not None and cap.availability is not CapabilityAvailability.READY:
            self.set_message("ERROR", f"断力矩前置条件不满足: {cap.reason}")
            return
        self.set_message("INFO", "五轴断力矩：执行中")

        def worker() -> None:
            try:
                completed = subprocess.run(
                    [self.python_path, str(TOOLS / "emergency_off_single_arm.py")],
                    cwd=PROJECT_ROOT,
                    check=False,
                    timeout=20,
                )
                ok = completed.returncode == 0
                self.messages.put(
                    ("plain", ok, "五轴断力矩" + ("完成" if ok else "失败"))
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                self.messages.put(("plain", False, f"断力矩失败：{error}"))

        threading.Thread(target=worker, daemon=True).start()

    def _select_competition_mode(self, mode: str) -> None:
        self.competition_mode = mode
        self._refresh_button_states()
        cap = self.backend.capabilities().get(mode)
        if cap is None or cap.availability is not CapabilityAvailability.READY:
            level = (
                "WARN"
                if cap and cap.availability is CapabilityAvailability.LOCKED
                else "ERROR"
            )
            reason = cap.reason if cap else "未知能力"
            self.set_message(level, f"{mode} 未就绪：{reason}")
        else:
            self.set_message("INFO", f"已选择 {mode}，可一键启动")

    def start_competition(self) -> None:
        if self.competition_mode is None:
            self.set_message("WARN", "请先选择比赛模式")
            return
        self.competition_started_monotonic = time.monotonic()
        self._start_backend_request(
            self.competition_mode,
            "比赛自动流程",
            {},
        )

    def safe_abort(self) -> None:
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            deadline = time.monotonic() + 1.0
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            if process.poll() is None:
                process.kill()
                process.wait()
        self.set_message("WARN", "安全中止：已终止当前子进程，磁铁仍请求 OFF")
        self._request_magnet_off()

    def manual_unlock(self) -> None:
        if self.state in (PageState.RUNNING, PageState.ESTOP, PageState.FAULT):
            self.set_message("WARN", "当前状态不允许解锁维护控制")
            return
        if self.backend.robot_motion_lock_reason() is not None:
            self.set_message("ERROR", "运动安全锁存在，禁止解锁维护控制")
            return
        if not self.backend.robot_available():
            self.set_message("ERROR", "机械臂设备离线，禁止解锁维护控制")
            return
        self.manual_unlocked = True
        self._refresh_button_states()
        self.set_message("INFO", "维护控制已解锁：动作仍受后端能力/门禁限制")

    def manual_read_state(self) -> None:
        self._start_backend_request("manual-read-state", "读取状态", {})

    def _jog_joint(self, joint_index: int, direction: int) -> None:
        if not self.manual_unlocked:
            self.set_message("WARN", "请先解锁维护控制")
            return
        feature = JOINT_ORDER[joint_index]
        self._start_backend_request(
            "manual-jog-joint",
            f"点动 {feature}",
            {"joint_feature": feature, "delta_deg": direction * self.joint_step_deg},
        )

    def _set_joint_step(self, step: float) -> None:
        self.joint_step_deg = step

    def _select_cartesian_axis(self, axis: str) -> None:
        self.selected_cartesian_axis = axis
        self.cartesian_is_angle = axis in {"pitch", "yaw"}
        self.cartesian_step_value = 1.0 if not self.cartesian_is_angle else 5.0

    def _set_cartesian_step(self, step: float, *, is_angle: bool) -> None:
        self.cartesian_step_value = step
        self.cartesian_is_angle = is_angle

    def _jog_cartesian(self, axis: str, direction: int) -> None:
        if not self.manual_unlocked:
            self.set_message("WARN", "请先解锁维护控制")
            return
        self._start_backend_request(
            "manual-jog-cartesian",
            f"点动 {axis}",
            {
                "axis": axis,
                "delta": direction * self.cartesian_step_value,
                "frame": "robot_base_mm",
            },
        )

    def _set_magnet_duration(self, duration: float) -> None:
        self.magnet_duration_s = duration

    def manual_magnet_pulse(self) -> None:
        if not self.manual_unlocked:
            self.set_message("WARN", "电磁铁脉冲需先解锁维护控制")
            return
        self._start_backend_request(
            "manual-magnet-pulse",
            "电磁铁脉冲",
            {"duration_s": self.magnet_duration_s},
        )

    def manual_magnet_off(self) -> None:
        self._start_backend_request("manual-magnet-off", "电磁铁立即断电", {})

    def _request_magnet_off(self) -> None:
        """尽最大努力请求磁铁 OFF；不阻塞 GUI。"""
        try:
            cap = self.backend.capabilities().get("manual-magnet-off")
            if cap is None or cap.availability is not CapabilityAvailability.READY:
                self.set_message("WARN", "无法请求电磁铁 OFF：后端未就绪")
                return
            request = self.backend.build_request(
                "manual-magnet-off",
                {},
                status_path=self.gui_run_dir / "magnet_off_status.json",
                result_path=self.gui_run_dir / "magnet_off_result.json",
            )
            subprocess.Popen(
                request.argv,
                cwd=PROJECT_ROOT,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, ValueError, RuntimeError) as error:
            self.set_message("ERROR", f"请求电磁铁 OFF 失败: {error}")

    def _motion_lock_reason(self) -> str | None:
        return robot_motion_lock_reason(self.robot_motion_lock)

    def _robot_available(self) -> bool:
        return self.backend.robot_available()

    def _robot_motion_ready(self) -> bool:
        return self._robot_available() and self._motion_lock_reason() is None

    # ---------------- 急停与复位 ----------------

    def emergency_stop(self) -> None:
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            deadline = time.monotonic() + 1.0
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            if process.poll() is None:
                process.kill()
                process.wait()
        self.state = PageState.ESTOP
        self._estop_reset_pending = False
        self.manual_unlocked = False
        self.set_message(
            "ERROR",
            "急停：已终止动作，正在请求磁铁 OFF 与五轴断力矩",
        )
        self._request_magnet_off()

        def off_worker() -> None:
            torque_ok = False
            try:
                completed = subprocess.run(
                    [self.python_path, str(TOOLS / "emergency_off_single_arm.py")],
                    cwd=PROJECT_ROOT,
                    check=False,
                    timeout=20,
                )
                torque_ok = completed.returncode == 0
            except (OSError, subprocess.TimeoutExpired) as error:
                self.messages.put(("estop-off", False, f"断力矩异常：{error}"))
                return
            self.messages.put(
                (
                    "estop-off",
                    torque_ok,
                    (
                        "急停锁定：磁铁/断力矩已确认；处置后回 PARK 并复位"
                        if torque_ok
                        else "急停锁定：五轴断力矩未确认；请人工支撑机械臂并切断舵机电源"
                    ),
                )
            )

        threading.Thread(target=off_worker, daemon=True).start()

    def reset_from_estop(self) -> None:
        if self.state != PageState.ESTOP:
            return
        self._estop_reset_pending = True
        self._start_backend_request("park", "回 PARK 并复位", {}, allow_in_estop=True)
        if self.state != PageState.RUNNING:
            self._estop_reset_pending = False

    # ---------------- 选择与目标角 ----------------

    def select(self, delta: int) -> None:
        if not self.pieces:
            self.set_message("WARN", "没有检测结果")
            return
        self.selected_index = (self.selected_index + delta) % len(self.pieces)
        piece = self.pieces[self.selected_index]
        self.set_message(
            "INFO",
            f"已选 {piece.get('piece_id', '?')} "
            f"({self.selected_index + 1}/{len(self.pieces)})",
        )

    def rotate_target(self, delta: float) -> None:
        self.target_rotation_deg = (self.target_rotation_deg + delta) % 180.0
        self.set_message("INFO", f"目标放置角度 {self.target_rotation_deg:.0f}°")

    def _toggle_debug_switch(self, attr: str) -> None:
        if attr in {"debug_generic_vision", "debug_texture", "debug_act"}:
            cap_id = {
                "debug_generic_vision": "detect",
                "debug_texture": "competition-card",
                "debug_act": "competition-card",
            }.get(attr)
            cap = self.capabilities.get(cap_id or "")
            if cap is None or cap.availability is CapabilityAvailability.UNAVAILABLE:
                self.set_message("WARN", f"{attr} 后端未就绪，开关仅在本次会话保留")
                return
        current = getattr(self, attr)
        setattr(self, attr, not current)
        self.set_message("INFO", f"{attr}={'开' if not current else '关'}")

    # ---------------- 检测数据 ----------------

    def _reload_detection(self) -> None:
        try:
            payload = json.loads(self.pieces_path.read_text(encoding="utf-8"))
            pieces = payload.get("pieces", [])
            self.pieces = pieces if isinstance(pieces, list) else []
        except (OSError, json.JSONDecodeError):
            self.pieces = []
        if self.selected_index >= len(self.pieces):
            self.selected_index = 0
        try:
            image = pygame.image.load(str(self.annotated_path))
            self.preview = image.convert()
        except (pygame.error, OSError):
            self.preview = None

    def _selected_piece(self) -> dict[str, object] | None:
        if not self.pieces or self.selected_index >= len(self.pieces):
            return None
        return self.pieces[self.selected_index]

    def _piece_center(self, piece: dict[str, object]) -> tuple[float, float]:
        center = piece.get("center_table_mm", [0.0, 0.0])
        if isinstance(center, list) and len(center) == 2:
            return display_float(center[0]), display_float(center[1])
        return 0.0, 0.0

    def _piece_pixel(self, piece: dict[str, object]) -> tuple[float, float] | None:
        diagnostics = piece.get("diagnostics")
        if not isinstance(diagnostics, dict):
            return None
        pixel = diagnostics.get("center_pixel")
        if isinstance(pixel, list) and len(pixel) == 2:
            return display_float(pixel[0]), display_float(pixel[1])
        return None

    # ---------------- 结果/状态文件 ----------------

    def _read_json_file(self, path: Path) -> dict[str, object] | None:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        return payload if isinstance(payload, dict) else None

    def _read_heartbeat(self) -> None:
        now = time.monotonic()
        if now - self._heartbeat_read_at < 0.25:
            return
        self._heartbeat_read_at = now
        action = self.current_action
        if action is None:
            return
        payload = self._read_json_file(action["status_path"])
        if payload is not None:
            self.heartbeat = payload

    def _finish_action(
        self, returncode: int | None, timed_out: bool, error: str
    ) -> None:
        action = self.current_action
        self.current_action = None
        self.heartbeat = None
        if action is None:
            return
        command = action["command"]
        label = action["label"]
        timeout_s = action["timeout_s"]
        if self.state == PageState.ESTOP and not self._estop_reset_pending:
            self._log("INFO", f"{label}：进程已随急停结束")
            return

        result_path = action["result_path"]
        payload = self._read_json_file(result_path)
        reset = self._estop_reset_pending
        self._estop_reset_pending = False

        def fail(text: str) -> None:
            self.state = PageState.ESTOP if reset else PageState.FAULT
            # 故障时自动取消维护解锁。
            self.manual_unlocked = False
            suffix = "；请人工检查后重新复位" if reset else "；建议回 PARK → 刷新检测"
            self.set_message("ERROR", f"{label}失败：{text}{suffix}")

        if error:
            fail(error)
            return
        if timed_out:
            fail(f"超过 {timeout_s:.0f}s 上限，已终止")
            return
        if payload is None:
            fail(f"结果文件缺失或损坏（returncode={returncode}）")
            return
        self.last_result = payload
        result = str(payload.get("result", ""))
        duration = display_float(payload.get("duration_s", 0.0))
        detail = str(payload.get("message", ""))
        if result == "PASS" and returncode == 0:
            self.state = PageState.IDLE
            if reset:
                self.set_message("INFO", "复位完成：机械臂已回 PARK，急停解除")
            else:
                self.set_message("INFO", f"{label}完成，用时 {duration:.1f}s")
            if command in {"detect", "red-baseline", "sweep-shortest"}:
                self._reload_detection()
            if command in {
                "competition-self-color",
                "competition-white",
                "competition-card",
            }:
                self.competition_started_monotonic = None
            return
        fail(detail or f"result={result or 'MISSING'} returncode={returncode}")

    def _drain(self) -> None:
        while not self.messages.empty():
            message = self.messages.get_nowait()
            kind = message[0]
            if kind == "done":
                returncode = message[1]
                if not isinstance(returncode, (int, type(None))):
                    returncode = None
                timed_out = bool(message[2])
                self._finish_action(returncode, timed_out, str(message[3]))
            elif kind == "output":
                self._log("INFO", f"子进程输出尾部：{message[1]}")
            elif kind == "plain":
                _, ok, text = message
                self.set_message("INFO" if ok else "ERROR", str(text))
            elif kind == "estop-off":
                _, _, text = message
                self.set_message("ERROR", str(text))

    # ---------------- 自检 ----------------

    def run_self_checks(self) -> None:
        checks: list[tuple[str, str, str]] = []
        checks.append(
            (
                "中文字体",
                "FAIL" if self.font_missing else "PASS",
                "缺失：中文将显示为方块" if self.font_missing else "已加载",
            )
        )
        tool_exists = Path(self.backend.supervisor_tool).is_file()
        checks.append(
            (
                "supervisor_run 工具",
                "PASS" if tool_exists else "WARN",
                "已部署" if tool_exists else "未部署（GUI 按合同调用）",
            )
        )
        checks.append(
            (
                "运动学标定",
                "PASS" if PARK_PATH.is_file() else "FAIL",
                PARK_PATH.name if PARK_PATH.is_file() else "缺失",
            )
        )
        robot_available = self._robot_available()
        checks.append(
            (
                "机械臂设备",
                "PASS" if robot_available else "FAIL",
                str(self.robot_port) if robot_available else "离线：运动动作锁定",
            )
        )
        motion_lock_reason = self._motion_lock_reason()
        checks.append(
            (
                "运动安全锁",
                "PASS" if motion_lock_reason is None else "FAIL",
                "无未解除事件" if motion_lock_reason is None else motion_lock_reason,
            )
        )
        checks.append(
            (
                "相机单应矩阵",
                "PASS" if HOMOGRAPHY_PATH.is_file() else "FAIL",
                HOMOGRAPHY_PATH.name,
            )
        )
        checks.append(
            (
                "正式 PARK 标定",
                "PASS" if PARK_PATH.is_file() else "FAIL",
                PARK_PATH.name,
            )
        )
        if self.pieces_path.is_file():
            age_s = time.time() - self.pieces_path.stat().st_mtime
            checks.append(
                (
                    "检测数据",
                    "PASS",
                    f"{len(self.pieces)} 片，{age_s / 60.0:.1f} 分钟前",
                )
            )
        else:
            checks.append(("检测数据", "WARN", "未检测"))
        checks.append(
            (
                "电磁铁控制链",
                "PASS",
                "BCM GPIO17 /dev/gpiochip0，默认关闭，软件上限 10s",
            )
        )
        lock_reason = pick_place_lock_reason()
        checks.append(
            (
                "正式抓放",
                "PASS" if lock_reason is None else "UNKNOWN",
                "已解锁" if lock_reason is None else f"锁定：{lock_reason}",
            )
        )
        self.self_checks = checks
        self._log("INFO", "自检已刷新")

    # ---------------- 事件处理 ----------------

    def _page_buttons(self) -> list[PageButton]:
        return self.pages[self.page]["buttons"]

    def _refresh_button_states(self) -> None:
        busy = self.state in (PageState.RUNNING, PageState.ESTOP)
        robot_motion_ready = self._robot_motion_ready()
        for entry in self.pages.values():
            for button in entry["buttons"]:
                if button.tab is not None and button.tab != self.manual_tab:
                    button.visible = False
                    button.enabled = False
                    continue
                button.visible = True
                if not button.action:
                    button.enabled = True
                    continue
                if busy:
                    button.enabled = False
                    continue
                if (
                    button.tab in {"joint", "cartesian"} or button.text == "启动电磁铁"
                ) and not self.manual_unlocked:
                    button.enabled = False
                    continue
                cap_id = self._button_capability_id(button.text)
                if cap_id:
                    cap = self.capabilities.get(cap_id)
                    if cap is not None:
                        ready = cap.availability is CapabilityAvailability.READY
                        if ready and cap.requires_robot:
                            ready = robot_motion_ready
                        button.enabled = ready
                        continue
                button.enabled = robot_motion_ready
        self.reset_button.enabled = self.state == PageState.ESTOP and robot_motion_ready

    def _button_capability_id(self, text: str) -> str | None:
        mapping = {
            "一键启动": self.competition_mode,
            "刷新检测": "detect",
            "红色基线": "red-baseline",
            "快速巡检": "sweep-shortest",
            "通用 Shadow": "puzzle-shadow",
            "到 20mm": "pregrasp",
            "到选中碎片 20mm": "pregrasp",
            "回 PARK": "park",
            "回正式 PARK": "park",
            "五轴断力矩": "park",
            "启动电磁铁": "manual-magnet-pulse",
            "立即断电": "manual-magnet-off",
            "读取状态": "manual-read-state",
        }
        return mapping.get(text)

    def handle_event(self, raw_event) -> None:
        event = touch_event_to_mouse_event(raw_event, (self.width, self.height))
        if getattr(event, "touch", False) and hasattr(event, "pos"):
            self.pointer_pos = event.pos
        if event.type == pygame.QUIT or (
            event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
        ):
            self.quit_requested = True
            return
        if self.estop_button.handle(event):
            return
        if self.state == PageState.ESTOP and self.reset_button.handle(event):
            return
        if (
            event.type == pygame.MOUSEBUTTONDOWN
            and self.page == "competition"
            and self.pieces
            and self.piece_list_rect.collidepoint(event.pos)
        ):
            row_height = 34
            index = (event.pos[1] - self.piece_list_rect.y) // row_height
            if 0 <= index < len(self.pieces):
                self.selected_index = index
                self.select(0)
            return
        for button in self._page_buttons():
            if button.handle(event):
                return

    # ---------------- 绘制 ----------------

    def _panel(self, rect, title: str = "") -> None:
        pygame.draw.rect(self.screen, C_PANEL, rect, border_radius=10)
        pygame.draw.rect(self.screen, C_LINE, rect, width=1, border_radius=10)
        if title:
            image = self.fonts["tiny"].render(title, True, C_DIM)
            self.screen.blit(image, (rect.x + 12, rect.y + 8))

    def _draw_block(self, rect, label: str, value: str, color) -> None:
        rect = pygame.Rect(rect)
        pygame.draw.rect(self.screen, C_PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, color, rect, width=2, border_radius=8)
        label_image = self.fonts["tiny"].render(label, True, C_DIM)
        self.screen.blit(label_image, (rect.x + 10, rect.y + 2))
        value_image = self.fonts["small"].render(value, True, color)
        self.screen.blit(value_image, (rect.x + 10, rect.y + 22))

    def _draw_top_bar(self) -> None:
        pygame.draw.rect(self.screen, C_PANEL, (0, 0, self.width, TOP_BAR_HEIGHT))
        pygame.draw.line(
            self.screen, C_LINE, (0, TOP_BAR_HEIGHT), (self.width, TOP_BAR_HEIGHT)
        )
        page_names = {
            "competition": "比赛",
            "run": "运行监视",
            "manual": "手动",
            "debug": "Debug",
        }
        title = self.fonts["title"].render(
            f"拼图装置 · {page_names[self.page]}", True, C_TEXT
        )
        self.screen.blit(title, (16, 12))
        lock_reason = pick_place_lock_reason()
        blocks = [
            (
                "页面状态",
                STATE_LABELS[self.state.value],
                STATE_COLORS[self.state.value],
            ),
            ("检测碎片", f"{len(self.pieces)} 片", C_TEXT if self.pieces else C_DIM),
            (
                "正式抓放",
                "已解锁" if lock_reason is None else "锁定",
                C_GREEN if lock_reason is None else C_ORANGE,
            ),
            (
                "维护解锁",
                "已解锁" if self.manual_unlocked else "锁定",
                C_GREEN if self.manual_unlocked else C_DIM,
            ),
        ]
        x = 336
        for label, value, color in blocks:
            self._draw_block((x, 3, 132, 50), label, value, color)
            x += 140
        self.estop_button.draw(self.screen, self.fonts)

    def _draw_bottom_bar(self) -> None:
        pygame.draw.rect(
            self.screen, C_PANEL, (0, BOTTOM_BAR_TOP, self.width, self.height)
        )
        pygame.draw.line(
            self.screen, C_LINE, (0, BOTTOM_BAR_TOP), (self.width, BOTTOM_BAR_TOP)
        )
        color = MESSAGE_COLORS.get(self.message_level, C_TEXT)
        message = self.fonts["big"].render(self.message_text[:30], True, color)
        self.screen.blit(message, (16, BOTTOM_BAR_TOP + 6))
        safety = self.fonts["tiny"].render(
            "Z_SAFE=60mm  PREGRASP=20mm  PICK=UNKNOWN（正式抓放锁定）",
            True,
            C_DIM,
        )
        self.screen.blit(safety, (16, BOTTOM_BAR_TOP + 40))

        if self.state == PageState.RUNNING and self.current_action is not None:
            action = self.current_action
            elapsed = time.monotonic() - action["started_monotonic"]
            timeout_s = action["timeout_s"]
            phase = "-"
            progress: float | None = None
            if self.heartbeat is not None:
                phase = str(self.heartbeat.get("phase", "-"))[:24]
                raw = self.heartbeat.get("progress")
                if isinstance(raw, (int, float)) and math.isfinite(raw):
                    progress = max(0.0, min(1.0, raw))
            info = self.fonts["small"].render(
                f"{action['label']}  {phase}  {elapsed:.0f}/{timeout_s:.0f}s",
                True,
                C_TEXT,
            )
            self.screen.blit(info, (688, BOTTOM_BAR_TOP + 8))
            bar = pygame.Rect(688, BOTTOM_BAR_TOP + 34, 320, 16)
            pygame.draw.rect(self.screen, C_BG, bar, border_radius=6)
            if progress is not None:
                fill = bar.copy()
                fill.width = round(bar.width * progress)
                if fill.width > 0:
                    pygame.draw.rect(self.screen, C_BLUE, fill, border_radius=6)
            pygame.draw.rect(self.screen, C_LINE, bar, width=1, border_radius=6)
        elif self.state == PageState.ESTOP:
            self.reset_button.draw(self.screen, self.fonts)

    def _draw_preview(self, rect, *, draw_selection: bool = True) -> None:
        pygame.draw.rect(self.screen, (8, 8, 10), rect)
        if self.preview is not None:
            image = pygame.transform.smoothscale(self.preview, rect.size)
            self.screen.blit(image, rect)
            scale_x = rect.width / PREVIEW_SOURCE_SIZE[0]
            scale_y = rect.height / PREVIEW_SOURCE_SIZE[1]
            for index, piece in enumerate(self.pieces):
                pixel = self._piece_pixel(piece)
                if pixel is None:
                    continue
                center = (
                    round(rect.x + pixel[0] * scale_x),
                    round(rect.y + pixel[1] * scale_y),
                )
                if draw_selection and index == self.selected_index:
                    diagnostics = piece.get("diagnostics")
                    area = (
                        display_float(diagnostics.get("pixel_area", 900.0))
                        if isinstance(diagnostics, dict)
                        else 900.0
                    )
                    radius = max(
                        14, min(56, round(math.sqrt(area / math.pi) * scale_x * 1.4))
                    )
                    pygame.draw.circle(self.screen, C_ORANGE, center, radius, width=4)
                    badge = self.fonts["small"].render(f"◆ {index}", True, C_ORANGE)
                    self.screen.blit(badge, (center[0] + radius + 4, center[1] - 12))
                else:
                    pygame.draw.circle(self.screen, C_DIM, center, 8, width=2)
        else:
            text = self.fonts["button"].render(
                '尚未检测：请点击"刷新检测"', True, C_DIM
            )
            self.screen.blit(text, text.get_rect(center=rect.center))
        border = (
            C_ORANGE
            if (draw_selection and self._selected_piece() is not None)
            else C_LINE
        )
        pygame.draw.rect(self.screen, border, rect, width=2)

    def _piece_info_lines(self) -> tuple[str, str]:
        piece = self._selected_piece()
        if piece is None:
            return "碎片：未检测", ""
        center_x, center_y = self._piece_center(piece)
        yaw = display_float(piece.get("yaw_deg", 0.0))
        confidence = display_float(piece.get("confidence", 0.0))
        first = (
            f"{piece.get('piece_id', '?')} "
            f"({self.selected_index + 1}/{len(self.pieces)})  "
            f"中心 ({center_x:.1f}, {center_y:.1f})mm"
        )
        second = (
            f"当前角 {yaw:.1f}° → 目标角 {self.target_rotation_deg:.0f}°  "
            f"置信度 {confidence:.2f}"
        )
        return first, second

    def _draw_competition(self) -> None:
        cards = [
            ("competition-self-color", "自备纯色碎片", 50, C_GREEN),
            ("competition-white", "现场白色碎片", 20, C_BLUE),
            ("competition-card", "现场扑克牌碎片", 30, C_ORANGE),
        ]
        for index, (mode, title, score, color) in enumerate(cards):
            y = 72 + index * 110
            selected = self.competition_mode == mode
            cap = self.capabilities.get(mode)
            status_text = (
                "READY"
                if cap and cap.availability.value == "READY"
                else cap.availability.value
                if cap
                else "UNKNOWN"
            )
            status_color = (
                C_GREEN
                if status_text == "READY"
                else C_ORANGE
                if status_text == "LOCKED"
                else C_RED
            )
            if selected:
                pygame.draw.rect(
                    self.screen, color, (12, y - 4, 516, 104), width=3, border_radius=10
                )
            label = self.fonts["tiny"].render(
                f"{title} · {score} 分 · 限时 {COMPETITION_TIMEOUT_S:.0f}s · {status_text}",
                True,
                status_color,
            )
            self.screen.blit(label, (24, y + 8))
            if selected:
                sel = self.fonts["small"].render("◆ 已选择", True, C_ORANGE)
                self.screen.blit(sel, (24, y + 66))
            if cap and cap.availability is not CapabilityAvailability.READY:
                reason = self.fonts["tiny"].render(cap.reason[:46], True, C_ORANGE)
                self.screen.blit(reason, (24, y + 66))

        check_rect = pygame.Rect(540, 72, 460, 304)
        self._panel(check_rect, "赛前检查（能力表）")
        for index, cap in enumerate(self.capabilities.values()):
            if index >= 8:
                break
            y = check_rect.y + 32 + index * 32
            color = (
                C_GREEN
                if cap.availability is CapabilityAvailability.READY
                else C_ORANGE
                if cap.availability is CapabilityAvailability.LOCKED
                else C_RED
            )
            text = f"{cap.command_id:22} {cap.availability.value:12} {cap.reason[:28]}"
            image = self.fonts["tiny"].render(text, True, color)
            self.screen.blit(image, (check_rect.x + 12, y))

        if self.competition_mode:
            cap = self.capabilities.get(self.competition_mode)
            if cap and cap.availability is not CapabilityAvailability.READY:
                reason_rect = pygame.Rect(540, 392, 460, 44)
                pygame.draw.rect(self.screen, C_LOCK, reason_rect, border_radius=8)
                pygame.draw.rect(
                    self.screen, C_ORANGE, reason_rect, width=2, border_radius=8
                )
                text = self.fonts["small"].render(
                    f"未就绪：{cap.reason}", True, C_ORANGE
                )
                self.screen.blit(text, (reason_rect.x + 10, reason_rect.y + 10))

    def _draw_run(self) -> None:
        mode_text = self.competition_mode or "未选择模式"
        elapsed = 0.0
        if (
            self.competition_started_monotonic is not None
            and self.state == PageState.RUNNING
        ):
            elapsed = time.monotonic() - self.competition_started_monotonic
        info = self.fonts["small"].render(
            f"模式：{mode_text}  官方限时 {COMPETITION_TIMEOUT_S:.0f}s  已用 {elapsed:.1f}s",
            True,
            C_TEXT,
        )
        self.screen.blit(info, (16, 72))
        if elapsed >= COMPETITION_TIMEOUT_S:
            warn = self.fonts["small"].render("已超时", True, C_RED)
            self.screen.blit(warn, (600, 72))

        stages = ["准备", "检测", "求解", "搬运", "视觉复核", "完成"]
        phase = "-"
        if self.heartbeat is not None:
            phase = str(self.heartbeat.get("phase", "-"))
        stage_y = 104
        for index, stage in enumerate(stages):
            x = 16 + index * 100
            color = C_BLUE if stage in phase else C_DIM
            image = self.fonts["tiny"].render(stage, True, color)
            self.screen.blit(image, (x, stage_y))
            if stage in phase:
                pygame.draw.circle(self.screen, C_BLUE, (x + 16, stage_y + 28), 5)

        preview_rect = pygame.Rect(16, 144, 420, 296)
        self._draw_preview(preview_rect)

        progress_rect = pygame.Rect(452, 144, 320, 296)
        self._panel(progress_rect, "阶段与进度")
        if self.heartbeat is not None:
            raw = self.heartbeat.get("progress")
            progress = (
                max(0.0, min(1.0, display_float(raw)))
                if isinstance(raw, (int, float))
                else 0.0
            )
            bar = pygame.Rect(progress_rect.x + 12, progress_rect.y + 36, 296, 16)
            pygame.draw.rect(self.screen, C_BG, bar, border_radius=6)
            fill = bar.copy()
            fill.width = round(bar.width * progress)
            if fill.width > 0:
                pygame.draw.rect(self.screen, C_BLUE, fill, border_radius=6)
            pygame.draw.rect(self.screen, C_LINE, bar, width=1, border_radius=6)
            phase_text = self.fonts["small"].render(f"阶段：{phase}", True, C_TEXT)
            self.screen.blit(phase_text, (progress_rect.x + 12, progress_rect.y + 64))
        pieces_text = self.fonts["small"].render(
            f"碎片数：{len(self.pieces)}", True, C_TEXT
        )
        self.screen.blit(pieces_text, (progress_rect.x + 12, progress_rect.y + 96))
        first, second = self._piece_info_lines()
        if first:
            self.screen.blit(
                self.fonts["tiny"].render(first[:42], True, C_DIM),
                (progress_rect.x + 12, progress_rect.y + 130),
            )
        if second:
            self.screen.blit(
                self.fonts["tiny"].render(second[:42], True, C_DIM),
                (progress_rect.x + 12, progress_rect.y + 154),
            )

        result_rect = pygame.Rect(784, 144, 216, 296)
        self._panel(result_rect, "最后结果")
        if self.last_result is not None:
            payload = self.last_result
            result = str(payload.get("result", "?"))
            color = C_GREEN if result == "PASS" else C_RED
            lines = [
                (str(payload.get("command", "-")), C_TEXT),
                (result, color),
                (f"用时 {display_float(payload.get('duration_s')):.1f}s", C_DIM),
                (
                    "磁铁 " + ("启用" if payload.get("magnet_enabled") else "关闭"),
                    C_DIM,
                ),
            ]
            error_code = str(payload.get("error_code", ""))
            if error_code and error_code != "NONE":
                lines.append((error_code, C_RED))
            for row, (text, line_color) in enumerate(lines):
                image = self.fonts["small"].render(text[:14], True, line_color)
                self.screen.blit(
                    image, (result_rect.x + 14, result_rect.y + 34 + row * 34)
                )
        else:
            image = self.fonts["small"].render("尚无记录", True, C_DIM)
            self.screen.blit(image, (result_rect.x + 14, result_rect.y + 34))

    def _draw_manual(self) -> None:
        lock_text = (
            "维护解锁：已解锁"
            if self.manual_unlocked
            else "维护解锁：未解锁（离开手动页/急停/任务自动取消）"
        )
        lock_color = C_GREEN if self.manual_unlocked else C_ORANGE
        self.screen.blit(
            self.fonts["small"].render(lock_text, True, lock_color),
            (472, 78),
        )

        if self.manual_tab == "joint":
            self._draw_manual_joint()
        else:
            self._draw_manual_cartesian()

        magnet_status = self.fonts["tiny"].render(
            f"磁铁脉冲：{self.magnet_duration_s:.0f}s（仅 1/3/5s，无保持入口）",
            True,
            C_DIM,
        )
        self.screen.blit(magnet_status, (540, 254))

    def _draw_manual_joint(self) -> None:
        row_height = 56
        for index, feature in enumerate(JOINT_ORDER):
            y = 132 + index * row_height
            names = {
                "shoulder_pan": "肩旋转",
                "shoulder_lift": "肩俯仰",
                "elbow_flex": "肘俯仰",
                "wrist_flex": "腕俯仰",
                "wrist_roll": "腕旋转",
            }
            name_text = f"{names.get(feature, feature)} · {feature}.pos"
            self.screen.blit(
                self.fonts["small"].render(name_text, True, C_TEXT),
                (156, y + 6),
            )
            readback = self.joint_readbacks.get(feature, 0.0)
            self.screen.blit(
                self.fonts["tiny"].render(f"读回 {readback:.2f}°", True, C_DIM),
                (156, y + 30),
            )
            if index == self.selected_joint_index:
                pygame.draw.rect(
                    self.screen,
                    C_ORANGE,
                    (152, y - 2, 340, 52),
                    width=2,
                    border_radius=6,
                )
        self.screen.blit(
            self.fonts["tiny"].render(
                f"关节步长：{self.joint_step_deg:g}°", True, C_DIM
            ),
            (16, 420),
        )

    def _draw_manual_cartesian(self) -> None:
        axes = [
            ("x", "X"),
            ("y", "Y"),
            ("z", "Z"),
            ("pitch", "Pitch"),
            ("yaw", "Yaw"),
        ]
        row_height = 56
        for index, (axis, label) in enumerate(axes):
            y = 132 + index * row_height
            frame = "robot_base_mm"
            unit = "mm" if axis in {"x", "y", "z"} else "deg"
            value = self.tcp_readback.get(axis, 0.0)
            text = f"{label} · {axis} · {frame} · 当前 {value:.2f} {unit}"
            self.screen.blit(
                self.fonts["small"].render(text, True, C_TEXT), (260, y + 6)
            )
            if axis == self.selected_cartesian_axis:
                pygame.draw.rect(
                    self.screen,
                    C_ORANGE,
                    (256, y - 2, 460, 52),
                    width=2,
                    border_radius=6,
                )
        unit = "deg" if self.cartesian_is_angle else "mm"
        self.screen.blit(
            self.fonts["tiny"].render(
                f"末端步长：{self.cartesian_step_value:g}{unit}（robot_base_mm）",
                True,
                C_DIM,
            ),
            (16, 420),
        )

    def _draw_debug(self) -> None:
        cap_rect = pygame.Rect(16, 132, 496, 220)
        self._panel(cap_rect, "能力矩阵")
        for index, cap in enumerate(self.capabilities.values()):
            if index >= 6:
                break
            y = cap_rect.y + 32 + index * 32
            color = (
                C_GREEN
                if cap.availability is CapabilityAvailability.READY
                else C_ORANGE
                if cap.availability is CapabilityAvailability.LOCKED
                else C_RED
            )
            text = f"{cap.command_id:22} {cap.availability.value:12} {cap.reason[:22]}"
            self.screen.blit(
                self.fonts["tiny"].render(text, True, color), (cap_rect.x + 12, y)
            )

        self_check_rect = pygame.Rect(528, 132, 472, 220)
        self._panel(self_check_rect, "自检摘要")
        for index, (name, status, detail) in enumerate(self.self_checks[:6]):
            y = self_check_rect.y + 32 + index * 32
            color = (
                C_GREEN if status == "PASS" else C_ORANGE if status == "WARN" else C_RED
            )
            text = f"{name:12} {status:6} {detail[:24]}"
            self.screen.blit(
                self.fonts["tiny"].render(text, True, color),
                (self_check_rect.x + 12, y),
            )

        switches = [
            ("轮廓覆盖", self.debug_overlays),
            ("详细日志", self.debug_verbose_logs),
            ("保存证据", self.debug_save_evidence),
            ("generic vision", self.debug_generic_vision),
            ("texture", self.debug_texture),
            ("ACT", self.debug_act),
        ]
        switch_rect = pygame.Rect(16, 364, 992, 120)
        self._panel(switch_rect, "本次会话开关（不修改环境/标定/运动锁）")
        for index, (label, value) in enumerate(switches):
            x = switch_rect.x + 12 + (index % 3) * 320
            y = switch_rect.y + 36 + (index // 3) * 44
            color = C_GREEN if value else C_DIM
            self.screen.blit(
                self.fonts["small"].render(
                    f"{'[开]' if value else '[关]'} {label}", True, color
                ),
                (x, y),
            )

        log_rect = pygame.Rect(16, 490, 992, 40)
        self._panel(log_rect, "日志尾部")
        for row, (level, text) in enumerate(self.logs[-2:]):
            color = MESSAGE_COLORS.get(level, C_TEXT)
            image = self.fonts["tiny"].render(f"[{level}] {text[:88]}", True, color)
            self.screen.blit(image, (log_rect.x + 14, log_rect.y + 24 + row * 18))

    def draw(self) -> None:
        self.screen.fill(C_BG)
        self._refresh_button_states()
        self._read_heartbeat()
        self._draw_top_bar()
        self.pages[self.page]["draw"]()
        for button in self._page_buttons():
            button.draw(self.screen, self.fonts)
        self._draw_bottom_bar()
        if self.font_missing:
            text = self.fonts["small"].render(
                "ERROR: CJK font missing - install fonts-wqy-zenhei "
                "or place NotoSansCJK under fonts/",
                True,
                C_RED,
            )
            background = text.get_rect(topleft=(16, 60)).inflate(16, 8)
            pygame.draw.rect(self.screen, (60, 16, 16), background, border_radius=6)
            pygame.draw.rect(self.screen, C_RED, background, width=2, border_radius=6)
            self.screen.blit(text, (16, 64))
        if self.pointer_pos is not None and self.framebuffer is not None:
            pygame.draw.circle(self.screen, C_TEXT, self.pointer_pos, 9, width=2)
            pygame.draw.circle(self.screen, C_RED, self.pointer_pos, 3)
        if self.framebuffer is not None:
            self.framebuffer.present(self.screen)
        else:
            pygame.display.flip()

    # ---------------- 主循环 ----------------

    def run(self) -> None:
        clock = pygame.time.Clock()

        def _handle_sigterm(_signum: int, _frame: object) -> None:
            self._request_magnet_off()
            self.quit_requested = True

        signal.signal(signal.SIGTERM, _handle_sigterm)
        try:
            while not self.quit_requested:
                events = list(pygame.event.get())
                if self.touch_reader is not None:
                    events.extend(self.touch_reader.poll())
                for raw_event in events:
                    self.handle_event(raw_event)
                self._drain()
                self.draw()
                clock.tick(20)
        finally:
            process = self.process
            if process is not None and process.poll() is None:
                process.terminate()
                deadline = time.monotonic() + 1.0
                while process.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                if process.poll() is None:
                    process.kill()
                    process.wait()
            self._request_magnet_off()
            subprocess.run(
                [self.python_path, str(TOOLS / "emergency_off_single_arm.py")],
                cwd=PROJECT_ROOT,
                check=False,
            )
            if self.touch_reader is not None:
                self.touch_reader.close()
            if self.framebuffer is not None:
                self.framebuffer.close()
            pygame.quit()


def main() -> None:
    page = PickPlacePage()
    if "--page" in sys.argv:
        page_index = sys.argv.index("--page")
        if page_index + 1 < len(sys.argv) and sys.argv[page_index + 1] in page.pages:
            page.page = sys.argv[page_index + 1]
    if "--screenshot" in sys.argv:
        screenshot_index = sys.argv.index("--screenshot")
        if screenshot_index + 1 >= len(sys.argv):
            raise SystemExit("--screenshot 需要输出路径")
        page.draw()
        pygame.image.save(page.screen, sys.argv[screenshot_index + 1])
        if page.touch_reader is not None:
            page.touch_reader.close()
        if page.framebuffer is not None:
            page.framebuffer.close()
        pygame.quit()
        return
    with contextlib.suppress(KeyboardInterrupt):
        page.run()


if __name__ == "__main__":
    main()
