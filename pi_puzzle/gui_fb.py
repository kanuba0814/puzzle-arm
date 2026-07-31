"""
机械臂控制 GUI — 无桌面环境版 (pygame / SDL2, 树莓派纯 tty 直显)

适用场景: 树莓派 OS Lite 没有 X11/桌面, 只有 tty。本程序通过 SDL2 的
kmsdrm 驱动直接渲染到屏幕 (不依赖 X, 不依赖桌面环境), 鼠标/触摸走 evdev。

界面结构 (适配 800x480 小屏):
    开机自检        : 连接/相机/标定/声光/字体检查
    主菜单          : 比赛模式 / 系统标定 / 手动调试 / 运行记录
    ├─ 比赛准备     : 任务配置 + 赛前检查 + 一键开始
    ├─ 自动运行     : 计时 + 阶段 + 碎片计数 + 最新相机帧 + 中止
    ├─ 运行结果     : 成功/失败原因 + 总耗时 + 再次运行
    ├─ 系统标定     : 本地相机工作区标定 + 触点式基座标定
    ├─ 手动调试     : Tab[关节] J1-J5 步进 + 力矩/电磁铁/Home
    │                Tab[末端] XYZ 步进
    └─ 系统日志     : 全屏日志 + 最近运行记录 + 导出/清空
    常驻            : 顶部状态栏 (连接/模式/力矩/电磁铁/标定/任务 + 关节角),
                     右上角红色 [急停] 按钮任何页面可点, ESC 退出

树莓派上运行:
    pip install pygame-ce          # 有 aarch64 官方 wheel, Lite 直接装
    python3 gui_fb.py              # 自动选 kmsdrm; 失败时回退窗口模式
    bash tools/start_gui_fb.sh     # 部署用启动脚本

开发机 (有桌面) 上调试:
    python3 gui_fb.py              # 自动用 x11/wayland 窗口模式
    python3 gui_fb.py --window 800x480   # 强制窗口 + 指定分辨率

中文字体: 自动搜索系统 CJK 字体 (Noto/WenQuanYi/Droid);
树莓派 Lite 若无中文字体: sudo apt install fonts-wqy-zenhei,
或把 ttf/ttc 放到项目 fonts/ 目录。

Pi启动脚本设置 PUZZLE_REAL=1 后启用本地硬件后端；系统标定页直接读取
/dev/video0 和本地舵机总线，不使用SSH或网络视频流。开发机默认仍使用Mock。
"""

from __future__ import annotations

import json
import os
import queue
import sys
import time
from pathlib import Path

# ---- 在 import pygame 前决定视频驱动 ----
# 无 DISPLAY 环境 = 树莓派纯 tty -> kmsdrm 直显; 有桌面 -> 走桌面窗口
if (
    not os.environ.get("DISPLAY")
    and not os.environ.get("WAYLAND_DISPLAY")
    and "--window" not in sys.argv
):
    os.environ.setdefault("SDL_VIDEODRIVER", "kmsdrm")
    os.environ.setdefault("SDL_NOMOUSE", "0")
    # 禁止SDL重复合成鼠标事件；FINGER事件由下方统一转换，原生mouse0仍直接使用。
    os.environ.setdefault("SDL_TOUCH_MOUSE_EVENTS", "0")

import pygame

from gui_backend import (
    TASK_PROFILES,
    RobotState,
    RunMetrics,
    create_backend,
    task_profile,
)
from gui_calibration import LocalCalibration  # type: ignore[import-not-found]

# ==================== 主题 ====================
C_BG = (24, 26, 32)
C_PANEL = (36, 39, 48)
C_BTN = (62, 68, 84)
C_BTN_PRESS = (90, 98, 120)
C_TEXT = (232, 234, 240)
C_DIM = (150, 155, 168)
C_GREEN = (46, 160, 90)
C_RED = (200, 60, 60)
C_RED_DARK = (150, 40, 40)
C_ORANGE = (210, 140, 40)
C_BLUE = (60, 130, 200)
C_LINE = (70, 75, 92)
C_TAB = (48, 52, 64)
C_TAB_ON = (62, 68, 84)

STATE_LABELS = {
    "IDLE": "空闲",
    "INIT": "初始化",
    "CALIBRATING": "标定中",
    "DETECTING": "检测中",
    "SOLVING": "求解中",
    "EXECUTING": "执行中",
    "DONE": "完成",
    "ERROR": "错误",
}

RUN_HISTORY_PATH = Path("out/gui_run_history.jsonl")

FONT_CANDIDATES = [
    "fonts/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
]


def find_font() -> str | None:
    for p in FONT_CANDIDATES:
        if Path(p).is_file():
            return p
    return None


def create_font(font_path, size):
    """创建中文字体；独立函数避免在GUI构造器内嵌套资源创建。"""
    fallback_size = size + size * 3 // 10
    if font_path:
        return pygame.font.Font(font_path, size)
    return pygame.font.Font(None, fallback_size)


def normalize_pointer_event(event, screen_size):
    """将SDL触摸事件转为现有控件使用的鼠标事件；鼠标事件原样返回。"""
    if event.type not in (pygame.FINGERDOWN, pygame.FINGERUP):
        return event
    width, height = screen_size
    pos = (
        max(0, min(width - 1, round(event.x * width))),
        max(0, min(height - 1, round(event.y * height))),
    )
    event_type = (
        pygame.MOUSEBUTTONDOWN
        if event.type == pygame.FINGERDOWN
        else pygame.MOUSEBUTTONUP
    )
    return pygame.event.Event(
        event_type, pos=pos, button=1, touch=True, finger_id=event.finger_id
    )


# ==================== 控件 ====================


class Button:
    def __init__(
        self, rect, text, on_click, color=C_BTN, text_color=C_TEXT, font_key="btn"
    ):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.on_click = on_click
        self.color = color
        self.text_color = text_color
        self.font_key = font_key
        self.pressed = False
        self.enabled = True
        self.subtitle: str | None = None
        self.selected = False
        self._tab: str | None = None

    def handle(self, ev) -> bool:
        if not self.enabled:
            return False
        if (
            ev.type == pygame.MOUSEBUTTONDOWN
            and ev.button == 1
            and self.rect.collidepoint(ev.pos)
        ):
            self.pressed = True
            return True
        if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1 and self.pressed:
            self.pressed = False
            if self.rect.collidepoint(ev.pos):
                self.on_click()
            return True
        return False

    def draw(self, surf, fonts):
        color = self.color if self.enabled else C_TAB
        if self.pressed and self.enabled:
            color = C_BTN_PRESS
        text_color = self.text_color if self.enabled else C_DIM
        pygame.draw.rect(
            surf,
            color,
            self.rect,
            border_radius=8,
        )
        pygame.draw.rect(surf, C_LINE, self.rect, width=1, border_radius=8)
        f = fonts[self.font_key]
        img = f.render(self.text, True, text_color)
        if getattr(self, "subtitle", None):
            main_rect = img.get_rect(center=(self.rect.centerx, self.rect.centery - 10))
            surf.blit(img, main_rect)
            sub = fonts["small"].render(self.subtitle, True, text_color)
            surf.blit(
                sub,
                sub.get_rect(center=(self.rect.centerx, self.rect.centery + 22)),
            )
        else:
            surf.blit(img, img.get_rect(center=self.rect.center))
        if getattr(self, "selected", False):
            pygame.draw.rect(surf, C_GREEN, self.rect, width=3, border_radius=8)


class Radio:
    """单选组: options=[(文本, 值)], get()/set(v) 读写当前值"""

    def __init__(self, x, y, options, get, set_, font_key="small", box=20, gap=10):
        self.x, self.y = x, y
        self.options = options
        self.get, self.set = get, set_
        self.font_key, self.box, self.gap = font_key, box, gap
        self.rects = []
        self._tab: str | None = None

    def layout(self, fonts):
        self.rects = []
        cx = self.x
        for text, _ in self.options:
            w = self.box + 8 + fonts[self.font_key].size(text)[0] + self.gap
            self.rects.append(pygame.Rect(cx, self.y, w, self.box))
            cx += w

    def handle(self, ev) -> bool:
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            for rect, (_, val) in zip(self.rects, self.options, strict=True):
                if rect.collidepoint(ev.pos):
                    self.set(val)
                    return True
        return False

    def draw(self, surf, fonts):
        cur = self.get()
        for rect, (text, val) in zip(self.rects, self.options, strict=True):
            cx, cy = rect.x + self.box // 2, rect.y + self.box // 2
            pygame.draw.circle(surf, C_TEXT, (cx, cy), self.box // 2, width=2)
            if val == cur:
                pygame.draw.circle(surf, C_GREEN, (cx, cy), self.box // 2 - 6)
            img = fonts[self.font_key].render(text, True, C_TEXT)
            surf.blit(
                img,
                (rect.x + self.box + 8, rect.y + self.box // 2 - img.get_height() // 2),
            )


# ==================== 主界面 ====================


class FbGUI:
    def __init__(self, win_size=None):
        pygame.init()
        if win_size:
            self.screen = pygame.display.set_mode(win_size)
        elif os.environ.get("SDL_VIDEODRIVER") == "kmsdrm":
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((960, 600), pygame.RESIZABLE)
        pygame.display.set_caption("拼图机械臂控制台")
        self.W, self.H = self.screen.get_size()

        font_path = find_font()
        self.font_path = font_path
        if font_path is None:
            print(
                "[GUI] 未找到中文字体, 中文将显示为方块! "
                "树莓派: sudo apt install fonts-wqy-zenhei"
            )
        base = max(16, self.H // 30)
        self.fonts = {
            "title": create_font(font_path, base + 8),
            "btn": create_font(font_path, base + 2),
            "small": create_font(font_path, base - 2),
            "log": create_font(font_path, base - 3),
        }

        self.backend = create_backend()
        self._q = queue.Queue()
        self.backend.set_callbacks(
            on_log=lambda level, message: self._q.put(("log", level, message)),
            on_state=lambda s: self._q.put(("state", s)),
            on_progress=lambda s, f: self._q.put(("progress", s, f)),
            on_task_done=lambda ok, m: self._q.put(("done", ok, m)),
        )
        self.backend.set_metrics_callback(
            lambda metrics: self._q.put(("metrics", metrics))
        )
        self.backend.set_frame_callback(lambda frame: self._q.put(("frame", frame)))

        self.state = RobotState()
        self.logs = []
        self.progress = 0.0
        self.progress_stage = "-"
        self.task_mode = "task1"
        self.joint_step = 5.0
        self.xyz_step = 10.0
        self.run_metrics = RunMetrics(expected_count=4)
        self.latest_frame = None
        self.run_started_at: float | None = None
        self.last_result: dict | None = None
        self.history: list[dict] = []
        self._load_history()
        self.checks: list[dict] = []
        self._timeout_sent = False

        # 页面: boot / main / auto / run / result / calibration / manual / log
        self.page = "boot"
        self.manual_tab = "joint"

        ok, msg = self.backend.connect()
        self.state = self.backend.get_state()
        self._log("INFO" if ok else "ERROR", f"后端连接: {msg}")
        self.calibration = LocalCalibration(
            getattr(self.backend, "_arm", None),
            lambda level, text: self._q.put(("log", level, text)),
        )
        self.checks = self._collect_checks()

        # 每页的控件: {page: {'buttons': [...], 'radios': [...], 'draw': fn}}
        self.pages = {}
        self._build_all()

    # ---------- 赛前检查 / 状态 ----------
    def _check_aruco(self) -> bool:
        try:
            import cv2

            return hasattr(cv2, "aruco")
        except (ImportError, AttributeError):
            return False

    def _check_gpiozero(self) -> bool:
        try:
            import importlib

            importlib.import_module("gpiozero")
            return True
        except (ImportError, RuntimeError):
            return False

    def _collect_checks(self) -> list[dict]:
        homography_ok = Path("homography.npy").is_file()
        base_ok = Path("arm_base.json").is_file()
        aruco_ok = self._check_aruco()
        gpio_ok = self.state.simulation or self._check_gpiozero()
        connected = self.state.connected
        calibrated = homography_ok and base_ok
        return [
            {
                "status": "ok" if connected else "fail",
                "label": "机械臂后端",
                "detail": "已连接" if connected else "未连接",
            },
            {
                "status": "ok" if aruco_ok else "fail",
                "label": "OpenCV ArUco",
                "detail": "可用" if aruco_ok else "不可用",
            },
            {
                "status": "ok" if homography_ok else "fail",
                "label": "工作区标定",
                "detail": "homography.npy" if homography_ok else "缺少 homography.npy",
            },
            {
                "status": "ok" if base_ok else "fail",
                "label": "基座标定",
                "detail": "arm_base.json" if base_ok else "缺少 arm_base.json",
            },
            {
                "status": "ok" if self.font_path else "warn",
                "label": "中文字体",
                "detail": self.font_path or "未找到，中文可能显示为方块",
            },
            {
                "status": "ok" if gpio_ok else "warn",
                "label": "声光提示",
                "detail": "模拟" if self.state.simulation else "gpiozero 不可用",
            },
            {
                "status": "ok" if calibrated or self.state.simulation else "warn",
                "label": "比赛准备",
                "detail": "可进入比赛模式" if calibrated else "模拟模式可跳过标定",
            },
        ]

    def _refresh_checks(self):
        self.backend.mark_calibrated()
        self.checks = self._collect_checks()
        self._log("INFO", "系统自检已刷新")

    def _task_ready(self) -> bool:
        return (
            self.state.connected
            and (self.state.calibrated or self.state.simulation)
            and not self._task_running()
        )

    def _task_running(self) -> bool:
        return self.run_started_at is not None and self.run_metrics.success is None

    def _reset_run(self):
        expected = task_profile(self.task_mode)["expected_pieces"]
        self.run_metrics = RunMetrics(expected_count=expected)
        self.latest_frame = None
        self.run_started_at = None
        self.last_result = None
        self._timeout_sent = False
        self.progress = 0.0
        self.progress_stage = "-"

    # ---------- 布局 ----------
    def _build_all(self):
        W, H = self.W, self.H
        m = 10
        self.rect_status = pygame.Rect(m, m, W - m * 2, max(50, H // 11))
        self.rect_body = pygame.Rect(
            m,
            self.rect_status.bottom + m,
            W - m * 2,
            H - self.rect_status.height - m * 3,
        )
        # 急停常驻 (状态栏右上角)
        self.btn_estop = Button(
            (
                self.rect_status.right - 104,
                self.rect_status.y + 7,
                96,
                self.rect_status.h - 14,
            ),
            "急停",
            self.backend.emergency_stop,
            color=C_RED,
            font_key="btn",
        )
        self.pages = {
            "boot": self._build_boot(),
            "main": self._build_main(),
            "auto": self._build_auto(),
            "run": self._build_run(),
            "result": self._build_result(),
            "calibration": self._build_calibration(),
            "manual": self._build_manual(),
            "log": self._build_log(),
        }

    def _back_btn(self):
        return Button(
            (self.rect_body.right - 100, self.rect_body.y + 6, 92, 40),
            "← 返回",
            lambda: self._goto("main"),
        )

    def _goto(self, page):
        if self._task_running() and page != "run":
            self._log("WARN", "任务运行中，页面已锁定；只允许中止或急停")
            return
        if self.page == "calibration" and page != "calibration":
            self.calibration.close()
        if page == "auto":
            self._reset_run()
            self.backend.mark_calibrated()
            self.checks = self._collect_checks()
        self.page = page
        if page == "calibration":
            ok, msg = self.calibration.open_camera()
            self._log("INFO" if ok else "ERROR", msg)
        # 切页时释放所有按钮按压态
        for p in self.pages.values():
            for b in p["buttons"]:
                b.pressed = False

    def _build_boot(self):
        r = self.rect_body
        y = r.bottom - 58
        bw = (r.w - 72) // 3
        buttons = [
            Button(
                (r.x + 20, y, bw, 46),
                "重新自检",
                self._refresh_checks,
                color=C_BTN,
            ),
            Button(
                (r.x + 36 + bw, y, bw, 46),
                "进入主界面",
                lambda: self._goto("main"),
                color=C_GREEN,
            ),
            Button(
                (r.x + 52 + bw * 2, y, bw, 46),
                "系统标定",
                lambda: self._goto("calibration"),
                color=C_BLUE,
            ),
        ]
        return {"buttons": buttons, "radios": [], "draw": self._draw_boot}

    def _build_main(self):
        r = self.rect_body
        gap = 16
        bw = (r.w - gap * 3) // 2
        bh = (r.h - gap * 3 - 28) // 2
        x1 = r.x + gap
        x2 = x1 + bw + gap
        y1 = r.y + gap
        y2 = y1 + bh + gap
        buttons = [
            Button(
                (x1, y1, bw, bh),
                "比赛模式",
                lambda: self._goto("auto"),
                color=C_BLUE,
                font_key="title",
            ),
            Button(
                (x2, y1, bw, bh),
                "系统标定",
                lambda: self._goto("calibration"),
                color=C_BLUE,
                font_key="title",
            ),
            Button(
                (x1, y2, bw, bh),
                "手动调试",
                lambda: self._goto("manual"),
                color=C_BTN,
                font_key="title",
            ),
            Button(
                (x2, y2, bw, bh),
                "运行记录/日志",
                lambda: self._goto("log"),
                color=C_BTN,
                font_key="title",
            ),
        ]
        buttons[0].subtitle = "赛前检查 · 一键启动 · 120秒计时"
        buttons[1].subtitle = "工作区单应 + 触点式基座标定"
        buttons[2].subtitle = "力矩 · 电磁铁 · 关节/末端步进"
        buttons[3].subtitle = "日志 · 历史 · 导出"
        return {"buttons": buttons, "radios": [], "draw": self._draw_main}

    def _build_auto(self):
        r = self.rect_body
        left_w = min(430, r.w * 56 // 100)
        x = r.x + 20
        y = r.y + 56
        profile_h = max(50, (r.h - 205) // 3)
        buttons = [self._back_btn()]
        self.profile_buttons = {}
        for mode in ("task1", "task2_white", "task2_card"):
            label = TASK_PROFILES[mode]["label"]
            btn = Button(
                (x, y, left_w, profile_h),
                label,
                lambda m=mode: self._set_task_mode(m),
                color=C_BTN,
            )
            self.profile_buttons[mode] = btn
            buttons.append(btn)
            y += profile_h + 10
        buttons.append(
            Button(
                (x, y, left_w, 42),
                "检查/重新标定",
                self._on_calibrate,
                color=C_BLUE,
            )
        )
        self.btn_start = Button(
            (x, r.bottom - 70, left_w, 60),
            "一 键 开 始 比 赛",
            self._on_start,
            color=C_GREEN,
            font_key="title",
        )
        buttons.append(self.btn_start)
        self.rect_checklist = pygame.Rect(
            x + left_w + 18,
            r.y + 56,
            r.right - (x + left_w + 18) - 16,
            r.h - 72,
        )
        return {"buttons": buttons, "radios": [], "draw": self._draw_auto}

    def _build_run(self):
        r = self.rect_body
        self.rect_run_view = pygame.Rect(
            r.x + 14, r.y + 56, max(320, r.w * 62 // 100), r.h - 104
        )
        side_x = self.rect_run_view.right + 14
        self.rect_run_side = pygame.Rect(
            side_x, r.y + 56, r.right - side_x - 12, r.h - 104
        )
        self.rect_run_progress = pygame.Rect(
            self.rect_run_side.x + 12,
            self.rect_run_side.bottom - 76,
            self.rect_run_side.w - 24,
            16,
        )
        buttons = [
            Button(
                (
                    self.rect_run_side.x + 12,
                    self.rect_run_side.bottom - 48,
                    self.rect_run_side.w - 24,
                    38,
                ),
                "中止并返回",
                self._on_abort,
                color=C_ORANGE,
            )
        ]
        return {"buttons": buttons, "radios": [], "draw": self._draw_run}

    def _build_result(self):
        r = self.rect_body
        bw = min(210, (r.w - 100) // 3)
        y = r.bottom - 66
        x = r.x + (r.w - (bw * 3 + 28)) // 2
        buttons = [
            self._back_btn(),
            Button(
                (x, y, bw, 48),
                "返回比赛准备",
                lambda: self._goto("auto"),
                color=C_BLUE,
            ),
            Button(
                (x + bw + 14, y, bw, 48),
                "查看日志",
                lambda: self._goto("log"),
                color=C_BTN,
            ),
            Button(
                (x + (bw + 14) * 2, y, bw, 48),
                "再次运行",
                self._on_start,
                color=C_GREEN,
            ),
        ]
        return {"buttons": buttons, "radios": [], "draw": self._draw_result}

    def _build_calibration(self):
        r = self.rect_body
        self.rect_calib_view = pygame.Rect(
            r.x + 12, r.y + 56, max(320, r.w * 62 // 100), r.h - 104
        )
        x = self.rect_calib_view.right + 12
        w = max(120, r.right - x - 12)
        top = r.y + 56
        gap = 7
        bh = max(38, (r.h - 104 - gap * 5) // 6)
        actions = [
            ("① 工作区标定", self._calibrate_workspace, C_BLUE),
            ("② 释放力矩", self._release_calib_torque, C_ORANGE),
            ("③ 记录触点", self._record_touch, C_GREEN),
            ("撤销触点", self._undo_touch, C_BTN),
            ("④ 解算基座", self._solve_base, C_BLUE),
            ("重新打开相机", self._open_calib_camera, C_BTN),
        ]
        buttons = [self._back_btn()]
        for i, (text, callback, color) in enumerate(actions):
            buttons.append(
                Button(
                    (x, top + i * (bh + gap), w, bh),
                    text,
                    callback,
                    color=color,
                    font_key="small",
                )
            )
        return {"buttons": buttons, "radios": [], "draw": self._draw_calibration}

    def _build_manual(self):
        r = self.rect_body
        x = r.x + 24
        w = r.w - 48
        y = r.y + 56  # 顶部留给返回按钮
        half = (w - 12) // 2
        bh = max(42, (r.h - 220) // 6)
        buttons = [
            self._back_btn(),
            Button(
                (x, y, half, bh),
                "力矩使能",
                lambda: self.backend.set_torque(True),
                color=C_GREEN,
            ),
            Button(
                (x + half + 12, y, half, bh),
                "力矩失能",
                lambda: self.backend.set_torque(False),
                color=C_ORANGE,
            ),
        ]
        y += bh + 10
        buttons += [
            Button(
                (x, y, half, bh), "电磁铁 吸", lambda: self.backend.set_magnet(True)
            ),
            Button(
                (x + half + 12, y, half, bh),
                "电磁铁 放",
                lambda: self.backend.set_magnet(False),
            ),
        ]
        y += bh + 10
        buttons.append(Button((x, y, w, bh), "回纸外驻车位 (Park)", self.backend.home))
        y += bh + 14
        # --- Tab 栏: 关节 | 末端 ---
        tab_w = half
        self.rect_tabs = {
            "joint": pygame.Rect(x, y, tab_w, 44),
            "xyz": pygame.Rect(x + tab_w + 12, y, tab_w, 44),
        }
        tab_y = y + 44 + 16

        # --- Tab[关节]: 步长 radio + J1..J5 两行排布 ---
        radios = [
            Radio(
                x,
                tab_y + 4,
                [(f"{s}°", s) for s in (1, 5, 10, 30)],
                lambda: self.joint_step,
                lambda v: setattr(self, "joint_step", v),
            ),
            Radio(
                x,
                tab_y + 4,
                [(f"{s}mm", s) for s in (1, 5, 10, 50)],
                lambda: self.xyz_step,
                lambda v: setattr(self, "xyz_step", v),
            ),
        ]
        radios[0]._tab = "joint"
        radios[1]._tab = "xyz"
        row_h = max(40, (r.bottom - tab_y - 60) // 3)
        jy = tab_y + 40
        bw2 = 64
        # J1-J3 第一行, J4-J5 第二行
        self._joint_rows = []
        per_row = 3
        for j in range(5):
            row, col = j // per_row, j % per_row
            cell_w = (w - 12 * (per_row - 1)) // per_row
            cx = x + col * (cell_w + 12)
            cy = jy + row * (row_h + 12)
            minus = Button(
                (cx, cy, bw2, row_h),
                "−",
                lambda j=j: self._jog(j, -1),
                font_key="title",
            )
            plus = Button(
                (cx + bw2 + 6, cy, bw2, row_h),
                "+",
                lambda j=j: self._jog(j, +1),
                font_key="title",
            )
            minus._tab = plus._tab = "joint"
            buttons += [minus, plus]
            self._joint_rows.append(
                (j, pygame.Rect(cx + bw2 * 2 + 14, cy, cell_w - bw2 * 2 - 14, row_h))
            )
        # --- Tab[末端]: XYZ 3x2 大按钮 ---
        cell_w = (w - 24) // 3
        xyz_h = max(48, (r.bottom - tab_y - 60) // 2)
        for i, (t, ax, sg) in enumerate(
            [
                ("X−", "x", -1),
                ("X+", "x", 1),
                ("Y−", "y", -1),
                ("Y+", "y", 1),
                ("Z−", "z", -1),
                ("Z+", "z", 1),
            ]
        ):
            b = Button(
                (
                    x + (i % 3) * (cell_w + 12),
                    jy + (i // 3) * (xyz_h + 14),
                    cell_w,
                    xyz_h,
                ),
                t,
                lambda a=ax, s=sg: self._jog_xyz(a, s),
                font_key="title",
            )
            b._tab = "xyz"
            buttons.append(b)
        for rd in radios:
            rd.layout(self.fonts)
        return {"buttons": buttons, "radios": radios, "draw": self._draw_manual}

    def _build_log(self):
        r = self.rect_body
        buttons = [
            self._back_btn(),
            Button((r.right - 306, r.y + 6, 92, 40), "导出", self._export_log),
            Button((r.right - 204, r.y + 6, 92, 40), "清空", self._clear_log),
        ]
        return {"buttons": buttons, "radios": [], "draw": self._draw_log_page}

    # ---------- 动作 ----------
    def _set_task_mode(self, mode):
        self.task_mode = mode
        profile = task_profile(mode)
        self.run_metrics.expected_count = profile["expected_pieces"]
        self._log("INFO", f"已选择 {profile['label']}")

    def _jog(self, j, sign):
        ok, msg = self.backend.jog_joint(j, sign * self.joint_step)
        if not ok:
            self._log("ERROR", msg)

    def _jog_xyz(self, axis, sign):
        d = sign * self.xyz_step
        self.backend.jog_xyz(*{"x": (d, 0, 0), "y": (0, d, 0), "z": (0, 0, d)}[axis])

    def _calib_result(self, result, mark=False):
        ok, msg = result
        self._log("INFO" if ok else "WARN", msg)
        if ok and mark:
            self.backend.mark_calibrated()

    def _open_calib_camera(self):
        self._calib_result(self.calibration.open_camera())

    def _calibrate_workspace(self):
        self._calib_result(self.calibration.calibrate_workspace(), mark=True)

    def _release_calib_torque(self):
        self._calib_result(self.backend.set_torque(False))

    def _record_touch(self):
        self._calib_result(self.calibration.record_touch())

    def _undo_touch(self):
        self._calib_result(self.calibration.undo_touch())

    def _solve_base(self):
        result = self.calibration.solve_base()
        if result[0]:
            self.backend.set_torque(True)
        self._calib_result(result, mark=True)

    def _on_calibrate(self):
        self.progress = 0.0
        self._goto("calibration")

    def _on_start(self):
        if self._task_running():
            self._log("WARN", "任务已在运行")
            return
        self.backend.mark_calibrated()
        self.state.calibrated = (
            Path("homography.npy").is_file() and Path("arm_base.json").is_file()
        )
        self.checks = self._collect_checks()
        if not self._task_ready():
            failed = [c for c in self.checks if c["status"] == "fail"]
            reason = "；".join(f"{c['label']}:{c['detail']}" for c in failed)
            self._log("WARN", f"比赛未就绪：{reason or '请检查连接和标定'}")
            return

        profile = task_profile(self.task_mode)
        self._reset_run()
        self.run_metrics.expected_count = profile["expected_pieces"]
        self.run_metrics.stage = "INIT"
        self.run_metrics.detail = "启动中"
        self.run_started_at = time.monotonic()
        ok, msg = self.backend.start_task(self.task_mode)
        self._log("INFO" if ok else "WARN", msg)
        if ok:
            self._goto("run")
        else:
            self._reset_run()

    def _on_abort(self):
        ok, msg = self.backend.abort()
        self._log("INFO" if ok else "WARN", msg)

    def _load_history(self):
        if not RUN_HISTORY_PATH.is_file():
            return
        try:
            with RUN_HISTORY_PATH.open("r", encoding="utf-8") as fh:
                for line in fh:
                    try:
                        item = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(item, dict):
                        self.history.append(item)
        except OSError:
            return
        self.history = self.history[-20:]

    def _persist_history(self, entry: dict):
        try:
            RUN_HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
            with RUN_HISTORY_PATH.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except OSError as exc:
            self._log("WARN", f"运行历史保存失败: {exc}")

    def _record_history(self, ok, msg):
        elapsed = self.run_metrics.elapsed_s
        if self.run_started_at is not None:
            elapsed = max(elapsed, time.monotonic() - self.run_started_at)
        entry = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": task_profile(self.task_mode)["label"],
            "ok": ok,
            "message": msg,
            "elapsed": elapsed,
            "detected": self.run_metrics.detected_count,
            "placed": self.run_metrics.placed_count,
        }
        self.history.append(entry)
        self.history = self.history[-20:]
        self._persist_history(entry)
        self.last_result = entry

    def _tick_run(self):
        if not self._task_running() or self.run_started_at is None:
            return
        elapsed = time.monotonic() - self.run_started_at
        self.run_metrics.elapsed_s = elapsed
        if elapsed >= self.run_metrics.limit_s and not self._timeout_sent:
            self._timeout_sent = True
            self._log("ERROR", "超过120秒限制，发送中止信号")
            self.backend.abort()

    def _export_log(self):
        try:
            log_dir = Path("out/gui_logs")
            log_dir.mkdir(parents=True, exist_ok=True)
            path = log_dir / f"gui_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
            lines = [
                f"# GUI日志导出 {time.strftime('%Y-%m-%d %H:%M:%S')}",
                "",
                "## 最近运行",
            ]
            for item in self.history[-10:]:
                lines.append(
                    f"[{item['time']}] {'成功' if item['ok'] else '失败'} "
                    f"{item['mode']} 用时{item['elapsed']:.1f}s: {item['message']}"
                )
            lines += ["", "## 日志"]
            lines += [f"[{level}] {msg}" for level, msg in self.logs]
            path.write_text("\n".join(lines), encoding="utf-8")
        except OSError as exc:
            self._log("ERROR", f"日志导出失败: {exc}")
            return
        self._log("INFO", f"日志已导出: {path}")

    # ---------- 日志 ----------
    def _log(self, level, msg):
        self.logs.append((level, msg))
        if len(self.logs) > 300:
            self.logs = self.logs[-300:]

    def _clear_log(self):
        self.logs.clear()

    # ---------- 渲染 ----------
    def _panel(self, rect, title=""):
        pygame.draw.rect(self.screen, C_PANEL, rect, border_radius=10)
        pygame.draw.rect(self.screen, C_LINE, rect, width=1, border_radius=10)
        if title:
            img = self.fonts["btn"].render(title, True, C_DIM)
            self.screen.blit(img, (rect.x + 16, rect.y + 10))

    def _draw_status(self):
        r = self.rect_status
        self._panel(r)
        st = self.state
        items = [
            (
                "连接",
                "已连接" if st.connected else "未连接",
                C_GREEN if st.connected else C_RED,
            ),
            (
                "模式",
                "模拟" if st.simulation else "硬件",
                C_ORANGE if st.simulation else C_GREEN,
            ),
            (
                "力矩",
                "使能" if st.torque_enabled else "失能",
                C_GREEN if st.torque_enabled else C_DIM,
            ),
            (
                "电磁铁",
                "吸合" if st.magnet_on else "释放",
                C_GREEN if st.magnet_on else C_DIM,
            ),
            (
                "标定",
                "已标定" if st.calibrated else "未标定",
                C_GREEN if st.calibrated else C_DIM,
            ),
            (
                "任务",
                STATE_LABELS.get(st.task_state, st.task_state),
                C_BLUE if st.task_state != "IDLE" else C_TEXT,
            ),
        ]
        f = self.fonts["small"]
        x = r.x + 16
        y = r.y + r.h // 2 - f.get_height() // 2
        for label, val, color in items:
            img = f.render(f"{label}: ", True, C_DIM)
            self.screen.blit(img, (x, y))
            x += img.get_width()
            img = f.render(val, True, color)
            self.screen.blit(img, (x, y))
            x += img.get_width() + 18
        joints = " ".join(f"J{j + 1} {d:.0f}°" for j, d in enumerate(st.joints_deg))
        img = f.render(joints, True, C_TEXT)
        jx = self.btn_estop.rect.x - img.get_width() - 16
        if jx > x + 8:  # 空间不足时不画, 防止与状态项重叠
            self.screen.blit(img, (jx, y))

    def _draw_boot(self):
        r = self.rect_body
        title = self.fonts["title"].render("系统自检", True, C_TEXT)
        title_rect = title.get_rect(topleft=(r.x + 24, r.y + 18))
        self.screen.blit(title, title_rect)
        fail_count = sum(c["status"] == "fail" for c in self.checks)
        warn_count = sum(c["status"] == "warn" for c in self.checks)
        summary = "核心检查失败，请先处理红色项目" if fail_count else "可以进入主界面"
        if warn_count and not fail_count:
            summary += f"（{warn_count}项警告）"
        color = C_RED if fail_count else C_GREEN
        img = self.fonts["small"].render(summary, True, color)
        self.screen.blit(
            img,
            (title_rect.right + 18, title_rect.centery - img.get_height() // 2),
        )

        colors = {"ok": C_GREEN, "warn": C_ORANGE, "fail": C_RED}
        labels = {"ok": "通过", "warn": "警告", "fail": "失败"}
        y = r.y + 74
        for item in self.checks:
            status = item["status"]
            color = colors[status]
            pygame.draw.circle(self.screen, color, (r.x + 36, y + 11), 7)
            text = self.fonts["btn"].render(
                f"{item['label']}：{labels[status]}", True, C_TEXT
            )
            self.screen.blit(text, (r.x + 54, y))
            detail = self.fonts["small"].render(item["detail"], True, C_DIM)
            self.screen.blit(detail, (r.x + 250, y + 4))
            y += 41

    def _draw_main(self):
        # 主菜单附加信息: 最近日志一行
        if self.logs:
            level, msg = self.logs[-1]
            colors = {"INFO": C_DIM, "WARN": C_ORANGE, "ERROR": C_RED}
            img = self.fonts["small"].render(
                f"[{level}] {msg}", True, colors.get(level, C_DIM)
            )
            r = self.rect_body
            self.screen.blit(img, (r.x + 20, r.bottom - img.get_height() - 8))

    def _draw_auto(self):
        r = self.rect_body
        title = self.fonts["title"].render("比赛准备", True, C_TEXT)
        title_rect = title.get_rect(topleft=(r.x + 20, r.y + 12))
        self.screen.blit(title, title_rect)
        profile = task_profile(self.task_mode)
        subtitle = self.fonts["small"].render(
            f"当前配置：{profile['label']} · 限时120秒 · 开始后立即移除摄像头遮挡",
            True,
            C_DIM,
        )
        self.screen.blit(
            subtitle,
            (title_rect.right + 16, title_rect.bottom - subtitle.get_height()),
        )

        cr = self.rect_checklist
        self._panel(cr, "赛前检查")
        colors = {"ok": C_GREEN, "warn": C_ORANGE, "fail": C_RED}
        labels = {"ok": "通过", "warn": "警告", "fail": "失败"}
        y = cr.y + 46
        for item in self.checks:
            status = item["status"]
            color = colors[status]
            pygame.draw.circle(self.screen, color, (cr.x + 20, y + 10), 6)
            img = self.fonts["small"].render(
                f"{item['label']}：{labels[status]}", True, C_TEXT
            )
            self.screen.blit(img, (cr.x + 36, y))
            y += 30
            detail = self.fonts["log"].render(item["detail"], True, C_DIM)
            self.screen.blit(detail, (cr.x + 36, y - 2))
            y += 24
        ready = self._task_ready()
        msg = "就绪：可一键开始" if ready else "未就绪：请处理失败项"
        img = self.fonts["btn"].render(msg, True, C_GREEN if ready else C_RED)
        self.screen.blit(img, (cr.x + 16, cr.bottom - img.get_height() - 12))

    def _format_seconds(self, seconds: float) -> str:
        return f"{seconds // 60:02.0f}:{seconds % 60:04.1f}"

    def _draw_wrapped(self, text, x, y, max_width, font, color, max_lines=4):
        line = ""
        lines = 0
        for ch in text:
            candidate = line + ch
            if font.size(candidate)[0] <= max_width:
                line = candidate
                continue
            img = font.render(line, True, color)
            self.screen.blit(img, (x, y))
            y += font.get_height() + 4
            lines += 1
            line = ch
            if lines >= max_lines:
                return y
        if line and lines < max_lines:
            img = font.render(line, True, color)
            self.screen.blit(img, (x, y))
            y += font.get_height() + 4
        return y

    def _draw_run(self):
        r = self.rect_run_view
        self._panel(r, "视觉画面")
        frame_rect = pygame.Rect(r.x + 8, r.y + 36, r.w - 16, r.h - 46)
        pygame.draw.rect(self.screen, (8, 8, 10), frame_rect)
        if self.latest_frame is not None:
            try:
                rgb = self.latest_frame[:, :, ::-1]
                surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
                surface = pygame.transform.smoothscale(surface, frame_rect.size)
                self.screen.blit(surface, frame_rect)
            except (TypeError, ValueError, pygame.error):
                self.latest_frame = None
        if self.latest_frame is None:
            text = self.fonts["btn"].render("等待自动任务相机帧", True, C_DIM)
            self.screen.blit(text, text.get_rect(center=frame_rect.center))
            note = self.fonts["small"].render("运行期间相机由视觉后端独占", True, C_DIM)
            self.screen.blit(
                note,
                note.get_rect(center=(frame_rect.centerx, frame_rect.centery + 34)),
            )
        pygame.draw.rect(self.screen, C_LINE, frame_rect, width=1)

        s = self.rect_run_side
        self._panel(s, "比赛运行")
        elapsed = self.run_metrics.elapsed_s
        if self.run_started_at is not None:
            elapsed = max(elapsed, time.monotonic() - self.run_started_at)
        remaining = max(0.0, self.run_metrics.limit_s - elapsed)
        time_color = C_GREEN
        if remaining <= 10:
            time_color = C_RED
        elif remaining <= 30:
            time_color = C_ORANGE
        img = self.fonts["title"].render(
            self._format_seconds(elapsed), True, time_color
        )
        self.screen.blit(img, (s.x + 16, s.y + 42))
        img = self.fonts["small"].render(
            f"剩余 {self._format_seconds(remaining)}", True, C_DIM
        )
        self.screen.blit(img, (s.x + 18, s.y + 82))

        m = self.run_metrics
        lines = [
            ("阶段", STATE_LABELS.get(m.stage, m.stage)),
            ("详情", m.detail),
            ("检测", f"{m.detected_count}/{m.expected_count}"),
            ("放置", f"{m.placed_count}/{m.expected_count}"),
            ("当前", f"碎片 {m.current_piece}" if m.current_piece else "-"),
        ]
        y = s.y + 116
        for label, value in lines:
            img = self.fonts["small"].render(f"{label}: ", True, C_DIM)
            self.screen.blit(img, (s.x + 16, y))
            val = self.fonts["small"].render(str(value), True, C_TEXT)
            self.screen.blit(val, (s.x + 72, y))
            y += 27

        stages = ["INIT", "CALIBRATING", "DETECTING", "SOLVING", "EXECUTING", "DONE"]
        current = stages.index(m.stage) if m.stage in stages else 0
        y = s.bottom - 116
        for i, stage in enumerate(stages):
            color = C_GREEN if i < current else C_BLUE if i == current else C_DIM
            pygame.draw.circle(self.screen, color, (s.x + 20, y + 8), 5)
            label = STATE_LABELS.get(stage, stage)
            img = self.fonts["log"].render(label, True, color)
            self.screen.blit(img, (s.x + 34, y))
            y += 17

        pr = self.rect_run_progress
        pygame.draw.rect(self.screen, C_BG, pr, border_radius=6)
        fill = pr.copy()
        try:
            fill.width = int(pr.w * max(0.0, min(1.0, m.progress)))
        except (TypeError, ValueError, OverflowError):
            fill.width = 0
        if fill.width > 0:
            pygame.draw.rect(self.screen, C_GREEN, fill, border_radius=6)
        pygame.draw.rect(self.screen, C_LINE, pr, width=1, border_radius=6)

    def _draw_result(self):
        r = self.rect_body
        entry = self.last_result or {
            "ok": False,
            "message": "暂无运行结果",
            "elapsed": 0.0,
            "mode": task_profile(self.task_mode)["label"],
            "detected": 0,
            "placed": 0,
        }
        ok = bool(entry["ok"])
        color = C_GREEN if ok else C_RED
        card = pygame.Rect(r.x + 70, r.y + 58, r.w - 140, r.h - 154)
        self._panel(card)
        pygame.draw.rect(self.screen, color, card, width=3, border_radius=10)
        title = "拼 图 完 成" if ok else "任 务 未 完 成"
        img = self.fonts["title"].render(title, True, color)
        self.screen.blit(img, img.get_rect(center=(card.centerx, card.y + 54)))
        info = [
            f"任务模式：{entry['mode']}",
            f"总耗时：{entry['elapsed']:.1f} 秒",
            f"检测碎片：{entry['detected']}/{self.run_metrics.expected_count}",
            f"成功放置：{entry['placed']}/{self.run_metrics.expected_count}",
        ]
        y = card.y + 104
        for text in info:
            img = self.fonts["btn"].render(text, True, C_TEXT)
            self.screen.blit(img, (card.x + 34, y))
            y += 32
        img = self.fonts["small"].render("结果说明：", True, C_DIM)
        self.screen.blit(img, (card.x + 34, y + 8))
        self._draw_wrapped(
            entry["message"],
            card.x + 34,
            y + 36,
            card.w - 68,
            self.fonts["small"],
            C_TEXT,
            max_lines=3,
        )

    def _draw_calibration(self):
        r = self.rect_calib_view
        frame = self.calibration.update()
        pygame.draw.rect(self.screen, (8, 8, 10), r)
        if frame is not None:
            rgb = frame[:, :, ::-1]
            surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            surface = pygame.transform.smoothscale(surface, r.size)
            self.screen.blit(surface, r)
        else:
            text = self.fonts["btn"].render("无本地相机画面", True, C_RED)
            self.screen.blit(text, text.get_rect(center=r.center))
        pygame.draw.rect(self.screen, C_LINE, r, width=2)
        steps = [
            ("工作区", self.calibration.homography is not None),
            ("释放力矩", not self.state.torque_enabled),
            ("触点≥3", len(self.calibration.pairs) >= 3),
            ("基座", Path("arm_base.json").is_file()),
        ]
        x = r.x
        y_steps = r.y - 48
        for i, (label, done) in enumerate(steps):
            color = C_GREEN if done else C_DIM
            pygame.draw.circle(self.screen, color, (x + 8, y_steps + 8), 5)
            img = self.fonts["log"].render(f"{i + 1}.{label}", True, color)
            self.screen.blit(img, (x + 20, y_steps))
            x += 118
        title = self.fonts["small"].render(
            "预览内点击目标点 → 手扶末端碰点 → 记录（至少3点）", True, C_DIM
        )
        self.screen.blit(title, (r.x, r.y - title.get_height() - 6))
        status = self.fonts["small"].render(
            f"{self.calibration.message}  已记录:{len(self.calibration.pairs)}",
            True,
            C_TEXT,
        )
        self.screen.blit(status, (r.x, r.bottom + 8))
        brightness = self.calibration.brightness
        sharpness = self.calibration.sharpness
        brightness_text = "-" if brightness is None else f"{brightness:.0f}"
        sharpness_text = "-" if sharpness is None else f"{sharpness:.0f}"
        quality_color = C_DIM
        if brightness is not None and sharpness is not None:
            if brightness < 45 or brightness > 220 or sharpness < 30:
                quality_color = C_ORANGE
            else:
                quality_color = C_GREEN
        quality = self.fonts["log"].render(
            f"亮度:{brightness_text}  清晰度:{sharpness_text}  "
            f"角点:{self.calibration.corner_count}",
            True,
            quality_color,
        )
        self.screen.blit(quality, (r.x, r.bottom + 29))

    def _draw_manual(self):
        # Tab 栏
        for key, rect in self.rect_tabs.items():
            on = self.manual_tab == key
            pygame.draw.rect(
                self.screen, C_TAB_ON if on else C_TAB, rect, border_radius=8
            )
            pygame.draw.rect(self.screen, C_LINE, rect, width=1, border_radius=8)
            img = self.fonts["btn"].render(
                "关节步进" if key == "joint" else "末端步进",
                True,
                C_TEXT if on else C_DIM,
            )
            self.screen.blit(img, img.get_rect(center=rect.center))
        # 关节读数 (仅 joint tab)
        if self.manual_tab == "joint":
            f = self.fonts["small"]
            for j, rect in self._joint_rows:
                deg = self.state.joints_deg[j] if j < len(self.state.joints_deg) else 0
                img = f.render(f"J{j + 1} {deg:+.1f}°", True, C_TEXT)
                self.screen.blit(
                    img, (rect.x + 6, rect.centery - img.get_height() // 2)
                )

    def _draw_log_page(self):
        r = self.rect_body
        title = self.fonts["btn"].render("最近运行", True, C_DIM)
        self.screen.blit(title, (r.x + 16, r.y + 10))
        y = r.y + 36
        for item in reversed(self.history[-3:]):
            color = C_GREEN if item["ok"] else C_RED
            img = self.fonts["small"].render(
                f"[{item['time']}] {'成功' if item['ok'] else '失败'} "
                f"{item['mode']} · {item['elapsed']:.1f}s · "
                f"检测{item['detected']} 放置{item['placed']}",
                True,
                color,
            )
            self.screen.blit(img, (r.x + 16, y))
            y += 24

        f = self.fonts["log"]
        lh = f.get_height() + 3
        top_reserved = 132 if self.history else 58
        n = max(1, (r.h - top_reserved) // lh)
        colors = {"INFO": C_TEXT, "WARN": C_ORANGE, "ERROR": C_RED}
        y = r.bottom - 10 - lh
        for level, msg in reversed(self.logs[-n:]):
            img = f.render(f"[{level}] {msg}", True, colors.get(level, C_TEXT))
            self.screen.blit(img, (r.x + 16, y))
            y -= lh

    def _visible_widgets(self):
        p = self.pages[self.page]
        buttons = list(p["buttons"])
        radios = list(p["radios"])
        if self.page == "auto":
            self.btn_start.enabled = self._task_ready()
            for mode, btn in self.profile_buttons.items():
                btn.selected = mode == self.task_mode
        if self._task_running():
            if self.page != "run":
                return [], []
            return buttons, []
        if self.page == "manual":
            buttons = [
                b
                for b in buttons
                if getattr(b, "_tab", self.manual_tab) == self.manual_tab
            ]
            radios = [
                r
                for r in radios
                if getattr(r, "_tab", self.manual_tab) == self.manual_tab
            ]
        return buttons, radios

    def draw(self):
        self.screen.fill(C_BG)
        self._draw_status()
        self._panel(self.rect_body)
        self.pages[self.page]["draw"]()
        buttons, radios = self._visible_widgets()
        for b in buttons:
            b.draw(self.screen, self.fonts)
        for rd in radios:
            rd.draw(self.screen, self.fonts)
        self.btn_estop.draw(self.screen, self.fonts)
        pygame.display.flip()

    # ---------- 主循环 ----------
    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            for raw_event in pygame.event.get():
                ev = normalize_pointer_event(raw_event, (self.W, self.H))
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    if self.page != "main":
                        self._goto("main")
                    else:
                        running = False
                elif ev.type == pygame.VIDEORESIZE:
                    self.W, self.H = ev.w, ev.h
                    self._build_all()
                elif self.btn_estop.handle(ev):
                    pass
                elif (
                    ev.type == pygame.MOUSEBUTTONDOWN
                    and self.page == "calibration"
                    and self.rect_calib_view.collidepoint(ev.pos)
                ):
                    self._calib_result(
                        self.calibration.set_target_from_view(
                            ev.pos, self.rect_calib_view
                        )
                    )
                elif ev.type == pygame.MOUSEBUTTONDOWN and self.page == "manual":
                    # Tab 点击
                    for key, rect in self.rect_tabs.items():
                        if rect.collidepoint(ev.pos):
                            self.manual_tab = key
                            break
                    else:
                        self._dispatch(ev)
                else:
                    self._dispatch(ev)
            self._drain()
            self._tick_run()
            self.draw()
            clock.tick(30)
        self.calibration.close()
        self.backend.disconnect()
        pygame.quit()

    def _dispatch(self, ev):
        buttons, radios = self._visible_widgets()
        for b in buttons:
            if b.handle(ev):
                return
        for rd in radios:
            rd.handle(ev)

    def _drain(self):
        try:
            while True:
                item = self._q.get_nowait()
                if item[0] == "log":
                    self._log(item[1], item[2])
                elif item[0] == "state":
                    self.state = item[1]
                elif item[0] == "progress":
                    self.progress_stage, self.progress = item[1], item[2]
                elif item[0] == "metrics":
                    self.run_metrics = item[1]
                elif item[0] == "frame":
                    self.latest_frame = item[1]
                elif item[0] == "done":
                    ok, msg = item[1], item[2]
                    self._record_history(ok, msg)
                    self.run_metrics.success = ok
                    self.run_metrics.message = msg
                    if self.last_result is not None:
                        self.run_metrics.elapsed_s = self.last_result["elapsed"]
                    self._log("INFO" if ok else "WARN", f"任务结束: {msg}")
                    self._goto("result")
        except queue.Empty as exc:
            _ = exc


def _parse_size():
    if "--window" in sys.argv:
        i = sys.argv.index("--window")
        if i + 1 < len(sys.argv) and "x" in sys.argv[i + 1]:
            w, h = sys.argv[i + 1].split("x")
            try:
                return int(w), int(h)
            except ValueError:
                return (960, 600)
        return (960, 600)
    return None


if __name__ == "__main__":
    try:
        FbGUI(_parse_size()).run()
    except pygame.error as e:
        # kmsdrm 失败时 (比如在桌面上跑) 回退窗口模式重试
        if os.environ.get("SDL_VIDEODRIVER") == "kmsdrm":
            print(f"[GUI] kmsdrm 打开失败 ({e}), 回退窗口模式")
            del os.environ["SDL_VIDEODRIVER"]
            os.environ["SDL_VIDEODRIVER"] = "x11,wayland"
            FbGUI((960, 600)).run()
        else:
            raise
