# pyright: reportMissingImports=false
# ruff: noqa: I001, E402

"""正式赛场 GUI（pick_place_page）headless 回归测试。

前置条件：
- `pip install pygame-ce`（宿主 conda base 已装）；
- 本文件自行把 `workspace/src`、`workspace/apps`、`workspace/rpi_gui_snapshot`
  加入 sys.path，并用 SDL dummy 驱动离屏运行，不需要真实 fb0/触摸屏。

运行：
    python3 workspace/tests/test_pick_place_page.py -v
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "src"))
sys.path.insert(0, str(WORKSPACE / "apps"))
sys.path.insert(0, str(WORKSPACE / "rpi_gui_snapshot"))

import pygame

import pick_place_page
from integration import competition_backend
from integration.competition_backend import (
    BackendCapability,
    CapabilityAvailability,
    CompetitionBackend,
    JOINT_ORDER,
    SupervisorRunBackend,
)
from pick_place_page import ActionInfo, PageState, PickPlacePage

FAKE_SUPERVISOR = """
import json, os, sys, time
from pathlib import Path


def arg(name):
    return sys.argv[sys.argv.index(name) + 1]


command = sys.argv[1]
status_path = Path(arg("--status-file"))
result_path = Path(arg("--result-file"))
sleep_s = float(os.environ.get("FAKE_SLEEP", "0.2"))
status_path.parent.mkdir(parents=True, exist_ok=True)
status_path.write_text(json.dumps({
    "schema_version": 1,
    "command": command,
    "phase": "fake-phase-1",
    "progress": 0.5,
    "updated_at_ns": time.time_ns(),
}), encoding="utf-8")
time.sleep(sleep_s)
mode = os.environ.get("FAKE_RESULT", "PASS")
if mode == "corrupt":
    result_path.write_text("{not json", encoding="utf-8")
elif mode != "none":
    result_path.write_text(json.dumps({
        "schema_version": 1,
        "command": command,
        "result": mode,
        "magnet_enabled": False,
        "message": os.environ.get("FAKE_MESSAGE", "fake detail"),
        "duration_s": sleep_s,
        "events": [{"phase": "fake-phase-1"}],
    }), encoding="utf-8")
sys.exit(int(os.environ.get("FAKE_EXIT", "0")))
"""

DEMO_PIECES = {
    "schema_version": 1,
    "frame_size": [1280, 720],
    "pieces": [
        {
            "piece_id": f"red-{index}",
            "timestamp_ns": 1,
            "frame": "table_mm",
            "polygon_table_mm": [[0, 0], [1, 0], [1, 1]],
            "pick_point_table_mm": [x, y],
            "center_table_mm": [x, y],
            "yaw_deg": yaw,
            "confidence": 0.9,
            "diagnostics": {"center_pixel": [px, py], "pixel_area": 1600.0},
        }
        for index, (x, y, yaw, px, py) in enumerate(
            [
                (52.0, 210.0, 12.5, 300.0, 200.0),
                (110.0, 150.0, -45.0, 640.0, 360.0),
                (168.0, 90.0, 78.0, 950.0, 520.0),
            ]
        )
    ],
}


def seed_detection(runtime_dir: Path) -> None:
    detection_dir = runtime_dir / "red_piece_detection"
    detection_dir.mkdir(parents=True, exist_ok=True)
    (detection_dir / "pieces.json").write_text(
        json.dumps(DEMO_PIECES, ensure_ascii=False), encoding="utf-8"
    )
    surface = pygame.Surface((1280, 720))
    surface.fill((12, 12, 16))
    for piece in DEMO_PIECES["pieces"]:
        pixel = piece["diagnostics"]["center_pixel"]
        pygame.draw.rect(
            surface,
            (180, 40, 40),
            (pixel[0] - 30, pixel[1] - 20, 60, 40),
        )
    pygame.image.save(surface, str(detection_dir / "annotated.jpg"))


class FakeBackend(CompetitionBackend):
    """测试用后端：可通过 ``caps`` 预置能力，并记录 build_request 调用。"""

    def __init__(
        self,
        fake_tool: Path,
        caps: dict[str, BackendCapability] | None = None,
    ) -> None:
        self.caps = caps or {}
        self.requests: list[tuple[str, dict[str, object]]] = []
        self._robot_available = True
        self._lock_reason: str | None = None
        self.fake_tool = fake_tool
        self.supervisor_tool = fake_tool

    def capabilities(self) -> dict[str, BackendCapability]:
        return dict(self.caps)

    def build_request(
        self,
        command_id: str,
        parameters: dict[str, object],
        *,
        status_path: Path,
        result_path: Path,
    ) -> competition_backend.BackendRequest:
        from integration.competition_backend import BackendRequest

        self.requests.append((command_id, parameters))
        return BackendRequest(
            command_id=command_id,
            argv=[
                sys.executable,
                str(self.fake_tool),
                command_id,
                "--status-file",
                str(status_path),
                "--result-file",
                str(result_path),
            ],
            timeout_s=60.0,
            status_path=status_path,
            result_path=result_path,
        )

    def robot_motion_lock_reason(self) -> str | None:
        return self._lock_reason

    def robot_available(self) -> bool:
        return self._robot_available


def all_ready_caps() -> dict[str, BackendCapability]:
    return {
        command_id: BackendCapability(
            command_id=command_id,
            availability=CapabilityAvailability.READY,
            reason="ready",
            requires_robot=command_id != "detect" and command_id != "puzzle-shadow",
            requires_manual_unlock=command_id
            in {"manual-jog-joint", "manual-jog-cartesian", "manual-magnet-pulse"},
        )
        for command_id in SupervisorRunBackend.COMMAND_TO_MODE
    }


class PickPlacePageTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self._env_backup = dict(os.environ)
        self.temp_dir = tempfile.TemporaryDirectory()
        self.runtime_dir = Path(self.temp_dir.name) / "runtime"
        self.fake_tool = Path(self.temp_dir.name) / "fake_supervisor_run.py"
        self.fake_tool.write_text(FAKE_SUPERVISOR, encoding="utf-8")
        self.robot_device = Path(self.temp_dir.name) / "epuzzle-arm"
        self.robot_device.touch()
        self.motion_lock = Path(self.temp_dir.name) / "robot-motion-lock.json"
        os.environ["EP_GUI_RUNTIME_DIR"] = str(self.runtime_dir)
        os.environ["EPUZZLE_ROBOT_PORT"] = str(self.robot_device)
        os.environ["EPUZZLE_ROBOT_MOTION_LOCK"] = str(self.motion_lock)
        os.environ.pop("EP_GUI_FONT", None)
        os.environ.pop("EPUZZLE_MAGNET_ENABLED", None)

        self.backend = FakeBackend(self.fake_tool, all_ready_caps())
        self.page = PickPlacePage(backend=self.backend)
        self.page.python_path = sys.executable
        self.page.supervisor_tool = str(self.fake_tool)

    def tearDown(self) -> None:
        self.page._stop_active_process(graceful_timeout_s=1.0)
        if self.page.worker is not None:
            self.page.worker.join(timeout=2.0)
        pygame.quit()
        self.temp_dir.cleanup()
        os.environ.clear()
        os.environ.update(self._env_backup)

    def pump(self, timeout_s: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_s
        worker = self.page.worker
        while worker is not None and worker.is_alive():
            self.page._drain()
            time.sleep(0.02)
            if time.monotonic() > deadline:
                self.fail("动作在测试时限内未结束")
        self.page._drain()

    def fake_action(self, command: str = "park") -> ActionInfo:
        result_path = self.runtime_dir / "gui" / f"result_{command}.json"
        result_path.parent.mkdir(parents=True, exist_ok=True)
        return ActionInfo(
            command=command,
            label="测试动作",
            argv=[],
            status_path=self.runtime_dir / "gui" / "status.json",
            result_path=result_path,
            timeout_s=60.0,
            started_monotonic=time.monotonic(),
        )

    def mouse_click(self, pos: tuple[int, int]) -> None:
        self.page.handle_event(
            pygame.event.Event(pygame.MOUSEBUTTONDOWN, {"pos": pos, "button": 1})
        )
        self.page.handle_event(
            pygame.event.Event(pygame.MOUSEBUTTONUP, {"pos": pos, "button": 1})
        )


class ScreenshotTest(PickPlacePageTestCase):
    def test_offscreen_pages_render_non_black_and_colorful(self) -> None:
        seed_detection(self.runtime_dir)
        self.page._reload_detection()
        for page_name in ("competition", "run", "manual", "debug"):
            with self.subTest(page=page_name):
                self.page.page = page_name
                self.page.draw()
                self.assertEqual(self.page.screen.get_size(), (1024, 600))
                colors = set()
                for x in range(0, 1024, 32):
                    for y in range(0, 600, 32):
                        colors.add(self.page.screen.get_at((x, y))[:3])
                self.assertGreater(len(colors), 10)
                self.assertNotEqual(colors, {(0, 0, 0)})
                output = Path(self.temp_dir.name) / f"shot_{page_name}.png"
                pygame.image.save(self.page.screen, str(output))
                self.assertGreater(output.stat().st_size, 5000)


class LayoutTest(PickPlacePageTestCase):
    def test_buttons_in_screen_no_overlap_and_estop_hot_area(self) -> None:
        estop_rect = self.page.estop_button.rect
        self.assertGreaterEqual(estop_rect.width, 96)
        self.assertGreaterEqual(estop_rect.height, 48)
        for manual_tab in ("joint", "cartesian"):
            self.page.manual_tab = manual_tab
            self.page._refresh_button_states()
            for page_name, entry in self.page.pages.items():
                with self.subTest(page=page_name, tab=manual_tab):
                    buttons = list(entry["buttons"]) + [self.page.estop_button]
                    for button in buttons:
                        if not button.visible:
                            continue
                        rect = button.rect
                        self.assertGreaterEqual(rect.left, 0)
                        self.assertGreaterEqual(rect.top, 0)
                        self.assertLessEqual(rect.right, 1024)
                        self.assertLessEqual(rect.bottom, 600)
                        self.assertGreaterEqual(rect.height, 44)
                        if button is not self.page.estop_button:
                            self.assertLessEqual(
                                rect.bottom,
                                pick_place_page.BOTTOM_BAR_TOP,
                                f"{button.text} 覆盖底栏",
                            )
                    visible_buttons = [button for button in buttons if button.visible]
                    for index, first in enumerate(visible_buttons):
                        for second in visible_buttons[index + 1 :]:
                            self.assertFalse(
                                first.rect.colliderect(second.rect),
                                f"{first.text} 与 {second.text} 重叠",
                            )


class ResultJsonTest(PickPlacePageTestCase):
    def finish(self, returncode, timed_out=False, error=""):
        self.page.state = PageState.RUNNING
        self.page._finish_action(returncode, timed_out, error)

    def test_pass_result_returns_to_idle_with_duration(self) -> None:
        self.page.current_action = self.fake_action("park")
        self.fake_action("park")["result_path"].write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "command": "park",
                    "result": "PASS",
                    "magnet_enabled": False,
                    "message": "",
                    "duration_s": 3.2,
                    "events": [],
                }
            ),
            encoding="utf-8",
        )
        self.finish(0)
        self.assertEqual(self.page.state, PageState.IDLE)
        self.assertEqual(self.page.message_level, "INFO")
        self.assertIn("3.2s", self.page.message_text)

    def test_missing_result_file_is_fault(self) -> None:
        self.page.current_action = self.fake_action("park")
        self.finish(0)
        self.assertEqual(self.page.state, PageState.FAULT)
        self.assertEqual(self.page.message_level, "ERROR")
        self.assertIn("结果文件", self.page.message_text)

    def test_corrupt_result_file_is_fault(self) -> None:
        action = self.fake_action("park")
        action["result_path"].write_text("{broken", encoding="utf-8")
        self.page.current_action = action
        self.finish(0)
        self.assertEqual(self.page.state, PageState.FAULT)

    def test_fail_result_uses_file_message(self) -> None:
        action = self.fake_action("sweep-shortest")
        action["result_path"].write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "command": "sweep-shortest",
                    "result": "FAIL",
                    "magnet_enabled": False,
                    "message": "机械臂不在正式 PARK，拒绝页面动作",
                    "duration_s": 1.0,
                    "events": [],
                }
            ),
            encoding="utf-8",
        )
        self.page.current_action = action
        self.finish(2)
        self.assertEqual(self.page.state, PageState.FAULT)
        self.assertIn("拒绝页面动作", self.page.message_text)

    def test_stdout_text_is_never_used_as_status(self) -> None:
        action = self.fake_action("detect")
        action["result_path"].write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "command": "detect",
                    "result": "PASS",
                    "magnet_enabled": False,
                    "message": "",
                    "duration_s": 1.0,
                    "events": [],
                }
            ),
            encoding="utf-8",
        )
        self.page.current_action = action
        self.finish(0)
        self.assertEqual(self.page.state, PageState.IDLE)
        self.assertNotIn("print", self.page.message_text)


class HeartbeatTest(PickPlacePageTestCase):
    def test_heartbeat_parsed_and_corrupt_ignored(self) -> None:
        action = self.fake_action("sweep-shortest")
        self.page.current_action = action
        self.page._heartbeat_read_at = 0.0
        self.page._read_heartbeat()
        self.assertIsNone(self.page.heartbeat)
        action["status_path"].write_text(
            json.dumps({"schema_version": 1, "phase": "p", "progress": 0.5}),
            encoding="utf-8",
        )
        self.page._heartbeat_read_at = 0.0
        self.page._read_heartbeat()
        self.assertEqual(self.page.heartbeat["phase"], "p")
        action["status_path"].write_text("{broken", encoding="utf-8")
        self.page._heartbeat_read_at = 0.0
        self.page._read_heartbeat()
        self.assertEqual(self.page.heartbeat["phase"], "p")


class ButtonStateTest(PickPlacePageTestCase):
    def test_actions_greyed_but_stop_remains_available_while_running(self) -> None:
        self.page.state = PageState.RUNNING
        self.page._refresh_button_states()
        for entry in self.page.pages.values():
            for button in entry["buttons"]:
                if not button.visible:
                    continue
                if button.action and button.text != "停止":
                    self.assertFalse(button.enabled, button.text)
                elif button.text == "停止":
                    self.assertTrue(button.enabled)
                else:
                    self.assertTrue(button.enabled, button.text)
        park_button = next(
            button
            for button in self.page.pages["manual"]["buttons"]
            if button.text == "回 PARK"
        )
        park_button.on_click = mock.Mock()
        self.mouse_click(park_button.rect.center)
        park_button.on_click.assert_not_called()
        self.assertEqual(self.page.state, PageState.RUNNING)

    def test_stop_before_worker_launch_prevents_popen(self) -> None:
        deferred_threads = []

        class DeferredThread:
            def __init__(self, *, target, daemon: bool) -> None:
                self.target = target
                self.daemon = daemon
                deferred_threads.append(self)

            def start(self) -> None:
                return None

            def is_alive(self) -> bool:
                return False

            def join(self, timeout: float | None = None) -> None:
                return None

        with (
            mock.patch.object(pick_place_page.threading, "Thread", DeferredThread),
            mock.patch.object(self.page, "_request_magnet_off"),
            mock.patch.object(pick_place_page.subprocess, "Popen") as popen,
        ):
            self.page._start_backend_request("detect", "刷新检测", {})
            self.page.safe_abort()
            deferred_threads[0].target()
        popen.assert_not_called()
        self.page._drain()
        self.assertEqual(self.page.state, PageState.FAULT)

    def test_actions_enabled_again_in_fault_for_recovery(self) -> None:
        self.page.state = PageState.FAULT
        self.page._refresh_button_states()
        for entry in self.page.pages.values():
            for button in entry["buttons"]:
                if not button.visible:
                    continue
                # 维护解锁按钮在 FAULT 下仍需要解锁条件，不强制启用。
                if button.text == "启动电磁铁" or button.tab in {
                    "joint",
                    "cartesian",
                }:
                    continue
                self.assertTrue(button.enabled, button.text)

    def test_missing_robot_locks_motion_but_keeps_detect_and_shadow(self) -> None:
        self.backend._robot_available = False
        self.page.run_self_checks()
        self.page._refresh_button_states()

        buttons = {
            button.text: button
            for entry in self.page.pages.values()
            for button in entry["buttons"]
        }
        for text in (
            "回 PARK",
            "五轴断力矩",
            "启动电磁铁",
        ):
            self.assertFalse(buttons[text].enabled, text)
        self.assertTrue(buttons["刷新检测"].enabled)
        arm_check = next(
            check for check in self.page.self_checks if check[0] == "机械臂设备"
        )
        self.assertEqual(arm_check[1], "FAIL")

    def test_estop_remains_available_when_robot_is_missing(self) -> None:
        self.backend._robot_available = False
        self.page._refresh_button_states()
        self.assertTrue(self.page.estop_button.enabled)

    def test_incident_lock_blocks_motion_even_if_device_reappears(self) -> None:
        self.motion_lock.write_text(
            json.dumps({"reason": "xHCI host controller died"}), encoding="utf-8"
        )
        self.page.run_self_checks()
        self.page._refresh_button_states()
        lock_check = next(
            check for check in self.page.self_checks if check[0] == "运动安全锁"
        )
        self.assertEqual(lock_check[1], "FAIL")
        self.assertIn("xHCI", lock_check[2])
        park_button = next(
            button
            for button in self.page.pages["manual"]["buttons"]
            if button.text == "回 PARK"
        )
        self.assertFalse(park_button.enabled)
        self.assertTrue(self.page.estop_button.enabled)


class SelectTest(PickPlacePageTestCase):
    def test_select_without_pieces_warns(self) -> None:
        self.page.select(1)
        self.assertEqual(self.page.selected_index, 0)
        self.assertEqual(self.page.message_level, "WARN")

    def test_select_wraps_in_both_directions(self) -> None:
        self.page.pieces = list(DEMO_PIECES["pieces"])
        self.page.select(-1)
        self.assertEqual(self.page.selected_index, 2)
        self.page.select(1)
        self.assertEqual(self.page.selected_index, 0)
        self.page.select(1)
        self.assertEqual(self.page.selected_index, 1)


class EstopTest(PickPlacePageTestCase):
    def test_estop_uses_no_sudo_and_latches_until_reset(self) -> None:
        run_calls: list[list[str]] = []
        popen_calls: list[list[str]] = []

        def fake_run(argv, **kwargs):
            run_calls.append(list(argv))
            return mock.Mock(returncode=0)

        def fake_popen(argv, **kwargs):
            popen_calls.append(list(argv))
            return mock.Mock(
                poll=lambda: 0,
                communicate=lambda: ("", ""),
                returncode=0,
            )

        with mock.patch.object(pick_place_page.subprocess, "run", fake_run):
            with mock.patch.object(pick_place_page.subprocess, "Popen", fake_popen):
                self.page.emergency_stop()
            deadline = time.monotonic() + 3.0
            while not run_calls and time.monotonic() < deadline:
                time.sleep(0.02)
        self.assertEqual(self.page.state, PageState.ESTOP)
        self.assertTrue(run_calls, "急停必须调用断力矩工具")
        flat = [str(part) for call in run_calls for part in call]
        self.assertNotIn("sudo", flat)
        self.assertFalse(any("systemctl" in part for part in flat))
        self.assertTrue(any("emergency_off_single_arm.py" in part for part in flat))
        # 急停必须同时请求磁铁 OFF。
        popen_flat = [str(part) for call in popen_calls for part in call]
        self.assertTrue(
            any("manual-magnet-off" in part for part in popen_flat),
            "急停必须请求 manual-magnet-off",
        )
        self.page._drain()
        self.assertEqual(self.page.message_level, "ERROR")
        self.assertIn("断力矩", self.page.message_text)

        self.page._refresh_button_states()
        self.assertTrue(self.page.reset_button.enabled)
        for entry in self.page.pages.values():
            for button in entry["buttons"]:
                if button.action:
                    self.assertFalse(button.enabled, button.text)

    def test_estop_reports_unconfirmed_torque_off_failure(self) -> None:
        def failed_run(argv, **kwargs):
            return mock.Mock(returncode=1)

        with mock.patch.object(pick_place_page.subprocess, "run", failed_run):
            self.page.emergency_stop()
            deadline = time.monotonic() + 3.0
            while self.page.messages.empty() and time.monotonic() < deadline:
                time.sleep(0.02)
        self.page._drain()
        self.assertEqual(self.page.state, PageState.ESTOP)
        self.assertEqual(self.page.message_level, "ERROR")
        self.assertIn("断力矩未确认", self.page.message_text)
        self.assertIn("切断舵机电源", self.page.message_text)

    def test_estop_button_reachable_by_touch_on_every_page(self) -> None:
        for page_name in ("competition", "run", "manual", "debug"):
            with self.subTest(page=page_name):
                self.page.page = page_name
                self.page.estop_button.on_click = mock.Mock()
                center = self.page.estop_button.rect.center
                self.page.handle_event(
                    pygame.event.Event(
                        pygame.FINGERDOWN,
                        {
                            "x": center[0] / 1024,
                            "y": center[1] / 600,
                            "finger_id": 1,
                            "touch_id": 1,
                        },
                    )
                )
                self.page.handle_event(
                    pygame.event.Event(
                        pygame.FINGERUP,
                        {
                            "x": center[0] / 1024,
                            "y": center[1] / 600,
                            "finger_id": 1,
                            "touch_id": 1,
                        },
                    )
                )
                self.page.estop_button.on_click.assert_called_once()
                self.page.estop_button.on_click = self.page.emergency_stop


class FakeSupervisorRunTest(PickPlacePageTestCase):
    def test_detect_pass_reloads_detection(self) -> None:
        seed_detection(self.runtime_dir)
        os.environ["FAKE_SLEEP"] = "0.1"
        self.page.detect()
        self.assertEqual(self.page.state, PageState.RUNNING)
        self.pump()
        self.assertEqual(self.page.state, PageState.IDLE)
        self.assertEqual(len(self.page.pieces), 3)
        self.assertIn("完成", self.page.message_text)
        self.assertIsNotNone(self.page.last_result)

    def test_timeout_becomes_fault(self) -> None:
        os.environ["FAKE_SLEEP"] = "5.0"
        self.page.timeout_overrides["park"] = 1.0
        self.page.park()
        self.pump(timeout_s=10.0)
        self.assertEqual(self.page.state, PageState.FAULT)
        self.assertIn("超过", self.page.message_text)
        self.assertIn("已终止", self.page.message_text)

    def test_fail_result_from_tool(self) -> None:
        os.environ["FAKE_RESULT"] = "FAIL"
        os.environ["FAKE_MESSAGE"] = "PARK 门拒绝"
        os.environ["FAKE_EXIT"] = "2"
        self.page.park()
        self.pump()
        self.assertEqual(self.page.state, PageState.FAULT)
        self.assertIn("PARK 门拒绝", self.page.message_text)

    def test_missing_result_file_with_zero_exit_is_fault(self) -> None:
        os.environ["FAKE_RESULT"] = "none"
        self.page.park()
        self.pump()
        self.assertEqual(self.page.state, PageState.FAULT)
        self.assertIn("结果文件", self.page.message_text)


class FontAndLockTest(PickPlacePageTestCase):
    def test_pick_place_lock_message_is_rendered_when_gate_rejects(self) -> None:
        with mock.patch.object(
            pick_place_page,
            "pick_place_lock_reason",
            return_value="PICK/PLACE 接触高度 UNKNOWN：未验收",
        ):
            self.page.run_self_checks()
            self.page.draw()
        check = next(item for item in self.page.self_checks if item[0] == "正式抓放")
        self.assertEqual(check[1], "UNKNOWN")
        self.assertIn("UNKNOWN", check[2])

    def test_missing_cjk_font_raises_on_screen_error(self) -> None:
        with mock.patch.object(pick_place_page, "find_font", return_value=None):
            page = PickPlacePage(backend=self.backend)
        self.assertTrue(page.font_missing)
        page.draw()
        pixel = page.screen.get_at((20, 62))
        self.assertGreater(pixel.r, 40)
        self.assertLess(pixel.g, 40)


class SelfCheckTest(PickPlacePageTestCase):
    def test_self_checks_cover_font_tool_magnet_and_lock(self) -> None:
        self.page.run_self_checks()
        names = [name for name, _, _ in self.page.self_checks]
        for expected in (
            "中文字体",
            "supervisor_run 工具",
            "运动学标定",
            "电磁铁控制链",
            "正式抓放",
        ):
            self.assertIn(expected, names)
        statuses = {name: status for name, status, _ in self.page.self_checks}
        self.assertEqual(statuses["电磁铁控制链"], "PASS")
        self.assertEqual(statuses["supervisor_run 工具"], "PASS")


class CompetitionPageTest(PickPlacePageTestCase):
    def test_three_modes_render_without_competition_timer(self) -> None:
        source = Path(pick_place_page.__file__).read_text(encoding="utf-8")
        self.assertNotIn("COMPETITION_TIMEOUT_S", source)
        self.assertNotIn("官方限时", source)
        self.page.page = "competition"
        self.page.draw()
        surface = self.page.screen
        # 采样更密以确保检测到多种颜色。
        colors = {
            surface.get_at((x, y))[:3]
            for x in range(0, 1024, 32)
            for y in range(0, 600, 32)
        }
        self.assertGreater(len(colors), 8)
        # 简单 OCR-free 断言：检查某行像素包含非黑非背景色。
        row_colors = {surface.get_at((x, 140))[:3] for x in range(1024)}
        self.assertGreater(len(row_colors), 3)

    def test_unavailable_mode_cannot_start(self) -> None:
        self.backend.caps["competition-self-color"] = BackendCapability(
            command_id="competition-self-color",
            availability=CapabilityAvailability.UNAVAILABLE,
            reason="未接入",
            requires_robot=True,
            requires_manual_unlock=False,
        )
        self.page._refresh_capabilities()
        self.page._refresh_button_states()
        self.page._select_competition_mode("competition-self-color")
        start_button = next(
            button
            for button in self.page.pages["competition"]["buttons"]
            if button.text == "一键启动"
        )
        self.assertFalse(start_button.enabled)
        start_button.on_click = mock.Mock()
        self.mouse_click(start_button.rect.center)
        start_button.on_click.assert_not_called()
        self.assertIn("未接入", self.page.message_text)

    def test_fake_ready_mode_generates_command(self) -> None:
        self.backend.caps["competition-self-color"] = BackendCapability(
            command_id="competition-self-color",
            availability=CapabilityAvailability.READY,
            reason="ready",
            requires_robot=True,
            requires_manual_unlock=False,
        )
        self.page._refresh_capabilities()
        self.page._select_competition_mode("competition-self-color")
        self.page.start_competition()
        self.assertEqual(self.page.state, PageState.RUNNING)
        self.assertEqual(len(self.backend.requests), 1)
        command_id, _ = self.backend.requests[0]
        self.assertEqual(command_id, "competition-self-color")
        self.page._refresh_button_states()
        stop_buttons = [
            button
            for entry in self.page.pages.values()
            for button in entry["buttons"]
            if button.text == "停止"
        ]
        self.assertEqual(len(stop_buttons), 2)
        self.assertTrue(all(button.enabled for button in stop_buttons))
        self.pump()


class ManualPageTest(PickPlacePageTestCase):
    def test_manual_readback_updates_joint_and_tcp_widgets(self) -> None:
        payload = {
            "final_joint_deg": {
                "shoulder_pan": 1.0,
                "shoulder_lift": 2.0,
                "elbow_flex": 3.0,
                "wrist_flex": 4.0,
                "wrist_roll": 5.0,
            },
            "tcp_pose": {
                "frame": "robot_base_mm",
                "position_mm": [10.0, 20.0, 30.0],
                "pitch_deg": -87.5,
                "yaw_deg": 12.0,
            },
        }
        self.page._apply_manual_readback(payload)
        self.assertEqual(self.page.joint_readbacks["wrist_roll"], 5.0)
        self.assertEqual(self.page.tcp_readback["z"], 30.0)
        self.assertEqual(self.page.tcp_readback["yaw"], 12.0)

    def test_manual_unlock_fails_when_locked_or_running(self) -> None:
        self.page.state = PageState.RUNNING
        self.page.manual_unlock()
        self.assertFalse(self.page.manual_unlocked)

        self.page.state = PageState.IDLE
        self.backend._lock_reason = "xHCI died"
        self.page.manual_unlock()
        self.assertFalse(self.page.manual_unlocked)

    def test_jog_rejected_without_unlock(self) -> None:
        self.page.manual_unlocked = False
        self.page._jog_joint(0, 1)
        self.assertEqual(self.page.message_level, "WARN")
        self.assertIn("解锁", self.page.message_text)
        self.assertEqual(len(self.backend.requests), 0)

    def test_fixed_joint_order_and_no_gripper(self) -> None:
        self.page.manual_unlocked = True
        self.page._jog_joint(0, 1)
        self.assertEqual(len(self.backend.requests), 1)
        command_id, params = self.backend.requests[0]
        self.assertEqual(command_id, "manual-jog-joint")
        self.assertEqual(params["joint_feature"], JOINT_ORDER[0])
        self.assertEqual(params["delta_deg"], 1.0)
        self.assertNotIn("gripper.pos", str(params))

    def test_cartesian_uses_robot_base_mm(self) -> None:
        self.page.manual_unlocked = True
        self.page._select_cartesian_axis("z")
        self.page._set_cartesian_step(5.0, is_angle=False)
        self.page._jog_cartesian("z", -1)
        _, params = self.backend.requests[0]
        self.assertEqual(params["axis"], "z")
        self.assertEqual(params["frame"], "robot_base_mm")
        self.assertEqual(params["delta"], -5.0)

    def test_shared_buttons_preserved_across_tabs(self) -> None:
        for tab in ("joint", "cartesian"):
            with self.subTest(tab=tab):
                self.page.manual_tab = tab
                self.page._refresh_button_states()
                texts = [
                    button.text
                    for button in self.page.pages["manual"]["buttons"]
                    if button.visible
                ]
                for shared in ("读取状态", "回 PARK", "五轴断力矩", "立即断电"):
                    self.assertIn(shared, texts)

    def test_magnet_duration_can_exceed_10_seconds(self) -> None:
        self.page._set_magnet_duration(5.0)
        self.assertEqual(self.page.magnet_duration_s, 5.0)
        backend = SupervisorRunBackend(
            python_path=sys.executable,
            supervisor_tool=Path("/dev/null"),
        )
        backend.override_capabilities(
            {
                "manual-magnet-pulse": BackendCapability(
                    command_id="manual-magnet-pulse",
                    availability=CapabilityAvailability.READY,
                    reason="ready",
                    requires_robot=True,
                    requires_manual_unlock=True,
                )
            }
        )
        request = backend.build_request(
            "manual-magnet-pulse",
            {"duration_s": 11.0},
            status_path=Path("/tmp/s.json"),
            result_path=Path("/tmp/r.json"),
        )
        self.assertIn("11.000", request.argv)
        self.assertEqual(request.timeout_s, 26.0)

    def test_magnet_off_does_not_require_unlock(self) -> None:
        self.page.manual_magnet_off()
        self.assertEqual(len(self.backend.requests), 1)
        self.assertEqual(self.backend.requests[0][0], "manual-magnet-off")


class DebugPageTest(PickPlacePageTestCase):
    def test_camera_calibration_starts_candidate_only_request(self) -> None:
        self.page.calibrate_camera()
        self.assertEqual(self.page.state, PageState.RUNNING)
        command_id, parameters = self.backend.requests[-1]
        self.assertEqual(command_id, "camera-calibration")
        self.assertEqual(parameters, {})
        self.assertIn("候选", self.page.current_action["label"])
        self.pump()

    def test_debug_switches_do_not_modify_environment(self) -> None:
        before = dict(os.environ)
        self.page._toggle_debug_switch("debug_overlays")
        self.page._toggle_debug_switch("debug_generic_vision")
        self.assertEqual(os.environ, before)

    def test_debug_generic_vision_disabled_when_backend_unavailable(self) -> None:
        self.backend.caps["detect"] = BackendCapability(
            command_id="detect",
            availability=CapabilityAvailability.UNAVAILABLE,
            reason="no backend",
            requires_robot=False,
            requires_manual_unlock=False,
        )
        self.page._refresh_capabilities()
        self.page._toggle_debug_switch("debug_generic_vision")
        self.assertFalse(self.page.debug_generic_vision)


class UnlockPageTest(PickPlacePageTestCase):
    def test_unlock_requires_every_current_lock_item_before_request(self) -> None:
        self.motion_lock.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "clear_requires": ["现场支撑机械臂", "确认磁铁 OFF"],
                },
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        self.page._goto("unlock")
        self.assertEqual(self.page.unlock_checked, [False, False])
        self.page._submit_motion_lock_clear()
        self.assertEqual(len(self.backend.requests), 0)
        self.page._toggle_unlock_requirement(0)
        self.page._toggle_unlock_requirement(1)
        self.page._submit_motion_lock_clear()
        self.assertEqual(self.backend.requests[-1][0], "manual-clear-motion-lock")
        confirmation = json.loads(
            (self.runtime_dir / "gui/motion_lock_confirmation.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertTrue(all(item["checked"] for item in confirmation["requirements"]))
        self.assertEqual(
            confirmation["operator_confirmation"],
            pick_place_page.LOCK_CLEAR_CONFIRMATION,
        )
        self.pump()

    def test_hold_button_requires_two_seconds(self) -> None:
        calls: list[str] = []
        button = pick_place_page.HoldPageButton(
            (0, 0, 100, 48),
            "hold",
            lambda: calls.append("confirmed"),
            hold_seconds=2.0,
            on_too_short=lambda: calls.append("short"),
        )
        down = pygame.event.Event(pygame.MOUSEBUTTONDOWN, {"pos": (10, 10), "button": 1})
        up = pygame.event.Event(pygame.MOUSEBUTTONUP, {"pos": (10, 10), "button": 1})
        with mock.patch.object(pick_place_page.time, "monotonic", side_effect=[1.0, 2.5]):
            button.handle(down)
            button.handle(up)
        self.assertEqual(calls, ["short"])
        with mock.patch.object(pick_place_page.time, "monotonic", side_effect=[3.0, 5.1]):
            button.handle(down)
            button.handle(up)
        self.assertEqual(calls, ["short", "confirmed"])


class MagnetBackendTest(unittest.TestCase):
    def test_backend_accepts_pulse_over_10_seconds(self) -> None:
        backend = SupervisorRunBackend(
            python_path=sys.executable,
            supervisor_tool=Path("/dev/null"),
        )
        backend.override_capabilities(
            {
                "manual-magnet-pulse": BackendCapability(
                    command_id="manual-magnet-pulse",
                    availability=CapabilityAvailability.READY,
                    reason="ready",
                    requires_robot=True,
                    requires_manual_unlock=True,
                )
            }
        )
        request = backend.build_request(
            "manual-magnet-pulse",
            {"duration_s": 11.0},
            status_path=Path("/tmp/s.json"),
            result_path=Path("/tmp/r.json"),
        )
        self.assertIn("11.000", request.argv)

    def test_backend_rejects_nonpositive_pulse(self) -> None:
        backend = SupervisorRunBackend(
            python_path=sys.executable,
            supervisor_tool=Path("/dev/null"),
        )
        backend.override_capabilities(
            {
                "manual-magnet-pulse": BackendCapability(
                    command_id="manual-magnet-pulse",
                    availability=CapabilityAvailability.READY,
                    reason="ready",
                    requires_robot=True,
                    requires_manual_unlock=True,
                )
            }
        )
        with self.assertRaises(ValueError):
            backend.build_request(
                "manual-magnet-pulse",
                {"duration_s": 0.0},
                status_path=Path("/tmp/s.json"),
                result_path=Path("/tmp/r.json"),
            )

    def test_camera_calibration_uses_frozen_camera_and_no_robot_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            camera = Path(temporary_directory) / "epuzzle-camera"
            camera.touch()
            backend = SupervisorRunBackend(
                python_path=sys.executable,
                supervisor_tool=Path("/dev/null"),
                robot_port=Path(temporary_directory) / "missing-arm",
                camera_device=camera,
            )
            capability = backend.capabilities()["camera-calibration"]
            self.assertEqual(capability.availability, CapabilityAvailability.READY)
            self.assertFalse(capability.requires_robot)
            request = backend.build_request(
                "camera-calibration",
                {},
                status_path=Path("/tmp/s.json"),
                result_path=Path("/tmp/r.json"),
            )
            self.assertEqual(request.timeout_s, 55.0)
            self.assertEqual(
                request.argv[request.argv.index("--camera") + 1], str(camera)
            )

    def test_backend_builds_jog_joint_argv(self) -> None:
        backend = SupervisorRunBackend(
            python_path=sys.executable,
            supervisor_tool=Path("/dev/null"),
        )
        backend.override_capabilities(
            {
                "manual-jog-joint": BackendCapability(
                    command_id="manual-jog-joint",
                    availability=CapabilityAvailability.READY,
                    reason="ready",
                    requires_robot=True,
                    requires_manual_unlock=True,
                )
            }
        )
        request = backend.build_request(
            "manual-jog-joint",
            {"joint_feature": "shoulder_pan", "delta_deg": 1.0},
            status_path=Path("/tmp/s.json"),
            result_path=Path("/tmp/r.json"),
        )
        self.assertIn("--joint-feature", request.argv)
        self.assertIn("shoulder_pan", request.argv)
        self.assertIn("--delta-deg", request.argv)
        self.assertNotIn("gripper.pos", request.argv)


if __name__ == "__main__":
    unittest.main()
