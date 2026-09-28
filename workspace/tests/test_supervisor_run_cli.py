# pyright: reportMissingImports=false

import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

WORKSPACE = Path(__file__).resolve().parents[1]
TOOL_PATH = WORKSPACE / "tools/supervisor_run.py"
SPEC = importlib.util.spec_from_file_location("supervisor_run", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
supervisor_run = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = supervisor_run
SPEC.loader.exec_module(supervisor_run)


class SupervisorRunCliTest(unittest.TestCase):
    def test_hardware_modes_are_exposed_in_parser(self) -> None:
        parser = supervisor_run.build_parser()
        for mode in (
            "detect",
            "red-baseline",
            "sweep-shortest",
            "sweep",
            "repeat",
            "pregrasp",
            "park",
            "puzzle-shadow",
            "shadow-plan",
            "pick-place",
            "commission-center",
            "competition-self-color",
            "competition-white",
            "competition-card",
            "camera-calibration",
            "manual-clear-motion-lock",
            "manual-read-state",
            "manual-jog-joint",
            "manual-jog-cartesian",
            "manual-magnet-pulse",
            "manual-magnet-off",
        ):
            self.assertEqual(parser.parse_args([mode]).mode, mode)

    def test_manual_magnet_duration_argument_parses(self) -> None:
        parser = supervisor_run.build_parser()
        args = parser.parse_args(["manual-magnet-pulse", "--duration-s", "5.0"])
        self.assertEqual(args.duration_s, 5.0)

    def test_manual_jog_arguments_parse(self) -> None:
        parser = supervisor_run.build_parser()
        args = parser.parse_args(
            [
                "manual-jog-joint",
                "--joint-feature",
                "shoulder_pan",
                "--delta-deg",
                "1.5",
            ]
        )
        self.assertEqual(args.joint_feature, "shoulder_pan")
        self.assertEqual(args.delta_deg, 1.5)
        args = parser.parse_args(
            ["manual-jog-cartesian", "--axis", "z", "--delta", "-2.5"]
        )
        self.assertEqual(args.axis, "z")
        self.assertEqual(args.delta, -2.5)

    def test_manual_and_competition_modes_fail_closed_without_robot(self) -> None:
        for mode in (
            "competition-self-color",
            "competition-white",
            "competition-card",
            "manual-read-state",
            "manual-jog-joint",
            "manual-jog-cartesian",
        ):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as tmp:
                result_file = Path(tmp) / "result.json"
                completed = subprocess.run(
                    [
                        sys.executable,
                        str(TOOL_PATH),
                        mode,
                        "--port",
                        "/definitely/missing/epuzzle-arm",
                        "--result-file",
                        str(result_file),
                    ],
                    cwd=WORKSPACE.parent,
                    env={
                        **dict(os.environ),
                        "PYTHONPATH": str(WORKSPACE / "src"),
                        "EPUZZLE_ROBOT_MOTION_LOCK": str(
                            Path(tmp) / "no-motion-lock.json"
                        ),
                    },
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertEqual(completed.returncode, 2, completed.stderr)
                payload = json.loads(result_file.read_text(encoding="utf-8"))
                self.assertEqual(payload["result"], "FAIL")
                self.assertEqual(payload["error_code"], "ROBOT_COMM_TIMEOUT")

    def test_manual_magnet_off_writes_result_even_without_gpio(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result_file = Path(tmp) / "result.json"
            subprocess.run(
                [
                    sys.executable,
                    str(TOOL_PATH),
                    "manual-magnet-off",
                    "--result-file",
                    str(result_file),
                ],
                cwd=WORKSPACE.parent,
                env={**dict(os.environ), "PYTHONPATH": str(WORKSPACE / "src")},
                text=True,
                capture_output=True,
                check=False,
            )
            payload = json.loads(result_file.read_text(encoding="utf-8"))
            # 在缺少 gpiod 的开发机上会失败，但必须写出结构化结果。
            self.assertEqual(payload["command"], "manual-magnet-off")
            self.assertIn(payload["result"], {"PASS", "FAIL"})
            self.assertIsInstance(payload["magnet_enabled"], bool)

    def test_detection_uses_stable_camera_device_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            pieces = Path(temporary_directory) / "pieces.json"
            pieces.write_text('{"pieces": []}', encoding="utf-8")
            with patch.dict(
                os.environ,
                {"EPUZZLE_CAMERA_DEVICE": "/dev/epuzzle-camera"},
            ):
                args = supervisor_run.build_parser().parse_args(
                    ["detect", "--pieces", str(pieces)]
                )
            with patch.object(
                supervisor_run.subprocess,
                "run",
                return_value=subprocess.CompletedProcess([], 0, "", ""),
            ) as run:
                supervisor_run.refresh_detection(args)
            command = run.call_args.args[0]
            camera_index = command.index("--camera") + 1
            self.assertEqual(command[camera_index], "/dev/epuzzle-camera")

    def test_explicit_usb_device_loss_maps_to_robot_comm_timeout(self) -> None:
        code = supervisor_run.classify_unhandled_error(
            OSError(19, "No such device"),
            mode="sweep",
            robot_port=Path("/definitely/missing/epuzzle-arm"),
        )
        self.assertEqual(code.value, "ROBOT_COMM_TIMEOUT")

    def test_non_hardware_or_camera_failure_stays_safety_rejected(self) -> None:
        missing_port = Path("/definitely/missing/epuzzle-arm")
        self.assertEqual(
            supervisor_run.classify_unhandled_error(
                RuntimeError("bad fixture"), mode="shadow-plan", robot_port=missing_port
            ).value,
            "SAFETY_REJECTED",
        )
        self.assertEqual(
            supervisor_run.classify_unhandled_error(
                RuntimeError("相机同步检测失败: camera gone"),
                mode="sweep",
                robot_port=missing_port,
            ).value,
            "SAFETY_REJECTED",
        )

    def test_competition_reuses_123_plan_then_four_piece_executor(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            port = temporary / "epuzzle-arm"
            port.touch()
            args = supervisor_run.build_parser().parse_args(
                ["competition-self-color", "--port", str(port)]
            )
            args.command = "competition-self-color"
            args.mode = "competition-self-color"
            calls: list[list[str]] = []

            def bounded(argv, *, timeout_s, log_path):
                calls.append(argv)
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.write_text("test", encoding="utf-8")
                if "live_detect_123.py" in str(argv[1]):
                    output_dir = Path(argv[argv.index("--out") + 1])
                    output_dir.mkdir(parents=True, exist_ok=True)
                    (output_dir / "result.json").write_text(
                        json.dumps(
                            {
                                "solve": {
                                    "status": "SOLVED",
                                    "placements": [
                                        {"piece_id": f"piece_{index:02d}"}
                                        for index in range(4)
                                    ],
                                }
                            }
                        ),
                        encoding="utf-8",
                    )
                return 0, False

            with (
                patch.object(supervisor_run, "GUI_COMPETITION_RUNTIME", temporary / "runtime"),
                patch.object(supervisor_run, "load_pick_place_heights"),
                patch.object(supervisor_run, "robot_motion_lock_reason", return_value=None),
                patch.object(supervisor_run, "_run_bounded_child", side_effect=bounded),
            ):
                payload = supervisor_run.run_competition(args, time.monotonic())
            self.assertTrue(payload["success"])
            self.assertEqual(payload["route"], [f"piece_{index:02d}" for index in range(4)])
            self.assertEqual(len(calls), 2)
            self.assertIn("live_detect_123.py", str(calls[0][1]))
            self.assertIn("run_assembly_4pieces.py", str(calls[1][1]))

    def test_manual_clear_motion_lock_uses_fixed_confirmation_and_helper(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            workspace = temporary / "workspace"
            confirmation = workspace / "runtime/gui/motion_lock_confirmation.json"
            confirmation.parent.mkdir(parents=True)
            confirmation.write_text("{}", encoding="utf-8")
            helper = temporary / "epuzzle-clear-motion-lock"
            helper.touch()
            lock = temporary / "robot-motion.lock.json"
            lock.write_text(json.dumps({"reason": "requires operator"}), encoding="utf-8")
            args = supervisor_run.build_parser().parse_args(["manual-clear-motion-lock"])
            args.command = "manual-clear-motion-lock"
            args.mode = "manual-clear-motion-lock"
            args.lock_confirmation = confirmation
            calls: list[list[str]] = []

            def bounded(argv, *, timeout_s, log_path):
                calls.append(argv)
                self.assertEqual(timeout_s, 20.0)
                lock.unlink()
                return 0, False

            with (
                patch.dict(os.environ, {"EPUZZLE_ROBOT_MOTION_LOCK": str(lock)}),
                patch.object(supervisor_run, "WORKSPACE", workspace),
                patch.object(supervisor_run, "MOTION_LOCK_CLEAR_HELPER", helper),
                patch.object(supervisor_run, "_run_bounded_child", side_effect=bounded),
            ):
                payload = supervisor_run.run_manual_clear_motion_lock(
                    args, time.monotonic()
                )
            self.assertTrue(payload["success"])
            self.assertEqual(payload["final_state"], "UNLOCKED")
            self.assertEqual(calls[0][0], "sudo")
            self.assertEqual(calls[0][2], "--confirmation")
            self.assertEqual(calls[0][3], str(confirmation))

    def test_manual_read_keeps_torque_evidence_when_tcp_is_outside_limits(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            port = temporary / "epuzzle-arm"
            port.touch()
            args = supervisor_run.build_parser().parse_args(
                ["manual-read-state", "--port", str(port)]
            )
            args.command = "manual-read-state"
            args.mode = "manual-read-state"

            class FakeBus:
                is_calibrated = True
                is_connected = True

                def connect(self) -> None:
                    return None

                def disconnect(self, *, disable_torque: bool) -> None:
                    self.is_connected = False

                def sync_read(self, register: str, **_kwargs: object) -> dict[str, float]:
                    if register == "Present_Position":
                        return {
                            name: 2048.0 if _kwargs.get("normalize") else 2048
                            for name in supervisor_run.JOINT_ORDER
                        }
                    if register == "Torque_Enable":
                        return {name: 0 for name in supervisor_run.JOINT_ORDER}
                    raise AssertionError(register)

            class FakeRobot:
                def __init__(self, _config: object) -> None:
                    self.bus = FakeBus()

            class FakeAdapter:
                def __init__(self, **_kwargs: object) -> None:
                    self.kinematics = types.SimpleNamespace(
                        forward=lambda _joints: (_ for _ in ()).throw(
                            ValueError("elbow_flex.pos outside frozen limit")
                        )
                    )

            fake_plugin = types.SimpleNamespace(
                SO101Follower5D=FakeRobot,
                SO101Follower5DConfig=lambda **_kwargs: object(),
            )
            with (
                patch.dict(sys.modules, {"lerobot_robot_so101_5d": fake_plugin}),
                patch.object(supervisor_run, "InspectionHardwareAdapter", FakeAdapter),
            ):
                payload = supervisor_run._manual_read_state(args, time.monotonic())
            self.assertTrue(payload["success"])
            self.assertEqual(
                payload["torque_enable"],
                {name: 0 for name in supervisor_run.JOINT_ORDER},
            )
            self.assertNotIn("tcp_pose", payload)
            self.assertIn("未计算 TCP", payload["message"])

    def test_camera_calibration_writes_candidate_without_hardware_adapter(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            camera = temporary / "epuzzle-camera"
            camera.touch()
            args = supervisor_run.build_parser().parse_args(
                ["camera-calibration", "--camera", str(camera)]
            )
            args.command = "camera-calibration"
            args.mode = "camera-calibration"

            def bounded(argv, *, timeout_s, log_path):
                self.assertEqual(timeout_s, 45.0)
                self.assertEqual(argv[argv.index("--camera") + 1], str(camera))
                self.assertNotIn(str(supervisor_run.HOMOGRAPHY_PATH), argv)
                candidate = Path(argv[argv.index("--output") + 1])
                summary = Path(argv[argv.index("--summary") + 1])
                candidate.parent.mkdir(parents=True, exist_ok=True)
                candidate.write_bytes(b"candidate")
                summary.write_text(
                    json.dumps(
                        {
                            "corners": 70,
                            "homography_residual_mm": {"rms": 0.185, "max": 0.411},
                            "homography_sha256": hashlib.sha256(
                                candidate.read_bytes()
                            ).hexdigest(),
                        }
                    ),
                    encoding="utf-8",
                )
                return 0, False

            with (
                patch.object(
                    supervisor_run,
                    "GUI_CAMERA_CALIBRATION_RUNTIME",
                    temporary / "candidate-runtime",
                ),
                patch.object(
                    supervisor_run, "_run_bounded_child", side_effect=bounded,
                ),
                patch.object(supervisor_run, "InspectionHardwareAdapter") as adapter,
            ):
                payload = supervisor_run.run_camera_calibration(args, time.monotonic())
            self.assertTrue(payload["success"])
            self.assertEqual(payload["final_state"], "CANDIDATE_READY")
            self.assertIn("candidate_homography", payload["artifacts"])
            adapter.assert_not_called()

    def test_sweep_refreshes_detection_before_any_hardware_adapter(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            robot_port = Path(temporary_directory) / "epuzzle-arm"
            robot_port.touch()
            args = supervisor_run.build_parser().parse_args(
                ["sweep", "--port", str(robot_port)]
            )
            sequence: list[str] = []

            class Adapter:
                def __init__(self, **kwargs: object) -> None:
                    sequence.append("adapter")

            with (
                patch.dict(
                    os.environ,
                    {
                        "EPUZZLE_ROBOT_MOTION_LOCK": str(
                            robot_port.parent / "no-lock.json"
                        )
                    },
                ),
                patch.object(
                    supervisor_run,
                    "refresh_detection",
                    side_effect=lambda _: sequence.append("detect"),
                ),
                patch.object(supervisor_run, "InspectionHardwareAdapter", Adapter),
                self.assertRaisesRegex(AttributeError, "robot"),
            ):
                supervisor_run.run_hardware(args, 0.0)
            self.assertEqual(sequence, ["detect", "adapter"])

    def test_incident_motion_lock_rejects_before_adapter(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            robot_port = temporary / "epuzzle-arm"
            robot_port.touch()
            motion_lock = temporary / "robot-motion-lock.json"
            motion_lock.write_text(
                json.dumps({"reason": "xHCI host controller died"}), encoding="utf-8"
            )
            args = supervisor_run.build_parser().parse_args(
                ["park", "--port", str(robot_port)]
            )
            with (
                patch.dict(
                    os.environ,
                    {"EPUZZLE_ROBOT_MOTION_LOCK": str(motion_lock)},
                ),
                patch.object(supervisor_run, "InspectionHardwareAdapter") as adapter,
                self.assertRaises(supervisor_run.SupervisorFailure) as caught,
            ):
                supervisor_run.run_hardware(args, 0.0)
            self.assertEqual(caught.exception.code.value, "SUPERVISOR_LOCKED")
            self.assertIn("xHCI", str(caught.exception))
            adapter.assert_not_called()

    def test_missing_robot_port_rejects_after_detection_before_adapter(self) -> None:
        args = supervisor_run.build_parser().parse_args(
            ["sweep", "--port", "/definitely/missing/epuzzle-arm"]
        )
        sequence: list[str] = []
        with (
            patch.dict(
                os.environ,
                {"EPUZZLE_ROBOT_MOTION_LOCK": "/definitely/missing/robot-lock.json"},
            ),
            patch.object(
                supervisor_run,
                "refresh_detection",
                side_effect=lambda _: sequence.append("detect"),
            ),
            patch.object(supervisor_run, "InspectionHardwareAdapter") as adapter,
            self.assertRaises(supervisor_run.SupervisorFailure) as caught,
        ):
            supervisor_run.run_hardware(args, 0.0)
        self.assertEqual(caught.exception.code.value, "ROBOT_COMM_TIMEOUT")
        self.assertEqual(sequence, ["detect"])
        adapter.assert_not_called()

    def test_park_failure_without_hardware_still_has_json_contract(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(TOOL_PATH), "park"],
            cwd=WORKSPACE.parent,
            env={**dict(os.environ), "PYTHONPATH": str(WORKSPACE / "src")},
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertNotEqual(completed.returncode, 0)
        payload = json.loads(completed.stdout)
        self.assertFalse(payload["success"])
        self.assertEqual(payload["mode"], "park")
        self.assertEqual(
            set(payload).intersection(
                {
                    "success",
                    "error_code",
                    "message",
                    "mode",
                    "route",
                    "duration_ms",
                    "final_state",
                    "events_path",
                }
            ),
            {
                "success",
                "error_code",
                "message",
                "mode",
                "route",
                "duration_ms",
                "final_state",
                "events_path",
            },
        )

    def test_shadow_plan_json_contract_without_hardware(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            pieces = temporary / "pieces.json"
            output = temporary / "result.json"
            pieces.write_text(
                json.dumps(
                    {
                        "pieces": [
                            {
                                "piece_id": "red-0",
                                "center_table_mm": [50, 30],
                                "polygon_table_mm": [
                                    [0, 0],
                                    [100, 0],
                                    [100, 60],
                                    [0, 60],
                                ],
                                "yaw_deg": 0,
                                "timestamp_ns": 1,
                                "equivalent_yaws_deg": [180],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(TOOL_PATH),
                    "shadow-plan",
                    "--pieces",
                    str(pieces),
                    "--output",
                    str(output),
                ],
                cwd=WORKSPACE.parent,
                env={**dict(os.environ), "PYTHONPATH": str(WORKSPACE / "src")},
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(
                set(payload).intersection(
                    {
                        "success",
                        "error_code",
                        "message",
                        "mode",
                        "route",
                        "duration_ms",
                        "final_state",
                        "events_path",
                    }
                ),
                {
                    "success",
                    "error_code",
                    "message",
                    "mode",
                    "route",
                    "duration_ms",
                    "final_state",
                    "events_path",
                },
            )
            self.assertTrue(payload["success"])
            self.assertEqual(payload["mode"], "shadow-plan")
            self.assertEqual(payload["report"]["solve_status"], "SOLVED")

    def test_shadow_no_solution_is_fail_not_false_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            pieces = temporary / "pieces.json"
            result_file = temporary / "result.json"
            pieces.write_text(
                json.dumps(
                    {
                        "pieces": [
                            {
                                "piece_id": "triangle",
                                "center_table_mm": [10, 10],
                                "polygon_table_mm": [[0, 0], [10, 0], [0, 10]],
                                "yaw_deg": 0,
                                "timestamp_ns": 1,
                                "equivalent_yaws_deg": [],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(TOOL_PATH),
                    "puzzle-shadow",
                    "--pieces",
                    str(pieces),
                    "--result-file",
                    str(result_file),
                ],
                cwd=WORKSPACE.parent,
                env={**dict(os.environ), "PYTHONPATH": str(WORKSPACE / "src")},
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 2, completed.stderr)
            payload = json.loads(result_file.read_text(encoding="utf-8"))
            self.assertEqual(payload["result"], "FAIL")
            self.assertEqual(payload["error_code"], "SOLVER_NO_SOLUTION")
            self.assertEqual(payload["report"]["solve_status"], "NO_SOLUTION")
            self.assertFalse(payload["magnet_enabled"])

    def test_pick_place_is_locked_before_hardware_construction(self) -> None:
        """pick-place 验收文件缺失时必须在构造硬件前拒绝。"""
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            result_file = temporary / "result.json"
            nonexistent_calibration = temporary / "nonexistent_calibration.json"
            with (
                patch.object(
                    sys,
                    "argv",
                    [
                        str(TOOL_PATH),
                        "pick-place",
                        "--result-file",
                        str(result_file),
                        "--place-table-mm",
                        "105,100",
                    ],
                ),
                patch.object(
                    supervisor_run,
                    "PICK_PLACE_HEIGHT_PATH",
                    nonexistent_calibration,
                ),
                patch.object(
                    supervisor_run,
                    "run_hardware",
                    side_effect=AssertionError("不得构造硬件"),
                ),
            ):
                self.assertEqual(supervisor_run.main(), 2)
            payload = json.loads(result_file.read_text(encoding="utf-8"))
            self.assertEqual(payload["result"], "FAIL")
            self.assertEqual(payload["error_code"], "SUPERVISOR_LOCKED")
            self.assertFalse(payload["magnet_enabled"])

    def test_gui_alias_writes_status_and_result_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            pieces = temporary / "pieces.json"
            result_file = temporary / "result.json"
            status_file = temporary / "status.json"
            pieces.write_text(
                json.dumps(
                    {
                        "pieces": [
                            {
                                "piece_id": "piece-0",
                                "center_table_mm": [50, 30],
                                "polygon_table_mm": [
                                    [0, 0],
                                    [100, 0],
                                    [100, 60],
                                    [0, 60],
                                ],
                                "yaw_deg": 0,
                                "timestamp_ns": 1,
                                "equivalent_yaws_deg": [],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(TOOL_PATH),
                    "puzzle-shadow",
                    "--pieces",
                    str(pieces),
                    "--result-file",
                    str(result_file),
                    "--status-file",
                    str(status_file),
                ],
                cwd=WORKSPACE.parent,
                env={**dict(os.environ), "PYTHONPATH": str(WORKSPACE / "src")},
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(result_file.read_text(encoding="utf-8"))
            status = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertEqual(result["command"], "puzzle-shadow")
            self.assertEqual(result["result"], "PASS")
            self.assertEqual(status["phase"], "完成")
            self.assertEqual(status["progress"], 1.0)


if __name__ == "__main__":
    unittest.main()
