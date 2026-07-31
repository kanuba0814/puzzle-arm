# pyright: reportMissingImports=false

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
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

    def test_unimplemented_manual_and_competition_modes_are_locked(self) -> None:
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
                self.assertEqual(payload["error_code"], "SUPERVISOR_LOCKED")

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
