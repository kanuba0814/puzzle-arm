# pyright: reportMissingImports=false

import tempfile
import unittest
from pathlib import Path

from control.kinematics import JointVector, ToolPose
from control.motion_controller import MotionConfig, MotionErrorCode, MotionResult
from integration import (
    PlacementPlanRef,
    Supervisor,
    SupervisorConfig,
    SupervisorErrorCode,
    SupervisorFailure,
    SupervisorState,
    VisionBatch,
)
from integration.sweep_planner import PieceRef


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def monotonic_ns(self) -> int:
        return int(self.now * 1_000_000_000)


class Vision:
    def __init__(self, batch: VisionBatch) -> None:
        self.batch = batch

    def scan(self) -> VisionBatch:
        return self.batch


class Magnet:
    def __init__(self) -> None:
        self.calls: list[bool] = []

    def set_enabled(self, enabled: bool) -> None:
        self.calls.append(enabled)

    def emergency_off(self) -> None:
        self.calls.append(False)


class SupervisorGateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = ManualClock()
        self.tempdir = tempfile.TemporaryDirectory()
        self.batch = VisionBatch("batch", "image-sha", 0, (PieceRef("p", (1.0, 2.0)),))
        self.config = SupervisorConfig(
            Path("calibration.json"),
            Path("park.json"),
            Path("transform.json"),
            False,
            MotionConfig(10, 5, 10, 5, 1, 0.01, 1, -20, -30, 180),
            60,
            20,
            None,
            None,
            4,
            8,
            2000,
            120,
            False,
            False,
            3,
            -87.5,
            Path(self.tempdir.name) / "events.jsonl",
        )

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def _supervisor(
        self, config: SupervisorConfig | None = None, magnet: Magnet | None = None
    ) -> Supervisor:
        return Supervisor(
            config or self.config,
            None,
            None,
            magnet,
            Vision(self.batch),
            None,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
        )

    def test_unknown_pick_place_locks_before_magnet_enable(self) -> None:
        magnet = Magnet()
        supervisor = self._supervisor(magnet=magnet)
        supervisor.scan()
        with self.assertRaises(SupervisorFailure) as caught:
            supervisor._transition(SupervisorState.MAGNET_ON)
        self.assertEqual(caught.exception.code, SupervisorErrorCode.SUPERVISOR_LOCKED)
        self.assertEqual(magnet.calls, [])

    def test_future_grasp_fsm_cannot_enable_magnet_while_heights_unknown(self) -> None:
        magnet = Magnet()
        supervisor = self._supervisor(magnet=magnet)
        supervisor.scan()
        pose = ToolPose("robot_base_mm", (150.0, 0.0, 0.0), -87.5, 0.0)
        with self.assertRaises(SupervisorFailure) as caught:
            supervisor.grasp_fsm(
                PlacementPlanRef("image-sha", 0),
                piece=PieceRef("piece-1", (105.0, 148.5)),
                pick_pose=pose,
                place_pose=pose,
            )
        self.assertEqual(caught.exception.code, SupervisorErrorCode.SUPERVISOR_LOCKED)
        self.assertEqual(magnet.calls, [])

    def test_unofficial_transform_is_rejected_after_other_grasp_prerequisites(
        self,
    ) -> None:
        config = SupervisorConfig(
            **{
                **self.config.__dict__,
                "z_pick_mm": 1.0,
                "z_place_mm": 1.0,
                "automatic_pick_place_enabled": True,
                "magnet_footprint_clearance_frozen": True,
            }
        )
        supervisor = self._supervisor(config)
        supervisor.scan()
        with self.assertRaises(SupervisorFailure) as caught:
            supervisor._transition(SupervisorState.MAGNET_ON)
        self.assertEqual(
            caught.exception.code, SupervisorErrorCode.TRANSFORM_UNOFFICIAL
        )

    def test_stale_batch_and_wrong_plan_are_rejected(self) -> None:
        supervisor = self._supervisor()
        supervisor.scan()
        self.clock.now = 3.0
        with self.assertRaises(SupervisorFailure) as stale:
            supervisor.validate_plan(PlacementPlanRef("image-sha", 0))
        self.assertEqual(stale.exception.code, SupervisorErrorCode.VISION_STALE)
        self.clock.now = 0.0
        with self.assertRaises(SupervisorFailure) as mismatch:
            supervisor.validate_plan(PlacementPlanRef("other", 0))
        self.assertEqual(mismatch.exception.code, SupervisorErrorCode.PLAN_STALE)

    def test_sweep_segment_preflight_rejects_before_park_or_servo_action(self) -> None:
        class Motion:
            def __init__(self) -> None:
                self.calls = 0

            def move(self, goal: object) -> object:
                self.calls += 1
                raise AssertionError("preflight should reject before move")

        class Park:
            def __init__(self) -> None:
                self.calls = 0

            def park_to_home(self) -> object:
                self.calls += 1
                raise AssertionError("preflight should reject before PARK transition")

        class Kinematics:
            def inverse(self, pose: ToolPose) -> object:
                raise ValueError("unreachable full sweep segment")

        motion = Motion()
        park = Park()
        supervisor = Supervisor(
            self.config,
            motion,  # type: ignore[arg-type]
            park,  # type: ignore[arg-type]
            None,
            Vision(self.batch),
            None,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
            sweep_kinematics=Kinematics(),
        )
        with self.assertRaisesRegex(ValueError, "unreachable"):
            supervisor.sweep(
                self.batch,
                start_table_mm=(0.0, 0.0),
                pose_for=lambda piece, z: ToolPose(
                    "robot_base_mm",
                    (piece.center_table_mm[0], piece.center_table_mm[1], z),
                    0.0,
                    0.0,
                ),
            )
        self.assertEqual(motion.calls, 0)
        self.assertEqual(park.calls, 0)

    def test_fake_robot_sweep_reads_every_center_and_returns_home_then_park(
        self,
    ) -> None:
        class FakeRobot:
            def __init__(self) -> None:
                self.goals: list[object] = []

            def move(self, goal: object) -> MotionResult:
                self.goals.append(goal)
                return MotionResult(
                    goal.goal_id,  # type: ignore[union-attr]
                    0,
                    True,
                    MotionErrorCode.NONE,
                    "fake actual readback",
                    "fake-calibration",
                    1,
                    1,
                    JointVector((0, 0, 0, 0, 0)).as_mapping(),
                )

        class FakeParkMover:
            def __init__(self) -> None:
                self.transitions: list[str] = []

            def park_to_home(self) -> object:
                self.transitions.append("park_to_home")
                return type(
                    "Result", (), {"success": True, "error_code": None, "message": "ok"}
                )()

            def home_to_park(self) -> object:
                self.transitions.append("home_to_park")
                return type(
                    "Result", (), {"success": True, "error_code": None, "message": "ok"}
                )()

        class Kinematics:
            def inverse(self, pose: ToolPose) -> JointVector:
                return JointVector((0, 0, 0, 0, 0))

        batch = VisionBatch(
            "batch",
            "image-sha",
            0,
            (PieceRef("a", (1.0, 2.0)), PieceRef("b", (3.0, 2.0))),
        )
        robot = FakeRobot()
        park_mover = FakeParkMover()
        supervisor = Supervisor(
            self.config,
            robot,  # type: ignore[arg-type]
            park_mover,  # type: ignore[arg-type]
            None,
            Vision(batch),
            None,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
            sweep_kinematics=Kinematics(),
        )
        home_pose = ToolPose("robot_base_mm", (0.0, 0.0, 60.0), 0.0, 0.0)
        route = supervisor.sweep(
            batch,
            start_table_mm=(0.0, 0.0),
            pose_for=lambda piece, z: ToolPose(
                "robot_base_mm",
                (piece.center_table_mm[0], piece.center_table_mm[1], z),
                0.0,
                0.0,
            ),
            home_pose=home_pose,
        )
        self.assertEqual([piece.piece_id for piece in route], ["a", "b"])
        self.assertEqual(park_mover.transitions, ["park_to_home", "home_to_park"])
        self.assertEqual(len(robot.goals), 5)  # high, both low centers, high, HOME
        self.assertEqual(supervisor.state, SupervisorState.PARKED)

    def test_shadow_rejects_hardware_sweep(self) -> None:
        shadow = SupervisorConfig(**{**self.config.__dict__, "shadow": True})
        supervisor = self._supervisor(shadow)
        with self.assertRaises(SupervisorFailure) as caught:
            supervisor.sweep(
                self.batch, start_table_mm=(0.0, 0.0), pose_for=lambda piece, z: None
            )  # type: ignore[arg-type]
        self.assertEqual(caught.exception.code, SupervisorErrorCode.SHADOW_MODE)


if __name__ == "__main__":
    unittest.main()
