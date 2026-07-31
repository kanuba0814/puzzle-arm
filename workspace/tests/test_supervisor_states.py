# pyright: reportMissingImports=false

import tempfile
import unittest
from pathlib import Path

from control.motion_controller import MotionConfig
from integration import Supervisor, SupervisorConfig, SupervisorState, VisionBatch
from integration.sweep_planner import PieceRef


class Clock:
    def monotonic(self) -> float:
        return 0.0

    def monotonic_ns(self) -> int:
        return 0


class Vision:
    def scan(self) -> VisionBatch:
        return VisionBatch("batch", "image", 0, (PieceRef("piece", (0.0, 0.0)),))


class ParkMover:
    def __init__(self, parked: bool) -> None:
        self.parked = parked

    def is_at_park(self) -> bool:
        return self.parked


class SupervisorStateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.config = SupervisorConfig(
            Path("calibration"),
            Path("park"),
            Path("transform"),
            False,
            MotionConfig(10, 5, 10, 5, 1, 0.01, 1, -20, -30, 180),
            60,
            20,
            None,
            None,
            4,
            8,
            100,
            120,
            False,
            False,
            3,
            -87.5,
            Path(self.directory.name) / "events.jsonl",
        )

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_state_is_read_only_and_estop_needs_confirmed_park_reset(self) -> None:
        mover = ParkMover(False)
        supervisor = Supervisor(
            self.config,
            None,
            mover,
            None,
            Vision(),
            None,
            monotonic=self.clock.monotonic,
            monotonic_ns=self.clock.monotonic_ns,
        )
        with self.assertRaises(AttributeError):
            supervisor.state = SupervisorState.COMPLETE  # type: ignore[misc]
        supervisor.emergency_stop()
        self.assertEqual(supervisor.state, SupervisorState.ESTOP)
        self.assertFalse(supervisor.reset_to_parked(operator_confirmed=False))
        self.assertFalse(supervisor.reset_to_parked(operator_confirmed=True))
        mover.parked = True
        self.assertTrue(supervisor.reset_to_parked(operator_confirmed=True))
        self.assertEqual(supervisor.state, SupervisorState.PARKED)


if __name__ == "__main__":
    unittest.main()
