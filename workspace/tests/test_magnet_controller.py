# pyright: reportMissingImports=false

import unittest

from control.magnet_controller import MagnetConfig, MagnetController


class FakeOutput:
    def __init__(self) -> None:
        self.active = None
        self.actions: list[bool] = []
        self.closed = False

    def set_active(self, active: bool) -> None:
        self.active = active
        self.actions.append(active)

    def close(self) -> None:
        self.closed = True


class MagnetControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.output = FakeOutput()
        self.statuses = []
        config = MagnetConfig(
            chip_path="/dev/gpiochip0",
            line_offset=17,
            active_high=False,
        )
        self.controller = MagnetController(
            config,
            output_factory=lambda _: self.output,
            on_status=self.statuses.append,
        )

    def test_connect_requests_output_in_safe_off_state(self) -> None:
        status = self.controller.connect()

        self.assertFalse(status.enabled)
        self.assertEqual(status.reason, "connected_safe_off")
        self.assertEqual(self.output.actions, [False])

    def test_set_enabled_keeps_magnet_enabled_until_explicit_off(self) -> None:
        self.controller.connect()

        status = self.controller.set_enabled(True)

        self.assertTrue(status.enabled)
        self.assertTrue(self.output.active)
        self.assertEqual(self.controller.get_status().reason, "command_on")

        off_status = self.controller.set_enabled(False)

        self.assertFalse(off_status.enabled)
        self.assertFalse(self.output.active)
        self.assertEqual(off_status.reason, "command_off")

    def test_repeated_enable_does_not_schedule_an_automatic_shutdown(self) -> None:
        self.controller.connect()
        self.controller.set_enabled(True)

        self.controller.set_enabled(True)

        self.assertTrue(self.output.active)
        self.assertTrue(self.controller.get_status().enabled)

    def test_emergency_off_and_close_are_fail_safe(self) -> None:
        self.controller.connect()
        self.controller.set_enabled(True)

        status = self.controller.emergency_off()
        self.controller.close()

        self.assertFalse(status.enabled)
        self.assertEqual(status.reason, "emergency_off")
        self.assertFalse(self.output.active)
        self.assertTrue(self.output.closed)

    def test_control_before_connect_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "尚未连接"):
            self.controller.set_enabled(True)


if __name__ == "__main__":
    unittest.main()
