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


class FakeTimer:
    def __init__(self, interval: float, callback) -> None:
        self.interval = interval
        self.callback = callback
        self.daemon = False
        self.started = False
        self.cancelled = False

    def start(self) -> None:
        self.started = True

    def cancel(self) -> None:
        self.cancelled = True

    def fire(self) -> None:
        if not self.cancelled:
            self.callback()


class MagnetControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.output = FakeOutput()
        self.timers: list[FakeTimer] = []
        self.statuses = []
        config = MagnetConfig(
            chip_path="/dev/gpiochip0",
            line_offset=17,
            active_high=False,
            max_on_seconds=10.0,
        )
        self.controller = MagnetController(
            config,
            output_factory=lambda _: self.output,
            timer_factory=self._make_timer,
            on_status=self.statuses.append,
        )

    def _make_timer(self, interval: float, callback) -> FakeTimer:
        timer = FakeTimer(interval, callback)
        self.timers.append(timer)
        return timer

    def test_connect_requests_output_in_safe_off_state(self) -> None:
        status = self.controller.connect()

        self.assertFalse(status.enabled)
        self.assertEqual(status.reason, "connected_safe_off")
        self.assertEqual(self.output.actions, [False])

    def test_set_enabled_uses_one_boolean_interface_and_arms_timeout(self) -> None:
        self.controller.connect()

        status = self.controller.set_enabled(True)

        self.assertTrue(status.enabled)
        self.assertEqual(self.output.actions, [False, True])
        self.assertEqual(self.timers[-1].interval, 10.0)
        self.assertTrue(self.timers[-1].started)
        self.assertTrue(self.timers[-1].daemon)

    def test_timeout_automatically_turns_magnet_off(self) -> None:
        self.controller.connect()
        self.controller.set_enabled(True)

        self.timers[-1].fire()

        self.assertFalse(self.output.active)
        self.assertFalse(self.controller.get_status().enabled)
        self.assertEqual(self.controller.get_status().reason, "timeout")

    def test_repeated_enable_rearms_timeout(self) -> None:
        self.controller.connect()
        self.controller.set_enabled(True)
        first_timer = self.timers[-1]

        self.controller.set_enabled(True)

        self.assertTrue(first_timer.cancelled)
        self.assertIsNot(self.timers[-1], first_timer)
        self.assertTrue(self.output.active)

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
