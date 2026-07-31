# pyright: reportMissingImports=false

"""PICK/PLACE 自动抓放门禁回归测试。

覆盖两条边界：
1. ``pick_place_lock_reason`` / 后端能力：验收文件缺失、未验收、成功率不足时锁定。
2. ``load_pick_place_heights``：只有完整实测的验收文件才能解出接触高度和安定时间；
   缺少磁铁安定时间或高度越界时必须 fail-closed，不得回退到估计值。
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

from integration.competition_backend import (
    CapabilityAvailability,
    SupervisorRunBackend,
    pick_place_lock_reason,
)
from integration.supervisor import SupervisorErrorCode, SupervisorFailure

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / "tools"))

from supervisor_run import load_pick_place_heights  # noqa: E402


def accepted_verification(**overrides: object) -> dict[str, object]:
    """完整实测的磁铁验证字段；传 None 表示删除该字段。"""
    verification: dict[str, object] = {
        "pickup_tested": True,
        "release_tested": True,
        "pickup_success_count": 9,
        "pickup_failure_count": 1,
        "release_success_count": 9,
        "release_failure_count": 1,
        "on_settle_s": 2.0,
        "off_settle_s": 1.0,
    }
    for key, value in overrides.items():
        if value is None:
            verification.pop(key, None)
        else:
            verification[key] = value
    return verification


def accepted_payload(**overrides: object) -> dict[str, object]:
    """一份完整通过验收的接触高度文件。"""
    payload: dict[str, object] = {
        "schema_version": 1,
        "calibration_id": "pick-place-contact-heights-test",
        "date": "2026-07-31",
        "reference_frame": "calibration_sheet_mm",
        "quality": {"status": "accepted", "reason": "test fixture"},
        "contact_heights_mm": {"pick": 4.0, "place": 6.0},
        "magnet_verification": accepted_verification(),
        "single_piece_success_rate": 0.9,
        "total_test_cycles": 10,
    }
    payload.update(overrides)
    return payload


class PickPlaceLockReasonTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "pick_place_contact_height.json"

    def write(self, payload: dict[str, object]) -> Path:
        self.path.write_text(json.dumps(payload), encoding="utf-8")
        return self.path

    def test_missing_file_is_locked(self) -> None:
        reason = pick_place_lock_reason(self.path)
        assert reason is not None
        self.assertIn("UNKNOWN", reason)

    def test_status_not_accepted_is_locked(self) -> None:
        path = self.write(accepted_payload(quality={"status": "rejected"}))
        reason = pick_place_lock_reason(path)
        assert reason is not None
        self.assertIn("accepted", reason)

    def test_success_rate_below_gate_is_locked(self) -> None:
        path = self.write(accepted_payload(single_piece_success_rate=0.75))
        reason = pick_place_lock_reason(path)
        assert reason is not None
        self.assertIn("80%", reason)

    def test_fully_accepted_file_unlocks(self) -> None:
        self.assertIsNone(pick_place_lock_reason(self.write(accepted_payload())))

    def test_backend_capability_tracks_lock_reason(self) -> None:
        locked = SupervisorRunBackend(pick_place_calibration=self.path).capabilities()[
            "pick-place"
        ]
        self.assertIs(locked.availability, CapabilityAvailability.LOCKED)
        self.assertTrue(locked.requires_robot)

        # 验收通过后，唯一剩余的阻塞原因只能来自机械臂/运动锁，而不是标定文件。
        backend = SupervisorRunBackend(
            pick_place_calibration=self.write(accepted_payload()),
            robot_port=Path(self._tmp.name) / "absent-arm",
        )
        capability = backend.capabilities()["pick-place"]
        self.assertIs(capability.availability, CapabilityAvailability.LOCKED)
        self.assertIn("机械臂", capability.reason)


class LoadPickPlaceHeightsTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "pick_place_contact_height.json"

    def write(self, payload: dict[str, object]) -> Path:
        self.path.write_text(json.dumps(payload), encoding="utf-8")
        return self.path

    def assert_locked(self, payload: dict[str, object], expected: str) -> None:
        with self.assertRaises(SupervisorFailure) as caught:
            load_pick_place_heights(self.write(payload))
        self.assertIs(caught.exception.code, SupervisorErrorCode.SUPERVISOR_LOCKED)
        self.assertIn(expected, str(caught.exception))

    def test_accepted_file_resolves_heights_and_settle_times(self) -> None:
        heights = load_pick_place_heights(self.write(accepted_payload()))
        self.assertAlmostEqual(heights.pick_mm, 4.0)
        self.assertAlmostEqual(heights.place_mm, 6.0)
        self.assertAlmostEqual(heights.magnet_on_settle_s, 2.0)
        self.assertAlmostEqual(heights.magnet_off_settle_s, 1.0)

    def test_missing_file_is_locked(self) -> None:
        with self.assertRaises(SupervisorFailure) as caught:
            load_pick_place_heights(self.path)
        self.assertIs(caught.exception.code, SupervisorErrorCode.SUPERVISOR_LOCKED)

    def test_wrong_reference_frame_is_locked(self) -> None:
        self.assert_locked(
            accepted_payload(reference_frame="table_mm"), "calibration_sheet_mm"
        )

    def test_unknown_height_is_locked(self) -> None:
        self.assert_locked(
            accepted_payload(contact_heights_mm={"pick": "UNKNOWN", "place": 6.0}),
            "接触高度无效",
        )

    def test_height_outside_safe_band_is_locked(self) -> None:
        self.assert_locked(
            accepted_payload(contact_heights_mm={"pick": 4.0, "place": 45.0}),
            "安全区间",
        )

    def test_missing_settle_time_is_locked_without_estimating(self) -> None:
        self.assert_locked(
            accepted_payload(
                magnet_verification=accepted_verification(on_settle_s=None)
            ),
            "磁铁安定时间",
        )

    def test_untested_magnet_is_locked(self) -> None:
        self.assert_locked(
            accepted_payload(
                magnet_verification=accepted_verification(release_tested=False)
            ),
            "实机验证",
        )


if __name__ == "__main__":
    unittest.main()
