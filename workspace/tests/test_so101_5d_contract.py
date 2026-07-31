# pyright: reportMissingImports=false

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, cast

import tomllib
from lerobot_robot_so101_5d import (
    JOINT_FEATURES,
    JOINT_NAMES,
    SO101Follower5D,
    SO101Follower5DConfig,
)

EXPECTED_FEATURES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
)


class _FakeBumplessBus:
    def __init__(self) -> None:
        self.is_connected = False
        self.is_calibrated = True
        self.calls: list[tuple[object, ...]] = []
        self.present_raw: dict[str, int] = {
            str(joint_name): 2048 for joint_name in JOINT_NAMES
        }
        self.goal_raw: dict[str, int] = {
            str(joint_name): 0 for joint_name in JOINT_NAMES
        }

    def connect(self) -> None:
        self.calls.append(("connect",))
        self.is_connected = True

    def disable_torque(self, num_retry: int = 0) -> None:
        self.calls.append(("disable_torque", num_retry))

    def enable_torque(self, num_retry: int = 0) -> None:
        self.calls.append(("enable_torque", num_retry))

    def configure_motors(self, **settings: int) -> None:
        self.calls.append(("configure_motors", settings))

    def write(self, register: str, motor: str, value: int) -> None:
        self.calls.append(("write", register, motor, value))

    def sync_read(
        self,
        register: str,
        *,
        normalize: bool,
        num_retry: int = 0,
    ) -> dict[str, float] | dict[str, int]:
        self.calls.append(("sync_read", register, normalize, num_retry))
        if register == "Goal_Position":
            return {
                str(joint_name): int(value)
                for joint_name, value in self.goal_raw.items()
            }
        if normalize:
            return {str(joint_name): 0.0 for joint_name in JOINT_NAMES}
        return {
            str(joint_name): int(value)
            for joint_name, value in self.present_raw.items()
        }

    def sync_write(
        self,
        register: str,
        values: dict[str, int],
        *,
        normalize: bool,
        num_retry: int,
    ) -> None:
        self.calls.append(("sync_write", register, normalize, num_retry))
        self.goal_raw = {
            str(joint_name): int(value) for joint_name, value in values.items()
        }

    def disconnect(self, disable_torque: bool) -> None:
        self.calls.append(("disconnect", disable_torque))
        self.is_connected = False


class SO101Follower5DContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration_directory = tempfile.TemporaryDirectory()
        config = SO101Follower5DConfig(
            port="/dev/epuzzle-arm",
            id="contract-test",
            calibration_dir=Path(self.calibration_directory.name),
        )
        self.robot = SO101Follower5D(config)

    def tearDown(self) -> None:
        self.calibration_directory.cleanup()

    def test_source_metadata_version_matches_pyproject(self) -> None:
        package_root = Path("workspace/src/robot_plugin/lerobot_robot_so101_5d")
        pyproject = tomllib.loads(
            (package_root / "pyproject.toml").read_text(encoding="utf-8")
        )
        metadata_lines = (
            (package_root / "lerobot_robot_so101_5d.egg-info" / "PKG-INFO")
            .read_text(encoding="utf-8")
            .splitlines()
        )
        metadata_version = next(
            line.removeprefix("Version: ")
            for line in metadata_lines
            if line.startswith("Version: ")
        )

        self.assertEqual(metadata_version, pyproject["project"]["version"])

    def test_bumpless_connect_aligns_goal_before_enabling_torque(self) -> None:
        fake_bus = _FakeBumplessBus()
        cast(Any, self.robot).bus = fake_bus

        observation = self.robot.connect_bumpless()

        goal_index = next(
            index
            for index, call in enumerate(fake_bus.calls)
            if call[0] == "sync_write"
        )
        enable_index = next(
            index
            for index, call in enumerate(fake_bus.calls)
            if call[0] == "enable_torque"
        )
        self.assertLess(goal_index, enable_index)
        self.assertEqual(fake_bus.goal_raw, fake_bus.present_raw)
        self.assertEqual(tuple(observation), EXPECTED_FEATURES)
        read_calls = [call for call in fake_bus.calls if call[0] == "sync_read"]
        self.assertTrue(all(call[3] == 2 for call in read_calls))

    def test_feature_order_and_motor_ids_are_exactly_five_axis(self) -> None:
        self.assertEqual(JOINT_FEATURES, EXPECTED_FEATURES)
        self.assertEqual(tuple(self.robot.action_features), EXPECTED_FEATURES)
        self.assertEqual(tuple(self.robot.observation_features), EXPECTED_FEATURES)
        self.assertEqual(tuple(self.robot.bus.motors), JOINT_NAMES)
        self.assertEqual(
            [motor.id for motor in self.robot.bus.motors.values()], [1, 2, 3, 4, 5]
        )
        self.assertNotIn("gripper.pos", self.robot.action_features)

    def test_send_action_rejects_six_axis_legacy_field_before_bus_access(self) -> None:
        action: dict[str, float] = dict.fromkeys(EXPECTED_FEATURES, 0.0)
        action["gripper.pos"] = 0.0
        with self.assertRaisesRegex(ValueError, "extra=.*gripper.pos"):
            self.robot.send_action(action)

    def test_send_action_rejects_missing_joint_before_bus_access(self) -> None:
        action: dict[str, float] = dict.fromkeys(EXPECTED_FEATURES[:-1], 0.0)
        with self.assertRaisesRegex(ValueError, "missing=.*wrist_roll.pos"):
            self.robot.send_action(action)

    def test_constructor_rejects_legacy_six_axis_calibration_file(self) -> None:
        calibration_dir = Path(self.calibration_directory.name)
        joint_names = (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        )
        payload = {
            joint_name: {
                "id": motor_id,
                "drive_mode": 0,
                "homing_offset": 0,
                "range_min": 0,
                "range_max": 4095,
            }
            for motor_id, joint_name in enumerate(joint_names, start=1)
        }
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            suffix=".json",
            dir=calibration_dir,
            delete=False,
        ) as calibration_file:
            json.dump(payload, calibration_file)
            robot_id = Path(calibration_file.name).stem
        config = SO101Follower5DConfig(
            port="/dev/epuzzle-arm",
            id=robot_id,
            calibration_dir=calibration_dir,
        )

        with self.assertRaisesRegex(ValueError, "extra=.*gripper"):
            SO101Follower5D(config)


if __name__ == "__main__":
    unittest.main()
