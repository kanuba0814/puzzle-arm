import json
import unittest
from pathlib import Path


class ControlDataContractsTest(unittest.TestCase):
    def test_all_json_schemas_parse(self) -> None:
        for schema_path in sorted(Path("data_contracts").glob("*.schema.json")):
            with self.subTest(schema=str(schema_path)):
                schema = json.loads(schema_path.read_text(encoding="utf-8"))
                self.assertEqual(
                    schema["$schema"],
                    "https://json-schema.org/draft/2020-12/schema",
                )

    def test_supervisor_contracts_are_strict_and_result_has_required_fields(
        self,
    ) -> None:
        config = json.loads(
            Path("data_contracts/supervisor_config.schema.json").read_text(
                encoding="utf-8"
            )
        )
        result = json.loads(
            Path("data_contracts/supervisor_result.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertFalse(config["additionalProperties"])
        self.assertFalse(result["additionalProperties"])
        self.assertIn("automatic_pick_place_enabled", config["required"])
        self.assertIn("error_code", result["required"])
        self.assertIn("duration_ms", result["required"])

    def test_commission_center_is_in_gui_status_and_result_contracts(self) -> None:
        for filename in (
            "supervisor_run_status.schema.json",
            "supervisor_run_result.schema.json",
        ):
            schema = json.loads(Path("data_contracts", filename).read_text())
            self.assertIn(
                "commission-center",
                schema["properties"]["command"]["enum"],
                filename,
            )

    def test_competition_and_manual_commands_are_in_gui_contracts(self) -> None:
        expected = {
            "competition-self-color",
            "competition-white",
            "competition-card",
            "manual-read-state",
            "manual-jog-joint",
            "manual-jog-cartesian",
            "manual-magnet-pulse",
            "manual-magnet-off",
        }
        for filename in (
            "supervisor_run_status.schema.json",
            "supervisor_run_result.schema.json",
        ):
            schema = json.loads(Path("data_contracts", filename).read_text())
            commands = set(schema["properties"]["command"]["enum"])
            self.assertTrue(
                expected.issubset(commands),
                f"{filename} 缺少命令: {expected - commands}",
            )

    def test_gui_backend_capability_schema_is_strict(self) -> None:
        schema = json.loads(
            Path("data_contracts/gui_backend_capability.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema",
        )
        self.assertFalse(schema["additionalProperties"])
        self.assertIn("availability", schema["required"])
        self.assertEqual(
            set(schema["properties"]["availability"]["enum"]),
            {"READY", "LOCKED", "UNAVAILABLE"},
        )

    def test_robot_motion_lock_schema_is_strict_and_current_lock_is_active(
        self,
    ) -> None:
        schema = json.loads(
            Path("data_contracts/robot_motion_lock.schema.json").read_text(
                encoding="utf-8"
            )
        )
        lock = json.loads(
            Path("workspace/runtime/robot_motion_lock.json").read_text(encoding="utf-8")
        )
        self.assertFalse(schema["additionalProperties"])
        self.assertTrue(lock["locked"])
        self.assertIn("xHCI", lock["reason"])
        self.assertGreaterEqual(len(lock["clear_requires"]), 5)

    def test_motion_result_final_joint_fields_are_exactly_five_axis(self) -> None:
        schema = json.loads(
            Path("data_contracts/motion_result.schema.json").read_text(encoding="utf-8")
        )
        joint_object = schema["properties"]["final_joint_deg"]["oneOf"][1]

        self.assertEqual(
            joint_object["required"],
            [
                "shoulder_pan.pos",
                "shoulder_lift.pos",
                "elbow_flex.pos",
                "wrist_flex.pos",
                "wrist_roll.pos",
            ],
        )
        self.assertNotIn("gripper.pos", joint_object["properties"])
        self.assertFalse(joint_object["additionalProperties"])

    def test_calibration_sheet_transform_has_explicit_frames_and_units(self) -> None:
        schema = json.loads(
            Path("data_contracts/calibration_sheet_to_robot.schema.json").read_text(
                encoding="utf-8"
            )
        )
        transform = json.loads(
            Path(
                "workspace/calibration/contact_sheet_to_robot_base_20260731.json"
            ).read_text(encoding="utf-8")
        )

        self.assertEqual(
            schema["properties"]["from_frame"]["const"],
            "calibration_sheet_mm",
        )
        self.assertEqual(transform["from_frame"], "calibration_sheet_mm")
        self.assertEqual(transform["to_frame"], "robot_base_mm")
        self.assertEqual(transform["translation_unit"], "mm")
        self.assertEqual(transform["rotation_unit"], "degree")
        self.assertEqual(len(transform["translation_mm"]), 3)

    def test_arm_calibration_schema_freezes_single_follower_origin(self) -> None:
        schema = json.loads(
            Path("data_contracts/arm_kinematic_calibration.schema.json").read_text(
                encoding="utf-8"
            )
        )

        self.assertEqual(
            schema["properties"]["source_arm"]["const"],
            "so101_follower_modified_single_arm",
        )
        self.assertEqual(schema["properties"]["joint_features"]["maxItems"], 5)
        self.assertIn("tool_orientation", schema["required"])
        orientation = schema["properties"]["tool_orientation"]
        self.assertFalse(orientation["additionalProperties"])
        self.assertEqual(
            orientation["properties"]["wrist_roll_yaw_sign"]["enum"], [-1, 1]
        )


if __name__ == "__main__":
    unittest.main()
