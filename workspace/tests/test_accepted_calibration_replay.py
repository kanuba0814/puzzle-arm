# pyright: reportMissingImports=false

import hashlib
import json
import math
import unittest
from pathlib import Path

from control.calibration import ArmCalibration
from control.kinematics import JointVector, PlanarArmKinematics


class AcceptedCalibrationReplayTest(unittest.TestCase):
    calibration_path = Path(
        "workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"
    )
    contacts_path = Path(
        "workspace/calibration/evidence/2026-07-31_single_arm_contacts_v2/"
        "single_arm_contacts_20260731.json"
    )
    transform_path = Path(
        "workspace/calibration/contact_sheet_to_robot_base_20260731.json"
    )
    report_path = Path(
        "workspace/calibration/evidence/2026-07-31_single_arm_contacts_v2/"
        "fit_report.json"
    )

    @staticmethod
    def sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as source_file:
            for block in iter(lambda: source_file.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    def test_evidence_hashes_match_archived_files(self) -> None:
        calibration = ArmCalibration.load(self.calibration_path)
        evidence_paths = (
            self.contacts_path,
            Path(
                "workspace/calibration/evidence/2026-07-31_tcp_measurement/"
                "tcp_measurement.json"
            ),
            Path(
                "workspace/calibration/evidence/2026-07-31_single_arm_motor/"
                "pi_epuzzle-single-arm.json"
            ),
            Path(
                "workspace/calibration/fixtures/single_arm_contact_sheet_v1.plan.json"
            ),
        )

        self.assertEqual(
            tuple(self.sha256(path) for path in evidence_paths),
            calibration.evidence_sha256,
        )

    def test_practical_limits_are_not_derived_from_contact_envelope(self) -> None:
        calibration = ArmCalibration.load(self.calibration_path)

        # 2026-08-01 实测放宽：elbow_flex 舵机行程 196.7°→[-95,95]；
        # wrist_roll 改装后无物理阻挡（Min/Max 寄存器 0..4095=360.4°）→[-180,180]。
        # 断言与标定文件一致（限位来自舵机行程实测，不是接触包络推导）。
        self.assertEqual(
            calibration.geometry.joint_limits_deg,
            (
                (-110.0, 110.0),
                (-100.0, 100.0),
                (-95.0, 95.0),
                (-95.0, 95.0),
                (-180.0, 180.0),
            ),
        )

    def test_production_fk_replays_independent_holdout_metrics(self) -> None:
        calibration = ArmCalibration.load(self.calibration_path)
        geometry = calibration.require_accepted_geometry()
        model = PlanarArmKinematics(geometry)
        contacts = json.loads(self.contacts_path.read_text(encoding="utf-8"))
        transform = json.loads(self.transform_path.read_text(encoding="utf-8"))[
            "inverse_sheet_from_robot"
        ]
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        self.assertFalse(report["holdout_used_during_fit"])

        yaw_rad = math.radians(transform["yaw_deg"])
        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        translation_x, translation_y, translation_z = transform["translation_mm"]
        holdout_errors: list[tuple[float, float, float]] = []
        for point in contacts["points"]:
            if point["role"] != "holdout":
                continue
            joint_deg = point["joint_deg"]
            joints = JointVector(
                tuple(
                    joint_deg[name]
                    for name in (
                        "shoulder_pan",
                        "shoulder_lift",
                        "elbow_flex",
                        "wrist_flex",
                        "wrist_roll",
                    )
                )
            )
            robot_x, robot_y, robot_z = model.forward(joints).position_mm
            predicted = (
                cosine * robot_x - sine * robot_y + translation_x,
                sine * robot_x + cosine * robot_y + translation_y,
                robot_z + translation_z,
            )
            target = point["target"]["position_mm"]
            holdout_errors.append(
                tuple(predicted[index] - target[index] for index in range(3))
            )

        xy_errors = [math.hypot(error[0], error[1]) for error in holdout_errors]
        xy_rms = math.sqrt(sum(error * error for error in xy_errors) / len(xy_errors))
        xy_max = max(xy_errors)
        z_max_abs = max(abs(error[2]) for error in holdout_errors)

        self.assertAlmostEqual(xy_rms, calibration.quality.holdout_xy_rms_mm, places=5)
        self.assertAlmostEqual(xy_max, calibration.quality.holdout_xy_max_mm, places=5)
        self.assertAlmostEqual(
            z_max_abs, calibration.quality.holdout_z_max_abs_mm, places=5
        )


if __name__ == "__main__":
    unittest.main()
