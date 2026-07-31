# pyright: reportMissingImports=false
import json
from pathlib import Path
import unittest

import cv2

from vision import detect_red_pieces, load_homography
from vision.legacy_red_adapter import red_observation_to_piece_observation
from vision.red_piece_detector import RedPieceObservation


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "calibration/evidence/2026-07-31_gui_a4_workspace"


class RedPieceReplayTest(unittest.TestCase):
    def test_replay_keeps_four_piece_centres_and_yaws(self):
        expected = json.loads(
            (EVIDENCE / "red_pieces_detection.json").read_text(encoding="utf-8")
        )["pieces"]
        image = cv2.imread(str(EVIDENCE / "red_pieces_frame.jpg"))
        observations, _, _ = detect_red_pieces(
            image,
            load_homography(ROOT / "calibration/table_homography_gui_20260731.npy"),
        )
        self.assertEqual(len(observations), 4)
        for observation, saved in zip(observations, expected):
            self.assertAlmostEqual(
                observation.center_table_mm[0], saved["center_table_mm"][0], delta=1.0
            )
            self.assertAlmostEqual(
                observation.center_table_mm[1], saved["center_table_mm"][1], delta=1.0
            )
            # Rectangle heading has a 90-degree orientation equivalence in legacy output.
            delta = min(
                abs(observation.yaw_deg - saved["angle_deg"] + shift)
                for shift in (-180, -90, 0, 90, 180)
            )
            self.assertLessEqual(delta, 2.0)

    def test_legacy_constructor_and_mapping_do_not_require_pixel_box(self):
        observation = RedPieceObservation(
            "red-0",
            1,
            (1.0, 2.0),
            (1.0, 2.0),
            ((0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)),
            0.0,
            0.5,
            (10.0, 11.0),
            400.0,
        )
        self.assertNotIn("pixel_box", observation.as_mapping())
        adapted = red_observation_to_piece_observation(observation)
        self.assertEqual(adapted.measured_contour.frame, "image_px")
        self.assertEqual(len(adapted.measured_contour.points), 4)


if __name__ == "__main__":
    unittest.main()
