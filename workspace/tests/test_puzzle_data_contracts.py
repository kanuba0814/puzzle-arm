import json
import unittest
from pathlib import Path


class PuzzleDataContractsTest(unittest.TestCase):
    def test_new_schemas_are_strict_and_parse(self) -> None:
        schemas = {
            path.name: json.loads(path.read_text(encoding="utf-8"))
            for path in Path("data_contracts").glob("*puzzle*.schema.json")
        }
        piece = json.loads(
            Path("data_contracts/piece_observation.schema.json").read_text(
                encoding="utf-8"
            )
        )
        placement = json.loads(
            Path("data_contracts/placement_plan.schema.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertIn("puzzle_solve_result.schema.json", schemas)
        self.assertFalse(piece["additionalProperties"])
        self.assertFalse(piece["$defs"]["measuredContour"]["additionalProperties"])
        self.assertEqual(
            piece["$defs"]["measuredContour"]["properties"]["frame"]["enum"],
            ["image_px", "rectified_px"],
        )
        self.assertEqual(piece["properties"]["yaw_deg"]["exclusiveMaximum"], 180)
        self.assertEqual(
            placement["$defs"]["pose2"]["properties"]["yaw_deg"]["exclusiveMaximum"],
            180,
        )
        self.assertFalse(placement["additionalProperties"])


if __name__ == "__main__":
    unittest.main()
