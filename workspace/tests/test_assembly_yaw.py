# pyright: reportMissingImports=false

import importlib.util
import sys
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
TOOL_PATH = WORKSPACE / "tools/run_assembly_4pieces.py"
SPEC = importlib.util.spec_from_file_location("run_assembly_4pieces", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
assembly = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = assembly
SPEC.loader.exec_module(assembly)


class AssemblyYawTest(unittest.TestCase):
    def test_relative_solver_yaw_is_applied_once_by_corrected_model(self) -> None:
        pick_yaw, place_yaw = assembly.relative_tool_yaws(40.0, 0.0, 125.0)

        self.assertEqual(pick_yaw, 40.0)
        self.assertEqual(place_yaw, 165.0)

    def test_relative_solver_yaw_uses_periodic_shortest_delta(self) -> None:
        pick_yaw, place_yaw = assembly.relative_tool_yaws(10.0, 170.0, -170.0)

        self.assertEqual(pick_yaw, 10.0)
        self.assertEqual(place_yaw, 30.0)


if __name__ == "__main__":
    unittest.main()
