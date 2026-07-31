# pyright: reportMissingImports=false

import json
import tempfile
import unittest
from pathlib import Path

from integration.event_log import JsonlEventLog


class ManualClock:
    def __init__(self) -> None:
        self.values = iter((5, 5, 3, 10))

    def monotonic_ns(self) -> int:
        return next(self.values)


class EventLogTest(unittest.TestCase):
    def test_jsonl_events_are_schema_shaped_and_monotonic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "runtime.jsonl"
            log = JsonlEventLog(path, ManualClock().monotonic_ns)
            log.record(state="INIT", event="transition:PARKED->INIT")
            log.record(
                state="SCAN", event="motion_actual_readback", payload={"actual": {}}
            )
            log.record(
                state="ESTOP",
                event="magnet:off",
                severity="ERROR",
                error_code="ESTOP_ACTIVE",
            )
            lines = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
        required = {"timestamp_ns", "state", "event", "severity"}
        self.assertTrue(all(required <= set(line) for line in lines))
        self.assertTrue(
            all(
                set(line) <= required | {"piece_id", "error_code", "payload"}
                for line in lines
            )
        )
        self.assertEqual([line["timestamp_ns"] for line in lines], [5, 6, 7])


if __name__ == "__main__":
    unittest.main()
