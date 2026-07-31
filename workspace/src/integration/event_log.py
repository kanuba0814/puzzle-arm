"""Monotonic JSONL runtime-event writer for the Supervisor."""

from __future__ import annotations

import json
import time
from collections.abc import Callable, Mapping
from pathlib import Path


class JsonlEventLog:
    """Append schema-compatible runtime events and preserve monotonic ordering."""

    def __init__(
        self, path: str | Path, monotonic_ns: Callable[[], int] = time.monotonic_ns
    ) -> None:
        self.path = Path(path)
        self._monotonic_ns = monotonic_ns
        self._last_timestamp_ns = -1

    def record(
        self,
        *,
        state: str,
        event: str,
        severity: str = "INFO",
        piece_id: str | None = None,
        error_code: str | None = None,
        payload: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        if severity not in {"DEBUG", "INFO", "WARNING", "ERROR", "FATAL"}:
            raise ValueError(f"未知事件严重性: {severity}")
        timestamp_ns = max(self._monotonic_ns(), self._last_timestamp_ns + 1)
        self._last_timestamp_ns = timestamp_ns
        item: dict[str, object] = {
            "timestamp_ns": timestamp_ns,
            "state": state,
            "event": event,
            "severity": severity,
        }
        if piece_id is not None:
            item["piece_id"] = piece_id
        if error_code is not None:
            item["error_code"] = error_code
        if payload is not None:
            item["payload"] = dict(payload)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as output:
            output.write(json.dumps(item, ensure_ascii=False, sort_keys=True) + "\n")
        return item
