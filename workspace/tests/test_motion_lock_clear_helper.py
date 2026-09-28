"""受限 root helper 的纯软件回归测试；不访问 Pi 的实际 /etc 锁。"""

from __future__ import annotations

import hashlib
import importlib.machinery
import importlib.util
import json
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
HELPER_PATH = (
    ROOT / "deployment/raspberry_pi/overlay/usr/local/sbin/epuzzle-clear-motion-lock"
)


def load_helper():
    loader = importlib.machinery.SourceFileLoader("motion_lock_clear_helper", str(HELPER_PATH))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class MotionLockClearHelperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.helper = load_helper()
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.temporary = Path(self.temporary_directory.name)
        self.lock_path = self.temporary / "robot-motion.lock.json"
        self.confirmation_path = self.temporary / "motion_lock_confirmation.json"
        self.audit_directory = self.temporary / "audit"
        self.lock_payload = {
            "schema_version": 1,
            "clear_requires": ["现场支撑机械臂", "确认磁铁 OFF"],
        }
        self.lock_bytes = json.dumps(
            self.lock_payload, ensure_ascii=False, indent=2
        ).encode("utf-8")
        self.lock_path.write_bytes(self.lock_bytes)
        self.confirmation_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "lock_sha256": hashlib.sha256(self.lock_bytes).hexdigest(),
                    "requirements": [
                        {"text": item, "checked": True}
                        for item in self.lock_payload["clear_requires"]
                    ],
                    "operator_confirmation": self.helper.OPERATOR_CONFIRMATION,
                },
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        os.chmod(self.confirmation_path, 0o600)

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def invoke(self) -> int:
        with (
            mock.patch.object(self.helper, "LOCK_PATH", self.lock_path),
            mock.patch.object(self.helper, "CONFIRMATION_PATH", self.confirmation_path),
            mock.patch.object(self.helper, "AUDIT_DIRECTORY", self.audit_directory),
            mock.patch.object(self.helper.os, "geteuid", return_value=0),
            mock.patch.object(
                self.helper.pwd,
                "getpwnam",
                return_value=types.SimpleNamespace(pw_uid=os.getuid()),
            ),
            mock.patch.object(
                sys,
                "argv",
                [str(HELPER_PATH), "--confirmation", str(self.confirmation_path)],
            ),
        ):
            return self.helper.main()

    def test_matching_checklist_writes_audit_before_removing_lock(self) -> None:
        self.assertEqual(self.invoke(), 0)
        self.assertFalse(self.lock_path.exists())
        audit_files = list(self.audit_directory.glob("clear-*.json"))
        self.assertEqual(len(audit_files), 1)
        audit = json.loads(audit_files[0].read_text(encoding="utf-8"))
        self.assertEqual(audit["cleared_lock"], self.lock_payload)

    def test_changed_lock_hash_keeps_lock(self) -> None:
        confirmation = json.loads(self.confirmation_path.read_text(encoding="utf-8"))
        confirmation["lock_sha256"] = "invalid"
        self.confirmation_path.write_text(json.dumps(confirmation), encoding="utf-8")
        os.chmod(self.confirmation_path, 0o600)
        self.assertEqual(self.invoke(), 2)
        self.assertTrue(self.lock_path.exists())
        self.assertFalse(self.audit_directory.exists())


if __name__ == "__main__":
    unittest.main()
