"""开发机静态覆盖赛场镜像的服务、脚本及自检合同。"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import json
import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEPLOYMENT_ROOT = ROOT / "deployment/raspberry_pi"
OVERLAY = DEPLOYMENT_ROOT / "overlay"
SYSTEMD = OVERLAY / "etc/systemd/system"
UDEV_RULES = OVERLAY / "etc/udev/rules.d/70-epuzzle-devices.rules"
SELFCHECK_PATH = OVERLAY / "usr/local/sbin/epuzzle-selfcheck"
START_SCRIPT = ROOT / "workspace/tools/start_pick_place_page.sh"


def load_selfcheck_module():
    loader = importlib.machinery.SourceFileLoader(
        "epuzzle_selfcheck", str(SELFCHECK_PATH)
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    if spec is None:
        raise RuntimeError("cannot import epuzzle-selfcheck")
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class DeploymentContractTests(unittest.TestCase):
    def test_gui_service_is_target_member_and_uses_no_privilege_escalation(
        self,
    ) -> None:
        gui_unit = (SYSTEMD / "epuzzle-gui.service").read_text(encoding="utf-8")
        target_unit = (SYSTEMD / "epuzzle.target").read_text(encoding="utf-8")
        self.assertIn("After=epuzzle-magnet-safe.service", gui_unit)
        self.assertIn("Wants=epuzzle-magnet-safe.service", gui_unit)
        self.assertNotIn("epuzzle-preflight.service", gui_unit)
        self.assertIn(
            "ExecStart=/opt/epuzzle/workspace/tools/start_pick_place_page.sh", gui_unit
        )
        self.assertIn("User=epuzzle", gui_unit)
        self.assertIn("SupplementaryGroups=video input render dialout gpio", gui_unit)
        self.assertIn("TTYPath=/dev/tty2", gui_unit)
        self.assertIn("WantedBy=multi-user.target", gui_unit)
        self.assertIn("epuzzle-gui.service", target_unit)
        self.assertNotIn("epuzzle-supervisor.service", target_unit)
        self.assertNotIn("sudo", gui_unit.lower())

    def test_supervisor_daemon_is_not_started_until_entrypoint_exists(self) -> None:
        supervisor_unit = (SYSTEMD / "epuzzle-supervisor.service").read_text(
            encoding="utf-8"
        )
        target_unit = (SYSTEMD / "epuzzle.target").read_text(encoding="utf-8")
        self.assertIn("ConditionFileIsExecutable=", supervisor_unit)
        self.assertNotIn("epuzzle-supervisor.service", target_unit)
        self.assertIn("Requires=epuzzle-preflight.service", supervisor_unit)

    def test_gui_script_is_executable_and_never_elevates_privileges(self) -> None:
        script = START_SCRIPT.read_text(encoding="utf-8")
        self.assertTrue(START_SCRIPT.stat().st_mode & stat.S_IXUSR)
        self.assertIn("EP_GUI_ON_VT", script)
        self.assertIn("SDL_VIDEODRIVER=dummy", script)
        self.assertIn("EPUZZLE_PYTHON", script)
        plugin_path = "$PROJECT_DIR/workspace/src/robot_plugin/lerobot_robot_so101_5d"
        venv_path = "/opt/epuzzle/venv/lib/python3.13/site-packages"
        self.assertLess(script.index(plugin_path), script.index(venv_path))
        self.assertNotIn("sudo", script.lower())
        self.assertNotIn("openvt", script.lower())

    def test_arm64_requirements_and_rebuild_verify_pygame(self) -> None:
        requirements = (DEPLOYMENT_ROOT / "requirements-arm64.in").read_text(
            encoding="utf-8"
        )
        reinstall = (OVERLAY / "usr/local/sbin/epuzzle-offline-reinstall").read_text(
            encoding="utf-8"
        )
        build_script = (DEPLOYMENT_ROOT / "build-image.sh").read_text(encoding="utf-8")
        self.assertIn("pygame-ce==2.5.5", requirements)
        self.assertIn("import cv2, lerobot, numpy, pygame", reinstall)
        self.assertIn("pygame_ce-2.5.5-*-aarch64.whl", build_script)
        self.assertIn("pygame-ce wheel 未由 SHA256SUMS 覆盖", build_script)
        self.assertNotIn("缺少 canonical Supervisor entrypoint", build_script)

    def test_environment_enables_only_confirmed_magnet_control_chain(self) -> None:
        environment = (OVERLAY / "etc/epuzzle/epuzzle.env").read_text(encoding="utf-8")
        registry = (ROOT / "configs/parameter_registry.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("EPUZZLE_MAGNET_ENABLED=1", environment)
        self.assertIn(
            "EPUZZLE_ROBOT_MOTION_LOCK=/etc/epuzzle/robot-motion.lock.json",
            environment,
        )
        self.assertIn('gpio_chip: "/dev/gpiochip0"', registry)
        self.assertIn("gpio_line_bcm: 17", registry)
        self.assertIn("relay_active_high: false", registry)
        self.assertIn("relay_control_ready: true", registry)
        self.assertIn("automatic_pick_place_enabled: false", registry)

    def test_selfcheck_uses_frozen_files_and_preserves_unknown_gate(self) -> None:
        selfcheck = load_selfcheck_module()
        self.assertEqual(selfcheck.EXPECTED_GUI_PACKAGES["pygame-ce"], "2.5.5")
        self.assertEqual(selfcheck.EXPECTED_GUI_PACKAGES["pygame"], "2.6.1")
        self.assertIn("pygame", selfcheck.EXPECTED_IMPORTS)
        self.assertIn(
            "workspace/calibration/table_homography_gui_20260731.npy",
            selfcheck.FROZEN_CALIBRATION_FILES,
        )
        self.assertIn(
            "workspace/calibration/contact_sheet_to_robot_base_20260731.json",
            selfcheck.FROZEN_CALIBRATION_FILES,
        )
        self.assertIn(
            "inspection_parameters", SELFCHECK_PATH.read_text(encoding="utf-8")
        )
        self.assertIn(
            "deferred_unknown_parameters",
            SELFCHECK_PATH.read_text(encoding="utf-8"),
        )
        self.assertIn(
            "bumpless_connect_contract", SELFCHECK_PATH.read_text(encoding="utf-8")
        )
        self.assertIn(
            'PYGAME_HIDE_SUPPORT_PROMPT", "1"',
            SELFCHECK_PATH.read_text(encoding="utf-8"),
        )
        self.assertIn('state = "WARN"', SELFCHECK_PATH.read_text(encoding="utf-8"))

    def test_camera_udev_rule_freezes_capture_node_identity(self) -> None:
        rules = UDEV_RULES.read_text(encoding="utf-8")
        self.assertIn('ATTRS{idVendor}=="359f"', rules)
        self.assertIn('ATTRS{idProduct}=="2120"', rules)
        self.assertIn('ATTRS{serial}=="c9c28f4513595968"', rules)
        self.assertIn('ATTR{index}=="0"', rules)
        self.assertIn('SYMLINK+="epuzzle-camera"', rules)
        self.assertNotIn('ATTRS{idVendor}=="ffff"', rules)

    def test_parameter_selfcheck_separates_inspection_and_deferred_gates(self) -> None:
        selfcheck = load_selfcheck_module()
        checks = selfcheck.parameter_registry_checks(ROOT)
        inspection = next(
            check for check in checks if check["name"] == "inspection_parameters"
        )
        deferred = next(
            check for check in checks if check["name"] == "deferred_unknown_parameters"
        )
        self.assertTrue(inspection["passed"], inspection["detail"])
        self.assertFalse(deferred["passed"])
        self.assertEqual(deferred["severity"], "warning")
        self.assertIn("remain gated", deferred["detail"])

    def test_selfcheck_motion_lock_blocks_full_hardware_readiness(self) -> None:
        selfcheck = load_selfcheck_module()
        with tempfile.TemporaryDirectory() as temporary_directory:
            lock_path = Path(temporary_directory) / "robot-motion.lock.json"
            unlocked = selfcheck.motion_lock_check(
                {"EPUZZLE_ROBOT_MOTION_LOCK": str(lock_path)}
            )
            lock_path.write_text(
                json.dumps({"reason": "xHCI host controller died"}), encoding="utf-8"
            )
            locked = selfcheck.motion_lock_check(
                {"EPUZZLE_ROBOT_MOTION_LOCK": str(lock_path)}
            )
        self.assertTrue(unlocked["passed"])
        self.assertFalse(locked["passed"])
        self.assertIn("xHCI", locked["detail"])

    def test_selfcheck_calibration_accepts_matching_frozen_manifest(self) -> None:
        selfcheck = load_selfcheck_module()
        check = selfcheck.calibration_check(ROOT)
        self.assertTrue(check["passed"], check["detail"])

    def test_selfcheck_feature_gates_fail_open_attempts(self) -> None:
        selfcheck = load_selfcheck_module()
        passing = selfcheck.feature_gate_checks(
            {
                "EPUZZLE_ACT_ENABLED": "0",
                "EPUZZLE_GENERIC_VISION_ENABLED": "0",
                "EPUZZLE_TEXTURE_ENABLED": "0",
                "EPUZZLE_MAGNET_ENABLED": "1",
            },
            ROOT,
        )
        failing = selfcheck.feature_gate_checks(
            {
                "EPUZZLE_ACT_ENABLED": "0",
                "EPUZZLE_GENERIC_VISION_ENABLED": "1",
                "EPUZZLE_TEXTURE_ENABLED": "0",
                "EPUZZLE_MAGNET_ENABLED": "1",
            },
            ROOT,
        )
        self.assertTrue(all(check["passed"] for check in passing))
        self.assertFalse(
            next(check for check in failing if "GENERIC" in check["name"])["passed"]
        )

    def test_frozen_manifest_is_valid_json(self) -> None:
        manifest = (
            ROOT / "workspace/calibration/runtime_calibration_manifest_20260731.json"
        )
        self.assertTrue(manifest.is_file())
        payload = json.loads(manifest.read_text(encoding="utf-8"))
        self.assertEqual(
            payload["artifacts"]["kinematic_calibration"]["status"], "accepted"
        )

    def test_start_script_rejects_non_tty_without_executing_gui(self) -> None:
        python_stub = Path(tempfile.mkdtemp()) / "python"
        python_stub.write_text("#!/bin/sh\nexit 99\n", encoding="utf-8")
        python_stub.chmod(0o755)
        completed = subprocess.run(
            ["bash", str(START_SCRIPT)],
            check=False,
            capture_output=True,
            env={
                **os.environ,
                "EP_GUI_ON_VT": "0",
                "EPUZZLE_PYTHON": str(python_stub),
            },
            text=True,
        )
        self.assertEqual(completed.returncode, 2)


if __name__ == "__main__":
    unittest.main()
