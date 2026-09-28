#!/usr/bin/env python3
"""实机舵机运动测试：PARK→HOME→PARK 完整往返，记录关节轨迹与耗时。

用法（Pi 上，sudo）：
    PYTHONPATH=... /opt/epuzzle/venv/bin/python tools/motion_selftest.py
"""

import json
import sys
import time
from pathlib import Path

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from control.motion_controller import MotionConfig  # noqa: E402
from control.magnet_controller import MagnetController  # noqa: E402
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402

CAL = "/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"
TRANS = "/opt/epuzzle/workspace/calibration/contact_sheet_to_robot_base_20260731.json"
PARK_POSE = "/opt/epuzzle/workspace/calibration/park_pose_single_arm_20260731.json"
CALIB_DIR = Path(
    "/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)

mc = MotionConfig(
    15.0,
    10.0,
    10.0,
    3.0,
    1,
    0.02,
    3.0,
    -20.0,
    -30.0,
    180.0,
    transit_cartesian_step_mm=25.0,
    follow_tolerance_deg=8.0,
    # 与 Supervisor 保持一致：空载/接近/持物分别 +25% 提速。
    goal_velocity_by_profile={
        "EMPTY_NORMAL": 250,
        "EMPTY_SLOW": 180,
        "HOLDING_SLOW": 100,
    },
)

adapter = InspectionHardwareAdapter(
    calibration_path=Path(CAL),
    transform_path=Path(TRANS),
    park_pose_path=Path(PARK_POSE),
    port=Path("/dev/epuzzle-arm"),
    lerobot_calibration_dir=CALIB_DIR,
    motion_config=mc,
    motion_detail_log_dir=Path("/opt/epuzzle/workspace/logs/motion_detail"),
)

records: list[dict] = []


def joint_snapshot(tag: str) -> dict:
    obs = adapter.robot.get_observation()
    j = {k: round(float(v), 2) for k, v in obs.items() if k.endswith(".pos")}
    rec = {"tag": tag, "t_s": round(time.monotonic(), 2), "joints": j}
    records.append(rec)
    print(f"[{tag}] {j}")
    return j


def main() -> None:
    magnet = MagnetController()
    try:
        print("=== 舵机运动自检：PARK→HOME→PARK ===")
        print("连接机械臂...")
        adapter.connect()
        magnet.connect()
        magnet.set_enabled(False)

        # 初始状态
        j0 = joint_snapshot("start")

        # PARK → HOME（大幅运动）
        print("\nPARK → HOME...")
        t0 = time.monotonic()
        res = adapter.park_mover.park_to_home()
        dt = time.monotonic() - t0
        j1 = joint_snapshot("home")
        print(f"  park_to_home: success={res.success} 耗时={dt:.1f}s")
        if not res.success:
            raise RuntimeError(f"park_to_home 失败: {res.message}")
        records.append(
            {"tag": "park_to_home", "duration_s": round(dt, 2), "success": res.success}
        )

        # 保持 1 秒后读回稳定
        time.sleep(1.0)
        j2 = joint_snapshot("home_stable")

        # HOME → PARK（回位）
        print("\nHOME → PARK...")
        t0 = time.monotonic()
        res2 = adapter.park_mover.home_to_park()
        dt2 = time.monotonic() - t0
        j3 = joint_snapshot("park_back")
        print(f"  home_to_park: success={res2.success} 耗时={dt2:.1f}s")
        if not res2.success:
            raise RuntimeError(f"home_to_park 失败: {res2.message}")
        records.append(
            {
                "tag": "home_to_park",
                "duration_s": round(dt2, 2),
                "success": res2.success,
            }
        )

        # 验证回到 PARK
        j4 = joint_snapshot("park_verify")
        drift = {k: round(j4[k] - j3[k], 2) for k in j3 if k in j4}
        print(f"\nPARK 回位漂移: {drift}")

        # 汇总
        total = sum(r.get("duration_s", 0) for r in records if "duration_s" in r)
        print(f"\n=== 完成：总运动耗时 {total:.1f}s ===")
        print(f"HOME 关节位移: " + ", ".join(f"{k}:{j1[k] - j0[k]:+.1f}°" for k in j0))

        out = Path("/home/epuzzle/motion_selftest.json")
        out.write_text(json.dumps(records, ensure_ascii=False, indent=2))
        print(f"记录已保存: {out}")

    finally:
        magnet.emergency_off()
        adapter.disconnect()


if __name__ == "__main__":
    main()
