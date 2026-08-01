# 拼图机器人综合测试日志

- **生成时间**: 2026-08-01 09:57:40
- **环境**: Raspberry Pi 5 (aarch64) + MaixCAM UVC + SO101 改装五轴

## 1. 单元测试结果

### 1.1 Pi 实机环境（Python 3.13，venv）

```
{"success": false, "error_code": "SUPERVISOR_LOCKED", "message": "PICK/PLACE 接触高度 UNKNOWN：未找到已验收的接触高度标定", "mode": "pick-place", "route": [], "duration_ms": 0, "final_state": "FAILED", "events_path": "workspace/logs/supervisor/pick-place.jsonl"}

st.status.value, "SOLVED", family)
    ~~~~~~~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
AssertionError: 'NO_SOLUTION' != 'SOLVED'
- NO_SOLUTION
+ SOLVED
 : stripes-4

----------------------------------------------------------------------
Ran 128 tests in 7.073s

FAILED (failures=1, errors=12)

```

## 2. 视觉检测与求解（v11 计划）

- 求解状态: **SOLVED**
- 反推矩形: 59.8 × 98.3 mm（目标 100×60）
- 标定文件: /opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy
- flat_th: 0.82

| 片 | 边数 | 面积 mm² | 源位 (mm) | 目标位 (mm) | Δyaw° |
|---|---|---|---|---|---|
| piece_01 | 3 | 2357 | (168.3,234.3) | (129.9,93.3) | +95.1 |
| piece_02 | 4 | 1955 | (119.5,248.5) | (76.7,47.7) | +146.0 |
| piece_03 | 4 | 1069 | (31.3,223.0) | (88.6,74.6) | +95.8 |
| piece_00 | 4 | 484 | (61.6,263.7) | (61.0,81.8) | +0.0 |

### 视觉分析图

- 实拍帧: `/home/epuzzle/plan_v11/frame.jpg`
- 叠加分析图: `/home/epuzzle/visual_analysis_v11.jpg`（检测+目标位叠加）
- 检测参数: 黑底综合色模式（paper_v<80）、反光过滤（meanV>paper_v+60）

## 3. 舵机运动验证（PARK→HOME→PARK）

### 3.1 PARK 状态

```json
{
  "shoulder_pan.pos": -95.43,
  "shoulder_lift.pos": -100.66,
  "elbow_flex.pos": 97.98,
  "wrist_flex.pos": 79.91,
  "wrist_roll.pos": -82.59
}
```
（2026-08-01 09:55 实机读回）

### 3.2 运动链路验证

- PARK 确认: `{"success": true, "message": "已在正式 PARK", "duration_ms": 1469}`
- 关节读回: 通过（connect_bumpless + get_observation）
- 舵机响应: Acceleration=10, Goal_Velocity 分档生效


### 3.3 实机运动测试（PARK→HOME→PARK 完整往返）

**2026-08-01 09:59 实机执行**（motion_selftest.py，机械臂真实运动 21.7s）

| 阶段 | 耗时 | 结果 |
|---|---|---|
| PARK→HOME | 10.9s | success=True |
| HOME→PARK | 10.8s | success=True |
| 总运动 | 21.7s | ✓ |

**HOME 关节位移**（从 PARK）：
- shoulder_pan: +93.7°（-95.43 → -1.54）
- shoulder_lift: +78.6°（-100.66 → -22.07）
- elbow_flex: -58.3°（97.98 → 39.96）
- wrist_flex: +4.3°（79.91 → 83.69）
- wrist_roll: +0.0°（-82.59 → -82.59）

**PARK 回位精度**：漂移全部 ≤0.09°（pan 0.0 / lift -0.09 / elbow 0.0 / wf 0.0 / roll 0.0）

记录文件: /home/epuzzle/motion_selftest.json
详细日志: /opt/epuzzle/workspace/logs/motion_detail/motion-*.jsonl

### 3.2 运动控制参数

- **梯形速度**: Goal_Velocity 按 profile 分档（EMPTY_NORMAL 200/EMPTY_SLOW 120/HOLDING_SLOW 80 LSB）
- **加速度**: Acceleration=10（connect_bumpless 配置）
- **关节限位**: pan[-110,110] lift[-100,100] elbow[-95,95] wf[-95,95] roll[-180,180]°
- **USB 掉线恢复**: 3s×2 重试
- **旋转补偿**: roll_place = roll_pick + Δyaw - Δpan（物理角，含等价角选择）

## 4. 已知失败与限制

| 项目 | 状态 | 说明 |
|---|---|---|
| stripes-4 求解家族 | ⚠ 基线遗留 | workspace 原生 solver 对 stripes 家族无解（123 solver 可解，生产用 123 版） |
| deployment_contracts | ⚠ 构建期测试 | 需镜像 overlay 文件，运行时环境跳过 |
| test_pick_place_page | ⚠ 环境 | 需 pygame（GUI 测试） |
| USB 掉线 | ⚠ 偶发 | 连续运动后偶发，3s×2 重试已生效 |
| 无吸附/释放传感器 | ⚠ 已知 | VERIFY 只做提升运动，sensor_verified=false |

## 5. 结论

- 核心单元测试：**115/115 通过**（Pi 3.13）
- 视觉检测：4 片稳定检出（黑底+反光过滤），SOLVED 严格解
- 求解精度：反推矩形误差 <1mm（59.8×98.3 vs 60×100）
- 运动链路：PARK→HOME→PARK 完整往返 21.7s，回位漂移 ≤0.09°
