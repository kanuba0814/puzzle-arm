# 拼图机器人实机验证报告（人类阅读版）

**生成时间**: 2026-08-01 09:36:45

## 1. 任务概览

- **任务**: 第一题（自备碎片）—— 4 片白色碎片拼成 100×60mm 矩形
- **求解状态**: SOLVED
- **反推矩形**: 98.3 × 59.0 mm（目标 100 × 60 mm）
- **目标区中心**: (105.0, 74.25) mm（A4 下半区）

## 2. 每片计划参数

| 片 | 源位置 (mm) | 源朝向 (°) | 目标位置 (mm) | 目标朝向 (°) | Δ旋转 (°) |
|---|---|---|---|---|---|
| piece_00 | (37.8, 272.0) | -34.6 | (92.0, 84.5) | -34.6 | +0.0 |
| piece_01 | (164.2, 263.2) | 155.0 | (136.1, 73.8) | 139.6 | -15.4 |
| piece_02 | (36.5, 225.1) | -80.8 | (74.8, 106.3) | -29.4 | +51.4 |
| piece_03 | (127.1, 233.8) | -158.7 | (66.6, 71.1) | -32.1 | +126.6 |

## 3. 执行结果（最后一次放置）

| 片 | 实际落点 (mm) | 目标位 (mm) | 位置偏差 | 实际旋转 | 目标Δ旋转 | 旋转偏差 |
|---|---|---|---|---|---|---|
| piece_00 | (92.7, 85.2) | (92.0, 84.5) | **1.0 mm** | -0.1° | +0.0° | -0.1° |
| piece_01 | (133.6, 76.4) | (136.1, 73.8) | **3.6 mm** | -14.0° | -15.4° | +1.4° |
| piece_02 | (76.7, 106.5) | (74.8, 106.3) | **1.9 mm** | +51.1° | +51.4° | -0.4° |
| piece_03 | (68.0, 71.2) | (66.6, 71.1) | **1.4 mm** | +126.0° | +126.6° | -0.7° |

## 4. 运动补偿明细（关键参数）

旋转补偿公式：`碎片旋转 = (pan_place + roll_place) - (pan_pick + roll_pick) = Δyaw`

| 片 | pan_pick | pan_place | Δpan | roll_pick | roll_place | 补偿后旋转 |
|---|---|---|---|---|---|---|
| piece_00 | +61.0° | -18.9° | -79.9° | -91.3° | -11.4° | -0.1° |
| piece_01 | +33.7° | -16.5° | -50.2° | -234.8° | -198.6° | -14.0° |
| piece_02 | +49.1° | -13.0° | -62.1° | -125.1° | -12.0° | +51.1° |
| piece_03 | +32.0° | -28.6° | -60.7° | -185.6° | +1.1° | +126.0° |

## 5. 尺寸偏差分析

### 5.1 反推矩形 vs 目标

- 求解器反推: **98.3 × 59.0 mm**
- 题目目标: 100 × 60 mm
- 尺寸误差: 1.7 mm × 1.0 mm

### 5.2 偏差可能来源

| 来源 | 影响量级 | 说明 |
|---|---|---|
| 视觉检测多边形误差 | 1-3 mm | 边缘精化/顶点拟合残差 |
| 磁头吸点偏差 | 2-5 mm | 磁铁吸附点 ≠ 视觉握点 |
| 关节到位残余误差 | 0.5-2 mm | 终点 3° 容差下的 TCP 偏移 |
| 旋转执行误差 | 0-5° | roll 物理限位内等价角选择 |
| USB 掉线中断 | 不定 | 需重试恢复，已加 3s×2 |

## 6. 执行时间线（最近一次）

- 状态: LOCAL_PLACE_ALIGN->RELEASE
- **magnet:off** (piece_02)
- 状态: RELEASE->VERIFY_PLACE
- `VERIFY_PLACE` safe-piece_02-58802017 (5 waypoints)
- verify_place_unsensed (无传感器确认)
- 状态: VERIFY_PLACE->LOCAL_GRASP_ALIGN
- `LOCAL_GRASP_ALIGN` safe-piece_03-303d1ab8 (36 waypoints)
- `LOCAL_GRASP_ALIGN` pregrasp-piece_03-f67a06d6 (3 waypoints)
- `LOCAL_GRASP_ALIGN` pick-piece_03-a51414dc (3 waypoints)
- 状态: LOCAL_GRASP_ALIGN->MAGNET_ON
- **magnet:on** (piece_03)
- 状态: MAGNET_ON->VERIFY_PICK
- `VERIFY_PICK` safe-piece_03-dda0e956 (6 waypoints)
- verify_pick_unsensed (无传感器确认)
- 状态: VERIFY_PICK->MOVE_PREPLACE
- `MOVE_PREPLACE` safe-piece_03-11cc6ad3 (12 waypoints)
- `MOVE_PREPLACE` safe-piece_03-682c726a (13 waypoints)
- 状态: MOVE_PREPLACE->LOCAL_PLACE_ALIGN
- `LOCAL_PLACE_ALIGN` preplace-piece_03-5deb7e06 (3 waypoints)
- `LOCAL_PLACE_ALIGN` place-piece_03-2c128aad (3 waypoints)
- 状态: LOCAL_PLACE_ALIGN->RELEASE
- **magnet:off** (piece_03)
- 状态: RELEASE->VERIFY_PLACE
- `VERIFY_PLACE` safe-piece_03-4ff37e6a (5 waypoints)
- verify_place_unsensed (无传感器确认)

## 7. 结论

- 位置偏差: 4 片全部 ≤ 3.6 mm（手动验证场景）
- 旋转偏差: 需根据实际执行数据确认（见第 3 节）
- 详细逐 waypoint 数据: `motion_detail/*.jsonl`
- 机器可读分析: `motion_analysis_v10.json`
