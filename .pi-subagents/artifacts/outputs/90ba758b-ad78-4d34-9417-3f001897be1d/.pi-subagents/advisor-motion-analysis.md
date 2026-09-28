# 机械臂启动与单片 grasp_fsm 运动路径只读审计（advisor）

审计范围：`workspace/src/control/motion_controller.py`、`workspace/src/integration/park_sequence.py`、`workspace/src/integration/supervisor.py`、`workspace/tools/run_assembly_4pieces.py`（对照 `workspace/tools/supervisor_run.py`、`workspace/src/integration/runtime_adapter.py`、五轴插件 `so101_5d.py`），证据取自 `workspace/calibration/evidence/2026-08-01_green_assembly/motion-151102.jsonl` 与 `2026-08-01_motion_rewrite_test/TEST_LOG.json`。未改任何文件、未触硬件。

## Inherited decisions（基线契约）

- PICK/PLACE 接触段恒禁止 follow 容差与 USB 自动重连（motion_controller.py `use_follow`、`reconnect_allowed`）。
- `max_joint_step_deg=10°` 绝不放宽；超限只能翻倍加密（AGENTS.md 明确记录）。
- 全路径 IK 预检必须在第一条运动命令之前完成；正式 PARK 末点用 4° 严格容差。
- 垂直 approach（30 mm）、z_safe=60 mm、HOME→PARK 收尾 + 断力矩验收为硬约束。
- 旋转分两段（平移保源 yaw → 目标上方原地旋转）是防撞架安全设计（supervisor.py:437-443 注释）。
- HOLDING_SLOW 是负载段的安全档位，不得为提速绕过。

## 1) 可量化的卡顿根因（按贡献排序）

### R1. 路点数 × 每路点固定开销：15 mm/10° 细分段 + dwell + 读回门控（主因）
- 实测（motion-151102.jsonl）：单片 9 段 move() 共 57 路点；piece_00 运动总耗时 **31.6 s**，piece_01 前 6 段已 32.7 s。15 个 goal 共 **113 路点 / 64.7 s**。
- 每路点成本 = `waypoint_min_dwell`（EMPTY_NORMAL 0.15 s / EMPTY_SLOW 0.20 s / HOLDING_SLOW 0.30 s，run_assembly_4pieces.py:107-113）+ 50 ms 轮询（实测读回间隔中位数 52 ms）+ 跟随收敛等待。旋转段实测 **1.09 s/路点、每路点 22 次读回**。
- **根因放大器：`run_assembly_4pieces.py` 的 `pick_place_motion_config()`（79-114 行）没有设置 `transit_cartesian_step_mm`**，非接触转移仍按 15 mm 切段；而兄弟工具 `supervisor_run.py:185` 已设 25 mm（注释记录 40 mm 曾致单步关节跳 10.14° 超限）。这是明显的工具间漂移：同一 `MotionConfig` 字段在装配主路径未启用。例：piece_00 转移段 15 步 / 6.48 s，改 25 mm 约 9 步。

### R2. PARK↔HOME 启动/收尾：1° 步进 × 逐步读回 ≈ 15.1 s × 2
- `runtime_adapter.py:206-214` 构造 `JointSpaceParkMover(step_deg=1.0, settle_s=0.06)`；`park_sequence.py:147` 步数 = ceil(最大关节差/1°)。
- 当前 PARK→upright 最大差 82.5°（83 步）+ upright→HOME pan 差 94.9°（95 步）= **178 个 1° 路点**，TEST_LOG.json 实测 `park_to_home` 与 `home_to_park` **各 15.1 s**（约 85 ms/步：send + settle 60 ms + 50 ms 轮询底）。
- 跟随容差是 8°，1° 步进相对容差过采样 8 倍；逐步读回门控在 1° 步距下买不到任何安全收益（步长只控制命令不连续性，MotionController 侧同类限制是 10°）。

### R3. 目标上方原地旋转段是单片最大单段耗时（物理 bound，勿硬压）
- piece_00 `safe-...-62a7896f`：12 步 / **13.03 s**；piece_01 同型段 12 步 / 11.50 s。
- HOLDING_SLOW=125 LSB ≈ 22 °/s（pan 无覆盖），10° joint 步距下单步运动 ~0.45 s + dwell 0.30 s + 轮询 ≈ 1.1 s。该段是两段式防撞架设计的必要成本，只能靠减少"路点固定开销"边际改善，不能靠提速（负载段）。

### R4. 非运动固定延迟（小头）
- `run_assembly_4pieces.py:515` 每片 `time.sleep(1.0)`（纯 pacing，4 片 ≈ 4 s）。
- 磁铁安定时间当前标定值 on/off 均 0.0 s（pick_place_contact_height.json `magnet_verification`），已非瓶颈。
- `_polling_drive_to_joints`（158 行起，step 3° / 0.12 s）只在断力矩漂移后触发，~1-2 s，保留。

### R5. 诊断残留配置（正确性/合规风险，顺带影响收敛）
- `run_assembly_4pieces.py:86` `joint_tolerance_deg=5.0`，注释自述"2026-08-01 诊断试验……**这不是最终安全标定**"；supervisor_run.py 用 3.0。5° 末点容差 + AGENTS.md 记录的"5°/单样本会使负载下 TCP 提前 10–20 mm 判到位"教训相冲突，应回到 3°（或经实测论证的 4°）并补记录。
- 同文件 `waypoint_timeout_s=30.0`（90 行）vs supervisor_run.py 的 3.0 s；30 s 是 wrist_roll 慢收敛的临时宽限，故障时会把"卡住"拖长 10 倍才报错。

## 2) 可合并/删除 vs 硬约束

### 可安全调整（不动安全语义）
| 项 | 现状 | 建议 | 依据 |
|---|---|---|---|
| transit 步长 | run_assembly 未设（=15 mm） | 设 `transit_cartesian_step_mm=25.0` | supervisor_run.py:185 已有同款先例与 40 mm 失败边界记录；PICK/PLACE 不受影响（motion_controller.py 相位判断） |
| PARK 步进 | `step_deg=1.0` → 178 步 | 4.0°（≤ follow 8°，远小于 MC 的 10° 关节步限） | park_sequence.py:147 纯步数公式；保留逐步读回门控与末点 4° 容差，安全性质不变 |
| 每片 `sleep(1.0)` | run_assembly_4pieces.py:515 | 删除或降为 0.2 s | 无安全语义，磁铁 settle 已由 `_settle` 覆盖 |
| `joint_tolerance_deg=5.0` | 诊断残留 | 回 3.0 与 supervisor_run 对齐 | 文件内注释自认非最终标定 |
| `waypoint_timeout_s=30.0` | 临时宽限 | 回到 3–5 s（wrist_roll 若需宽限按关节单独论证） | supervisor_run.py 先例 |
| 末段 VERIFY_PLACE 用 EMPTY_SLOW | supervisor.py:466 | 可评 EMPTY_NORMAL（已空载） | 收益 ~0.5 s/片，可选 |

### 硬约束，禁止删/绕
- grasp_fsm 9 段骨架：safe→pregrasp(垂直 approach 30 mm)→PICK→抬 z_safe→两段式转移（保源 yaw 平移 + 目标上方原地旋转）→preplace→PLACE→抬 z_safe（supervisor.py:414-466）。垂直 lift-before-translate 与旋转拆分都是防横扫/防撞架设计，不可合并。
- PICK/PLACE 相位禁用 follow 容差（motion_controller.py `use_follow` 排除 PICK/PLACE）、接触段禁 USB 重连。
- 入口门禁链：`pick_place_lock_reason` → `robot_motion_lock_reason` → 设备节点 → 连接（run_assembly_4pieces.py main 前段）；grasp_fsm 的 heights/transform/磁头净空/计划绑定门禁（supervisor.py `_grasp_gates`）。
- 全路径 IK 预检先于首条运动命令；`max_joint_step_deg=10°` 不放宽（只翻倍加密）。
- PARK 相关：`is_at_park` 前置（PARK_MISMATCH）、HOME→PARK 末点 4° 正式容差（park_sequence.py:170-174）、断力矩漂移后的 `_polling_drive_to_joints` 归位。
- `finally` 中 `magnet.emergency_off()`（supervisor.py:468-477、run_assembly finally）。

### 发现的预检覆盖缺口（非性能项，顺带报告）
- run_assembly_4pieces.py 预检路点用 `PREGRASP_HEIGHT_MM=20`（yaw_pose(src_ref, PREGRASP_HEIGHT_MM, ...)），但 grasp_fsm 实际执行的 pregrasp 是 `z_pick+30 = 36 mm`（supervisor.py:399-401）。预检未覆盖实际执行点；虽然 `MotionController._plan` 对每个目标端点仍做 IK 与高度门禁，不会放到不可达点，但"预检=执行路径"的契约被削弱，建议预检改用与 grasp_fsm 相同的 `vertical_approach` 构造。

## 3) 推荐最小安全改动与验收指标

按风险从低到高，全部落在工具层/参数层，不动 control/integration 安全逻辑：

1. **run_assembly_4pieces.py `pick_place_motion_config()`**：加 `transit_cartesian_step_mm=25.0`；`joint_tolerance_deg` 回 3.0；`waypoint_timeout_s` 回 5.0。
2. **run_assembly_4pieces.py:515**：删 `time.sleep(1.0)`（或 0.2 s）。
3. **runtime_adapter.py:212**：`step_deg=1.0 → 4.0`（park_sequence 末点 4° 正式容差与逐步读回保持不变）。
4. 预检路点改用 `vertical_approach` 高度（36 mm）替代 20 mm，消除覆盖缺口。
5. 附带修正文档漂移：AGENTS.md 记录新高度文件 status=pending 且锁定，但工作区文件已是 `accepted` v2（10/10 operator 报告）——以实物记录为准更新 AGENTS.md。

预估收益（4 片完整 run）：R1 约 −6~9 s/片（转移/抬升段路点 −40%），R2 约 −22 s（PARK 双向 178→45 步），R4 约 −3 s；总时长从约 180–200 s 降至约 130–140 s（−25~30%），旋转段 ~13 s/片为物理下限不承诺改善。

验收指标（Pi 上实测，先 `--dry-run` 再单片再四片）：
- 运动明细日志：单片总路点数 ≤ 45（现 ~66）；单片运动时间 ≤ 25 s（现 ~32 s）；无一路点 sent→converged > 2 s（HOLDING_SLOW 段除外，其 ≤ 1.5 s）。
- `park_to_home` / `home_to_park` 各 ≤ 6 s（现 15.1 s），且 HOME→PARK 末点正式误差 ≤ 4°、五轴 Torque_Enable 全 0。
- 单片抓放 10 次监督循环成功率 ≥ 0.8（与验收文件门禁一致），全程目视确认吸附/落位（无传感器，success=true 不代表抓放成功）。
- 回归：workspace/tests 的 test_park_sequence、test_motion_controller、test_pick_place_gate、test_supervisor_gates 全绿；`max_joint_step_deg=10°` 与 PICK/PLACE follow 禁用未被触碰（diff 审查确认只动参数层）。

## Risks / 残留不确定性
- transit 25 mm 在个别大重构转移（HOME→首航点型）仍可能触发加密重试（已有 4 轮翻倍兜底，属预期行为非故障）。
- PARK 4° 步进 + Acceleration=30 的冲击未在实机验证，需按验收指标实测后再冻结；若观察到下垂/过冲，退回 2–3° 而非 1°。
- wrist_roll 跨 ±180° 连续性（AGENTS.md 记录 piece_01/03 失败）是否已被 run_assembly 的等价角选择完全覆盖，本次只读审计无法闭环；相关失败现场仍要求运动锁流程，不得因提速改动而跳过。
- 5°→3° 末点容差回退可能重现 2026-08-01 的 4.147° 收敛失败（当时靠放宽到 5° 通过）；若重现，应查 wrist_roll 速度/收敛而非再次放宽容差。

## Need from main agent
无需决策即可执行 1–4 项（均为任务内参数修正）；第 3 项（PARK 步进）涉及实机验证节奏，建议与主代理确认是否同批部署。
