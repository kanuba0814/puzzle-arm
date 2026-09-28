# 机械臂运动只读安全审计

## Review
- **Correct**：非接触步长和接触步长已隔离；轨迹仍受 10° 关节步长预检、超限加密和终点读回约束，见 `workspace/src/control/motion_controller.py:552-600`。
- **Blocker（高）**：启动时 `_polling_drive_to_joints()` 把任何“不在 PARK”都推断为断力矩下垂，并自动发送最多 120 秒的纠正动作，绕过 `JointSpaceParkMover.park_to_home()` 的 PARK mismatch 门禁。位置：`workspace/tools/run_assembly_4pieces.py:158-185,437-448`。下次硬件运行前应删除这条自动恢复路径，异常起姿必须 fail-closed，另走人工监督恢复。
- **Blocker（高）**：装配异常清理只有磁铁关闭和断开连接，没有调用已有的 `InspectionHardwareAdapter.attempt_park()`，见 `workspace/tools/run_assembly_4pieces.py:517-527` 对比 `workspace/src/integration/runtime_adapter.py:256-266`。两份事故证据均记录正式 PARK 未确认：`2026-08-01_card_123_attempt/robot-motion-tol5.lock.json:3-12`、`2026-08-01_card_123_retry/robot-motion.lock.json:3-12`。
- **Note（中）**：超时摘要不是最后实际读回。`MotionController.move()` 用上一个已收敛的 `final` 写入 timeout/error/result，且收敛读数硬编码为 `reads=1`，见 `workspace/src/control/motion_controller.py:483-504`。例如实际 wrist_roll 误差已达 118.716°，摘要却写 17.53°并返回旧关节值：`2026-08-01_card_123_attempt/motion-143627-tol5.jsonl:1822-1824`。参数验收应直接分析逐条 `readback`，不能依赖当前 timeout/result 摘要。

## 安全可做

1. **恢复 `transit_cartesian_step_mm=25.0`：批准。**
   - 当前 `pick_place_motion_config()` 未设置该值，因而非接触动作也使用 15 mm，见 `workspace/tools/run_assembly_4pieces.py:79-110`。
   - 25 mm 仅作用于非 `PICK/PLACE`；接触段继续使用 15 mm。规划仍取笛卡尔、姿态和关节跨度约束的最大步数，并逐点验证 `max_joint_step_deg=10`，超限则翻倍加密，见 `workspace/src/control/motion_controller.py:552-600`。
   - 它主要减少距离主导的路点；对大 yaw 或 IK 非线性路径未必有效。例如 evidence 曾出现 36 个 SAFE 路点，见 `2026-08-01_card_123_attempt/assembly-4pieces.jsonl:120`。不得为追求路点减少而放宽 10° 关节限制。

2. **删除每片末尾 `time.sleep(1.0)`：批准。**
   - sleep 位于 `grasp_fsm()` 完成、状态写盘之后：`workspace/tools/run_assembly_4pieces.py:510-515`。
   - 在返回前，Supervisor 已关闭磁铁、执行正式磁铁断电安定时间，并提升至 `place_safe`，见 `workspace/src/integration/supervisor.py:457-470`。
   - 日志直接显示约 1 秒空档：`verify_place_unsensed` 到下一片状态转换相差 1.000387 秒，见 `2026-08-01_card_123_attempt/assembly-4pieces.jsonl:27-28`。该 sleep 没有传感器验证或安全职责；四片可节省约 4 秒。

3. **把 30 秒 waypoint 超时收回到约 8 秒：批准作为当前证据支持的保守回退。**
   - 30 秒是每个路点的等待窗口，不是整条轨迹窗口，见 `workspace/src/control/motion_controller.py:686-733`；错误方向或卡死时会明显放大“卡住”时间。
   - 既有日志的异常路点均在约 8 秒被截断：`motion-143627-tol5.jsonl:1668,1823` 为 8.010 秒，`green_assembly/motion-151102.jsonl:1446,1601` 为 8.008 秒。两次 wrist_roll 误差均持续增大，延长到 30 秒不会促进收敛。
   - USB 成功重连已有独立 deadline 延期逻辑，见 `workspace/src/control/motion_controller.py:724-731`，不应靠所有正常路点统一放宽到 30 秒。

4. **PARK 1°→4°：仅批准为受控候选，不可直接宣称已验收。**
   - 现值位于 `workspace/src/integration/runtime_adapter.py:206-214`。按当前正式 PARK，PARK→upright 和 upright→HOME 分别由 83/95 个路点降至 21/24 个，总数约 178→45；仅 60 ms 最小保持时间的理论下限由 10.68 秒降至 2.7 秒。
   - PARK 执行仍逐点读回，并对最终 PARK 使用正式 4° 容差，见 `workspace/src/integration/park_sequence.py:147-187`。
   - 但现有 `follow_tolerance_deg=8` 配合 4° 新步长时，下一命令相对实际读回可能形成至多约 12° 的跟踪包络。因此更安全的实现应同步验证或收紧巡航容差（至少保证 follow tolerance + step 不超过冻结的关节步长风险预算），并增加滞后读回测试。
   - Evidence 只证明旧流程单向约 15.1 秒，并明确最终 PARK 修复后没有硬件复测：`2026-08-01_motion_rewrite_test/TEST_LOG.json:56-86`。因此不得把 4° 写成实机已验证结论。

## 不该做

1. **不应直接把 PARK 改为 4°、同时原样保留 8° 跟随容差并直接上机。**  
   这是代码级优化候选，不是当前 evidence 已覆盖的正式参数。

2. **不应简单全局恢复 `joint_tolerance_deg=3.0`。**
   - 当前 5°明确标记为诊断值：`workspace/tools/run_assembly_4pieces.py:84-90`，继续作为正式参数也不合适。
   - 但旧 3°试验中 SAFE 抬升稳定停在 4.147°，最终超时：`2026-08-01_card_123_attempt/motion-142846.jsonl:240-313`；盲目恢复会复现已知假失败。
   - 同时，5°试验会接受 3.992°等终点：`motion-143627-tol5.jsonl:55-67`。由于 `PICK/PLACE` 全部路点禁用宽松 follow tolerance，当前单一 5°也直接作用于接触段，见 `workspace/src/control/motion_controller.py:455-466`。
   - 正确方向是按 phase/终点拆分：PICK/PLACE 保持精确门槛，SAFE 对低压带载残差使用单独、经验证的容差；不要继续用一个全局诊断值兼顾两者。

3. **不应为消除“启动冗余动作”删除 PARK→upright→HOME 安全序列。**  
   `JointSpaceParkMover._move_via_upright()` 的两段路径是避碰结构，见 `workspace/src/integration/park_sequence.py:101-124`。应删除的是未经授权的启动自动纠偏，而不是安全 upright 过渡。

4. **不应把 30 秒等待当作 wrist_roll 环绕问题的修复。**  
   Evidence 中误差从约 90°持续增至 118°，是方向/连续角问题而非速度不足，见 `motion-143627-tol5.jsonl:1810-1824`；绿色复测也重复出现同类增长，见 `green_assembly/motion-151102.jsonl:1590-1602`。

## 证据与验证

- 只进行了本机只读检查；未 SSH、未同步、未访问 Pi、未实例化硬件对象、未发送任何动作。
- 静态计算确认当前正式 PARK 两段最大跨度为 82.505495°和 94.901099°，1°共 178 路点、4°共45路点。正式 PARK 数据见 `workspace/calibration/park_pose_single_arm_20260731.json:21-39`。
- 2026-08-01 实测显示旧 PARK/HOME 单向均为 15.1 秒；最终 7.648°误差暴露了宽 8°跟随容差问题，见 `TEST_LOG.json:56-80`。
- `git diff --check` 对指定文件通过。未运行 pytest，以遵守“不得修改任何文件”及禁止硬件访问要求。
- **建议验证顺序**：先做无硬件单测（4°路点数量、滞后读回、最终 PARK 4°门槛、25 mm 接触隔离、8秒超时），之后只能在未来获得明确现场授权、完成 Pi 门禁和急停监护时进行 PARK 往返实测。