# PICK/PLACE 功能快速参考

## 当前状态

✅ **已实现**: 完整的自动 PICK/PLACE 执行路径  
⏸️ **待测**: 需要真实硬件运行标定工具  

## 快速使用

### 1. 首次标定（需真实硬件）

```bash
# 在树莓派上运行
cd /home/epuzzle/puzzle
python workspace/tools/calibrate_pick_place_heights.py \
  --port /dev/epuzzle-arm \
  --test-cycles 10
```

**输出**: `workspace/calibration/pick_place_contact_height.json`

### 2. 验证门禁状态

```bash
# 检查 pick-place 能力
python -c "
from integration.competition_backend import SupervisorRunBackend
backend = SupervisorRunBackend()
cap = backend.capabilities()['pick-place']
print(f'状态: {cap.availability}')
print(f'原因: {cap.reason}')
"
```

### 3. 执行单片抓放

```bash
# CLI 方式（需要验收文件）
python workspace/tools/supervisor_run.py pick-place \
  --pieces workspace/runtime/red_piece_detection/pieces.json \
  --piece-index 0 \
  --place-table-mm 105,100 \
  --port /dev/epuzzle-arm
```

### 4. GUI 使用

- GUI Manual 页会显示 "Pick/Place" 按钮
- 验收通过后按钮自动解锁
- 点击后执行（当前需要 CLI 方式传递 `--place-table-mm` 参数）

## 门禁条件

PICK/PLACE 功能解锁需要满足：

1. ✅ 验收文件存在: `workspace/calibration/pick_place_contact_height.json`
2. ✅ 验收状态: `quality.status == "accepted"`
3. ✅ 成功率: `single_piece_success_rate >= 0.8`
4. ✅ 接触高度: `pick` 和 `place` 在 [0, 20] mm 安全区间
5. ✅ 磁铁验证: `pickup_tested` 和 `release_tested == true`
6. ✅ 安定时间: `on_settle_s` 和 `off_settle_s` 在 [0, 5] 秒范围
7. ✅ 机械臂在线: `/dev/epuzzle-arm` 存在
8. ✅ 无运动锁: `/etc/epuzzle/robot-motion.lock.json` 不存在或已解除

任一条件不满足 → `pick-place` 能力状态为 **LOCKED**

## 安全保护

### 自动保护

- 磁铁吸合后保持到显式释放；异常时由 `emergency_off()` 强制断电
- 运动全局超时: 120 秒
- 异常时强制 `emergency_off()` + 回 PARK

### 手动保护

- GUI 急停按钮随时可用
- CLI 按 Ctrl+C 触发急停
- 标定工具按 Ctrl+C 中止测试

## 故障排查

### 问题: pick-place 按钮禁用

**检查**:

```bash
cd /home/gxxl/nudec-re101
ls -lh workspace/calibration/pick_place_contact_height.json
cat workspace/calibration/pick_place_contact_height.json | jq '.quality.status, .single_piece_success_rate'
```

**常见原因**:

- 文件不存在 → 运行标定工具
- `status != "accepted"` → 重新标定或手动修改
- `success_rate < 0.8` → 重新标定，改进测试条件

### 问题: 标定工具连接失败

**检查**:

```bash
ls -l /dev/epuzzle-arm
# 应显示符号链接到实际串口

# 如果不存在，检查 USB 设备
ls -l /dev/serial/by-id/ | grep 1a86
```

### 问题: 运动安全锁

**检查**:

```bash
ls -l /etc/epuzzle/robot-motion.lock.json
cat /etc/epuzzle/robot-motion.lock.json
```

**解除** (需要现场操作者确认完成所有 `clear_requires` 步骤):

```bash
sudo rm /etc/epuzzle/robot-motion.lock.json
```

## 文件位置

### 代码

- 标定工具: `workspace/tools/calibrate_pick_place_heights.py`
- Supervisor 状态机: `workspace/src/integration/supervisor.py:grasp_fsm()`
- CLI 主流程: `workspace/tools/supervisor_run.py:run_pick_place()`
- 后端门禁: `workspace/src/integration/competition_backend.py:_capability()`

### 配置

- 验收文件: `workspace/calibration/pick_place_contact_height.json`
- 数据契约: `data_contracts/pick_place_contact_height.schema.json`
- 参数注册表: `configs/parameter_registry.yaml`

### 文档

- 实现总结: `docs/PICK_PLACE_SUMMARY.md`
- 详细设计: `docs/PICK_PLACE_IMPLEMENTATION.md`
- GUI 接口: `docs/GUI_INTERFACE.md`

## 测试命令

```bash
cd /home/gxxl/nudec-re101
export PYTHONPATH="workspace/src:workspace/src/robot_plugin/lerobot_robot_so101_5d:$PYTHONPATH"

# 单元测试
python -m pytest workspace/tests/test_supervisor_gates.py -v
python -m pytest workspace/tests/test_supervisor_run_cli.py -v

# 门禁测试
python /tmp/test_pick_place_capability.py  # 临时脚本，见下方"待补测试"

# 编译检查
python -m py_compile workspace/tools/calibrate_pick_place_heights.py
python -m py_compile workspace/src/integration/supervisor.py
python -m py_compile workspace/tools/supervisor_run.py
```

## 下一步行动

### P0 (阻塞功能使用)

1. **真实硬件运行标定工具** → 生成验收文件
2. **单片抓放测试** → 验证完整流程

### P1 (改进体验)

1. GUI 参数输入界面 → 支持直接在 GUI 输入 `place_table_mm`
2. 传感器验证 → 替代当前的 `sensor_verified: false` 假设
3. 多次重试逻辑 → 提高鲁棒性

### P2 (完整比赛流程)

1. 视觉检测 + 拼图求解 + 逐片搬运循环
2. 集成到 `competition-self-color` / `competition-white` / `competition-card`
3. 120 秒内完整拼图

## 相关规则

- 项目规则与硬约束: `AGENTS.md`
- 未实测的硬件参数一律写 `UNKNOWN`，不得以估计值代替实测值。
