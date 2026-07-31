# Pi 上手动测试 PICK/PLACE 完整指南

## 前置条件

- Pi 已刷最新镜像并首启完成
- 开发机有线口设置为 `10.99.0.1/24`
- SSH 私钥位于 `deployment/raspberry_pi/output/epuzzle-image-access-ed25519`

## 1. 同步文件到 Pi

在**开发机**执行：

```bash
cd /home/gxxl/nudec-re101

# 设置变量
PI_HOST="epuzzle@10.99.0.2"
PI_KEY="deployment/raspberry_pi/output/epuzzle-image-access-ed25519"

# 同步验收文件和测试数据
rsync -avz \
  workspace/calibration/pick_place_contact_height.json \
  workspace/runtime/red_piece_detection/pieces.json \
  -e "ssh -i $PI_KEY" "$PI_HOST:/home/epuzzle/puzzle/workspace/"

# 同步代码更新（仅更新的文件）
rsync -avz \
  workspace/src/integration/supervisor.py \
  workspace/src/integration/competition_backend.py \
  workspace/tools/supervisor_run.py \
  -e "ssh -i $PI_KEY" "$PI_HOST:/home/epuzzle/puzzle/workspace/"
```

## 2. 连接到 Pi

```bash
ssh -i deployment/raspberry_pi/output/epuzzle-image-access-ed25519 epuzzle@10.99.0.2
cd /home/epuzzle/puzzle
```

## 3. 检查硬件状态

```bash
# 检查设备节点
ls -l /dev/epuzzle-arm /dev/epuzzle-camera

# 检查运动安全锁（应该不存在）
ls -l /etc/epuzzle/robot-motion.lock.json 2>&1

# 如果锁存在，查看原因
cat /etc/epuzzle/robot-motion.lock.json

# 按照 clear_requires 完成步骤后，root 解锁：
# sudo rm /etc/epuzzle/robot-motion.lock.json
```

## 4. 验证验收文件

```bash
python3 << 'PYEOF'
import json
from pathlib import Path

path = Path("workspace/calibration/pick_place_contact_height.json")
if not path.exists():
    print("✗ 验收文件缺失")
    exit(1)

data = json.load(path.open())
print(f"质量状态: {data['quality']['status']}")
print(f"成功率: {data['single_piece_success_rate']:.1%}")
print(f"PICK: {data['contact_heights_mm']['pick']} mm")
print(f"PLACE: {data['contact_heights_mm']['place']} mm")
print(f"磁铁安定: on={data['magnet_verification']['on_settle_s']}s, off={data['magnet_verification']['off_settle_s']}s")

if data['quality']['status'] != 'accepted':
    print("✗ 验收未通过")
    exit(1)
if data['single_piece_success_rate'] < 0.8:
    print(f"✗ 成功率 {data['single_piece_success_rate']:.1%} < 80%")
    exit(1)

print("✓ 验收文件有效")
PYEOF
```

## 5. 检查后端能力

```bash
python3 << 'PYEOF'
import sys
sys.path.insert(0, 'workspace/src')
sys.path.insert(0, 'workspace/src/robot_plugin/lerobot_robot_so101_5d')

from integration.competition_backend import SupervisorRunBackend

backend = SupervisorRunBackend()
cap = backend.capabilities()['pick-place']

print(f"状态: {cap.availability.value}")
print(f"原因: {cap.reason}")
print(f"需要机械臂: {cap.requires_robot}")

if cap.availability.value == "LOCKED":
    print("\n⚠️ pick-place 仍然锁定，检查:")
    print("  1. 验收文件是否存在且 status='accepted'")
    print("  2. /dev/epuzzle-arm 是否存在")
    print("  3. /etc/epuzzle/robot-motion.lock.json 是否存在")
    exit(1)

print("\n✓ pick-place 已解锁，可以执行")
PYEOF
```

## 6. 执行真实 PICK/PLACE

**⚠️ 安全提示**：

- 确认工作区无障碍物
- 确认碎片在测试位置 (105, 148.5) mm
- 随时可按 Ctrl+C 急停
- 全程监督机械臂运动
- 先手扰在机械臂旁，准备随时断舵机电源

`pick-place` 会先刷新相机检测；若检测不到碎片则回退到 `--pieces` 文件内容。
首次测试建议把碎片实际放在文件标的位置，使两者一致。

```bash
# 执行单片抓放
python3 workspace/tools/supervisor_run.py pick-place \
  --pieces workspace/runtime/red_piece_detection/pieces.json \
  --piece-index 0 \
  --place-table-mm 105,100 \
  --port /dev/epuzzle-arm
```

**预期行为**：

1. 加载验收文件（z_pick=4mm, z_place=8mm）
2. 刷新相机检测
3. 连接继电器和机械臂
4. 扫描视觉批次
5. 全路径 IK 预检
6. PARK → HOME
7. **执行 8 步磁吸序列**：
   - SAFE(60mm) → PREGRASP(20mm) → PICK(4mm)
   - 磁铁通电（即时响应，无等待）
   - SAFE(60mm) → SAFE(60mm，横移到目标）
   - PREPLACE(20mm) → PLACE(8mm)
   - 磁铁断电（即时释放，无等待）
   - SAFE(60mm)
8. HOME → PARK
9. 输出成功 JSON

## 7. 查看结果

```bash
# 成功示例
cat workspace/logs/supervisor/pick-place.jsonl | tail -20

# 检查事件日志中的磁铁调用
grep "magnet:" workspace/logs/supervisor/pick-place.jsonl
```

## 8. 故障排查

### 问题：ROBOT_COMM_TIMEOUT

**症状**：`机械臂设备离线: /dev/epuzzle-arm`

**检查**：

```bash
ls -l /dev/epuzzle-arm
# 应该是到 /dev/ttyUSB0 或类似串口的符号链接

# 检查 USB 设备
lsusb | grep 1a86
ls -l /dev/serial/by-id/ | grep 1a86
```

**修复**：

- 检查 USB 连接
- 重新插拔 USB 转串口
- 检查 udev 规则

### 问题：SUPERVISOR_LOCKED (运动安全锁)

**症状**：`机械臂运动安全锁: ...`

**修复**：

```bash
# 1. 查看锁定原因
cat /etc/epuzzle/robot-motion.lock.json

# 2. 按照 clear_requires 完成所有步骤
#    例如：支撑机械臂、切断舵机电源、确认 PARK 状态

# 3. root 解锁
sudo rm /etc/epuzzle/robot-motion.lock.json
```

### 问题：IK_NO_SOLUTION

**症状**：全路径预检失败

**可能原因**：

- 目标点超出工作区
- 关节限位冲突
- 运动学标定过期

**修复**：

- 检查 `--place-table-mm` 是否在 A4 范围内
- 检查运动学标定文件时间戳
- 尝试更近的目标点

### 问题：碎片掉落或粘连

**症状**：吸附后碎片掉落，或释放后粘连

**调整**：
如果需要安定时间（当前 0.0s），修改验收文件：

```bash
python3 << 'PYEOF'
import json
path = "workspace/calibration/pick_place_contact_height.json"
data = json.load(open(path))
data["magnet_verification"]["on_settle_s"] = 0.5  # 吸附后等 0.5s
data["magnet_verification"]["off_settle_s"] = 0.5  # 释放后等 0.5s
json.dump(data, open(path, 'w'), ensure_ascii=False, indent=2)
PYEOF

# 重新测试
python3 workspace/tools/supervisor_run.py pick-place ...
```

## 9. 验收通过标准

- [ ] 成功加载验收文件（z_pick=4mm, z_place=8mm, settle=0s）
- [ ] 后端能力状态为 READY
- [ ] 全路径 IK 预检通过
- [ ] 8 步序列完整执行
- [ ] 磁铁在 PICK 后通电，PLACE 后断电
- [ ] 最终回到 PARK
- [ ] 无异常、无掉落、无碰撞

## 10. 清理

测试完成后：

```bash
# 回开发机
exit

# （可选）备份日志
rsync -avz -e "ssh -i deployment/raspberry_pi/output/epuzzle-image-access-ed25519" \
  epuzzle@10.99.0.2:/home/epuzzle/puzzle/workspace/logs/supervisor/ \
  workspace/logs/pi-tests/
```

---

**实施状态**: 代码已就绪，等待真实硬件验证  
**验收文件**: workspace/calibration/pick_place_contact_height.json（z_pick=4mm, z_place=8mm, on_settle=0s, off_settle=0s）  
**回归测试**: 77 个测试通过 ✓
