# 树莓派 5 舵机部署 Runbook (阶段 1-8)

> 前置: 阶段 0(开发机准备)已完成, 项目已传输到 Pi。
> 图例: ⛔=必须人工操作/确认 · ✅=阶段验收标准
> 每个阶段通过后再做下一阶段; 出问题先查文末"故障速查"。

---

## 阶段 0.5: 项目传输到 Pi (开发机上执行一次)

```bash
# 开发机执行 (Pi 与开发机同一局域网):
rsync -av --exclude __pycache__ --exclude .git --exclude venv \
  /home/zhengjie/123/ pi@<PI_IP>:~/puzzle/
# 无网络条件时用 U 盘拷贝整个 123 目录到 Pi 的 ~/puzzle
```

⛔ 传输完成后**硬件保持断电**, 从阶段 1 开始。

---

## 阶段 1: Pi 环境就位 (无需舵机)

```bash
ssh pi@<PI_IP>          # 或在 Pi 上直接开终端
cd ~/puzzle
cat /etc/os-release | head -1; python3 --version; uname -m
bash tools/setup_pi.sh  # 离线安装 + 自检, 按提示处理 ⛔ 项
```

⛔ 可能被脚本要求执行(各需联网一次, 比赛前完成):
- `sudo apt install python3-venv` (venv 创建失败时)
- `sudo apt install python3-lgpio` (lgpio 缺失时)
- `sudo usermod -aG dialout $USER` → **重新登录** (不在 dialout 组时)

✅ 验收: 脚本末尾"自检通过 ✅", 且 4 项 import 全 ✓

**环境信息记录** (回填此处):
| 项目 | 值 |
|---|---|
| OS 版本 | |
| Python 版本 | |
| 舵机串口设备名 | (阶段2填) |

---

## 阶段 2: 硬件接线 ⛔

```
树莓派5 ──USB线──> [Waveshare总线舵机驱动板] ──舵机线──> [舵机 x1] (标定时只插1个)
                        ↑
                   舵机独立电源 (⚠️电压见下)
```

⛔ **第一步永远是确认电压**: 看舵机铭牌/电源适配器标签。
**本项目套件已确认**: 该版 SO-ARM101 主从臂共用同一型号 **5V 电源**(读数 4.9V 正常),
无 12V 供电 — 表现为运动偏慢、J2 带载下垂约 4°, 属预期, 软件侧补偿。
其他版本注意: Waveshare 套件 ST3215 多为 12V, Feetech 原厂有 7.4V/12V 两种。
**电压不符禁止通电** (过压烧舵机)。

⛔ 接线顺序:
1. 驱动板 USB → Pi (此时不开舵机电源)
2. `ls /dev/ttyUSB* /dev/ttyACM*` 确认出现设备 (记录到上表)
3. **只插 1 个舵机** (出厂 ID 全是 1, 多插会总线冲突)
4. 最后接舵机电源

```bash
source venv/bin/activate
python3 tools/servo_tool.py scan
```

✅ 验收: scan 发现恰好 1 个舵机 (通常 ID=1, 型号=777)

---

## 阶段 3: ID 标定 ⛔ (逐个插拔, 约 10 分钟)

对 i = 1..5 重复 (机械臂关节定义: **1=底座, 2=肩, 3=肘, 4=腕俯仰, 5=腕旋转**):

1. ⛔ 舵机电源断电 → 插上第 i 个舵机 → 上电
2. `python3 tools/servo_tool.py setid 1 <i>`
3. ⛔ 看到 `[OK]` 后**立即在舵机外壳贴标签写 ID 号**
4. ⛔ 断电, 换下⼀个

全部标完后: ⛔ 5 个舵机**按关节位置插好**(底座舵机=ID1...腕旋转=ID5), 然后:

```bash
python3 tools/servo_tool.py scan
```

✅ 验收: 恰好报告 `{1,2,3,4,5}`, 无重复 ID 警告

---

## 阶段 4: 通信验证 (只读, 不会动)

```bash
python3 tools/servo_tool.py ping 1 2 3 4 5
python3 tools/servo_tool.py read
```

✅ 验收:
- 5/5 通信正常, 型号全为 **777** (若是怪值如 58881=777字节交换, 报给我修协议端序)
- 电压读数与电源标称一致 (±10%), 位置全在 0-4095

---

## 阶段 5: 中位与方向标定 ⛔ (首次真实运动!)

> 全程 ⛔ 手扶机械臂, 手放电源开关旁, 异常立刻断电。

**方式A (推荐): 标定向导一键完成** —— 按中文提示掰关节约 5 分钟,
自动完成 范围记录/映射识别/方向判定/零位计算:
```bash
cd ~/puzzle && source venv/bin/activate
python3 tools/calibrate_arm.py
```
向导输出 `calibration_result.json` + 汇总表 → ⛔ **把汇总发给 Claude**
(计划中的停止点B), 我远程回填 `config.py` 的 SERVO_SIGNS/OFFSETS/LIMITS。

向导内部流程 (参考 LeRobot so_follower.calibrate, 全程手掰无自转):
1. 摆基准零位按回车 → 记录零偏 (≈LeRobot set_half_turn_homings, 但零偏存软件不写EPROM)
2. 每关节向"正方向"掰一下 → 判定 SERVO_SIGNS
3. 所有关节依次掰到整个行程极限, 实时 MIN|POS|MAX 表格, 回车结束 (≈LeRobot record_ranges_of_motion, 时间不限)
4. (可选) 小幅自转验证 ID↔关节映射 (钳制在已测范围内)

**方式B (手动逐步, 向导不可用时):**
```bash
python3 tools/servo_tool.py torque off     # 释放力矩, 手摆基准零位
python3 tools/servo_tool.py read           # 记录各舵机值 = OFFSETS来源
python3 tools/servo_tool.py center         # ⚠️慢速全部到中位, 手扶!
python3 tools/servo_tool.py nudge 1 +100   # ⚠️逐关节微动核对方向
python3 tools/servo_tool.py torque on      # 恢复力矩
```

✅ 验收: 回填后 `center` 姿态=基准姿态; 每关节 `nudge +100` 方向与软件定义一致

---

## 阶段 6: 运动联调 (安全高度)

```bash
python3 arm_controller.py
# 交互命令: h(回home) -> pos(读位置) -> m 150 0 80 -> m 120 80 80 -> m 120 -80 80
```

⛔ 首次运行速度已在安全范围; 仍保持手在电源旁。
⛔ 用尺量末端实际到达位置 vs 命令目标。

✅ 验收: `go_home` 不超时; 3 个测试点末端误差 ≤10mm (连杆参数未实测前的容忍度)

---

## 阶段 7: 电磁铁 ⛔ (电路接线)

```
Pi GPIO24 ──> MOSFET栅极(G)     [逻辑电平型: IRLZ44N / AO3400模块]
电磁铁电源(+) ──> 电磁铁一端
电磁铁另一端 ──> MOSFET漏极(D)
MOSFET源极(S) ──> GND ──> Pi GND  (必须共地!)
续流二极管: 反接并联在电磁铁两端 (负极接电源+侧) [1N4007/SS34]
消磁电路: 按你的硬件设计接入
```

⛔ 接线完成断电检查一遍后上电, 然后:

```bash
python3 magnet_controller.py     # 通断自测: 3次循环, 听吸合声/测漏极电压
# 实物测试: 电磁铁贴住 0.3mm 镀锌铁片碎片
python3 -c "from magnet_controller import MagnetController as M; import time; m=M(); m.on(); time.sleep(1); m.off()"
```

✅ 验收: 吸-放循环 10 次, 10/10 可靠; 释放无残磁粘连
(有粘连 → 调大 config.py `MAGNET_RELEASE_MS` 或检查消磁电路)

---

## 阶段 8: 端到端集成

```bash
# 1. 空跑拾放流程 (无碎片)
python3 -c "
from arm_controller import ArmController
from motion_planner import MotionPlanner
p = MotionPlanner(ArmController())
p.pick_and_place((150, 30), 0, (105, 100), 0)
"

# 2. 放真实碎片, 单片抓取验证
# 3. 全流程: 视觉求解 -> 机械臂执行
python3 solve_paths.py --camera      # 视觉端生成 paths.json
python3 main.py task2                # 主状态机全流程
```

⛔ 人工实测并回填 config.py: **L1-L4 连杆长**、底座相对 A4 纸位置 `ARM_BASE_X/Y`

✅ 验收: 单片"吸取→搬运→释放"成功; 2 分钟时限内 4 片全流程演练完成

---

## 故障速查

| 现象 | 排查 |
|---|---|
| scan 找不到设备 | `lsusb` 看驱动板; `ls /dev/ttyUSB*`; 换 USB 口; `groups` 看 dialout (改了要重新登录) |
| scan 全是超时 | 舵机电源没接; 波特率被改过 → `scan --baud 1000000,115200` |
| ping 型号是怪值 | 协议端序错误 → 告诉我修 `PacketHandler` 参数 |
| setid 后不生效 | 断电重试; 仍不行 → 检查 0x37 EPROM 锁 |
| 舵机乱抖/疯转 | **立刻断电**; 多半是方向/零偏未标定 → 回阶段5 |
| 舵机发热 | 负载过大或顶到机械限位; `read` 看负载列, 检查姿态 |
| 电磁铁不吸 | 万用表测 MOSFET 栅极有无 3.3V; 共地; 电磁铁电源电压 |
| 释放后粘连 | 加大 `MAGNET_RELEASE_MS`; 检查消磁电路/续流二极管极性 |
| import lgpio 失败 | ⛔ `sudo apt install python3-lgpio` (需联网一次) |
