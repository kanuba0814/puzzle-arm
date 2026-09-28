# E 题单臂控制重构上下文

> 最后更新：2026-07-31。本文件用于跨上下文续接；恢复任务时先读本文件、根目录 `AGENTS.md`、`docs/23_单臂运动控制重构.md`。

## 1. 用户当前意图

用户要求：通过 SSH 检查树莓派上现有实现，分析奇怪的机械臂末端 XYZ 偏差，并按诊断建议开始 rebuild。

硬件事实（用户确认）：

- 当前只有一台机械臂；
- 机械臂由原 SO-101 follower（从臂）改装；
- 五个舵机，ID 1–5；
- 固定字段顺序：
  1. `shoulder_pan.pos`
  2. `shoulder_lift.pos`
  3. `elbow_flex.pos`
  4. `wrist_flex.pos`
  5. `wrist_roll.pos`
- 没有 leader/follower 双臂运行关系；
- 没有 ID 6 `gripper.pos`；
- 电磁铁独立于机械臂动作，冻结为 `/dev/gpiochip0` BCM GPIO17，低电平触发；默认高电平关闭，软件不再设置连续通电上限，必须显式断电；
- `docs/14_三人分工与交接.md` 被用户明确判定为废纸，不再作为架构依据，已标记 DEPRECATED。

## 2. 树莓派连接和路径

当前可用 SSH：

```bash
ssh -i deployment/raspberry_pi/output/epuzzle-image-access-ed25519 \
  epuzzle@192.168.1.110
```

设备：`epuzzle-pi`，Wi-Fi `192.168.1.110`。

树莓派上存在两套代码：

1. `/home/epuzzle/puzzle`：当天快速迭代的 legacy 实现，Git 工作区大量未提交修改；它使用 `/home/epuzzle/puzzle/venv`，没有 LeRobot。
2. `/opt/epuzzle/workspace`：镜像内的规范基线；使用 `/opt/epuzzle/venv`，已安装：
   - LeRobot `0.6.0`
   - `lerobot_robot_so101_5d==0.2.0`（本次重构后已升级）
   - 五轴插件字段正确、ID 1–5、无 `gripper.pos`

当前 `/dev/ttyACM0` 为机械臂串口：

- USB：`1a86:55d3`
- Serial：`5AB9067408`
- 稳定路径：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00`
- udev 规则已按 VID/PID/serial 冻结，`/dev/epuzzle-arm -> ttyACM0` 已在 Pi 验证。

当前没有 GUI、Supervisor、`main.py` 或其他控制进程运行。禁止在重构阶段自动下发舵机目标或开启电磁铁。

## 3. 已完成诊断及证据

从树莓派复制的只读分析快照位于：

```text
/tmp/epuzzle-rpi-live
```

规范仓库内已固化证据：

```text
workspace/calibration/evidence/2026-07-31_single_arm_contacts/
├── fit_A-H.json
├── holdout_I-L.json
├── legacy_arm_base_rejected.json
└── README.md
```

SHA256 已写入证据 README。

### 3.1 独立触点重放

旧模型以 A–H 拟合；I/J/K/L 未参与该拟合，可作为独立重放点。旧模型结果：

| 点 | XY 误差 | Z 误差 |
| --- | ---: | ---: |
| I | 10.44 mm | +2.08 mm |
| J | 6.72 mm | -5.52 mm |
| K | 6.34 mm | +4.63 mm |
| L | 11.42 mm | -5.51 mm |

汇总：

- XY RMS：9.011 mm
- XY max：11.421 mm
- Z RMS：4.650 mm
- 三次离线重放结果完全一致
- 旧 A–H 拟合自身记录：XY RMS 6.96 mm、max 9.02 mm

因此旧标定固定为 `rejected`，已生成：

```text
workspace/calibration/arm_kinematics_legacy_rejected_20260731.json
```

### 3.2 偏差不是常量

以旧基座为原点分解径向误差：

- 源区 A–D：约 +2.9 ～ +8.4 mm；
- 装配区 E–H：约 -2.3 ～ -8.5 mm；
- 独立点 I：+10.4 mm；
- 独立点 L：-9.0 mm。

旧代码只根据 E 点加入 `ARM_TARGET_RADIAL_COMP_MM=3.125`。误差随区域翻转，因此单一全局补偿必然在不同位置反向过补偿。

### 3.3 旧运动学结构问题

legacy `/home/epuzzle/puzzle/config.py` 使用：

- 物理 `L1=116.6 mm`，但实际 FK 使用 `L1_EFFECTIVE=104.1573 mm`；
- `KINEMATIC_JOINT_OFFSETS_DEG=(0, 0.9248, 3.5343, 28.6032)`，总俯仰修正 33.0623°；
- `L4=98.1 mm` 单直杆；
- 上述值来自 A–H 约束拟合，不是独立物理测量闭环。

代码声称依据的官方 `so101_new_calib.urdf` 中腕滚转关节原点包含 `xyz=(0,-61.1,18.1) mm`，不能简单把所有 TCP 偏置压成一根共线杆。当前自定义电磁吸头尺寸/装配方向仍需正式实测和版本化记录。

### 3.4 旧执行层确定性问题

- 当前 `out/paths.json` 的 10 个抓取/放置端点，在 `APPROACH_Z=80 mm` 时只有 2 个 IK 可达；
- `motion_planner.pick_and_place()` 多处忽略 `move_to_xyz()` / `move_line()` 失败，仍继续吸合或释放；
- `move_line()` 最终目标发送后没有实际位置收敛检查；
- production `wait_for_stop()` 只看 Moving 位，不验证实际 raw 是否到达目标；
- GUI XYZ 点动静默把腕姿态重置为 -90°；5 mm 点动可导致 J4 一次变化约 5–10°；
- GUI 忙碌或 IK 失败时仍可能报告“步进已启动”；
- 驻车 raw 跨越软件角度周界，旧 `servo_to_deg()` 可把 J2 解成约 201°，不能直接用于普通 FK/jog；
- 旧 GUI 只检查标定文件存在就显示“已标定”，不检查质量。

### 3.5 已排除/尚未确认

已观察：

- 相机单应残差 RMS 0.194 mm、max 0.49 mm，不是主要误差源；
- 舵机只读 8 次 raw 完全一致；
- Pi `vcgencmd get_throttled=0x0`；
- 未见机械臂串口 USB 断连证据。

尚未确认：

- 舵机供电电压、电流、带载变形；
- 回差、重复到达精度；
- 自定义电磁吸头完整 TCP 三维偏置；
- 新 LeRobot 五轴 calibration；
- M/N/O/P 独立触点（legacy 文件只有目标，`points=[]`）。

## 4. 重构决策

### 4.1 外部 seam

沿用规范接口：

```text
MotionGoal(robot_base_mm) -> MotionResult
```

该接口也定义于 `docs/10_接口与数据契约.md`，不依赖已废弃的人员分工文档。

### 4.2 模块边界

- `control`：FK/IK、全路径预检、软限位、安全执行、MotionResult；
- LeRobot 五轴插件：唯一真实 JointRobot adapter，单 follower，ID 1–5；
- Supervisor：唯一控制全局状态和电磁铁；
- ACT：不得绕过安全过滤；
- vision/puzzle 不直接驱动机械臂。

### 4.3 运动学模型

新模型不再使用“L4 单杆 + 单点径向补偿”：

- 几何参数全部从版本化 JSON 标定加载；
- 分开表达 `tool_forward_mm` 与 `tool_normal_mm`；
- 支持肩部 radial/lateral offset；
- 明确 LeRobot joint_deg 到物理关节角的 `directions` 和 `zero_offsets_deg`；
- MotionGoal `pitch_deg` 保持，不在点动时静默重置；
- 工具 yaw 暂按 `pan + wrist_roll` 建模，最终需实物姿态验收。

### 4.4 标定门禁

真实运行只能通过：

```python
MotionController.from_calibration_file(...)
```

门槛：

- `quality.status == "accepted"`；
- 至少 3 个拟合点、4 个未参与拟合的独立点；
- holdout XY RMS <= 5 mm；
- holdout XY max <= 8 mm；
- holdout |Z|max <= 3 mm；
- 五轴字段和来源必须精确匹配；
- 证据 SHA256 必须存在。

旧标定即使手工把状态改为 accepted，指标仍超限，会继续被拒绝。

## 5. 已修改文件

### 文档与证据

- 新增 `CONTEXT.md`（本文件）
- 新增 `docs/23_单臂运动控制重构.md`
- 更新 `docs/14_三人分工与交接.md`：标记 DEPRECATED
- 新增触点证据目录及 SHA256 README
- 新增 `workspace/calibration/evidence/2026-07-31_single_arm_motor/`：保存同机 LeRobot 历史标定包、精确五轴 follower 候选、SHA256 和 J1 实测交叉验证
- 新增 `workspace/calibration/evidence/2026-07-31_tcp_measurement/`：保存 J5 安装面到磁铁工作面的卡尺照片、单次 `40.17 mm` 读数、同轴定性观察和 SHA256
- 新增 `workspace/calibration/evidence/2026-07-31_single_arm_contacts_v2/`：保存新 8 fit + 4 holdout 手工触点、完整会话日志、手绘治具计划、拟合报告和 SHA256
- 新增 accepted `workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json` 与 `contact_sheet_to_robot_base_20260731.json`
- 新增 rejected legacy calibration JSON

### 控制核心

- 新增 `workspace/src/control/kinematics.py`
  - `JOINT_FEATURES`
  - `JointVector`
  - `ToolPose`
  - `ArmGeometry`
  - `PlanarArmKinematics.forward/inverse`
- 新增 `workspace/src/control/calibration.py`
  - `ArmCalibration`
  - `CalibrationQuality`
  - JSON 加载、五轴来源检查、质量门禁
- 新增 `workspace/src/control/motion_controller.py`
  - `MotionGoal`
  - `MotionConfig`
  - `MotionResult`
  - `MotionErrorCode`
  - `MotionController`
  - 全轨迹在首次发送前预检
  - 每个路径点实际五轴连续稳定读回
  - timeout / safety / IK 显式失败
  - 不包含任何电磁铁调用
- 新增 `workspace/src/control/motor_calibration.py`
  - 由 J1–J4 两个机械端点 raw 自动计算中点、Homing_Offset 和居中范围
  - 不依赖目测中位或外部刻度；端点顺序无关，小跨度/越界显式拒绝
- 更新 `workspace/src/control/__init__.py` 导出新接口

### 数据契约与部署配置

- 新增 `data_contracts/motion_result.schema.json`
- 新增 `data_contracts/arm_kinematic_calibration.schema.json`
- 更新 `configs/parameter_registry.yaml`：记录单臂来源、USB VID/PID/serial、稳定端口；供电、软限位等未实测值继续为 UNKNOWN
- 更新 `deployment/raspberry_pi/overlay/etc/udev/rules.d/70-epuzzle-devices.rules`：按实机 `1a86:55d3` / `5AB9067408` 生成 `/dev/epuzzle-arm`
- 新增并调整 `workspace/tools/calibrate_single_arm.py`：默认只预检；实际流程必须交互 TTY、`--execute` 和双重确认；J1–J4 记录两个机械端点并按 12-bit 环形编码器长弧计算 raw 中位，J5 单独记录电缆安全零向；全程不发送 `Goal_Position`
- 新增 `workspace/tools/install_single_arm_motor_calibration.py`：严格验证同机来源为精确 J1–J5、交互确认后只写 Homing_Offset/范围并回读，不发送位置目标
- 新增 `workspace/tools/preflight_single_arm_motion.py`：只读检查串口、精确五轴 LeRobot calibration 和运动学质量门禁，不打开串口
- 更新 `docs/10_接口与数据契约.md`、`docs/12_测试与验收计划.md`、control/plugin README

### 测试

- 新增 `workspace/tests/test_single_arm_kinematics.py`
- 新增 `workspace/tests/test_motion_controller.py`
- 新增 `workspace/tests/test_arm_calibration.py`
- 新增 `workspace/tests/test_control_data_contracts.py`
- 扩展 `workspace/tests/test_so101_5d_contract.py`：拒绝遗留六轴 LeRobot calibration
- 新增 `workspace/tests/test_centered_motor_calibration.py`：覆盖无刻度端点居中、端点逆序、小跨度和 12-bit 越界

## 6. 当前测试里程碑

TDD red 已观察：

- 新模块不存在时 3 个测试模块 ImportError；
- 关节步长超限曾被错误归类为 `IK_NO_SOLUTION`；
- 高度安全配置加入前，`MotionConfig` 不接受高度门槛字段。

以上已分别修复。当前本地验证：

```text
控制/契约/磁铁测试：Ran 27 tests，OK
ruff check：clean
compileall：OK
JSON Schema / calibration JSON：parse OK
LSP：16 个既有相关 Python 文件 0 diagnostics
lens_diagnostics mode=all：No issues across 35 files
新增端点居中单测：Ran 5 tests，OK；包含跨 `4095 -> 0` 的长弧回归；相关 4 文件 LSP 0 diagnostics
```

树莓派 `/opt/epuzzle` 离线验证：

```text
初次部署：Ran 31 tests in 0.010s，OK
同机电机标定导入工具及环绕修复部署后：Ran 36 tests in 0.009s，OK
compileall：OK（PYTHONPYCACHEPREFIX=/tmp，源码保持 root 只读）
lerobot_robot_so101_5d：0.2.0
五轴 feature contract：PASS
/usr/local/sbin/epuzzle-selfcheck --software-only：READY
标定工具默认 dry-run：PASS，未打开串口
旧 rejected 运动学 preflight：返回 2 / NOT_READY（预期）
pip check：No broken requirements
udev test：Added SYMLINK 'epuzzle-arm'
```

覆盖行为包括：显式五轴、拒绝 gripper/遗留六轴 calibration、FK/IK、分离 tool forward/normal、数值标量兼容、不可达全路径零发送、关节步长安全错误码、phase 高度门禁、实际读回连续稳定、超时失败、标定质量门禁、JSON Schema 五轴约束、GPIO17 既有回归。

## 7. 下一步（竞赛最小闭环）

1. **先恢复机械臂 USB 安全状态**：现场人工支撑机械臂并切断舵机电源；之后才能恢复 USB/重启 Pi，并先做只读枚举与正式 PARK 状态检查。当前禁止自动重绑 USB 或重试运动。
2. USB/PARK 安全确认后，再执行一次磁铁关闭的最短巡检；必须得到全部中心实际读回、正式 PARK 和断力矩证据。
3. 在正式页面选一片碎片，用人工旁站流程确定 PICK/PLACE 接触高度；当前抓取按钮因高度 `UNKNOWN` 安全锁定。
4. 接触高度冻结后，只补磁铁吸取→抬升→旋转→放回单片闭环，然后直接转完整拼图。
5. 完整自碰模型、TCP 重复量测和 ACT 后置。

## 8. 安全和回退

- 2026-07-31 已开始真实硬件：先完成 2 秒 bumpless torque hold，再完成 `park → high_home → P上方60 mm → Q上方60 mm → P → high_home → park` 往返；磁铁始终关闭，最终回到停放位并断力矩。
- 2026-07-31 首次启动官方 LeRobot 通用标定时，程序在“目测移动到中位”提示处释放了力矩；用户指出无刻度无法可靠找中位后立即 `Ctrl+C` 中止。该流程尚未执行 `set_half_turn_homings`、未生成标定文件；机械臂已由用户安全放稳。
- 已决定不用目测中位：J1–J4 可逐轴采两个机械端点并按环形编码器长弧计算 Homing_Offset；J5 只定义电缆安全零向。
- 用户提供 `/home/gxxl/nudec-ro101-E/calibration/lerobot_calibration_bundle_20260731.zip` 并确认其中 follower 是当前同一台实物。原包 SHA256 `52c40e9cd1c6a366a512c35ae27d7e5733ca39df18b930da134dfa64319c948e`，包内全部哈希通过。
- J1 在 Homing_Offset=0 下实测端点 `2751/1372`；历史 follower 候选反推端点 `2762/1387`，仅差 `11/15` counts。两端跨编码器周界，真实长弧约 `2717` counts，与候选 `2721` 一致。因此此前将 `1379` 误判为跨度是算法错误，不是用户未到端点；已增加环绕回归测试。
- 同机候选已通过交互工具导入 Pi：`~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d/epuzzle-single-arm.json`，SHA256 `89ea5079eb9bb35012866d736d0438d419ee57c201e51e11a61530e3e6440ca8`；舵机寄存器回读逐字段一致。
- 独立只读验证未发送 `Goal_Position`。首次 J2=716 比范围下限723低7 counts；用户在断力矩下移离止挡后复读 J2=761，最终五轴全部在导入范围内。机械臂已安全放稳，力矩仍关闭。
- 使用 rejected 运动学标定执行 preflight 仍为 `NOT_READY` / exit 2（预期）；电机标定没有解除 TCP/运动学门禁。
- TCP 新增单次卡尺实测：J5 旋转输出/安装平面到磁铁工作面 `40.17 mm`；断力矩手转 J5 时磁铁基本绕自身圆心旋转。照片和 SHA256 已归档。结合历史 URDF 的 forward `61.1 mm` / normal `18.1 mm`，触点拟合初值为 `tool_forward=101.27 mm`、`tool_normal=18.1 mm`；因重复性及 URDF 原件 SHA256 仍未知，只能标为 provisional。
- 新触点采集已完成：A/B/D/E/F/G/H/I 共 8 个 fit，M/N/O/P 共 4 个独立 holdout；每点 11 次 raw，最终文件 SHA256 `11bf8221865f96242963d58e5af839eb78249ed481c7e0ca829abb5cabb433d4`。首次 F 与 E raw 完全相同，已当场丢弃并重采，错误样本未进入最终 JSON。
- 采点前重跑 GPIO17 fail-safe OFF，Supervisor inactive；全过程断力矩、无 `Goal_Position`、无磁铁激活。采后机械臂已安全放稳。
- 固定几何模型仅用 8 个 fit 点拟合 J2–J4 零偏及纸面刚体变换，M/N/O/P 不参与拟合、模型选择或软限位生成。holdout：XY RMS `2.480912 mm`、XY max `3.406456 mm`、|Z|max `2.458668 mm`，均通过 5/8/3 mm 门槛。
- 用户明确要求“标定结果直接接受”。当前 accepted v2 标定 SHA256 `c207bae180e187b6c639cc848320195dafce1f468920a0f2eba7f00c621eab38`；纸面→基座变换 SHA256 `151d447b7a1ff144295aabcbb11e60ade643cbad908b5b65f06b4d8f06f8b271`。生产 FK 独立重放与报告指标一致。
- 为避免竞赛中过度工程，v2 软限位改为同机机械端点内缩后的实用范围：J1 ±110°、J2 ±100°、J3 ±90°、J4 ±95°、J5 `[-145,-20]°`；它们不是完整自碰包络。
- 首次真实最小往返 PASS：最大跟随误差 `4.341246°`，各阶段最终最大误差 `2.637363°`，停放位返回最大误差 `1.054945°`。production `MotionController` 的 P/Q/HOME 所有 waypoint 也已成功并由实际五轴读回确认。
- P 点完成 60/20/10/20/60 mm、每点停留 5 秒；操作者确认 10 mm 过低或碰纸。随后 20 mm 单独停留 10 秒，操作者确认“高度合适，作为安全低位”。因此 Z_SAFE=60 mm、PREGRASP=20 mm，PICK 接触高度仍为 `UNKNOWN`。
- 12 点 20 mm 快速巡航全部通过，但宽松 5°/单样本到位门槛造成负载下提前判定；操作者观察到各点向机械臂方向缩 10–20 mm。D 点精确保持后显著改善，最终采用径向远离机械臂 3 mm、命令 pitch `-87.5°` 的终点补偿；两次实际 pitch 均为 `-90.03°`，操作者确认位置和垂直度正常。
- 快速巡航和精确抓放必须使用不同到位策略；宽松巡航门槛不得作为吸放成功条件。硬件证据在 `workspace/calibration/evidence/2026-07-31_first_hardware_motion/`。
- 悬空阶段之间断力矩会因重力下垂；成功往返在一个受监控序列内保持力矩，并回到有支撑的停放位后断力矩。
- 当前姿态已冻结为正式 PARK：`workspace/calibration/park_pose_single_arm_20260731.json`，5 次只读均值，SHA256 `1c14bd574e0a1976d8ae5e7b303aecdcd4ec3a08aeefefc2531f82eaf00276d9`。PARK 是有支撑的独立恢复姿态，可位于普通笛卡尔软限位之外。
- GUI A4 标定已归档为 `table_homography_gui_20260731.npy`，1280×720 image_px→table_mm，RMS `0.257541 mm`、max `1.029862 mm`、57 角点，SHA256 `d82e6514f9943d63f34649b1b6b228c1fda5a35be17ae6463d6b029fca9f3694`。
- 黑底红色场景检测到 4 个碎片。选中 `red-0` 重复到达两次：五轴最大差 `0.263736°`，FK XY 重复差 `1.104762 mm`；4 个中心全部扫过并回正式 PARK。证据在 `workspace/calibration/evidence/2026-07-31_red_piece_center_sweep/`。
- 当前完整运行标定总表：`workspace/calibration/runtime_calibration_manifest_20260731.json`。正式 7 英寸页面：`workspace/apps/pick_place_page.py`，按钮为刷新检测、重复两次、巡检全部、20 mm 预抓取、选片、目标角、抓取旋转放置、PARK、急停。页面已在 Pi tty2 启动，只持有 `/dev/fb0`，不常驻占用串口/GPIO。
- 自动抓取/旋转/放置仍安全锁定，因为 PICK/PLACE 接触高度是 `UNKNOWN`；页面不会启用磁铁或伪报成功。
- 诊断时 legacy `ArmController` 打印初始化 GPIO24，说明 legacy 与冻结 GPIO17 冲突；新控制核心不得复用 legacy 电磁铁路径。
- 本地 Todo：`TODO-d45a43a8`，状态 `in_progress`。
- 部署前备份已完成：`/home/epuzzle/backups/control-rebuild-pre-20260731T094018+0800.tar.gz`，SHA256 `41ce2435ff4438ffe34f1a91c15b7e8d55bfb199bd357e085de882f1b7d1a82f`。
- 新核心已部署到 `/opt/epuzzle`；正式抓取/旋转/放置页面已在 tty2 运行。页面的检测、重复、巡检、20 mm 预抓取、PARK 和急停已接 canonical 工具；`epuzzle-supervisor.service` 仍为 inactive/disabled，自动磁吸抓放仍锁定。
- `/home/epuzzle/puzzle` legacy dirty Git 工作区未覆盖、未 reset、未 clean。
- `/opt/epuzzle/venv` 为安装 0.2.0 wheel 启用了 ensurepip 25.1.1；离线 wheelhouse/requirements/SHA256 已同步 0.2.0，可回退到备份中的 0.1.0。
- 2026-07-31 全栈已审查部署至 `/opt/epuzzle`；部署前备份 `/home/epuzzle/backups/full-stack-reviewed-pre-20260731T191057+0800.tar.gz`，SHA256 `4c8f73946545e7a87864ab39f4fcfe0ea9f2d9da6dbbe6a0d5011e66feff810a`。Pi software selfcheck PASS；GUI 三页 1024×600 和 fb0 像素验证通过；正式 GUI active/enabled 且不持有串口/GPIO。
- 用户确认继电器控制链路已就绪：`/dev/gpiochip0`、BCM GPIO17、低电平触发；部署 feature gate 已设 `magnet_enabled=true`。这不等于吸取/释放、电气/热或耐久验收通过；`automatic_pick_place_enabled=false`、PICK/PLACE 高度仍 `UNKNOWN`，本次所有检测/巡检结果均 `magnet_enabled=false`。
- 新鲜相机检测 PASS，观察到 4 个红片。随后唯一一次 `sweep-shortest` 在连接阶段失败：内核 `xhci-hcd.1` 报 `xHCI host controller not responding` / `HC died`，机械臂 `cdc_acm` 写入返回 `-19 ENODEV`，整个 bus 3 消失。route 为空，没有中心被报告完成；磁铁未启用。
- USB 故障后机械臂 `/dev/ttyACM0` 和 `/dev/epuzzle-arm` 均缺失，软件无法确认最终 PARK 或断力矩；`throttled=0x0`，磁铁安全服务 active，串口/GPIO 无占用，GUI 已恢复诊断运行。完整证据在 `workspace/calibration/evidence/2026-07-31_full_stack_deployment/`。
- 已安装 root 管理的持久运动事故锁 `/etc/epuzzle/robot-motion.lock.json`（root:root 0644）。即使 USB 节点自行恢复，GUI 的巡检/PREGRASP/PARK/普通断力矩仍禁用，CLI 在 adapter 构造前返回 `SUPERVISOR_LOCKED`；检测、Shadow 和急停仍可用。Pi 锁门禁测试 36/36 通过；解除必须完成锁文件 `clear_requires` 并由现场操作者明确批准。
- 两级自检语义已冻结：`--software-only` 表示依赖/五轴接口可加载；完整 `--json` 还必须检查运动事故锁与硬件节点，禁止把软件可运行误写成整机 READY。
- 现场操作者随后重启 Pi、恢复舵机供电并手动删除 `/etc/epuzzle/robot-motion.lock.json`，视为明确批准解除该次 xHCI 事故锁；程序没有自动删除或绕过锁。只读检查确认五轴校准寄存器一致、最终在正式 PARK 4° 容差内且断力矩。
- 恢复后的首次巡检因 deadline 时钟域混用而 fail-closed：Supervisor 用 `time.monotonic_ns()` 生成 deadline，而 MotionController 默认用 `time.time_ns()` 比较。统一为 monotonic 并添加回归后，`sweep-shortest` PASS，路线 `red-1 -> red-2 -> red-0 -> red-3`，四个中心均有实际五轴读回；最终 PARK 最大误差 `3.516483°`、Torque_Enable 五轴全 0、magnet=false、当前启动 USB 错误 0。证据在 `workspace/calibration/evidence/2026-07-31_usb_recovery_and_sweep/`。
- 相机 identity 已冻结为 Sipeed MaixCAM `359f:2120` / serial `c9c28f4513595968` / capture index 0，稳定链接 `/dev/epuzzle-camera -> /dev/video0`。Supervisor/检测工具默认使用该稳定路径；独立检测再次得到 4 片、1280×720。完整 selfcheck 现为 `passed=true`，37 个与 PICK/PLACE、磁吸带载、generic/texture 等相关的 `UNKNOWN` 以 warning 保留并继续由各功能门禁阻塞。
