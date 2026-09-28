# 26. 机械臂运行中 USB 偶发断连数秒自动恢复 — 故障研究

> 数据来源：`epuzzle-pi` 2026-07-31 20:21 启动以来的 `journalctl` 内核与用户日志、
> `workspace/logs/supervisor/assembly-4pieces.jsonl`、evidence 目录与 git 提交记录。
> 所有时间均为 Pi 本地时间（UTC+8）。推断项均明确标注。

## 1. 结论摘要

"正在运行的机械臂突然断开几秒钟恢复"在本机有两个不同层级的证据：

1. **端口级信号完整性事件（本问题的主因）**
   内核签名：`usb usb3-port2: disabled by hub (EMI?), re-enabling...` → `USB disconnect`
   → 约 1 秒后同一设备（`1a86:55d3` / serial `5AB9067408`）自动重枚举。
   恢复机制：RP1 集线器自动 re-enable 端口 → CH343 重枚举 → udev 重建
   `/dev/epuzzle-arm -> ttyACM0` → 应用层重试续上。**无需人工干预，即"几秒钟恢复"。**
   触发条件与机械臂运行强相关：08:09–08:43 四片拼装期间共 5 次（含 1 次显式 EMI 消息），
   而空闲时段（20:21–22:18 近 2 小时、22:19–08:09 近 10 小时）零事件。

2. **主机控制器级故障（另一类，2026-07-31 19:18 已记录）**
   签名：`xHCI host not responding to stop endpoint command` → `assume dead` →
   `HC died; cleaning up`。整条 bus 3 消失，**不会自动恢复**，必须现场支撑机械臂、
   断电、重启 Pi、人工解除运动锁。本次启动 0 次此类事件。

两类故障不能混为一谈：前者端口自动恢复；后者必须走既有事故流程。

## 2. 本次启动（2026-07-31 20:21 起）全部机械臂 USB 事件时间线

| 时间 | 端口 | 内核事件 | 软件侧表现 | 归类 |
|---|---|---|---|---|
| 22:18:25 / 27 / 35, 22:19:33 | 3-2 | 4× `USB disconnect` + ~1s 重枚举（device 2→6） | 无（无运行任务） | 疑似人工拔插/触碰（间隔 2s/8s/58s 人尺度，无 EMI 消息，操作者 SSH 在场） |
| 08:09:08 | 3-2 | **`disabled by hub (EMI?), re-enabling...`** + 断开 + 0.26s 重枚举（6→7） | **正在运行的装配 Run 4 中断**（RELEASE 后 0.01s，无 VERIFY_PLACE/无 PARK 收尾），操作者 08:17 重启 Run 5 | EMI 端口禁用 |
| 08:18:09 | 3-2 | 断开 + 重枚举（7→8） | Run 5 运行中，随后多次 `PARKED->SCAN` 重启 | 端口级（无 EMI 字样） |
| 08:31:36 | 3-2 | 断开 + 重枚举（8→9） | 装配运行中 | 端口级 |
| 08:43:04 / 08:43:56 | 3-2 | 断开 + 重枚举（9→10） | 装配运行中 | 端口级 |
| ~08:5x–09:0x | — | 操作者将适配器物理换到 3-1 口 | — | 人为（换口排查） |
| 09:21:58, 09:29:10 | 3-1 | 断开 + 重枚举（11→12→13） | 09:27:53 `ROBOT_COMM_TIMEOUT`（重试逻辑 09:34 才部署） | 疑似人工拔插 + 其后果 |
| 10:25:38 | 3-1 | 断开 + 重枚举（13→14） | 10:32:56 / 10:34:17 / 10:36:54 三次 `ROBOT_COMM_TIMEOUT`（重试已部署但耗尽） | 端口级（全量断开，旧 fd 不可恢复） |
| 10:40 之后 | — | 无内核事件 | 11:19:53 `ROBOT_COMM_TIMEOUT` + `magnet:emergency_off` | 纯用户态/总线级（无 USB 断开） |

同一窗口的旁证：

- 08:09:49–08:14:54 相机口 1-2（`xhci-hcd.0`，另一个控制器）出现 `Cannot enable. Maybe the USB cable is bad?`、
  `attempt power cycle`、`device descriptor read/64, error -71`（EPROTO）——**两个独立控制器的端口在
  同一噪声窗口内同时出问题**，指向公共扰动源（电源/地/辐射 EMI），而非单一设备故障。
- 00:37 / 03:52 / 03:53 相机 1-2 干净断开+重枚举（无错误消息、机械臂空闲）——与装配无关，
  符合 MaixCAM 内部系统自重启的已知行为，另记。

## 3. 根因分析

### 3.1 直接机制（证据充分）

`disabled by hub (EMI?), re-enabling...` 是 Linux hub 驱动对端口信号完整性违规
（babble / 包损坏 / 端口状态异常）的标准处置：禁用端口 → 自动 re-enable → 设备重新枚举。
本次启动该类事件恰落在装配抓放运动段（08:09:08 与 RELEASE/place 段相差 0.01s 量级），
装配期间 5 次断开 vs 空闲期 0 次，相关性明确。

### 3.2 扰动源（推断，需实测确认）

按可能性排序：

1. **舵机总线电流 + 电磁铁继电器/线圈开关瞬态，经共享地/供电路径耦合进 USB 信号**
   - 适配器 TTL 侧必须与舵机总线共地；若舵机电源地回 Pi 地的唯一通路经过适配器细地线，
     伺服回程电流（STS3215 多轴同时动作可达数 A 级）直接抬升适配器局部地 → 信号完整性劣化。
   - 电磁铁继电器（GPIO17 低电平触发）与线圈的通断产生 dI/dt 尖峰；若继电器 VCC 取自 Pi 5V、
     或线圈续流保护缺失，瞬态会沿供电轨扩散。装配 cycle 中 08:09:08 事件发生在磁铁吸合/释放窗口。
   - Pi 本身无 throttling（`vcgencmd get_throttled=0x0`），说明扰动在适配器局部路径，不在 Pi 主轨。
2. **线缆/连接器机械接触**：22:18 窗口 4 次断开间隔为人尺度（2s/8s/58s）且无 EMI 消息，
   不能排除操作者拔插/触碰；装配期间操作者也在场。机械振动 + 松接头在运行中同样会间歇失联。
3. **RP1/xHCI 固件怪癖**：本次启动 0 条 xHCI 错误、消息干净 → 证据不足，不列为本次根因。

### 3.3 换口无效

操作者把适配器从 3-2 换到 3-1 后，09:29 / 10:25 仍断开 → 不是单口问题，支持公共扰动源。

## 4. 为什么"几秒钟恢复"以及软件现状

恢复链：hub 自动 re-enable（毫秒级）→ CH343 重枚举（~0.3–1 s）→ udev 重建 symlink →
应用下一次读写落在新节点（若应用能重开）或旧 fd 报错。

软件现状（`motion_controller.py`，09:34 部署，3s 间隔 ×2 重试）：

- 对 **URB 级/termios 瞬时错误**（设备未整体断开）有效——多数现场掉线属于此类，重试即恢复。
- 对 **全量断开+重枚举**无效：`cdc_acm` 断开即销毁 tty，旧 fd 永久 EIO/ENODEV，
  当前代码**没有任何 reopen/重连路径**（`runtime_adapter` 单次 `connect()`，装配工具每 run 一次）。
  证据：10:25:38 全量断开后 10:32–10:36 三次 `ROBOT_COMM_TIMEOUT` 重试耗尽；08:09–08:43
  五次断开各杀死一个装配 run（run 边界与断开时刻一一对应，且均无干净 PARK 收尾）。
- 11:19:53 的 `ROBOT_COMM_TIMEOUT`（VERIFY_PICK，随后 `magnet:emergency_off`）无伴随内核事件，
  属于第三类：纯总线/收敛问题（舵机负载抖动导致稳定样本达不到），与 USB 无关，需单独观察。

## 5. 建议

### 5.1 硬件（根治方向，需现场实测）

1. **接地审计**：舵机电源地 → Pi USB 地加粗直连（星形接地），禁止伺服回程电流走适配器细地线；
   用示波器抓适配器 VBUS/GND 在运动+磁铁动作时的跌落。
2. 适配器供电端加去耦（≥100 µF 电解 + 0.1 µF 陶瓷）；舵机电源输出端加大容量电解。
3. 电磁铁继电器确认续流二极管；继电器 VCC 不取自 Pi 5V 轨。
4. USB 线远离舵机/磁铁动力线；换屏蔽线、固定两端接头防振动。

### 5.2 软件（已实施，2026-08-01，详见下方第 7 节）

1. **新增设备重枚举检测 + 自动 reopen**：监听 udev/`/dev/epuzzle-arm` 节点变更（inode/mtime），
   全量断开后按安全流程重新 `connect_bumpless()` 再续跑，而不是在死 fd 上耗尽重试。
   注意与既有安全语义对齐：低高度/接触段断连不得自动续跑，必须回退安全高度后重连。
2. 保留 3s×2 重试（覆盖 URB 级瞬时错误）。
3. 自检/监控增加 `journalctl -k | grep -c "disabled by hub (EMI?)"` 计数，
   与现有 USB 故障计数并列展示，区分"端口级"与"控制器级"两类。
4. 记录区分三类失败（全量断开 / URB 级 / 收敛超时），避免把收敛超时误报为 USB 故障。

### 5.3 验证

- 按 5.1 整改后，跑一轮完整四片拼装（含磁铁），对照本启动基线：
  装配窗口 5 次/约 35 分钟 → 目标 0 次。
- 主动拔插测试：运行中断开 USB 2–3 s，验证新 reopen 路径在安全高度段自动恢复、
  在接触段 fail-closed。

## 6. 相关文件

- `workspace/calibration/evidence/2026-07-31_full_stack_deployment/kernel_usb_failure.log`（19:18 xHCI HC 死亡）
- `workspace/calibration/evidence/2026-07-31_usb_recovery_and_sweep/kernel_usb_recovery.log`
- `workspace/logs/supervisor/assembly-4pieces.jsonl`（08:00–11:19 装配全程，685 事件）
- `workspace/src/control/motion_controller.py`（3s×2 重试）
- git 4c4e095 / 2c2a89f（重试提交与现场描述）

## 7. 软件整改落地记录（2026-08-01）

### 已实现并测试

1. **插件轻量重连 `reconnect_lite()`**（`robot_plugin/.../so101_5d.py`）：
   打开串口 → 校验冻结标定 → 读 Present → 对齐 Goal=Present（读回校验 ≤1 LSB）→
   校验 Torque_Enable 五轴全 1（任一为 0 即 fail-closed 抛错）。
   **不断力矩、不重配电机、不碰标定寄存器**——与 connect_bumpless 的关键区别，
   保证空中重连零掉力矩。
2. **有界重连包装器 `ResilientJointRobot`**（`control/usb_resilience.py`）：
   实现 JointRobot 协议；每次调用最多 2 次轻量重连（预算耗尽抛最后一次异常）；
   旧连接尽力清理（可能已死，吞错）；`reconnect_epoch()` 供 deadline 宽限。
3. **控制器门禁与宽限**（`motion_controller.py`）：
   - `move()` 按 `goal.phase` 设置许可：**PICK/PLACE 接触段恒禁止自动重连**，
     其余相位按 `MotionConfig.allow_usb_reconnect`（默认 True，可全局关闭）；
   - `_wait_for_target` 检测到重连成功后重新给满一个收敛窗口（重连期间舵机仍按
     最后 Goal_Position 运动，需留出收敛时间）。
4. **接线**（`runtime_adapter.py`）：`connect()` 用包装器包住机器人，重连工厂走
   `reconnect_lite`；`attempt_park()` 清理路径显式开启重连许可，使 USB 掉线后
   仍能完成正式 PARK 与断力矩验收。

### 验收（2026-08-01）

- `tests/test_usb_resilience.py`：10 项（包装器 7 + 控制器集成 3）——非接触段掉线
  自动恢复、PICK/PLACE fail-closed 不重连、预算有界、deadline 宽限、旧连接清理。
- `tests/test_so101_5d_contract.py` 新增 2 项：`reconnect_lite` 不断力矩/不重配电机、
  扭矩未全开时 fail-closed。
- 宿主与 Pi 实机 venv 均通过；Pi 上插件从工作区源码路径导入（AGENTS.md 部署要求）。
- 全量回归与改动前基线一致：16 项宿主预先存在失败（fixture/合同/分支 mid-refactor），
  零新增回归。

### 待办

- **硬件整改**（5.1，示波器实测接地/去耦/续流二极管/线缆走线）。
- Pi 实机拔插验证：工具已就绪 `workspace/tools/run_verify_usb_reconnect.sh`
  （`--mode recovery` 非接触段自动恢复、`--mode contact-fail-closed` 接触段
  fail-closed + 清理 PARK 恢复；编排逻辑有单测），需现场操作者按提示执行拔插。
- 自检 `usb_port_faults` 已实施（见下方第 8 节）。

## 8. 监控落地记录（2026-08-01）

`/usr/local/sbin/epuzzle-selfcheck` 新增 `usb_port_faults` 检查项：

- `port_emi`：`disabled by hub (EMI?)` 端口级事件数（warning，不阻断）；
- `port_disconnect`：机械臂所在 bus 3 的 `usb 3-[12]: USB disconnect` 数（warning）；
- `controller_death`：xHCI `not responding` / `HC died` 数（error）。

Pi 实跑（2026-08-01）：`usb_port_faults: False warning port_emi=1; port_disconnect=12; controller_death=0`，
与本文时间线逐条吻合（EMI 1 次 + 断开 12 次 + 控制器级 0 次）。
overall `passed` 不受 port 级事件阻断（自动恢复机制在）；控制器级事件仍由运动锁机制阻断。
测试：`test_deployment_contracts.py` 新增 3 项（计数区分、warning 不阻断、干净启动 info）。
