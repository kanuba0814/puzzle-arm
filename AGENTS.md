# AGENTS.md — E题项目工作规则

## 目标与优先级

本仓库用于 2026 年电赛 E 题。优先级固定为：

1. 赛题硬约束与安全；
2. OpenCV + IK + 电磁吸头确定性基线；
3. 自备 50 分功能；
4. 现场白色 20 分功能；
5. 扑克牌 30 分功能；
6. ACT 局部增强。

任何模型或实验功能不得破坏已通过的确定性基线。

## 数据真实性

- 题面事实只以 `official_docs/competition/E题_拼图装置.pdf` 为准。
- LeRobot 接口只以 v0.6.0 标签、官方 v0.6.0 文档和 `sources.lock.json` 中的固定来源为准。
- 所有测试数据必须来自日志，不得把估计值写成实测值。
- 不能确认的硬件型号、电压、尺寸和性能必须写为 `UNKNOWN`，等待实物标签或实测确认。
- “工程目标”“保守假设”“官方要求”必须明确区分。

## 架构边界

- `vision`：只输出观察，不直接驱动机械臂。
- `puzzle`：只输出放置方案，不发送动作。
- `control`：负责 IK、轨迹、安全过滤与执行。
- `policy`：ACT 仅输出局部五维动作建议，必须经过安全过滤。
- `integration`：Supervisor 是唯一可切换全局状态的模块。
- 电磁铁由 Supervisor/MagnetController 控制，不进入 ACT action。

## 固定五轴顺序

1. `shoulder_pan.pos`
2. `shoulder_lift.pos`
3. `elbow_flex.pos`
4. `wrist_flex.pos`
5. `wrist_roll.pos`

禁止依赖字典隐式顺序。数据集、配置、日志和 checkpoint 必须同时保存显式字段名。

## 坐标与单位

任何跨模块数据必须携带坐标系和单位。允许的主要坐标系：

- `image_px`
- `rectified_px`
- `table_mm`
- `robot_base_mm`
- `joint_deg` 或已冻结的 LeRobot normalization mode

禁止使用未注明含义的 `x/y/z/theta`。

## 开发与变更

- LeRobot 固定 `v0.6.0`；不以 `main` 为比赛依赖。
- 五轴支持通过独立插件实现，不直接改系统安装目录。
- 新功能先写验收条件，再实现。
- 每次跨模块接口变更必须同步更新 `data_contracts/`、`docs/10_接口与数据契约.md` 和测试。
- ACT 必须可通过配置完全关闭。
- 所有低高度动作必须可超时、可急停、可退回安全高度。

## 树莓派远程调试固定流程

### 主从边界

- 本机 `/home/gxxl/nudec-re101` 是唯一源码真源，只负责编辑、纯计算、静态检查、单元测试、同步、发命令和回收日志；树莓派 `/opt/epuzzle` 是硬件执行副本。
- **任何可能访问机械臂串口、舵机、相机、GPIO、继电器或电磁铁的代码都只能在树莓派执行。** 本机禁止实例化 `SO101Follower5D`、`InspectionHardwareAdapter`、`MagnetController` 等真实硬件对象，禁止用本机设备节点冒充 Pi；本机只能运行明确不接硬件的 mock、纯算法或 schema 测试。
- Pi 上只使用规范树 `/opt/epuzzle` 和 `/opt/epuzzle/venv`；禁止在 legacy `/home/epuzzle/puzzle` 运行，禁止把临时代码丢到 `/home/epuzzle/*.py` 后直接控制机械臂。
- 本机源码始终覆盖 Pi 副本：不得在 Pi 上用 `sed`、编辑器或 heredoc 热修生产代码。若现场必须修复，先改本机、测试，再按原相对路径同步；Pi 生成的日志、标定和实测证据反向拉回仓库后才能引用。
- 同一时刻只允许一个主会话下发硬件动作。子代理可做只读分析和代码审查，但不得登录 Pi、同步文件、启停服务或执行机械臂/磁铁命令。

### 固定连接与运行环境

每个 Pi 调试会话先在仓库根目录使用以下固定变量；当前现场固定通过局域网 SSH 目标 `epuzzle@192.168.1.110` 连接，不使用 mDNS 或其他地址。

```bash
cd /home/gxxl/nudec-re101
PI_TARGET="epuzzle@192.168.1.110"
PI_KEY="$PWD/deployment/raspberry_pi/output/epuzzle-image-access-ed25519"
PI_SSH="ssh -i $PI_KEY -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3 -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=/tmp/epuzzle-ssh-%C"
PI_ROOT="/opt/epuzzle"
PI_PYTHON="/opt/epuzzle/venv/bin/python"
PI_PYTHONPATH="/opt/epuzzle/workspace/src/robot_plugin/lerobot_robot_so101_5d:/opt/epuzzle/workspace/src"
```

- 连通性只用 `$PI_SSH "$PI_TARGET" 'hostname && test -d /opt/epuzzle/workspace'` 检查；机械臂控制、磁铁和相机相关脚本一律通过该 SSH 目标在 Pi 上执行。
- 所有项目命令都从 `/opt/epuzzle` 执行，因为 Supervisor 的 `workspace/...` 相对路径以这里为基准；不得从 `/opt/epuzzle/workspace` 启动依赖该约定的 CLI。
- 固定解释器为 `/opt/epuzzle/venv/bin/python`，固定 `PYTHONPATH` 顺序如上，确保工作区五轴插件早于 site-packages。每次部署插件后必须打印并断言实际 `lerobot_robot_so101_5d.__file__` 位于 `/opt/epuzzle/workspace/src/robot_plugin/`。

### 最短迭代闭环

1. **本机修改和无硬件验证**：只改任务范围文件，先运行相关单测、compile/schema 检查。不要为了部署而处理或覆盖仓库中其他未提交改动。
2. **精确同步**：用户已明确授权部署当前改动后，从仓库根使用 `rsync -azR --itemize-changes` 只同步本次涉及的显式文件，保持相对路径落到 `$PI_TARGET:/opt/epuzzle/`；统一排除 `__pycache__/`、`*.pyc`、`.pytest_cache/`、`.ruff_cache/` 和 `*.egg-info/`。禁止全仓覆盖、禁止 `--delete`、禁止仅凭 `git status` 把他人的 dirty 文件一起推送。

   ```bash
   # Pi 规范树的上级目录可能由 root 持有；以下参数避免仅因目录属组/时间戳返回 code 23。
   rsync -azR --no-owner --no-group --omit-dir-times --itemize-changes \
     --exclude='__pycache__/' --exclude='*.pyc' \
     --exclude='.pytest_cache/' --exclude='.ruff_cache/' --exclude='*.egg-info/' \
     -e "$PI_SSH" \
     workspace/src/<module>/<file>.py workspace/tests/<test_file>.py \
     "$PI_TARGET:/opt/epuzzle/"
   ```

3. **Pi 软件验证**：在 Pi 清理本次改动模块的旧 `__pycache__`，从 `/opt/epuzzle` 用固定解释器和 `PYTHONPATH` 跑相关测试、import 路径断言及 `/usr/local/sbin/epuzzle-selfcheck --software-only`。涉及 arm64 wheel、设备库或实际 import 的结论只认 Pi 结果。
4. **硬件前只读门禁**：检查 `/etc/epuzzle/robot-motion.lock.json`、`/dev/epuzzle-arm`、`/dev/epuzzle-camera`、完整 `epuzzle-selfcheck --json`、`fuser`/运行进程、最近 USB/xHCI 日志以及磁铁安全状态。锁文件存在、设备身份不符、端口被占用或出现 `HC died` 时立即停止；不得自动删锁、杀进程、重绑 USB 或重启 Pi。
5. **实际动作**：只有用户在当前任务中明确授权具体动作且现场操作者已准备好急停/支撑后才执行。使用一个前台 SSH 命令和 Pi 端 `timeout --signal=INT --kill-after=10s`；需要交互确认的工具加 `ssh -tt`。超时必须包在 Pi 端程序外，不能只在本机包 `timeout ssh ...`，以免 SSH 退出后远端动作仍在运行。真实运动输出不得经 `head`、`tail`、`grep` 截断，也不得后台运行。
6. **收尾验收**：同时检查命令退出码、结构化日志、最终正式 PARK、五轴 `Torque_Enable`、磁铁 OFF 和事故锁；不能把“SSH 返回 0”写成抓放成功。随后把日志/结果精确拉回对应 `workspace/calibration/evidence/` 或 `docs/experiments/`，记录 Pi 时间、命令、版本/哈希和人工观察。

硬件命令统一采用此形态（具体工具和超时按任务替换）：

```bash
$PI_SSH -tt "$PI_TARGET" \
  "cd $PI_ROOT && env PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$PI_PYTHONPATH \
   timeout --signal=INT --kill-after=10s 300s $PI_PYTHON workspace/tools/<tool>.py <args>"
```

- SSH 中断或本机超时后，**先假设 Pi 上的进程和机械臂状态未知**；先只读检查进程、运行状态文件、日志、设备节点、PARK/力矩/磁铁状态，禁止盲目重发动作。
- `usb ... disabled by hub (EMI?), re-enabling` 后约 1 秒重枚举属于端口级事件，只能走已有有界重连和 phase 门禁；`xHCI ... HC died` 属于控制器级事故，严格执行人工支撑、断舵机电源和持久运动锁流程。
- 为了快，优先复用 SSH ControlMaster、精确 rsync、目标测试和既有规范 CLI；不得以远端热修、跳过门禁、隐藏完整输出或复用 legacy 脚本换速度。

## 树莓派镜像构建经验

- arm64 wheelhouse 必须在 aarch64 Python 环境解析条件依赖；宿主 x86 pip 可能错误拉取 PyTorch CUDA 包。
- 不要在宿主机对 arm64 缓存 venv 使用 `pip --python <venv>`：该 venv 的 Python 软链接会解析到宿主解释器，可能把包错误写入 `lib/python<宿主版本>`。纯 Python 插件也应先从无 `__pycache__` 的干净源目录构建 wheel，再在 aarch64 环境安装，或校验后解包到明确的 CPython 目标目录。
- wheel 可下载不等于能运行；必须在目标 rootfs 系统库或 Pi 5 实机中逐项 import OpenCV、torch、torchvision、LeRobot 和插件。
- `pkexec` 启动的 root 子进程可能脱离外层任务；写盘完成以实际 `dd` 退出和 raw SHA256 回读一致为准，不以外层返回码或时长判断。
- 64GB 卡使用较小交付镜像并保留 Raspberry Pi OS `resize` 首启扩容；写盘后只读回读整个镜像覆盖范围校验 SHA256。
- NetworkManager 有线维护 profile 不得设置负 `ipv4.dns-priority`；负值会排除其他连接的 DNS，使 Wi-Fi 已有默认路由却无法解析域名。
- 当前 Raspberry Pi OS 基础镜像的 `/etc/cloud/cloud.cfg` 引用了未安装的 `netplan_nm_patch`；构建时移除该 final module，避免首启 `cloud-init status` 显示 degraded。
- 长时间运行的 shell 构建脚本启动后不要修改同一脚本；bash 会继续读取后半段，运行中编辑可能造成只在收尾阶段出现的解析错误。先停止或等待构建结束，再修改并从稳定版本重跑。
- 电磁铁控制已冻结为 `/dev/gpiochip0` 的 BCM GPIO17，经低电平触发继电器控制；3.3 V 高电平默认关闭，软件不再设置 10 秒自动断电，必须由调用方显式关闭。`config.txt` 使用 `gpio=17=op,dh,pu` 尽早拉高，但官方说明上电到该设置生效仍可能延迟数秒，因此继电器输入外部硬件上拉到 3.3 V 才是最早期默认关闭的基础。线圈电压、电流、温升和实测连续通电能力仍为 `UNKNOWN`。
- pygame/SDL2 `kmsdrm` GUI 从 SSH 启动时必须通过 `openvt` 绑定活动 VT；内层不要使用启用了 `use_pty` 的 `sudo -u`，应由 root 侧 `setpriv` 降权，否则 Python 的标准输入仍指向 SSH pts，可能只显示 DRM 硬件鼠标层。
- QDTECH MPI700 触摸屏应具有 `ID_INPUT_TOUCHSCREEN=1`；若 udev 规则晚于设备枚举，需对对应 event 节点执行 `udevadm trigger --action=change`。GUI 不依赖 SDL 隐式触摸鼠标合成，应显式把 `FINGER*` 归一化坐标映射到屏幕像素。
- 当前 SDL 2.32 KMSDRM 在 Pi 5 上表现为主 framebuffer 正常翻页但物理屏全黑，仅硬件光标可见；修改 pygame 软件 Surface 的 alpha 不会修复 SDL 后续生成的 GBM buffer。7 英寸正式 GUI 使用 `SDL_VIDEODRIVER=dummy` 离屏绘图，转 RGB565 后直接写 `/dev/fb0`，并直接读取 QDTECH MPI700 evdev；验收需同时检查 fb0 非黑像素/颜色数和实屏触摸。
- 7 英寸 GUI 手动页的 `_tab=None` 表示关节/末端页签共享控件；过滤时必须保留 `None`，否则“← 返回”等公共按钮会消失。该语义必须有回归测试。
- QDTECH MPI700 实屏逻辑分辨率固定为 1024×600；Pi 5 KMS 曾因 EDID 选择 fb0 1024×768，使屏幕布局与 evdev y 轴映射错位。GUI 必须以 1024×600 作为逻辑画布并在 fb0 输出层显式缩放；镜像 kernel cmdline 固定 `video=HDMI-A-2:1024x600@60`，该启动参数变更须重启后只读核验。触摸必须同时支持 `BTN_TOUCH` 与 protocol-B `ABS_MT_TRACKING_ID`，节点未就绪/热拔不得令 GUI 退出。
- Pi 运行树是精简部署副本，不含源码 `deployment/` 树和某些 `workspace/runtime` 测试 fixture；`/opt/epuzzle/venv` 也不保证安装 `pytest`/`pygame`。GUI 服务和离屏截图须使用 `/usr/bin/python3`，并显式将 venv site-packages 放入 `PYTHONPATH`；Pi 只运行可脱离源码树的 GUI/触摸测试、编译和 software-only 自检，完整部署契约测试只在本机源码树运行。
- `manual-read-state` 是排障只读通道：断力矩后的重力下垂可能使关节落到冻结软限位外；此时必须仍返回五轴 raw/degree 与 `Torque_Enable`，仅省略不可安全计算的 TCP 并明确警告，严禁借由读取动作修正姿态或使能力矩。
- Pi 的 `test_supervisor_run_cli.py` 若断言“缺设备 → ROBOT_COMM_TIMEOUT”，必须在子进程环境显式指向临时不存在的 `EPUZZLE_ROBOT_MOTION_LOCK`；现场真实持久锁优先返回 `SUPERVISOR_LOCKED`，不能让设备离线 fixture 假设覆盖该安全门禁。
- 机械臂 commissioning 的悬空阶段之间不得断力矩；否则关节会因重力下垂，下一阶段的起点即失效。首次使能必须先在断力矩状态把 `Goal_Position` 对齐当前 `Present_Position`，受监控序列结束前先返回有支撑的停放位，再断力矩。
- 断力矩触点拟合与舵机编码器收敛不能单独保证带力矩时的 TCP 离面间隙；本机 `calibration_sheet_mm` 目标 10 mm 经现场观察过低或碰纸，20 mm 经 10 秒停留确认可作为竞赛 PREGRASP/观察高度。PICK 接触高度必须单独标定。
- 快速巡航与精确终点使用不同到位门槛：5°/单样本只适合扫点，会使负载下 TCP 提前 10–20 mm 判到位，禁止用于抓放。当前 D 点精确终点实测补偿为径向远离机械臂 3 mm、命令 pitch `-87.5°`，负载后实际 pitch `-90.03°`；推广到全工作区前至少抽查近/中/远点。
- 2026-08-01 Pi 只读寄存器复核：ID1–5 均为 STS3215、固件 3.10、Torque_Enable 全 0、`Present_Voltage` 原始值 48–49（约 4.8–4.9 V）；当前运行寄存器为 P=24、Acceleration=30。官方 LeRobot `SOFollower.configure()` 使用 P=16 以降低发抖，已将运行时 P 回退为 16；5 V 供电下重力带载能力不足仍是硬件风险，不能用软件保证不下垂，电机铭牌和电源规格必须现场确认。
- 2026-08-01 运动明细显示：宽跟随门槛下首个 `Present_Position` 读回在约 2 ms 内被接受，导致中间 `Goal_Position` 路点以总线最快速度连发；`Goal_Velocity` 也不应每个路点重复写。轨迹执行必须配置 profile 最小保持时间并将速度寄存器按 profile 缓存。
- 2026-08-01 实机回归：P=16、Acceleration=30、50 ms 轮询和路点 dwell 下 PARK→HOME 成功；HOME→正式 PARK 末点误差 7.648°，旧实现因使用 8° follow 容差提前放行，已修复为最后 PARK 路点直接使用正式 4° 容差。随后断力矩/重新连接期间 wrist_flex 从约 79.3°下垂到 70.77°，P=16 在 4.8–4.9 V 下无法独立恢复，仍需供电/负载或有支撑的上电流程。经现场操作者明确批准，已用 5 次稳定只读样本将当前姿态登记为新的正式 PARK，并同步 manifest；这不是新的电机/运动学标定。实测记录见 `workspace/calibration/evidence/2026-08-01_motion_rewrite_test/TEST_LOG.json`。
- 2026-08-01 绿色碎片单片实机抓放：相机新鲜帧偶发 `Corrupt JPEG`/`VISION_MERGED_BLOBS`，已现场检查帧并显式使用已检查快照回放一次；Supervisor 完成 `piece-1 -> (105,100) table_mm` 的完整动作/磁铁时序，最终扭矩全 0、最大 PARK 误差 2.637°、无吸附/落位传感器，现场目视确认仍是验收必需条件。默认 `pick-place` 仍强制刷新相机。
- 2026-08-01 现场请求将三档速度提高 25%（EMPTY_NORMAL=250、EMPTY_SLOW=150、HOLDING_SLOW=100），并将 PICK/PLACE 调为 6/12 mm；新高度文件已置 `quality.status=pending`、成功率归零，未完成新高度复测前，`pick-place`/`run_assembly_4pieces.py` 必须保持锁定。`competition-card` 一键后端仍未接入，不能把命令存在写成扑克牌完整排列已完成。
- `MotionGoal.timestamp_ns`、`deadline_ns` 和控制器 deadline 比较必须处于同一 `time.monotonic_ns()` 时钟域；禁止 Supervisor 用 monotonic 生成 deadline、控制器却用 `time.time_ns()` 比较。PARK 途中宽跟随门槛也不得代替最终正式 PARK 容差验收。
- 相机运行时必须使用冻结的 `/dev/epuzzle-camera` 稳定链接（Sipeed MaixCAM `359f:2120`、serial `c9c28f4513595968`、capture index 0），不得依赖可漂移的数字 index；更换相机必须重新冻结 udev identity。
- 若 Pi 内核报告 `xHCI host controller not responding` / `HC died` 并使机械臂串口节点消失，异常清理无法再确认断力矩或正式 PARK。禁止自动重绑 USB、重启 Pi 或重试运动；必须先由现场操作者支撑机械臂并切断舵机电源，再恢复 USB，随后先做只读枚举和 PARK 状态检查。
- 运行中断连数秒自动恢复与 HC 死亡是两类故障：`usb usb3-port2: disabled by hub (EMI?), re-enabling...` → `USB disconnect` → ~1 s 自动重枚举是**端口级信号完整性事件**（本机实测集中在机械臂运动+磁铁动作窗口，空闲期零事件，换口无效），自动恢复无需人工；`HC died` 是控制器级，不自动恢复。重试（3s×2）只对 URB 级瞬时错误有效，`cdc_acm` 全量断开后旧 fd 永久 EIO，装配 run 会整段被杀。**已实施**：插件 `reconnect_lite()`（重开串口+对齐 Goal=Present+校验 Torque_Enable 全 1，不断力矩/不重配电机）+ `ResilientJointRobot` 有界重连（每次调用最多 2 次）+ 控制器按 phase 门禁（PICK/PLACE 恒禁止，其余按 `allow_usb_reconnect`）+ 收敛 deadline 宽限 + `attempt_park` 清理路径可重连。详见 `docs/26_USB断连故障研究.md`；硬件整改（接地审计/去耦/续流二极管/线缆）与 Pi 实机拔插验证仍待办。
- Pi 的工作区五轴插件源码路径必须排在 venv site-packages 前；包版本号相同不能证明安装内容一致。软件自检必须同时断言实际导入路径来自 `/opt/epuzzle/workspace/src/robot_plugin/`，且 `SO101Follower5D.connect_bumpless` 存在。
- 真实硬件事件导致 PARK/断力矩无法确认时，使用 root 管理的 `/etc/epuzzle/robot-motion.lock.json` 持久锁定运动；设备节点重新出现、GUI 重启或 Pi 重启都不得自动解锁。只有现场完成锁文件 `clear_requires` 全部步骤并明确批准后，才可人工解除。
- 笛卡尔插值步数不能只按 XYZ 距离和 pitch/yaw 分段：HOME→首航点这类笛卡尔位移小、关节大幅重构的动作（实测 pan −49.9°、elbow +35.6°、wrist −46.9°）会使单步关节跳变超 `max_joint_step_deg`。终点 IK 的关节位移只是下界，IK 非线性使中间段单步跳变大于均值，`ceil(span/limit)` 甚至加 2% 余量都仍可能失败（实测 10.14°/10.15°/10.137° 反复卡在 10° 门槛外）。正确做法是超限即翻倍加密重试（上限 4 轮），绝不放宽 `max_joint_step_deg`；本机 piece-0 最终以 6 点通过。
- `sudo` 执行的 CLI 中 `Path.home()` 解析为 `/root`，会使 `~/.cache/huggingface/lerobot/calibration/...` 默认标定目录落空并报出误导性的“标定寄存器不一致”。必须显式传 `--lerobot-calibration <目录>`（要目录，不是 `.json` 文件），或先独立断言 `is_calibrated` 与逐关节 `Homing_Offset`/`Min_Position_Limit`/`Max_Position_Limit` 一致，再判断故障归属。
- 同步 Pi 时只推 `src/` 不够：`workspace/tools/`、`tests/`、`apps/` 与 `data_contracts/` 曾整体漏同步，导致 `MotionConfig` 字段名不匹配和 CLI 缺参数。rsync 覆盖也会抹掉在 Pi 上手工打的临时补丁，改动必须回到宿主源码再同步；同步 `control/` 后要清 `__pycache__` 才能确认新代码生效。
- Supervisor 的 `refresh_detection` 原先硬编码红色检测，会在 pick-place 前用红色结果清空其他材料的 `pieces.json`。现由 `--material` 选择 `detect_generic_pieces.py`（white/green/blue）或 legacy `detect_red_pieces.py`。`WORKSPACE = Path("workspace")` 是相对路径，CLI 必须从 `/opt/epuzzle` 项目根执行；不要用自引用软链接 `workspace -> .` 绕过，它会造成目录递归。
- 当前无吸附/释放传感器：`VERIFY_PICK`/`VERIFY_PLACE` 只做提升运动并记 `verify_pick_unsensed`/`verify_place_unsensed`，`success: true` 仅代表运动与磁铁时序完成，不代表碎片真被吸起或放下。抓放成败必须由现场目视确认后才可写入验收记录。
- 旋转基准 0° 约定（2026-08-01 起）：不测量碎片绝对朝向。`PlacementPlan.source.yaw_deg` 恒为 `0`（相机拍下初始位置一律视为 0°），`target.yaw_deg` 即求解器相对旋转角（拼图只需碎片间相对方向）。删除“质心→最远顶点”几何朝向的理由：对轮廓噪声敏感、接近对称形状时跳变 180°，且电磁吸头点接触下绝对朝向无夹持约束。执行端 `run_assembly_4pieces.py` 的 roll 补偿（碎片旋转 = Δpan + Δroll = Δyaw，模 360）是该约定的唯一消费方；求解器绝对角输出已废弃。
- 旋转方向标准视角（2026-08-01 统一）：所有水平面旋转角以“俯视 A4 工作台”为准（z 向上、磁吸面朝下），正值 = 俯视逆时针。本机实测硬件方向：`shoulder_pan` 物理角增大 = 俯视逆时针（与 FK 一致）；`wrist_roll` 物理角增大 = 俯视**顺时针**（与 FK 假设相反，位置标定测不出朝向）。因此实际碎片旋转（俯视顺时针为正）= `Δroll_phys − Δpan_phys`，IK 只保证 `Δpan + Δroll = 命令 Δyaw`；执行端命令增量必须取 `Δyaw_cmd = 2*Δpan − Δyaw`（已在 run_assembly_4pieces 实现）。验证/报告工具（verify_rotation2、motion_analysis、report_human）统一比较“实际碎片旋转 vs 目标 −Δyaw”，严禁退回 `Δpan+Δroll` 旧口径（它自洽但无法发现方向错误，v10 实测 4 片方向全错即此原因）。
- 黑底白片间距只有数像素时，二值掩码 `MORPH_CLOSE` 的 9×9 核会把 4 个独立连通域桥接成 1 块（随后因面积 >5000mm² 被全部拒绝）；黑底分支固定使用 3×3 close，暗光白纸分支才使用 9×9。排障必须用 `live_detect_123.py --frame <已保存帧>` 回放同一输入，不能用新抓帧替代失败样本。

## 提交前最低检查

- JSON Schema 可解析；
- 配置中无单位缺失；
- 没有六轴旧字段 `gripper.pos` 混入五轴数据；
- 没有把官方阈值与内部工程目标混写；
- 新行为有对应测试记录或明确标为未验证。

## PICK/PLACE 自动抓放实施记录

### 接触高度门禁（已实施）

- PICK/PLACE 接触高度必须通过人工监督标定工具实测，写入 `workspace/calibration/pick_place_contact_height.json` 验收文件。
- 验收条件：`quality.status == "accepted"` 且 `single_piece_success_rate >= 0.8`，接触高度在 [0, 20] mm 安全区间，磁铁吸附/释放已实测，安定时间在 [0, 5] 秒范围。
- 后端 `pick_place_lock_reason()` 与 Supervisor `load_pick_place_heights()` 双重 fail-closed 检查；任一条件不满足时 `pick-place` 能力状态为 LOCKED。
- 验收未通过时，不得回退到估计值或硬编码值。

### Supervisor grasp_fsm 状态机（已实施）

- 完整 8 状态序列：LOCAL_GRASP_ALIGN → MAGNET_ON → VERIFY_PICK → MOVE_PREPLACE → LOCAL_PLACE_ALIGN → RELEASE → VERIFY_PLACE。
- 入口先验证 `z_pick_mm` 和 `z_place_mm` 非 UNKNOWN（fail-closed），再验证姿态高度与配置一致。
- `finally` 块确保任何异常后磁铁必定 `emergency_off()`。
- 当前无吸附/释放传感器：VERIFY_PICK/PLACE 只做提升运动，事件日志显式标记 `sensor_verified: false`。

### supervisor_run.py 主流程（已实施）

- `run_pick_place()` 门禁顺序严格：先验收文件，再运动锁，再设备节点，最后才连接机械臂和继电器。
- 全路径 IK 预检必须在第一条运动命令之前完成；任何不可达点立即拒绝。
- `finally` 块：强制 `emergency_off()` + 回 PARK + 关闭继电器。
- CLI `--place-table-mm X,Y` 参数显式指定落位点；本模式不从求解器推断。

### 标定工具安全规范（已实施）

- 标定工具 `calibrate_pick_place_heights.py` 全程人工监督，Ctrl+C 随时急停。
- 电磁铁吸合后必须由标定工具流程或调用方显式释放；`MagnetController` 不再设置自动断电时限。
- 至少 10 次完整循环测试（可配置），自动生成验收文件。
- 磁铁安定时间（吸附 2.0s、释放 1.0s）由标定工具实测并写入验收文件，Supervisor 加载后使用。

### 回归测试必须覆盖（已实施）

- `test_pick_place_gate.py`：验收文件缺失/未验收/成功率不足 → LOCKED；高度越界/安定时间缺失 → SupervisorFailure。
- `test_supervisor_gates.py`：高度 UNKNOWN 时 `grasp_fsm` 锁定，磁铁不被调用。
- 任何 `grasp_fsm` 签名变更必须同步更新测试。

### GUI 自动适配（无需修改）

- GUI 通过 `backend.capabilities()` 动态查询按钮状态；验收通过后 `pick-place` 按钮自动解锁。
- 当前 GUI 调用 `pick-place` 需要补充参数输入界面（`--place-table-mm`）。

### 真实硬件首次验证清单（待完成）

1. 在树莓派运行标定工具，生成验收文件。
2. 检查验收文件 `quality.status` 和 `single_piece_success_rate`。
3. CLI 执行单片抓放，观察完整 8 状态序列。
4. 验证急停、异常清理和 PARK 回退路径。
5. 确认磁铁安定时间是否需要调整（当前 2.0s/1.0s）。

### 2026-08-01 容差 5° 诊断实测

- 将 `run_assembly_4pieces.py` 抓放末点 `joint_tolerance_deg` 从 3° 临时放宽到 5° 后，piece_01/piece_02 完成，证明此前 4.147° 失败确实受 3° 门槛触发；piece_03 的 `place_safe` 随后在 wrist_roll 跨越 -180°/+180° 表示边界时失败，误差从约 40°持续增大到 118°，其余四轴约 1° 内，不能再归因于 3°/5° 容差。该路径存在 wrist_roll 角度连续性/发送方向问题，修复前禁止继续运动。
- 本次失败后的串口断开清理抛出 `SerialException: device reports readiness to read but returned no data`；只读复核发现 wrist_flex/wrist_roll 的 `Torque_Enable=1` 且未确认正式 PARK，已重新建立持久运动锁。禁止自动回 PARK、重试或清锁，先由现场切断舵机电源并完成人工支撑和只读门禁。
- 同日扑克重摆后完整重试：视觉/求解成功，piece_00/piece_01 完成；piece_02 的 `place_safe` 阶段收到 `termios.error: (5, Input/output error)`。Pi 内核在同一时刻确认 `usb 3-1: USB disconnect` 后约 1 秒重新枚举 ttyACM0（不是 HC died）；随后只读复核五轴 Torque_Enable 全 1、正式 PARK 未确认，已建立持久运动锁。该次失败属于串口桥断连，不能通过继续放宽收敛容差解决。
- USB 容错已部署：`ResilientJointRobot` 把 `termios.error` 纳入通信异常，非 PICK/PLACE 阶段重连前等待窗口提升到 10 秒；接触段仍 fail-closed，禁止用自动重连掩盖低位抓放风险。
- 绿色第一题重试使用新 ChArUco 标定（70 角点，RMS 0.185mm，max 0.411mm）后，piece_00 完成，piece_01 `place_safe` 仍因 wrist_roll 从 -177.316° 跨到 +173.027° 的环绕路径无法收敛（误差约 83–85°）；本次无新的 USB 断连，说明必须先修 wrist_roll 连续角度路径，不能继续靠容差或 USB 等待规避。
