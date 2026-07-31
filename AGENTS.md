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

## 树莓派镜像构建经验

- arm64 wheelhouse 必须在 aarch64 Python 环境解析条件依赖；宿主 x86 pip 可能错误拉取 PyTorch CUDA 包。
- 不要在宿主机对 arm64 缓存 venv 使用 `pip --python <venv>`：该 venv 的 Python 软链接会解析到宿主解释器，可能把包错误写入 `lib/python<宿主版本>`。纯 Python 插件也应先从无 `__pycache__` 的干净源目录构建 wheel，再在 aarch64 环境安装，或校验后解包到明确的 CPython 目标目录。
- wheel 可下载不等于能运行；必须在目标 rootfs 系统库或 Pi 5 实机中逐项 import OpenCV、torch、torchvision、LeRobot 和插件。
- `pkexec` 启动的 root 子进程可能脱离外层任务；写盘完成以实际 `dd` 退出和 raw SHA256 回读一致为准，不以外层返回码或时长判断。
- 64GB 卡使用较小交付镜像并保留 Raspberry Pi OS `resize` 首启扩容；写盘后只读回读整个镜像覆盖范围校验 SHA256。
- NetworkManager 有线维护 profile 不得设置负 `ipv4.dns-priority`；负值会排除其他连接的 DNS，使 Wi-Fi 已有默认路由却无法解析域名。
- 当前 Raspberry Pi OS 基础镜像的 `/etc/cloud/cloud.cfg` 引用了未安装的 `netplan_nm_patch`；构建时移除该 final module，避免首启 `cloud-init status` 显示 degraded。
- 长时间运行的 shell 构建脚本启动后不要修改同一脚本；bash 会继续读取后半段，运行中编辑可能造成只在收尾阶段出现的解析错误。先停止或等待构建结束，再修改并从稳定版本重跑。
- 电磁铁控制已冻结为 `/dev/gpiochip0` 的 BCM GPIO17，经低电平触发继电器控制；3.3 V 高电平默认关闭，单次软件通电上限 10 秒。`config.txt` 使用 `gpio=17=op,dh,pu` 尽早拉高，但官方说明上电到该设置生效仍可能延迟数秒，因此继电器输入外部硬件上拉到 3.3 V 才是最早期默认关闭的基础。线圈电压、电流、温升和实测连续通电能力仍为 `UNKNOWN`。
- pygame/SDL2 `kmsdrm` GUI 从 SSH 启动时必须通过 `openvt` 绑定活动 VT；内层不要使用启用了 `use_pty` 的 `sudo -u`，应由 root 侧 `setpriv` 降权，否则 Python 的标准输入仍指向 SSH pts，可能只显示 DRM 硬件鼠标层。
- QDTECH MPI700 触摸屏应具有 `ID_INPUT_TOUCHSCREEN=1`；若 udev 规则晚于设备枚举，需对对应 event 节点执行 `udevadm trigger --action=change`。GUI 不依赖 SDL 隐式触摸鼠标合成，应显式把 `FINGER*` 归一化坐标映射到屏幕像素。
- 当前 SDL 2.32 KMSDRM 在 Pi 5 上表现为主 framebuffer 正常翻页但物理屏全黑，仅硬件光标可见；修改 pygame 软件 Surface 的 alpha 不会修复 SDL 后续生成的 GBM buffer。7 英寸正式 GUI 使用 `SDL_VIDEODRIVER=dummy` 离屏绘图，转 RGB565 后直接写 `/dev/fb0`，并直接读取 QDTECH MPI700 evdev；验收需同时检查 fb0 非黑像素/颜色数和实屏触摸。
- 7 英寸 GUI 手动页的 `_tab=None` 表示关节/末端页签共享控件；过滤时必须保留 `None`，否则“← 返回”等公共按钮会消失。该语义必须有回归测试。
- 机械臂 commissioning 的悬空阶段之间不得断力矩；否则关节会因重力下垂，下一阶段的起点即失效。首次使能必须先在断力矩状态把 `Goal_Position` 对齐当前 `Present_Position`，受监控序列结束前先返回有支撑的停放位，再断力矩。
- 断力矩触点拟合与舵机编码器收敛不能单独保证带力矩时的 TCP 离面间隙；本机 `calibration_sheet_mm` 目标 10 mm 经现场观察过低或碰纸，20 mm 经 10 秒停留确认可作为竞赛 PREGRASP/观察高度。PICK 接触高度必须单独标定。
- 快速巡航与精确终点使用不同到位门槛：5°/单样本只适合扫点，会使负载下 TCP 提前 10–20 mm 判到位，禁止用于抓放。当前 D 点精确终点实测补偿为径向远离机械臂 3 mm、命令 pitch `-87.5°`，负载后实际 pitch `-90.03°`；推广到全工作区前至少抽查近/中/远点。
- `MotionGoal.timestamp_ns`、`deadline_ns` 和控制器 deadline 比较必须处于同一 `time.monotonic_ns()` 时钟域；禁止 Supervisor 用 monotonic 生成 deadline、控制器却用 `time.time_ns()` 比较。PARK 途中宽跟随门槛也不得代替最终正式 PARK 容差验收。
- 相机运行时必须使用冻结的 `/dev/epuzzle-camera` 稳定链接（Sipeed MaixCAM `359f:2120`、serial `c9c28f4513595968`、capture index 0），不得依赖可漂移的数字 index；更换相机必须重新冻结 udev identity。
- 若 Pi 内核报告 `xHCI host controller not responding` / `HC died` 并使机械臂串口节点消失，异常清理无法再确认断力矩或正式 PARK。禁止自动重绑 USB、重启 Pi 或重试运动；必须先由现场操作者支撑机械臂并切断舵机电源，再恢复 USB，随后先做只读枚举和 PARK 状态检查。
- Pi 的工作区五轴插件源码路径必须排在 venv site-packages 前；包版本号相同不能证明安装内容一致。软件自检必须同时断言实际导入路径来自 `/opt/epuzzle/workspace/src/robot_plugin/`，且 `SO101Follower5D.connect_bumpless` 存在。
- 真实硬件事件导致 PARK/断力矩无法确认时，使用 root 管理的 `/etc/epuzzle/robot-motion.lock.json` 持久锁定运动；设备节点重新出现、GUI 重启或 Pi 重启都不得自动解锁。只有现场完成锁文件 `clear_requires` 全部步骤并明确批准后，才可人工解除。
- 笛卡尔插值步数不能只按 XYZ 距离和 pitch/yaw 分段：HOME→首航点这类笛卡尔位移小、关节大幅重构的动作（实测 pan −49.9°、elbow +35.6°、wrist −46.9°）会使单步关节跳变超 `max_joint_step_deg`。终点 IK 的关节位移只是下界，IK 非线性使中间段单步跳变大于均值，`ceil(span/limit)` 甚至加 2% 余量都仍可能失败（实测 10.14°/10.15°/10.137° 反复卡在 10° 门槛外）。正确做法是超限即翻倍加密重试（上限 4 轮），绝不放宽 `max_joint_step_deg`；本机 piece-0 最终以 6 点通过。
- `sudo` 执行的 CLI 中 `Path.home()` 解析为 `/root`，会使 `~/.cache/huggingface/lerobot/calibration/...` 默认标定目录落空并报出误导性的“标定寄存器不一致”。必须显式传 `--lerobot-calibration <目录>`（要目录，不是 `.json` 文件），或先独立断言 `is_calibrated` 与逐关节 `Homing_Offset`/`Min_Position_Limit`/`Max_Position_Limit` 一致，再判断故障归属。
- 同步 Pi 时只推 `src/` 不够：`workspace/tools/`、`tests/`、`apps/` 与 `data_contracts/` 曾整体漏同步，导致 `MotionConfig` 字段名不匹配和 CLI 缺参数。rsync 覆盖也会抹掉在 Pi 上手工打的临时补丁，改动必须回到宿主源码再同步；同步 `control/` 后要清 `__pycache__` 才能确认新代码生效。
- Supervisor 的 `refresh_detection` 原先硬编码红色检测，会在 pick-place 前用红色结果清空其他材料的 `pieces.json`。现由 `--material` 选择 `detect_generic_pieces.py`（white/green/blue）或 legacy `detect_red_pieces.py`。`WORKSPACE = Path("workspace")` 是相对路径，CLI 必须从 `/opt/epuzzle` 项目根执行；不要用自引用软链接 `workspace -> .` 绕过，它会造成目录递归。
- 当前无吸附/释放传感器：`VERIFY_PICK`/`VERIFY_PLACE` 只做提升运动并记 `verify_pick_unsensed`/`verify_place_unsensed`，`success: true` 仅代表运动与磁铁时序完成，不代表碎片真被吸起或放下。抓放成败必须由现场目视确认后才可写入验收记录。

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
- 单次磁铁通电不超过 10 秒（`MagnetController` 硬件保护）。
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
