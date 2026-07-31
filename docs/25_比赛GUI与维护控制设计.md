# 25 比赛 GUI 与维护控制设计

## 1. 目标与验收条件

本设计把 `workspace/apps/pick_place_page.py` 从开发期巡检页面升级为 1024×600、7 英寸触屏上的正式比赛控制台。界面必须同时满足：

1. 正式比赛首页直接覆盖题面三种得分模式：
   - 自备纯色碎片，50 分，最长 120 秒；
   - 现场白色碎片，20 分，最长 120 秒；
   - 现场扑克牌碎片，30 分，最长 120 秒。
2. 选择模式后只有一个明确的“一键启动”入口；计时使用 `time.monotonic()`，运行监视页显示阶段、已用时间、120 秒上限、进度、当前碎片和最后结果。
3. 所有页面保留不小于 96×48 px 的全局急停热区。运行中禁止启动第二个动作；ESTOP 继续锁存，只有正式 PARK 复位成功才能解除。
4. 手动页包括固定五轴关节点动、末端点动、PARK、断力矩、电磁铁定时吸合和立即断电。所有跨模块值显式标注 `joint_deg`、`robot_base_mm` 和单位。
5. 电磁铁界面不提供无限时“保持吸合”：只允许后端声明支持的定时脉冲，参数上限 10 秒；结束、异常、急停和进程退出都必须请求关闭。
6. Debug 页包括能力矩阵、自检、相机/视觉/求解调试入口、仅本次运行的可视化/日志开关、最后结果和日志。Debug 不得修改冻结标定、软限位、运动锁或 feature gate，也不得把未实现功能显示为成功。
7. 后端尚未完成的比赛全流程、手动关节点动和末端点动必须留下稳定接口，并在 GUI 中显示 `未接后端` 或 `安全锁定`；按钮禁用且点击无副作用。
8. 1024×600 离屏渲染覆盖全部四页；按钮不越界、不相互覆盖；关节/末端页签的公共控件必须始终可见。
9. 原有 `detect`、`red-baseline`、`sweep-shortest`、`puzzle-shadow`、`pregrasp`、`park` 合同和测试继续兼容。

## 2. 信息架构

正式 GUI 固定四页：

### 2.1 比赛页 `competition`

- 顶部常驻：页面/全局状态、机械臂、比赛模式、120 秒计时、电磁铁、急停。
- 页面导航：比赛、运行监视、手动、Debug。
- 三张正式模式卡写明题面对象、分值、最长时间和后端状态。
- 赛前检查区至少显示相机、机械臂、运动安全锁、运动学、PARK、电磁铁控制链、当前模式后端。
- 选中模式后显示唯一“一键启动”。若后端未就绪，按钮禁用并在原因区显示具体缺口。
- 既有红色基线不冒充 50 分自动拼图；它只放在 Debug/基线工具区。

### 2.2 运行监视页 `run`

- 显示模式、分值、120 秒上限、单调时钟计时和阶段进度。
- 阶段轨迹：准备 → 检测 → 求解 → 搬运 → 视觉复核 → 完成；以后端心跳原文为准，不凭 GUI 推测成功。
- 显示相机标注图、碎片计数、当前碎片、最后结构化结果和错误码。
- 提供“安全中止”；急停仍是独立全局动作。中止不是比赛正常交互，不提供继续/跳步。

### 2.3 手动页 `manual`

进入手动页不等于允许动作。先点击“解锁维护控制”，仅在非 RUNNING/ESTOP、无运动锁且能力满足时进入本次 GUI 会话的维护解锁状态。离开手动页、开始自动任务、发生故障或急停时自动取消解锁。

手动页有两个页签：

- `joint`：五轴固定顺序为
  1. `shoulder_pan.pos`
  2. `shoulder_lift.pos`
  3. `elbow_flex.pos`
  4. `wrist_flex.pos`
  5. `wrist_roll.pos`

  每轴显示中文名、字段名、实际读回、软限位与 `−/+`。步长仅允许后端合同中的离散值，默认 1°。

- `cartesian`：只显示 `robot_base_mm`；提供 X/Y/Z、pitch、yaw 的 `−/+`，毫米与角度步长分开。不得出现未注明含义的裸 `x/y/z/theta`。

公共控件包括返回、页签、维护锁状态、读取状态、回 PARK、断力矩和磁铁区。按钮模型使用 `_tab: str | None`；`None` 表示两个页签共享，过滤时必须保留 `None`。

手动动作的安全规则：

- GUI 只提交相对增量，不在本地推算最终关节/TCP 状态；实际值以后端读回为准。
- 一个动作一个短命后端请求，动作串行化并有超时；后端必须完成软限位、IK、全路径预检、稳定读回和清理。
- 运动安全锁存在或设备离线时，所有手动运动、PARK、普通断力矩均禁用；检测、Debug 只读动作和急停仍可用。
- 低高度动作必须由后端拒绝或经过冻结门禁；GUI Debug 选项不能覆盖。

电磁铁区：

- 显示 `OFF / PULSE / UNKNOWN` 和剩余自动关闭时间（若后端提供）。
- 提供 1/3/5 秒脉冲选择、“启动电磁铁”和“立即断电”。后端参数合同允许 `(0, 10]` 秒，GUI 默认 3 秒。
- 启动前需维护解锁；立即断电与急停不依赖维护解锁。
- 当前继电器控制链就绪不等于吸取、电气、温升和连续通电验收通过；界面必须保留此警告。

### 2.4 Debug 页 `debug`

分为三块：

1. 能力/自检：每项显示 `READY / LOCKED / UNAVAILABLE` 与原因。
2. 工具：刷新检测、红色基线巡检、快速巡检、通用拼图 Shadow、到选中碎片 20 mm、回 PARK、重新自检。
3. 本次会话选项：轮廓/坐标覆盖层、详细日志、保存证据、generic vision、texture、ACT。前 3 项可以是 GUI 本地开关；后 3 项只有后端能力为 READY 才可选择，且不得写回部署环境或冻结参数。

日志与最后 result JSON 要可在屏幕内查看；stdout 仅作为日志，不参与状态判定。

## 3. GUI 后端深模块与命令接口

GUI 只依赖一个小接口，后端复杂度集中在 Adapter：

```python
class CompetitionBackend(Protocol):
    def capabilities(self) -> Mapping[str, BackendCapability]: ...
    def build_request(
        self,
        command_id: str,
        parameters: Mapping[str, JsonValue],
        *,
        status_path: Path,
        result_path: Path,
    ) -> BackendRequest: ...
```

`BackendCapability` 至少含：

- `command_id`
- `availability`: `READY | LOCKED | UNAVAILABLE`
- `reason`
- `requires_robot`
- `requires_manual_unlock`

`BackendRequest` 至少含 argv、超时和结构化 status/result 路径。GUI 的线程、进程和状态机只处理这一种请求，不为每个未来功能复制一套执行逻辑。

命令 ID 冻结如下：

| 命令 ID | 用途 | 当前策略 |
| --- | --- | --- |
| `competition-self-color` | 自备纯色 50 分全流程 | 后端未完成则 UNAVAILABLE |
| `competition-white` | 现场白色 20 分全流程 | 通用视觉/正式抓放未冻结则 LOCKED |
| `competition-card` | 扑克牌 30 分全流程 | texture/正式抓放未冻结则 LOCKED |
| `manual-read-state` | 五轴/TCP 实际读回 | 后端未完成则 UNAVAILABLE |
| `manual-jog-joint` | 单轴相对点动 | 后端未完成则 UNAVAILABLE |
| `manual-jog-cartesian` | TCP 相对点动 | 后端未完成则 UNAVAILABLE |
| `manual-magnet-pulse` | 电磁铁定时吸合 | 仅 Supervisor/MagnetController Adapter 可 READY |
| `manual-magnet-off` | 立即断电 | 应尽量 READY，失败必须显式上屏 |
| 既有七个 GUI 命令 | 检测/巡检/Shadow/PARK 等 | 保持现有合同 |

未来命令统一继续使用：

```bash
supervisor_run.py <command-id> --status-file PATH --result-file PATH [参数]
```

未来参数：

- `manual-jog-joint --joint-feature NAME --delta-deg VALUE`
- `manual-jog-cartesian --axis x|y|z|pitch|yaw --delta VALUE --frame robot_base_mm`
- `manual-magnet-pulse --duration-s VALUE`
- 比赛命令可追加 `--debug-options-json PATH`，但 Debug 参数不能覆盖安全配置。

未实现命令不要求 CLI 先接受；默认 Adapter 应通过能力表在构造 argv 前拒绝。未来后端接入时同时更新能力表、CLI parser、status/result Schema、接口文档和测试。

## 4. 状态与安全

- GUI 状态仍为 `IDLE / RUNNING / FAULT / ESTOP`，另有 `manual_unlocked` 会话标志；维护解锁不是新的全局 Supervisor 状态。
- 自动比赛或手动动作开始时清除维护解锁。
- 只有结构化 result 为 `PASS` 且 returncode=0 才成功。
- 比赛 120 秒是官方计时上限；GUI 到 120 秒变红并显示超时。后端仍负责有界恢复/PARK，GUI 子进程 watchdog 可以为清理留出小幅余量，但不得把超过 120 秒显示为比赛通过。
- 急停顺序：终止当前子进程 → 请求磁铁关闭 → 请求五轴断力矩 → 锁存 ESTOP。任何一步未确认都保持 ESTOP 并显示人工断电提示。
- 页面不得删除、忽略或改写 `/etc/epuzzle/robot-motion.lock.json`。

## 5. 测试合同

至少新增/更新以下自动测试：

1. 四页离屏截图非黑且颜色足够；所有可见按钮在屏幕内、互不重叠，急停每页可触摸。
2. 三种正式模式及其分值/120 秒信息存在；未实现后端时一键启动禁用且不创建子进程。
3. 假后端把某比赛能力设为 READY 后，一键启动生成正确命令，并用单调时钟显示计时。
4. 手动模式未解锁时点动/磁铁吸合被拒绝；运动锁存在时不能解锁运动。
5. 固定五轴字段顺序无 `gripper.pos`；关节点动 argv 明确带字段名和 degree。
6. 页签过滤保留 `_tab=None` 公共按钮，特别是“← 返回”、PARK、断力矩和磁铁立即断电。
7. 磁铁时长只允许 `(0, 10]`，UI 没有无限保持入口；急停总会请求磁铁关闭。
8. Debug 本地开关不修改 `os.environ`、参数注册表或标定文件；锁定 feature gate 不能由 GUI 打开。
9. 既有 result JSON、heartbeat、离线设备、运动锁、ESTOP 和字体测试继续通过。
10. JSON Schema 全部可解析，文档中的命令枚举与代码能力表一致。
