# 25 比赛 GUI 与维护控制设计

## 1. 目标与验收条件

本设计把 `workspace/apps/pick_place_page.py` 从开发期巡检页面升级为 1024×600、7 英寸触屏上的正式比赛控制台。界面必须同时满足：

1. 正式比赛首页直接覆盖题面三种得分模式：
   - 自备纯色碎片，50 分，最长 120 秒；
   - 现场白色碎片，20 分，最长 120 秒；
   - 现场扑克牌碎片，30 分，最长 120 秒。
2. 选择模式后提供“一键启动”与运行中可用的“停止”入口；运行监视页显示阶段、进度、当前碎片和最后结果，但不显示比赛 120 秒倒计时/已用时间。题面 120 秒上限仍只由 Supervisor 后端以 `time.monotonic()` 强制执行，GUI 不得放宽它。
3. 所有页面保留不小于 96×48 px 的全局急停热区。运行中禁止启动第二个动作；ESTOP 继续锁存，只有正式 PARK 复位成功才能解除。
4. 手动页包括固定五轴关节点动、末端点动、PARK、断力矩、电磁铁定时吸合和立即断电。所有跨模块值显式标注 `joint_deg`、`robot_base_mm` 和单位。
5. 电磁铁界面不提供无限时“保持吸合”：只允许后端声明支持的定时脉冲，时长为大于 0 的有限值；结束、异常、急停和进程退出都必须请求关闭。
6. Debug 页包括能力矩阵、自检、相机/视觉/求解调试入口、仅本次运行的可视化/日志开关、最后结果和日志。Debug 不得修改冻结标定、软限位或 feature gate，也不得把未实现功能显示为成功；运动锁只能跳转到独立的本地人工解除页，不能一键删除。
7. 比赛全流程、手动只读和维护点动必须通过 `supervisor_run.py` 的结构化请求接通；任一运动锁、设备离线、接触高度未验收或安全预检失败时，GUI 必须显示 `安全锁定` 并禁用相应动作。
8. 1024×600 离屏渲染覆盖全部四页；按钮不越界、不相互覆盖；关节/末端页签的公共控件必须始终可见。
9. 原有 `detect`、`red-baseline`、`sweep-shortest`、`puzzle-shadow`、`pregrasp`、`park` 合同和测试继续兼容。
10. Debug 页提供“相机标定（候选）”入口：只能打开冻结 `/dev/epuzzle-camera`、采集 ChArUco 板并生成候选单应与摘要；至少 50 个角点、RMS ≤ 1.0 mm、max ≤ 3.0 mm 才返回 PASS。该入口不得连接机械臂/GPIO，也不得覆盖当前冻结单应、运行 manifest 或参数注册表；候选须经人工核验后才能按独立变更流程提升为正式标定。

## 2. 信息架构

正式 GUI 固定五页：

### 2.1 比赛页 `competition`

- 顶部常驻：页面/全局状态、机械臂、比赛模式、电磁铁、急停；不显示比赛计时。
- 页面导航：比赛、运行监视、手动、Debug。
- 三张正式模式卡写明题面对象、分值和后端状态；不显示最长时间。
- 赛前检查区至少显示相机、机械臂、运动安全锁、运动学、PARK、电磁铁控制链、当前模式后端。
- 选中模式后显示唯一“一键启动”。若后端未就绪，按钮禁用并在原因区显示具体缺口。
- 既有红色基线不冒充 50 分自动拼图；它只放在 Debug/基线工具区。

### 2.2 运行监视页 `run`

- 显示模式、分值和阶段进度；不显示比赛计时。题面时限仅在 Supervisor 后端执行。
- 阶段轨迹：准备 → 检测 → 求解 → 搬运 → 视觉复核 → 完成；以后端心跳原文为准，不凭 GUI 推测成功。
- 显示相机标注图、碎片计数、当前碎片、最后结构化结果和错误码。
- 提供醒目的“停止”；它向 GUI 启动的整个 Supervisor 子进程组发送 SIGINT、等待有限清理，并请求磁铁 OFF。急停仍是独立全局动作。停止后不提供继续/跳步。

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

- 显示 `OFF / PULSE / UNKNOWN` 和当前脉冲剩余时间（若后端提供）。
- 提供 1/3/5 秒脉冲选择、“启动电磁铁”和“立即断电”。后端参数合同允许任意大于 0 的有限时长，GUI 默认 3 秒。
- 启动前需维护解锁；立即断电与急停不依赖维护解锁。
- 当前继电器控制链就绪不等于吸取、电气、温升和连续通电验收通过；界面必须保留此警告。

### 2.4 Debug 页 `debug`

分为三块：

1. 能力/自检：每项显示 `READY / LOCKED / UNAVAILABLE` 与原因。
2. 工具：刷新检测、相机标定（候选）、红色基线巡检、快速巡检、通用拼图 Shadow、到选中碎片 20 mm、回 PARK、重新自检。相机标定只使用固定相机和 ChArUco 板，将候选写入 `workspace/runtime/camera_calibration/`；不覆盖当前正式单应或 manifest。
3. 本次会话选项：轮廓/坐标覆盖层、详细日志、保存证据、generic vision、texture、ACT。前 3 项可以是 GUI 本地开关；后 3 项只有后端能力为 READY 才可选择，且不得写回部署环境或冻结参数。

日志与最后 result JSON 要可在屏幕内查看；stdout 仅作为日志，不参与状态判定。

### 2.5 人工解锁页 `unlock`

- 只在本地触屏操作，读取当前 `/etc/epuzzle/robot-motion.lock.json` 的 `clear_requires`；锁不存在、无法读取、项目数不在 1–5 或内容异常时不显示解除控件。
- 操作者必须逐项勾选当前清单，并持续按住“人工解除运动锁”2 秒。GUI 写入权限为 0600、有效期为 5 分钟的确认文件，包含当前锁 SHA256 和每项勾选结果。
- `supervisor_run.py` 只把固定确认文件转发给 root helper；helper 再次校验文件路径、owner、权限、时效、锁 SHA256 和逐项文本，先写 root-only 审计记录后才删除锁。任何校验失败都保留锁。
- 人工解除只移除持久锁，不证明 PARK、力矩、磁铁或姿态安全；后续每项动作仍重走原有门禁。

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
| `competition-self-color` | 自备纯色 50 分全流程 | 新鲜 123 视觉 → 严格求解 → 四片抓放；接触高度/运动锁/设备任一失败则 LOCKED |
| `competition-white` | 现场白色 20 分全流程 | 同一确定性 123 链路；需要每次画面得到四片严格 `SOLVED` |
| `competition-card` | 扑克牌 30 分全流程 | 同一确定性 123 链路；不把“命令存在”表述为牌面纹理验收已完成 |
| `camera-calibration` | ChArUco 相机标定候选 | 仅固定相机；生成候选单应/摘要，不连接机械臂/GPIO、不覆盖冻结标定 |
| `manual-clear-motion-lock` | 现场人工解除持久锁 | 固定确认文件、逐项 checklist、2 秒长按和 root helper 二次校验缺一不可；写 root-only 审计记录 |
| `manual-read-state` | 五轴/TCP 实际读回 | 只读总线，不写舵机寄存器；运动锁下仍可用于排障 |
| `manual-jog-joint` | 单轴相对点动 | 仅 PARK→HOME→安全高度小步→PARK，须维护解锁 |
| `manual-jog-cartesian` | TCP 相对点动 | 仅 `robot_base_mm` 安全高度小步→PARK，须维护解锁 |
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

GUI 只依赖结构化 `status/result`，不解析 stdout。任何新增命令仍须同时更新能力表、CLI parser、status/result Schema、接口文档和测试。

## 4. 状态与安全

- GUI 状态仍为 `IDLE / RUNNING / FAULT / ESTOP`，另有 `manual_unlocked` 会话标志；维护解锁不是新的全局 Supervisor 状态。
- 自动比赛或手动动作开始时清除维护解锁。
- 只有结构化 result 为 `PASS` 且 returncode=0 才成功。
- 比赛 120 秒是官方计时上限；GUI 不显示该计时，Supervisor 后端仍负责以单调时钟硬性停止、有限恢复/PARK。GUI 子进程 watchdog 可以为清理留出小幅余量，但不得把超过 120 秒显示为比赛通过。
- 急停顺序：终止当前子进程 → 请求磁铁关闭 → 请求五轴断力矩 → 锁存 ESTOP。任何一步未确认都保持 ESTOP 并显示人工断电提示。
- 除 `manual-clear-motion-lock` 的受限 root helper 外，页面不得删除、忽略或改写 `/etc/epuzzle/robot-motion.lock.json`。该 helper 只接受本地 GUI 的固定确认文件，并在清单、SHA256、权限和时效均匹配后才允许删除。

## 5. 测试合同

至少新增/更新以下自动测试：

1. 四页离屏截图非黑且颜色足够；所有可见按钮在屏幕内、互不重叠，急停每页可触摸。
2. 三种正式模式及其分值存在、页面不显示 120 秒计时；未实现后端时一键启动禁用且不创建子进程。
3. 假后端把某比赛能力设为 READY 后，一键启动生成正确命令；比赛运行中两个“停止”入口都可用，其他普通动作禁用。
4. 手动模式未解锁时点动/磁铁吸合被拒绝；运动锁存在时不能解锁运动，但只读状态仍可查询。
5. 固定五轴字段顺序无 `gripper.pos`；关节点动 argv 明确带字段名和 degree。
6. 页签过滤保留 `_tab=None` 公共按钮，特别是“← 返回”、PARK、断力矩和磁铁立即断电。
7. 磁铁时长只允许大于 0 的有限值，UI 没有无限保持入口；急停总会请求磁铁关闭。
8. Debug 本地开关不修改 `os.environ`、参数注册表或标定文件；锁定 feature gate 不能由 GUI 打开。
9. 既有 result JSON、heartbeat、离线设备、运动锁、ESTOP 和字体测试继续通过。
10. JSON Schema 全部可解析，文档中的命令枚举与代码能力表一致。
