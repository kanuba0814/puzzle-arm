# Supervisor 深模块设计上下文（只读分析产物）

任务：为 production Supervisor/安全状态机给出可实现的设计与代码合同。本文件供规划/实现 agent 直接使用，不要重新挖掘以下证据。

## 0. 关键结论（TL;DR）

- `workspace/src/integration/` 目前**只有 README.md**，Supervisor 是全新深模块；不存在已有 Supervisor 代码可改。
- 现有 `workspace/tools/control_red_piece_centers.py` 是 legacy 页面工具（被正式 GUI 以子进程方式调用），其中 `direct_move`/`park_to_home`/`home_to_park` 是**页面级临时实现，不得搬进 Supervisor**（任务硬约束："不得把 legacy controller 带入"）。但它固化了两个必须复用的**行为事实**：PARK↔HOME 的安全过渡序列、精确终点补偿参数（径向 +3 mm、pitch −87.5°）。
- `MotionController` 已实现"全路径预检 + 读回收敛 + phase 门禁"，Supervisor 必须**复用它而非重写运动**；缺口只有：关节空间的 PARK↔HOME 过渡（MotionController 只接受笛卡尔 `robot_base_mm` 目标）、扫块排序、视觉时效、shadow、磁铁状态机 seam、急停恢复、事件日志——这些全部属于 integration 层。
- PICK/PLACE 接触高度仍为 `UNKNOWN`（`configs/parameter_registry.yaml` `motion.z_pick_mm/z_place_mm`；`runtime_calibration_manifest_20260731.json` `safety.automatic_pick_place_enabled=false`）。Supervisor 对磁吸抓放状态必须 **fail-closed**，门禁只能由"registry 中出现实测值"解除，不能加旁路开关。
- GUI 接法应保持现有"页面不常驻占用串口/GPIO、动作走子进程 CLI"的架构（`workspace/apps/pick_place_page.py` + `workspace/src/integration/README.md` 明确记录）。Supervisor 对 GUI 暴露为薄 CLI 包装 `workspace/tools/supervisor_run.py`。

---

## 1. 现状证据（文件 + 符号 + 关键行）

### 1.1 控制层（只读复用，不改）

`workspace/src/control/motion_controller.py`

- `_ALLOWED_PHASES = {HOME, SAFE, PREGRASP, PICK, PREPLACE, PLACE, RECOVERY}`（文件头，~L20-27）；`_ALLOWED_SPEED_PROFILES = {EMPTY_NORMAL, EMPTY_SLOW, HOLDING_SLOW, LOCAL_POLICY}`（~L28-33）。
- `MotionErrorCode = {NONE, IK_NO_SOLUTION, ROBOT_COMM_TIMEOUT, SAFETY_REJECTED, GLOBAL_TIMEOUT}`（~L61-67）。
- `MotionController.move(goal)`：**先在 `_plan()` 中对全部路径点做 IK + 软限位 + 关节步长预检，全部通过才发送第一个舵机目标**；逐点 `send_action` 后用 `_wait_for_target` 读回连续 `stable_samples` 个样本在 `joint_tolerance_deg` 内才算到位；`deadline_ns` 全程可中断。目标 z 门禁：phase ∈ {PICK, PLACE} 用 `contact_min_z_mm`，其余用 `safe_min_z_mm`。
- `MotionController.from_calibration_file()` 强制 `ArmCalibration.require_accepted_geometry()`（`calibration.py`：status=accepted、fit≥3、holdout≥4、XY RMS≤5 mm、XY max≤8 mm、|Z|max≤3 mm）。
- `JointRobot` Protocol：`get_observation() -> Mapping`、`send_action(dict) -> Mapping`。这就是 Supervisor 测试 fake 的 seam。

`workspace/src/control/kinematics.py`

- `JOINT_FEATURES` 固定五轴顺序；`JointVector.from_mapping` 精确校验五字段；`PlanarArmKinematics.forward/inverse` 只处理 `robot_base_mm`。

`workspace/src/control/magnet_controller.py`

- `MagnetController.set_enabled(bool)` 是唯一控制接口；`MagnetConfig(chip_path="/dev/gpiochip0", line_offset=17, active_high=True, max_on_seconds=10.0)`；10 s 软件上限调用方不可覆盖；`emergency_off()`、`close()` 先断电后释放；`on_status: StatusCallback` 回调天然是事件日志 hook；`MagnetStatus(enabled, reason, monotonic_ns, line_offset)`。
- output_factory/timer_factory 可注入 → 无 GPIO 的单元测试 seam（见 `workspace/tests/test_magnet_controller.py`）。

### 1.2 legacy 页面工具（不得带入，但承载必须复用的行为事实）

`workspace/tools/control_red_piece_centers.py`

- `HOME`（~L36-42）：五个关节的常量角度；`CALIBRATION_PATH`/`TRANSFORM_PATH`/`PARK_PATH` 硬编码到 20260731 标定族。
- PARK 门禁（`main()` 内）：连接后 `connect_bumpless(p=24, accel=30)`，起始位与 PARK 偏差 >4° 即拒绝（"机械臂不在正式 PARK，拒绝页面动作"）。
- `park_to_home()`：先把 `shoulder_pan` 固定为 PARK 值、其余四轴升到 HOME，再动 pan 到 HOME；`home_to_park()` 反向。这是**关节空间、非笛卡尔**的安全过渡，MotionController 不覆盖。
- `direct_move()`：sleep 轮询 + 8° 容差的页面级插补；`finally` 中未回 PARK 时兜底 `home_to_park()`。
- `target_pose()`：table_mm → robot_base_mm 变换（`contact_sheet_to_robot_base_20260731.json`），**径向远离机械臂 3 mm 补偿 + pitch −87.5°**（与 `parameter_registry.yaml` `motion.precision_terminal_*` 和 manifest 的 `precision_terminal_compensation` 一致，sha256 `b0a19ff5…`）。
- `sweep` 模式排序：`sorted(pieces, key=lambda p: (-p["y"], p["x"]))`（从后往前光栅，非最短路径）。
- 全程 `magnet_enabled=False`，`MotionConfig(10.0, 5.0, 10.0, 5.0, 1, 0.02, 3.0, -20.0, -30.0, 180.0)`。
- 时间戳用 `time.time_ns()`（非单调，PLAN Phase 0 已登记为遗留问题）。

### 1.3 GUI（接入目标，最小改动）

`workspace/apps/pick_place_page.py`

- 所有动作通过 `_run(name, command)` 启动**子进程**（`PYTHON=/opt/epuzzle/venv/bin/python`，cwd=项目根），页面自身不持有串口/GPIO；`_control(mode,label)` 调 `workspace/tools/control_red_piece_centers.py <mode> --piece-index N --output …`。
- `pick_rotate_place()` 当前只设置状态文本"PICK接触高度仍未冻结：已禁止磁铁和自动抓放"——**锁定已在 GUI 层存在，Supervisor 层必须独立再锁一次**（纵深防御）。
- `emergency_stop()`：terminate/kill 子进程 → `sudo systemctl restart epuzzle-magnet-safe.service` → 跑 `workspace/tools/emergency_off_single_arm.py`（五轴断力矩）。systemd fail-safe 服务见 `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-magnet-safe.service`（oneshot，启停都执行 `magnet-off.py`）。
- 检测结果读 `workspace/runtime/red_piece_detection/pieces.json`（由 `workspace/tools/detect_red_pieces.py` 产出，含 `schema_version/homography_file/pieces[]`）。

### 1.4 契约与配置

- `data_contracts/runtime_event.schema.json`：**已存在**，字段 `timestamp_ns(int≥0)/state/event/severity(DEBUG..FATAL)/piece_id?/error_code?/payload(object)`，`additionalProperties: false`。Supervisor 事件日志直接遵循此 schema。
- `data_contracts/magnet_command.schema.json`、`motion_goal/motion_result/piece_observation/placement_plan.schema.json` 已存在；`placement_plan` 的 yaw 是 `minimum:-180, maximum:180`（PLAN Phase 1 要改成 `exclusiveMaximum:180`，Supervisor 消费时按新契约）。
- `docs/10_接口与数据契约.md` 错误码基线：`VISION_NO_PIECE, VISION_LOW_CONFIDENCE, SOLVER_NO_SOLUTION, IK_NO_SOLUTION, ROBOT_COMM_TIMEOUT, PICK_VERIFY_FAILED, RELEASE_VERIFY_FAILED, ACT_TIMEOUT, SAFETY_REJECTED, GLOBAL_TIMEOUT`。PLAN Phase 1 将增补 `VISION_MERGED_BLOBS, VISION_SURFACE_MISMATCH, SOLVER_AMBIGUOUS, SOLVER_TIMEOUT, TEXTURE_INSUFFICIENT, TEXTURE_AMBIGUOUS`。
- `docs/11_运行状态机与故障恢复.md`：17 状态（INIT/WAIT_START/SCAN/SOLVE/PLAN_PREGRASP/MOVE_PREGRASP/LOCAL_GRASP_ALIGN/MAGNET_ON/VERIFY_PICK/MOVE_PREPLACE/LOCAL_PLACE_ALIGN/RELEASE/VERIFY_PLACE/NEXT_PIECE/COMPLETE/RECOVERY/FAILED）及抓取失败/掉落/释放失败/放置偏差/ACT 异常的恢复规则——这是 Supervisor 状态机的**文档基线**。
- `configs/parameter_registry.yaml`：`motion.z_safe_mm=60, z_pregrasp_mm=20, z_pick_mm=UNKNOWN, z_place_mm=UNKNOWN`；`supervisor.global_timeout_s=120, max_pick_retries=UNKNOWN, act_enabled=false`；正式 `table_to_robot_file` 路径已登记但**文件尚不存在**（当前运行用 contact-sheet 变换 + manifest 显式等价假设）。
- `workspace/calibration/runtime_calibration_manifest_20260731.json`：所有标定 artifact 的 sha256；`safety.automatic_pick_place_enabled=false, automatic_pick_place_blocker="PICK and PLACE contact heights remain UNKNOWN"`。

### 1.5 计划文档对本任务的要求

`.taskman/plans/adopt-puzzle-vision-v2-2/PLAN.md` Phase 6（安全启用）：

- 启用顺序固定：离线 replay → Pi benchmark → GUI shadow → Supervisor 只读 plan → Gate C/E → 受监控实机。
- Supervisor 每放一片后必须重新拍照和求解；不得沿用整局初始理想位姿。
- `SOLVER_AMBIGUOUS/SOLVER_TIMEOUT`/纹理不足歧义/低置信/视觉过期/标定失效均禁止动作。
- 真机执行需冻结正式 `table_mm→robot_base_mm`（含 SHA256）、PICK/PLACE 接触高度、磁头 footprint/净空门槛；**contact-sheet 变换不得冒充正式视觉链路变换**。
- 明确不改：`workspace/src/control/`、`robot_plugin/`、`policy/`、GPIO17/磁铁超时/五轴顺序。

### 1.6 测试模式（必须沿用）

`workspace/tests/test_motion_controller.py`：`ManualClock`（monotonic/sleep/timestamp_ns 注入）+ `FakeRobot`（实现 `JointRobot` Protocol）+ `unittest`。验收命令（PLAN §5）：

```bash
PYTHONPATH=workspace/src python -m unittest discover -s workspace/tests -v
python -m compileall -q workspace/src workspace/tools
ruff check workspace/src workspace/tools workspace/tests
```

---

## 2. 模块设计（深模块，小公共面）

### 2.1 文件布局（新增，均在 integration 层）

```
workspace/src/integration/
├── __init__.py            # 只导出公共符号（见 2.2）
├── supervisor.py          # Supervisor 类：状态机 + 编排（唯一公共入口）
├── safety_gates.py        # 私有实现可内联进 supervisor.py；若拆分则不导出
├── event_log.py           # JsonlEventLog：写 runtime_event schema 兼容 JSONL
├── park_sequence.py       # JointSpaceParkMover：PARK↔HOME 关节空间过渡
├── sweep_planner.py       # 扫块最短路径排序（纯函数，无硬件依赖）
└── README.md              # 更新：记录公共面与门禁
workspace/tools/
└── supervisor_run.py      # GUI 子进程薄包装：解析参数→构造真实 robot/magnet→调 Supervisor→写结果 JSON
workspace/tests/
├── test_supervisor_states.py
├── test_supervisor_gates.py
├── test_supervisor_event_log.py
├── test_sweep_planner.py
└── test_park_sequence.py
data_contracts/
├── supervisor_config.schema.json      # 新增（SupervisorConfig 序列化面）
├── supervisor_result.schema.json      # 新增（每次运行的结果记录）
└── runtime_event.schema.json          # 已存在，不放宽
configs/parameter_registry.yaml        # 追加 supervisor.* 键（见 2.5）
docs/10_接口与数据契约.md              # 增补 Supervisor 错误码段
```

注意：PLAN §6 明确"不改 `workspace/src/control/`"是对 adopt-puzzle-vision-v2-2 计划而言；本 Supervisor 任务若需要向 control 增加东西，**优先不加**——PARK↔HOME 的关节空间过渡放在 `integration/park_sequence.py`（它是编排逻辑，不是通用控制原语），这样 control 保持冻结。

### 2.2 公共类型与接口（建议公开且只公开）

```python
# integration/__init__.py 导出
Supervisor           # 类
SupervisorConfig     # frozen dataclass
SupervisorState      # str Enum
SupervisorErrorCode  # str Enum
SupervisorResult     # frozen dataclass（单次运行结果）
ShadowPlanReport     # frozen dataclass（shadow 模式输出）
JsonlEventLog        # 事件日志（GUI/工具也需要读事件时可导出）
```

**SupervisorState**（以 docs/11 的 17 态为基线，增补页面/PARK 态与 shadow 态；保留全部原状态名以便文档对齐）：

```
PARKED            # 正式 PARK，已断/保持力矩的唯一确定入口态
INIT              # 连接、标定门禁、读取 registry
PARK_TO_HOME / HOME_TO_PARK   # 过渡态（docs/11 未覆盖，新增）
WAIT_START
SCAN              # 拍照 + vision.observe_pieces / detect_red_pieces adapter
SHADOW_PLAN       # 只计算 PuzzleSolveResult/扫描建议并落盘，绝不允许后续动作
SOLVE
PLAN_PREGRASP / MOVE_PREGRASP
LOCAL_GRASP_ALIGN / MAGNET_ON / VERIFY_PICK
MOVE_PREPLACE / LOCAL_PLACE_ALIGN / RELEASE / VERIFY_PLACE
NEXT_PIECE / COMPLETE
ESTOP             # 急停锁定（新增，替代静默退出）
RECOVERY / FAILED
```

迁移规则硬约束：

- 只有 Supervisor 可切换状态（状态机私有 `_transition()`，外部只读 `state`）。
- 每个状态有 `max_duration_s`；超时 → RECOVERY。
- `MAGNET_ON/VERIFY_PICK/…/RELEASE/VERIFY_PLACE` 这 5 个态在进入前必须过 `grasp_gates()`（见 2.4）；gate 失败 → `SUPERVISOR_LOCKED`，不进态。
- ESTOP 只能从外部急停回调进入；退出 ESTOP 必须经 `RESET → PARKED`（人工确认 + 重新 PARK 校验），禁止自动恢复。

**SupervisorErrorCode**（Supervisor 自有，叠加 docs/10 基线；全部进 docs/10 与 schema enum）：

```
SUPERVISOR_LOCKED          # PICK/PLACE 高度或正式变换 UNKNOWN，抓放被锁
PARK_MISMATCH              # 起始位不在正式 PARK（>4°）
PARK_TRANSITION_FAILED     # PARK↔HOME 过渡跟随误差超限/超时
VISION_STALE               # 视觉结果超过 max_vision_age_ms 或 source_image 绑定不符
VISION_NO_PIECE / VISION_LOW_CONFIDENCE   # 复用 docs/10
PLAN_STALE                 # PlacementPlan 绑定的 source image/timestamp 已过期
SOLVER_NO_SOLUTION / SOLVER_AMBIGUOUS / SOLVER_TIMEOUT  # 复用 PLAN Phase 1
TRANSFORM_UNOFFICIAL       # 只有 contact-sheet 变换而任务要求正式 table_mm→robot_base_mm
CALIBRATION_REJECTED       # ArmCalibration 门禁失败（透传）
ESTOP_ACTIVE               # 急停中拒绝一切新命令
GLOBAL_TIMEOUT / SAFETY_REJECTED / IK_NO_SOLUTION / ROBOT_COMM_TIMEOUT  # 复用
SHADOW_MODE                # shadow 模式下收到执行请求（防御性，正常路径不应触发）
PICK_VERIFY_FAILED / RELEASE_VERIFY_FAILED  # 未来磁吸 FSM 用（docs/10 已有）
```

**SupervisorConfig**（frozen dataclass，字段全部来自 registry，禁止默认值捏造 UNKNOWN）：

```python
@dataclass(frozen=True)
class SupervisorConfig:
    calibration_path: Path            # arm_kinematics_single_arm_accepted_20260731.json
    park_pose_path: Path              # park_pose_single_arm_20260731.json
    transform_path: Path              # table_mm->robot_base_mm；正式文件缺失时只能 shadow
    transform_is_official: bool       # False=contact-sheet 等价假设 → 禁真机抓放
    motion_config: MotionConfig       # 复用 control 的（safe_min_z 等）
    z_safe_mm: float                  # 60.0（相对 calibration_sheet_mm；z 由变换 translation_z+height 得出）
    z_pregrasp_mm: float              # 20.0
    z_pick_mm: float | None           # None=UNKNOWN → grasp gate 锁
    z_place_mm: float | None          # None=UNKNOWN → grasp gate 锁
    park_tolerance_deg: float         # 4.0（沿用页面证据）
    park_follow_tolerance_deg: float  # 8.0（沿用 direct_move 证据）
    max_vision_age_ms: int            # registry UNKNOWN 前以保守值(如 2000)登记并标注工程假设
    global_timeout_s: float           # 120（官方上限）
    shadow: bool                      # True 时 hardware 接口必须同时为 None/Stub
    automatic_pick_place_enabled: bool  # 默认 False；与 z_pick/z_place 非 None 共同构成 gate
    precision_radial_compensation_mm: float  # 3.0（冻结证据）
    precision_pitch_command_deg: float       # -87.5（冻结证据）
    event_log_path: Path
```

**Supervisor 构造 seam**（与 control 测试同一模式）：

```python
class Supervisor:
    def __init__(
        self,
        config: SupervisorConfig,
        motion: MotionController,          # 已用 from_calibration_file 构造
        park_mover: JointSpaceParkMover,   # 内部模块，见 2.3
        magnet: MagnetController | None,   # shadow/锁定路径传 None
        vision: VisionProvider,            # Protocol: scan() -> VisionBatch
        solver: PuzzleSolver | None,       # Protocol: solve(batch) -> PuzzleSolveResult；红色基线期可为 None
        event_log: JsonlEventLog,
        monotonic: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,   # 不用 time.time_ns()
        sleep: Callable[[float], None] = time.sleep,
    ) -> None: ...
```

所有依赖注入 → 全部单元测试可无硬件运行。

**SupervisorResult**：`schema_version, run_id, final_state, success, error_code, message, events_path, vision_batch_ref(sha256), plan_ref(sha256), duration_ms` —— 落盘到 `workspace/runtime/supervisor/<run_id>.json`，与 legacy 工具的 `--output` 模式同构，GUI 解析方式不变。

### 2.3 PARK↔HOME（`park_sequence.py`，新增，复用行为证据、不复制 legacy 代码）

`JointSpaceParkMover(robot: JointRobot, kinematics, *, park_deg: dict, home_deg: dict, tolerance_deg, step_deg=1.0, settle_s, monotonic, sleep)`：

- `park_to_home()`：读当前位 → 校验与 PARK 偏差 ≤ `park_tolerance_deg`（否则 `PARK_MISMATCH`）→ 中间 upright = HOME 但 `shoulder_pan` 取 PARK 值 → 关节线性插补到 upright → 插补到 HOME。
- `home_to_park()`：HOME → upright（pan 取 PARK 值）→ PARK。
- 插补：每步 ≤1°，发后读回，任一点偏差 > `park_follow_tolerance_deg`(8°) 或超时 → `PARK_TRANSITION_FAILED`。
- 序列与容差取自 `control_red_piece_centers.py` 的 `park_to_home/home_to_park/direct_move`（已实机验证），但实现为独立、可注入时钟、有 unittest 的新类；**不 import、不调用 legacy 工具**。
- 断力矩安全（AGENTS.md commissioning 教训）：Supervisor 正常流程不断力矩；只有 ESTOP/emergency_off 路径断力矩，且恢复时必须 `connect_bumpless`（Goal_Position 对齐 Present_Position）再重新 PARK 校验。

### 2.4 安全门禁（`grasp_gates()`，fail-closed，无旁路）

进入 MAGNET_ON/PICK/PREPLACE/PLACE/RELEASE/VERIFY_* 前逐项检查，任一失败抛 `SUPERVISOR_LOCKED`/`TRANSFORM_UNOFFICIAL`/`VISION_STALE`/`PLAN_STALE`：

1. `config.z_pick_mm is not None and config.z_place_mm is not None`（registry 实测冻结）。
2. `config.automatic_pick_place_enabled` 为 True 且该值只能来自 registry 实测区（不是 CLI 参数）。
3. `config.transform_is_official` 为 True（正式 `table_mm→robot_base_mm` + sha256 在 manifest 中登记）；contact-sheet 等价假设只允许 SCAN/PREGRASP/巡检，不允许抓放。
4. 磁头 footprint/净空门槛已在 registry 冻结（当前 UNKNOWN；gate 预留检查点）。
5. 视觉/计划时效与 source image 绑定（见 2.5）。
6. MotionResult 成功且 calibration_id 与 registry `kinematic_calibration_sha256` 对应文件一致。

**门禁不得放宽**：GUI 已锁按钮（`pick_rotate_place`），Supervisor 独立再锁；两层锁的理由写进 `integration/README.md`。禁止引入"调试旁路"参数。

### 2.5 视觉结果时效与 shadow

**时效**（docs/10 "超过有效期的视觉目标必须丢弃"）：

- `VisionBatch = {batch_id, source_image_sha256, monotonic_ns, observations[], homography_sha256}`。生产 adapter 用 `time.monotonic_ns()`（legacy `red_piece_detector.py` 用 `time.time_ns()`，PLAN Phase 1 才迁移；Supervisor  adapter 层现在就把接收时间补记为单调时戳，并在 diagnostics 标注来源时钟）。
- 任何 MOVE_PREGRASP 之前：`monotonic_ns() - batch.monotonic_ns <= config.max_vision_age_ms*1e6`，否则 `VISION_STALE` → 回 SCAN 重拍。
- PlacementPlan 绑定 `source_image_sha256`；执行第 k 片前校验 plan 的 batch 未过期、且"每放一片后重新拍照求解"（PLAN Phase 6.3）→ 旧 plan 立即 `PLAN_STALE`。

**shadow**（PLAN Phase 4.3 / Phase 6）：

- `config.shadow=True`：Supervisor 只允许 PARKED→INIT→（可选 PARK_TO_HOME）→SCAN→SHADOW_PLAN→COMPLETE/HOME_TO_PARK；`motion`/`magnet` 以 Stub 注入，任何 `send_action`/`set_enabled` 调用会 raise（防御性 `SHADOW_MODE` 错误）。
- shadow 产物：`ShadowPlanReport{solve_status, placements[], scores, solver_version, vision_batch_ref, error_code?}` 落盘 JSON，供 GUI shadow 按钮展示；feature flags 从 registry 读（`generic_vision_enabled=false, texture_enabled=false` 当前冻结值）。

### 2.6 快速最短路径扫块（`sweep_planner.py`，纯函数）

现状：legacy sweep 用 `(-y, +x)` 光栅排序（`control_red_piece_centers.py`），非最短路径。设计：

```python
def plan_sweep_order(
    pieces: Sequence[PieceRef],          # piece_id + center_table_mm + pick_point_table_mm
    start_table_mm: tuple[float, float], # HOME 上方 TCP 投影（由 FK 算一次并冻结进 config 证据）
) -> tuple[PieceRef, ...]
```

- 成本函数：相邻点 table_mm 欧氏距离（同一 Z_SAFE 平面巡航，水平距离是时间的一阶代理；z 抬升成本对所有候选相同，可省）。用**最近邻 + 2-opt 改进**（≤4 片，确定性强；也可直接全排列 4!=24 取最优，最简最确定——**建议直接全排列穷举**，n≤4 时最优且零启发式风险；n>4 退化最近邻并记录）。
- tie-break：同成本取 `(piece_id)` 字典序，保证字节级稳定（PLAN 对确定性的要求）。
- 每个扫块点：Z_SAFE(60mm) 巡航 → 目标上方 → 降至 PREGRASP(20mm) → 停 `sweep_dwell_s`（legacy 为 1.0 s）→ 回 Z_SAFE。全部走 `MotionController`（phase=SAFE/PREGRASP，speed_profile=EMPTY_NORMAL），**继承全路径预检**。
- 整个 sweep 在发送第一个目标前，对所有点的 IK 可达性做一轮"干跑"预检（对每个目标 pose 调 `kinematics.inverse`），不可达的点直接 `IK_NO_SOLUTION` 拒绝整个 sweep，而不是扫到一半失败——这是"全路径预检"在 Supervisor 层的对应物（MotionController 管单段，Supervisor 管整段序列）。
- 排序结果与理由（成本矩阵）记入事件日志 payload，便于审计。

### 2.7 未来磁吸抓取状态机接口（ seam，现在定义、现在锁定）

Supervisor 内部 `_grasp_fsm(plan_step)` 子状态：`MAGNET_ON → (settle) → VERIFY_PICK → MOVE_PREPLACE → …`，接口合同：

- 磁铁只通过注入的 `MagnetController.set_enabled(bool)`；Supervisor 是 docs/10 指定的唯一调用方；MagnetStatus 的 `on_status` 回调直接接 `JsonlEventLog`。
- 每次 `set_enabled(True)` 前记录 `magnet_on_deadline_ns = monotonic_ns() + 10s`，Supervisor 自己也要在 9.5 s 前主动断电（硬件 10 s 是兜底不是时序依据）；`on_settle_ms/off_settle_ms` 当前 UNKNOWN → settle 时长先登记 registry 保守值并标注工程假设。
- VERIFY_PICK/VERIFY_PLACE 的判定方式当前 UNKNOWN（无负载传感器）→ 这两个状态**只定义接口和错误码（PICK_VERIFY_FAILED/RELEASE_VERIFY_FAILED），实现留 `NotImplementedError` 变体并 gate 锁定**；禁止伪造"验证通过"。
- 抓取失败恢复按 docs/11：抬升 Z_SAFE → 重拍 → 重算抓取点 → 重试上限 `max_pick_retries`（registry UNKNOWN，未冻结前 gate 锁死整条路径）。

### 2.8 急停与故障恢复

- 急停入口（GUI 急停按钮已存在）：子进程收到 SIGTERM → Supervisor `finally` 路径 = `magnet.emergency_off()`（已连接时）→ 若持有负载则**保持当前位**还是抬升由状态决定（搬运中掉落规则见 docs/11：停止横向动作并抬升）→ `home_to_park()` 兜底（legacy `finally` 同款行为）→ 断连断力矩。GUI 侧另有 systemd `epuzzle-magnet-safe.service` 独立关磁铁（双保险，保持不动）。
- 任何 MotionResult 失败：记录事件 → 抬升 Z_SAFE（RECOVERY）→ 重拍重求解 → 超过重试 → FAILED → HOME_TO_PARK → PARKED。禁止在中低高度直接重试。
- ESTOP 态退出必须人工复位 + PARK 校验，防止"自动继续"。

### 2.9 JSON 事件日志（`event_log.py`）

- 格式：JSONL，每行一个符合 `data_contracts/runtime_event.schema.json` 的对象；`state`=当前 SupervisorState，`event`=迁移名/动作名（如 `transition:PARKED->INIT`、`motion_goal_accepted`、`magnet:on`），`severity`、`piece_id`、`error_code`、`payload`（自由 object：成本矩阵、MotionResult.as_mapping()、MagnetStatus、sha256 引用等）。
- 时戳一律 `time.monotonic_ns()`（注入时钟，测试可控）。
- 落盘路径 `workspace/logs/supervisor/<run_id>.jsonl`（runtime 产物，不入库的证据也复制 sha256 进 SupervisorResult）。
- 单元测试用 `jsonschema`（或现有测试的轻量校验方式，见 `workspace/tests/test_control_data_contracts.py` 的模式）校验每行。

### 2.10 如何接现有 GUI（最小 diff）

1. 新增 `workspace/tools/supervisor_run.py`：CLI `scan|sweep|pregrasp|shadow-plan|park`，参数与 `control_red_piece_centers.py` 同构（`--piece-index --output --pieces`），退出码 0/非 0 + 末行 JSON —— 页面 `_run/_control` 机制**零改动**可复用。
2. `pick_place_page.py` 增量改动（新任务内）：
   - 新增按钮"shadow 求解（不发动作）"→ `supervisor_run.py shadow-plan`，结果 JSON 路径固定 `workspace/runtime/shadow_plan.json`，页面只读展示。
   - `pick_rotate_place` 保持锁定文案；当 Supervisor 结果中出现 `SUPERVISOR_LOCKED` 时在状态栏透传（页面已按末行文本显示）。
   - 现有 repeat/sweep/pregrasp/park 按钮可逐步切到 `supervisor_run.py`；**迁移期间 legacy 工具保留不动**（context.md blast radius：不删除现有符号）。
3. 急停链路不改：SIGTERM 子进程 + magnet-safe.service + emergency_off_single_arm.py。

### 2.11 registry 追加键（configs/parameter_registry.yaml `supervisor:` 段）

```yaml
supervisor:
  global_timeout_s: 120
  max_pick_retries: "UNKNOWN"
  max_release_retries: "UNKNOWN"
  act_enabled: false
  max_vision_age_ms: "UNKNOWN"        # 工程假设值需标注并从日志冻结
  park_tolerance_deg: 4.0             # 页面实测沿用
  park_follow_tolerance_deg: 8.0
  sweep_dwell_s: 1.0
  magnet_on_settle_ms: "UNKNOWN"
  magnet_off_settle_ms: "UNKNOWN"
  automatic_pick_place_enabled: false # 与 manifest safety 段一致
  transform_official: false           # 正式 table_mm->robot_base_mm 未冻结
```

### 2.12 测试清单（先写失败测试）

| 文件 | 关键断言 |
| --- | --- |
| `test_supervisor_states.py` | 合法迁移表穷举；非 Supervisor 路径无法改 state；每态超时→RECOVERY；ESTOP 只能人工复位退出 |
| `test_supervisor_gates.py` | z_pick/z_place=None 时进入 MAGNET_ON/PICK/PLACE → `SUPERVISOR_LOCKED`（含 GUI 已锁的双层验证）；transform_is_official=False → `TRANSFORM_UNOFFICIAL`；过期 batch → `VISION_STALE`；plan 换 batch → `PLAN_STALE`；shadow 下任何 send_action/set_enabled → `SHADOW_MODE` |
| `test_sweep_planner.py` | 4 片穷举最优（对照暴力解）；同输入字节级稳定；tie-break 确定性；单点不可达 → 整体 IK_NO_SOLUTION 且零动作 |
| `test_park_sequence.py` | FakeRobot + ManualClock：起始偏离 PARK>4° → PARK_MISMATCH；插补跟随误差>8° → PARK_TRANSITION_FAILED；upright 中 shoulder_pan 恒等于 PARK 值；断力矩后恢复必须先 bumpless 对齐 |
| `test_supervisor_event_log.py` | 每行通过 runtime_event.schema.json 校验；迁移/motion/magnet/错误四类事件齐全；时间戳单调 |
| 契约测试扩展 | `supervisor_config/result.schema.json` 正/反例；`additionalProperties:false` 不放宽 |
| 回归 | 现有 8 个测试文件全绿；legacy 工具与红色检测零改动（diff 证明） |

---

## 3. Meta-prompt 合同（给实现 agent）

**goal**：在 `workspace/src/integration/` 实现上述 Supervisor 深模块与 `workspace/tools/supervisor_run.py`，含全部测试与 registry/docs 同步，默认配置下行为 = shadow + 扫描/巡检/PREGRASP 可用、磁吸抓放双层锁定。

**context/evidence**：本文件 §1（现状与行号）、§2（类型/状态/错误码/门禁/测试）。权威来源：`docs/10/11/19`、`configs/parameter_registry.yaml`、`runtime_calibration_manifest_20260731.json`、`data_contracts/runtime_event.schema.json`、`.taskman/plans/adopt-puzzle-vision-v2-2/PLAN.md` Phase 6。

**success criteria**：

1. §2.12 测试全绿 + 现有测试零回归（三条验收命令通过）。
2. PICK/PLACE 高度 None 时任何路径都无法触发 `MagnetController.set_enabled(True)`（有测试证明）。
3. shadow 模式对硬件接口零调用（Stub raise 证明）。
4. sweep 排序确定性 + 整段 IK 干跑预检（有测试证明）。
5. 事件日志每行过 schema；时间戳全部单调时钟。
6. legacy `control_red_piece_centers.py`、`control/`、GUI 急停链路零 diff。

**hard constraints**：

- 不放宽磁铁 10 s 上限、GPIO17、标定质量门禁、z 高度门禁；不给 gate 加旁路。
- 不 import/调用 legacy 工具；不从 `control_red_piece_centers.py` 复制实现（只复用其已验证参数与序列语义）。
- 五轴顺序固定；跨模块字段带 frame/单位；`additionalProperties:false` 不放宽。
- UNKNOWN 值不捏造；工程假设显式标注。

**suggested approach**：先写 `SupervisorState`/错误码/gates 的失败测试 → sweep_planner 与 park_sequence 纯逻辑 + 测试 → Supervisor 编排 + 事件日志 → supervisor_run.py 薄壳 → registry/docs 同步 → GUI 按钮增量。

**validation**：§1.6 三条命令；无 Pi 硬件时用 FakeRobot/Stub 全量单测，真机验证另列未验证项。

**stop/escalation**：若发现需要改 `control/` 才能满足需求（例如 MotionController 必须支持关节空间目标）→ 停下来经 intercom 请示，不要静默扩大改动面；若 registry 缺键导致 gate 语义不清 → 按 fail-closed 处理并在结果中标注，不请示。

**已解决问题/假设**：

- PARK↔HOME 放 integration 而非 control（保持 control 冻结）——已按此设计。
- 扫块 n≤4 直接全排列穷举——已定为建议默认。
- `max_vision_age_ms` 等 UNKNOWN：实现必须支持 None=拒绝依赖该参数的动作路径，工程占位值须标注。
- `table_mm→robot_base_mm` 正式变换缺失：扫描/巡检/PREGRASP 可继续用 manifest 登记的等价假设，抓放一律 `TRANSFORM_UNOFFICIAL`。

---

## 4. 残留风险（供 reviewer 复核）

1. `max_vision_age_ms`、`max_pick_retries`、磁吸 settle 时间为 UNKNOWN；本设计用 fail-closed 兜住，但 shadow 展示路径若误用占位值需在 UI 标注"工程假设"。
2. VERIFY_PICK/VERIFY_PLACE 无传感手段，接口已定义但判定逻辑待实物方案；冻结前整链锁死，风险可控但意味着磁吸 FSM 无法提前联调。
3. 关节空间 PARK↔HOME 放 integration 是权衡决策；若未来需要第二处关节空间运动，应重新评估是否下沉 control（需另立计划）。
4. legacy 工具与 Supervisor 双轨期间，两套 PARK 容差/补偿参数可能漂移；迁移完成前以 `parameter_registry.yaml` 为唯一事实源，Supervisor 不得自带常量副本（应从 registry/manifest 读）。