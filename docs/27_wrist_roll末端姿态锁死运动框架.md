# 27 wrist_roll 末端姿态锁死运动框架

> 状态：设计已确认，尚未实现、尚未实机验收。不得据此直接执行带碎片动作。

## 1. 目标与验收条件

目标只覆盖 PICK/PLACE 接触附近：

- 接触前最后 30 mm：磁头沿 `robot_base_mm` Z 轴垂直下降；
- PICK/PLACE 后最初 30 mm：沿原路径垂直抬升；
- 低空段中工具世界系 yaw 与 pitch 均保持锁定；
- yaw、pitch 允许误差均为 ±5°；
- 每 5 mm 一个路点，共 6 段；
- J5 距软件限位必须至少保留 15°；
- 任何路点预检失败时，在第一条位置命令前整体拒绝；
- 运行中超差时停止深入，并沿同一锁定路径逆序退回 30 mm。

验收必须同时满足：

1. 规划路点 `x/y` 恒定，Z 单调，每步 5 mm；
2. 规划路点 yaw/pitch 恒定；
3. 每个实际读回路点的 yaw/pitch 误差不超过 ±5°；
4. J5 全程不跨 `[-180°, 180°]` 表示边界，且保留 15°余量；
5. PICK/PLACE 下降与抬升均覆盖；
6. 超差恢复路径经过相同检查；
7. 完成后仍执行正式 PARK、断力矩和磁铁状态验收。

## 2. 已确认语义

### 2.1 “姿态锁死”不是“ID5 数值不变”

锁定对象是工具相对桌面/世界坐标的俯视方向。若 `shoulder_pan` 改变，ID5 必须补偿；单独固定 `wrist_roll.pos` 不能保证世界系朝向不变。

但本设计的低空段固定 `x/y`，解析 IK 下 `shoulder_pan` 理论上不变，因此 J5 也应基本不变。框架仍按世界系 yaw 校验，避免把偶然的关节不变误称为姿态正确。

### 2.2 pitch 与 yaw 分工

- ID2–ID4 联合保持磁铁工作面垂直向下；
- ID1 与 ID5 联合决定俯视世界系 yaw；
- ID5 不负责磁铁工作面的垂直度。

### 2.3 没有绝对 0°

磁头没有物理基准标记，因此不能宣称“A4 +X 为绝对 0°”。每次进入低空段时，从最后一个已收敛的实际五轴读回计算当前 yaw/pitch，并锁存为本段目标。

规范术语是 **入口姿态保持（entry-pose hold）**，避免再写“固定全局零度”。

## 3. 不变量

### 3.1 坐标和姿态

- 位置只使用 `robot_base_mm`；
- 低空路径：`x=x_entry`、`y=y_entry`、`z` 每步变化 5 mm；
- `pitch=pitch_lock`；
- `yaw=yaw_lock`；
- 任务空间 yaw 是周期量，误差使用最短角差；
- 有界 J5 关节不是周期命令量，关节误差和步长不做 wrap。

### 3.2 J5 单圈语义

STS3215 普通位置模式按单圈有界关节处理：

- 合法软件范围：`[-180°, 180°]`；
- 低空可用范围：`[-165°, 165°]`（15°余量）；
- 禁止把 `177° -> -178°` 当作 5°关节步长；
- 禁止发送前把连续角重新 fold 到另一端；
- 高空规划若找不到同一有界分支，必须拒绝或在无碰撞安全高度重新选姿态，不能在低空回绕。

### 3.3 失败语义

- 姿态约束是硬门禁，不能运行中关闭编码器读回；
- ±5°是唯一允许带，不允许“先继续再说”；
- 预检失败：零发送；
- 跟踪超差但通信正常：逆序退回已执行路点；
- USB/总线不可用：沿用现有 PICK/PLACE fail-closed，不假装已退回；
- ESTOP 优先级高于自动退回。

## 4. 运动学符号统一

### 4.1 当前问题

当前运动学写成：

```text
yaw_fk = pan_phys + wrist_roll_phys
```

但实测方向表明，标准俯视逆时针为正时应表达为：

```text
yaw_robot = yaw_zero + pan_phys - wrist_roll_phys
```

目前四片工具在调用点使用 `2*Δpan-Δyaw` 补丁绕过符号错误。完整框架禁止“模型一套、调用方再补一套”。

### 4.2 设计

在运动学标定中显式加入：

```text
wrist_roll_yaw_sign = -1
tool_yaw_zero_deg = 0.0  # 无物理标记，仅作相对参考；绝对值不宣称已标定
```

统一公式：

```text
yaw_robot = wrap(
    tool_yaw_zero_deg
    + pan_phys
    + wrist_roll_yaw_sign * wrist_roll_phys
)
```

IK 使用同一公式反解 J5。`joint_directions` 继续只描述电机逻辑角到物理关节角的映射；不得再拿它隐式承担工具 yaw 轴方向。

实施前先做安全高度、空载、低速 J5 `±10°` 方向验收。验证通过后才删除：

- `run_assembly_4pieces.py` 的 `2*Δpan-Δyaw` 补偿；
- `_adapter_123.py` 中等价的调用点补丁；
- 所有基于旧 `pan+roll` 自洽计算的报告逻辑。

## 5. 最小模块设计

不增加第三方依赖，不引入第二套规划器。保持外部 seam：

```text
MotionGoal(robot_base_mm) -> MotionResult
```

只新增一个可选路径约束值对象：

```python
@dataclass(frozen=True)
class LowHeightPoseLock:
    yaw_robot_deg: float
    pitch_robot_deg: float
    entry_xy_robot_mm: tuple[float, float]
    vertical_distance_mm: float = 30.0
    cartesian_step_mm: float = 5.0
    yaw_tolerance_deg: float = 5.0
    pitch_tolerance_deg: float = 5.0
    wrist_limit_margin_deg: float = 15.0
```

`MotionGoal` 只增加：

```python
path_constraint: LowHeightPoseLock | None = None
```

复杂逻辑全部留在现有 `MotionController` 内部：

- 生成纯 Z 路点；
- 用修正后的解析 IK 求五轴；
- 选择 J5 有界分支；
- 全路径姿态/限位预检；
- 逐点实际 FK 校验；
- 保存已执行路点供逆序恢复。

调用方只需知道是否带 `LowHeightPoseLock`，不接触 angle unwrap、J5 分支或恢复路径细节。

## 6. 状态流程

### 6.1 PICK 下降

```text
SAFE
  -> 到达 pick 正上方 30 mm
  -> 从 MotionResult.final_joint_deg 锁存 entry yaw/pitch
  -> 验证 pitch 相对垂直方向在 ±5°内
  -> 验证 J5 限位余量 >=15°
  -> 预检 6 个 5 mm 纯 Z 路点
  -> LOCKED_DESCENT_EMPTY
  -> PICK 接触
  -> MAGNET_ON
```

### 6.2 PICK 抬升

```text
MAGNET_ON
  -> 复用同一个 lock（禁止重新捕获并带入新误差）
  -> LOCKED_ASCENT_HOLDING，逆向走 6 个路点
  -> 到达 +30 mm 后才允许普通 SAFE 运动
```

### 6.3 PLACE 下降/抬升

在安全高度完成计划要求的碎片旋转；到达 place 正上方 30 mm 后重新锁存实际 yaw/pitch：

```text
ROTATE_AT_SAFE_Z
  -> place 正上方 +30 mm
  -> acquire place lock
  -> LOCKED_DESCENT_HOLDING
  -> PLACE 接触
  -> MAGNET_OFF
  -> 复用 place lock 执行 LOCKED_ASCENT_EMPTY
  -> +30 mm 后恢复普通 SAFE 运动
```

锁只覆盖四段：

1. PICK 最后 30 mm 下降；
2. PICK 最初 30 mm 抬升；
3. PLACE 最后 30 mm 下降；
4. PLACE 最初 30 mm 抬升。

高空平移和目标上方原地旋转保持现有策略，不扩展姿态锁范围。

## 7. 规划算法

### 7.1 捕获

从 approach 终点的实际五轴读回构造 `JointVector`，经修正后的 FK 得到：

```text
yaw_lock = actual_pose.yaw_deg
pitch_lock = actual_pose.pitch_deg
entry_xy = planned approach x/y
```

入口门禁：

- `angular_error(pitch_lock, -90°) <= 5°`；
- 实际 FK 的 `x/y` 与计划 approach `x/y` 偏差小于冻结门槛；初始工程目标 2 mm，实测前标记 pending；
- J5 位于 `[-165°,165°]`；
- 当前 observation 新鲜且五轴字段完整。

### 7.2 六段纯 Z 路径

下降时：

```text
z_i = z_entry - 5*i, i=1..6
```

抬升时反向。每个路点：

```text
pose_i = (entry_x, entry_y, z_i, pitch_lock, yaw_lock)
q_i = corrected_analytic_IK(pose_i, reference=q_{i-1})
```

J5 候选选择：

1. 在 yaw 允许带 `yaw_lock ±5°` 内求可行解；
2. 只保留 J5 `[-165°,165°]` 的解；
3. 使用普通绝对差选择距上一 J5 最近者；
4. 同分时选择离限位更远者；
5. 无候选则整段拒绝。

由于 x/y 和 yaw 固定，正常结果应使 J1/J5 基本恒定。预检增加断言：

```text
max(|ΔJ1|, |ΔJ5|) <= numerical_lock_step_limit
```

初始工程门槛建议 0.5°；这只是发现模型/分支错误的内部门禁，不是实测精度指标。

### 7.3 全路径预检

第一条命令前检查全部路点：

- 数值有限；
- IK 可达；
- 五轴软限位；
- J5 15°余量；
- 相邻关节步长不超过现有 10°；
- FK 回放的 x/y 恒定；
- FK 回放 yaw/pitch 相对 lock 不超过数值容差；
- Z 单调且总位移恰为 30 mm；
- phase 高度门禁成立。

## 8. 执行与读回

不使用 Ruckig。复用现有速度档和 `Goal_Velocity`：

| 低空段 | 速度档 |
|---|---|
| PICK 下降，空载 | `EMPTY_SLOW` |
| PICK 抬升，持物 | `HOLDING_SLOW` |
| PLACE 下降，持物 | `HOLDING_SLOW` |
| PLACE 抬升，空载 | `EMPTY_SLOW` |

低空锁定段禁用中间点 `follow_tolerance`。每个 5 mm 路点：

1. 发送完整五轴目标；
2. 实际读取五轴；
3. 用普通绝对差检查有界关节目标；
4. FK 得到实际 x/y/z/pitch/yaw；
5. 检查 yaw/pitch 均在 ±5°；
6. 连续两个样本合格后进入下一点。

任务空间 yaw 误差使用 wrap；J5 关节误差不 wrap。两者不得复用同一个 `angle_error()`。

## 9. 超差恢复

### 9.1 通信正常

任一实际路点 yaw/pitch 超差：

1. 不再发送更深路点；
2. 保留已经成功执行的路点栈；
3. 沿该栈逆序、使用同一个 lock 退回 approach；
4. 恢复过程继续执行 ±5°姿态校验；
5. 返回结构化错误 `POSE_LOCK_TRACKING_ERROR`。

磁铁语义：

- 尚未吸合：保持 OFF；
- 已吸合且通信正常：退回期间保持 ON，避免低空掉落；
- 到达 approach 后进入人工恢复锁，不自动继续下一片；
- ESTOP、控制器级事故或退回失败仍服从最高级安全流程，并记录磁铁最终状态是否可确认。

该策略仅用于姿态跟踪超差；不得改变现有 PICK/PLACE USB 断连 fail-closed 规则。

### 9.2 结构化错误

新增错误码建议：

- `POSE_LOCK_PRECHECK_FAILED`
- `POSE_LOCK_LIMIT_MARGIN`
- `POSE_LOCK_TRACKING_ERROR`
- `POSE_LOCK_RECOVERY_FAILED`

错误结果至少记录：lock yaw/pitch、最大实际误差、失败路点、已执行路点数、J5 最小限位余量和是否成功退回。

## 10. 测试与上线顺序

### 10.1 离线单测

1. J5 对世界系 yaw 的贡献符号：J1 与 J5 分别 ±10°的已知方向；
2. 修正后 FK/IK round-trip；
3. `177° -> -178°` 被当作 355°有界关节路径并拒绝；
4. 任务空间 `+179°/-179°` yaw 误差为 2°；
5. 30 mm/5 mm 恰好生成 6 段；
6. 六段 x/y/yaw/pitch 恒定、Z 单调；
7. J5 15°余量门禁；
8. 任一路点不可达时发送数为 0；
9. 第 N 点超差时只逆序执行前 N−1 个已确认路点；
10. PICK/PLACE 四个低空方向均启用 lock；
11. 删除调用点旋转补偿后，四片历史计划的最终物理旋转结果保持一致。

### 10.2 Shadow/日志回放

对现有绿色、白色扑克和历史四片计划回放：

- 全路径均在 J5 有界分支内；
- 不再出现 `-177° -> +173°` 形式的低空跨边界命令；
- 低空计划 J1/J5 近似常量；
- 旋转只发生在安全高度显式 ROTATE 段。

### 10.3 实机验收（必须逐级授权）

1. 安全高度、空载、低速 J5 `±10°`，冻结方向证据；
2. 空载四象限各做一次 30 mm 下/上，磁铁 OFF；
3. 日志验证每路点 yaw/pitch ≤5°，并测量 x/y 漂移；
4. 单片 PICK 下/上；
5. 单片 PLACE 下/上；
6. 四片完整回归；
7. 正式 PARK、五轴 Torque_Enable=0、磁铁 OFF、无运动锁才算完成。

## 11. 实施切片

为降低风险，按以下小步实施，不一次性重写控制器：

1. **模型真源**：加入 J5 yaw 符号字段、离线测试；暂不启用真机。
2. **方向验收**：安全高度 ±10°，确认后删除调用点补丁。
3. **有界角语义**：J5 步长/误差不 wrap，加入 15°余量和历史事故回放。
4. **低空锁规划**：新增 `LowHeightPoseLock`，仅 shadow。
5. **低空锁执行/恢复**：接入四个 30 mm 段，先空载再带片。
6. **默认启用**：完整四片验收通过后再切换生产默认。

每一步都必须保持旧确定性基线可通过配置关闭，不与 ACT 耦合。

## 12. 明确不做

- 不引入 MoveIt、ROS、OMPL、Drake、Pinocchio 或 Ruckig；
- 不实现通用 6D 姿态规划；
- 不把 J5 改成 continuous joint；
- 不关闭编码器读回；
- 不在低空自动回零或跨 ±180°；
- 不改变现有 10°最大关节步长；
- 不把无物理标记的入口方向称为绝对全局 0°；
- 不让 vision/puzzle 直接发送姿态或关节命令。

相关开源算法依据见：`docs/research/wrist_roll_pose_lock_open_source.md`。
