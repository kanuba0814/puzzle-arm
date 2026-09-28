# 5 自由度 SO-101 类机械臂 wrist_roll / 末端 yaw 姿态锁死运动框架研究

> 研究日期：2026-08；仅引用官方文档、官方 GitHub、原始论文等一手来源。
> 本项目背景：LeRobot SO-101 五轴（shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll），电磁吸头抓放，树莓派执行。核心痛点是 wrist_roll 跨越 ±180° 表示边界时轨迹发散（piece_02/piece_03 实测误差 40°→118°），需要"末端 yaw/姿态锁死 + 连续角度路径"的确定性运动框架。

---

## 1. 结论摘要（TL;DR）

1. **工业界对"姿态锁死"没有银弹**：MoveIt 官方明确警告 legacy `computeCartesianPath` 不执行硬约束、时间参数化不可靠，PickNik 官方指南建议改用 OMPL 约束规划（`OrientationConstraint`）或 Pilz 规划器。这些全栈方案对树莓派过重，但其**语义可以直接借鉴**。
2. **±180° 发散是角度表示问题，不是规划问题**。ROS 官方 `angles` 包（`shortest_angular_distance`）和 `joint_trajectory_controller` 的 wraparound 机制证明：只要在**误差计算**和**轨迹段插值**两处做 [−π, π] 归一化，跨 ±180° 问题即消失。这是本项目最小可行修复，纯 Python 实现只需十几行。
3. **对本项目 5DoF 平面模型，推荐自研轻量栈**：最短角差展开 + 关节空间五次样条/梯形插值 + 单圈 waypoint 拆分（每段 ≤ π）+ Ruckig（pip 安装、纯 C++ 内核、无 ROS 依赖）做 jerk 限幅。Pinocchio/Drake/MoveIt/OMPL 全栈在 Pi 5 上不值得引入。

---

## 2. 分主题调研结果

### 2.1 任务空间姿态约束 / Cartesian path

#### MoveIt `computeCartesianPath`（legacy Cartesian 插值器）

- 接口（MoveIt 1，C++）：`MoveGroupInterface::computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions, error_code)`；MoveIt 2 中 `jump_threshold` 已从 `MoveGroupInterface` 重载中移除。返回 `fraction ∈ [0,1]`，执行前必须检查 `== 1.0`。
- 每个 waypoint 是完整位姿；位置线性插值 + 姿态 SLERP 式插值。**锁姿态 = 所有 waypoint 复制同一四元数**。
- 关键警告（官方教程与维护者）：
  - `jump_threshold = 0.0`（禁用）在真机上会导致冗余关节不可预测的大幅跳变，**真机必须设阈值**；
  - 输出轨迹**无时间参数化**，速度/加速度缩放因子不生效，必须自行 retime（MoveIt 2 用 TOTG）；
  - `path_constraints` 参数不被 legacy 插值器投影到约束流形，仅经 validity 回调间接生效，**不可靠**；
  - 内部 `CartesianInterpolator::MaxEEFStep` 平移/旋转分设阈值。
- PickNik 官方博客《Guide to Cartesian Planners in MoveIt》明确**建议避免使用 legacy 插值器**，推荐 Pilz planner / Descartes / OMPL 约束规划。

#### MoveIt 硬姿态约束的正确姿势：`OrientationConstraint` + 规划器

```cpp
moveit_msgs::OrientationConstraint oc;
oc.link_name = move_group.getEndEffectorLink();
oc.orientation = current_pose.orientation;   // 锁定目标姿态
oc.absolute_x_axis_tolerance = 0.1;  // rad
oc.absolute_y_axis_tolerance = 0.1;
oc.absolute_z_axis_tolerance = 0.1;  // 想绕某轴自由旋转时设 ~2π
oc.weight = 1.0;
move_group.setPathConstraints(constraints);  // 经 OMPL 投影规划
```

- 该路径在 MoveIt 1 Noetic 也可工作（官方 issue #3327 确认）。
- **对本项目的语义迁移**："锁 wrist_roll 相对世界系的 yaw"等价于 `OrientationConstraint` 只放开 z 轴容差（`absolute_z_axis_tolerance = 2π`）、锁死 x/y。但 MoveIt 栈本身不适用 Pi（见 §3）。

**一手来源**：
- [MoveIt 2 Move Group 教程](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [MoveIt 1 Noetic 教程](https://moveit.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html)
- [`CartesianInterpolator` API](https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html)
- [Using OMPL Constrained Planning（MoveIt 官方指南）](https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html)
- [PickNik: Guide to Cartesian Planners](https://picknik.ai/cartesian%20planners/moveit/motion%20planning/2021/01/07/guide-to-cartesian-planners-in-moveit.html)
- [moveit issue #3327](https://github.com/ros-planning/moveit/issues/3327)

---

### 2.2 SO(3)/SE(3) 插值与角度解包

#### 四元数 SLERP 与双重覆盖

- Shoemake 1985 SLERP：`q(t) = sin((1−t)Ω)/sinΩ · q1 + sin(tΩ)/sinΩ · q2`，`cosΩ = q1·q2`，沿 S³ 大圆、恒定角速度。
- **双重覆盖陷阱**：q 与 −q 表示同一旋转。若 `dot(q1,q2) < 0` 必须先取负，否则走 >180° 长弧。多关键帧序列必须**沿轨迹累积取负**（保持相邻帧点积非负），否则每段独立最短路径会产生可见翻转（glTF spec issue #2073 讨论的正是此问题）。
- Eigen：`qa.slerp(t, qb)`（成员调用语法，仅最短路径）；tf2：`tf2::slerp(q1, q2, t)`（内部做符号翻转，近重合时退化为 lerp）。Drake `PiecewiseQuaternionSlerp` 是"沿轨迹累积符号修正"的官方实现样例。
- **对本项目**：本项目姿态约束本质上是"末端 pitch 锁 −90°（俯视）+ yaw 单自由度"，**根本不需要完整 SO(3) 插值**——yaw 是标量，用 §2.3 的角度解包即可。四元数 SLERP 列为参考，**不适用**（引入不必要的表示复杂度与双重覆盖风险）。

#### ROS `angles` 包（角度解包的标准工具）

- 头文件库，`normalize_angle(angle)` → [−π, π]；`normalize_angle_positive` → [0, 2π]；核心：
  ```cpp
  static inline double shortest_angular_distance(double from, double to) {
      return normalize_angle(to - from);   // 结果 ∈ [−π, π]
  }
  ```
- Python 绑定同名可用。典型用法：`err = shortest_angular_distance(current, target)`，符号即最短旋转方向，`|err| < tol` 判到位。
- **对本项目：直接适用且是核心修复**。wrist_roll 反馈误差、IK 目标差、轨迹插值三处统一用 `shortest_angular_distance` 即可消除 ±180° 发散。无需引入 ROS，公式只有一行：`((to − from + π) mod 2π) − π`。

**一手来源**：
- [ROS angles 文档（Noetic）](https://docs.ros.org/en/noetic/api/angles/html/index.html)
- [ROS 2 angles 文档](https://docs.ros.org/en/ros2_packages/jazzy/api/angles/angles.html)
- [Eigen Quaternion::slerp](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html)
- [tf2::slerp 文档](https://docs.ros.org/en/jazzy/p/tf2/generated/function_namespacetf2_1afa98ebc8d8d08247c89f906267ea65f3.html)
- [Drake PiecewiseQuaternionSlerp](https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_quaternion_slerp.html)
- Shoemake, "Animating Rotation with Quaternion Curves", SIGGRAPH 1985（原始论文）

---

### 2.3 连续/单圈 revolute joint 的 winding 与反馈误差

#### joint_trajectory_controller 的 wraparound 机制（ROS 官方现行做法）

- **Jazzy 起**：URDF 声明 `type="continuous"` 即自动启用 wraparound，`angle_wraparound` 参数被移除（迁移指南 #949）。实现内部就是 `angles::shortest_angular_distance(from, to)`，位置误差归一化到 [−π, π]，走最短路径到每个 waypoint。
- **Humble 及更早 / ROS 1**：需显式 `gains.<joint>.angle_wraparound: true`，语义相同。
- **关键 caveats（官方文档明确）**：
  1. wraparound 是**逐段**的——当前状态到首个 waypoint、相邻 waypoint 之间各走最短路径；
  2. 若要让关节转多圈，**必须插入中间 waypoint 使每段 ≤ π**，否则永远走短弧。

#### 对本项目的映射（直接适用）

本项目 wrist_roll 是 STS3215 单圈位置舵机（不是物理 continuous joint），但**表示层面**存在 ±180° 环绕（FK/IK 输出的 yaw 会跨边界）。JTC 的模式给出三条确定规则：

1. **误差计算**：收敛判定 `err = normalize(to − from)`，禁止直接 `abs(to − from)`。当前代码在 −177°→+173° 时算出 350° 误差正是此处 bug。
2. **轨迹生成**：发送给舵机的路点序列必须做**连续性展开**（unwrap）：相邻路点差归一化到 [−π, π] 后累加成连续角，发送前再按舵机量程折回（fold）。展开/折回分离是核心不变量。
3. **多段拆分**：单段跨度 > π 时插入中间 waypoint（JTC 多圈规则的同源推论）。这也与项目既有"超限即翻倍加密重试"的插值加密策略兼容——加密判据应改用展开后的关节位移。

**一手来源**：
- [joint_trajectory_controller 用户文档（Jazzy）](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [ros2_controllers Humble→Jazzy 迁移指南](https://control.ros.org/jazzy/doc/ros2_controllers/doc/migration.html)
- [JTC 参数文档（Humble，含 angle_wraparound）](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)

---

### 2.4 微分 IK / Jacobian 伺服与奇异性

#### Pinocchio DLS-CLIK（官方示例即生产模板）

官方 [CLIK 教程](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html)：

```python
pin.forwardKinematics(model, data, q)
err = pin.log(oMdes.actInv(data.oMi[JOINT_ID])).vector   # SE(3) 对数误差，6 维 twist
J = pin.computeJointJacobian(model, data, q, JOINT_ID)   # 需先 FK；world 系 6×nv
v = -J.T @ solve(J @ J.T + damp*np.eye(6), err)          # 阻尼伪逆（Levenberg–Marquardt）
q = pin.integrate(model, q, v*DT)                        # 流形感知积分
```

要点：DLS 用 `J^T(JJ^T + λI)^{-1}` 代替裸伪逆，**奇异附近关节速度有界**（误差换稳定）；`solve` 代替显式求逆；固定小阻尼 1e-12 仅示例，生产应 1e-4~1e-2 或按可操作度自适应；`computeJointJacobian` 返回 world 系，local 系用 `getJointJacobian(..., pin.LOCAL, J)`。维护级替代：[pink](https://github.com/stephane-caron/pink)（基于 Pinocchio 的 QP 任务 IK，含关节限位）。

#### Drake DiffIK

`DoDifferentialInverseKinematics(q, v, V_WE_desired, frame_E, params)`：给定末端期望空间速度解关节速度（QP，内嵌关节位置/速度限位）；`DifferentialInverseKinematicsSystem` 可作为 LeafSystem 接入实时框图。`InverseKinematics::AddOrientationConstraint(frameAbar, R_AbarA, frameBbar, R_BbarB, theta_bound)` 以 `trace(R_AB) ≥ 2cos(θ_bound)+1` 形式硬约束姿态差，`theta_bound=0` 即完全对齐——这是"锁死姿态"在数值 IK 里的标准表述。

#### 对本项目的适用性判定

- **5DoF 平面模型不需要 6D Jacobian 伺服**：末端任务空间实际为 4 维（x, y, z, yaw），pitch 恒 −90°。全 SE(3) DLS 会引入两个永远不可达的行（roll/pitch），浪费阻尼预算并掩盖真实奇异性。
- **可借鉴的部分**：DLS 思想可用在 4×5 简化 Jacobian 上（若将来做笛卡尔伺服）；当前项目的解析/半解析 IK + 路点加密已够用，**不建议引入 Pinocchio/Drake 依赖**（arm64 wheel、安装体积、启动时间均不划算）。
- **奇异性实践**：5R 平面臂奇异（肘伸直、肩正上方）应像 DLS 一样用**阻尼 + 速度上限**降级，而非报错中止；本项目确定性基线可直接在 IK 失败时翻倍加密/拒绝该 plan（现状），保留。

**一手来源**：
- [Pinocchio CLIK 教程](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html)
- [Drake InverseKinematics（AddOrientationConstraint）](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html)
- [Drake DiffIK（C++）](https://drake.mit.edu/doxygen_cxx/differential__inverse__kinematics_8h.html) / [pydrake.inverse_kinematics](https://drake.mit.edu/pydrake/pydrake.multibody.inverse_kinematics.html)
- [pink（Pinocchio 之上的 QP IK）](https://github.com/stephane-caron/pink)

---

### 2.5 约束规划（constrained planning）

#### OMPL 约束流形规划

- 核心抽象：`ompl::base::Constraint`（实现 `function(q)=0`，可选解析 `jacobian()`）+ `ompl::base::ConstrainedStateSpace`（所有采样/投影流经约束）。
- 规划器：`AtlasRRT`（Jaillet & Porta, T-RO 2013，切空间图集 + 投影回流形；关键参数 `delta/lambda/rho/alpha`）、`TB-RRT`（tangent-bundle，难流形上更稳）、`ConstrainedRRT`（投影式）。
- 实践要点：起终点必须先 `project` 到流形；解析 Jacobian 显著加速投影；`setTolerance` 影响投影收敛与图集密度。
- MoveIt 通过 `setPathConstraints` + OMPL 约束规划对外暴露（见 §2.1）。

#### 适用性判定

- **对本项目不适用（重量维度）**：OMPL 采样规划是非确定性的（含随机种子可复现但运行时间不可复现），与项目"确定性基线"原则冲突；C++ 构建链对 Pi 过重。
- **概念可借用**：本项目的"约束流形"其实只是一条直线——`(x, y, z) ∈ 桌面区域, pitch = −90°, yaw = 自由标量`。约束检验 = 每个路点 FK 后断言 pitch 偏差与 z 高度，O(1) 完成，无需流形投影。

**一手来源**：
- [OMPL Constrained Planning 教程](https://ompl.kavrakilab.org/constrainedPlanning.html)
- [ompl::base::Constraint](https://ompl.kavrakilab.org/classompl_1_1base_1_1Constraint.html)
- [ompl::geometric::AtlasRRT](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1AtlasRRT.html)

---

### 2.6 时间参数化（Ruckig 等）

#### Ruckig（官方一手）

- [github.com/pantor/ruckig](https://github.com/pantor/ruckig)，[docs.ruckig.com](https://docs.ruckig.com)；Community 版 BSD-2，`pip install ruckig` 有官方 Python 绑定，**无 ROS 依赖**，纯 C++ 内核。
- 接口三件套：`Ruckig<DOFs>(cycle_time)`、`InputParameter`（当前/目标 pos/vel/acc + max vel/acc/jerk）、`OutputParameter`；循环 `update(input, output) == Result::Working`，每拍 `output.pass_to_input(input)`。
- 在线 OTG：每个控制周期从任意当前状态算出到目标状态的 jerk 限幅时间最优轨迹；目标中途改变自动重规划；省略 `max_jerk` 则退化为加速度限幅（梯形/S 曲线）。Franka Emika 官方控制器即使用 Ruckig。
- Community 版不含本地中间 waypoint（走云 API），多段路径需自行分段调用。

#### 对本项目的适用性（适用，推荐）

- 当前项目用固定速度档（EMPTY_NORMAL=250 等）+ 路点 dwell 的原始时间参数化，2026-08-01 实测已暴露"宽跟随门槛下 Goal_Position 以最快速度连发"问题。Ruckig 可以：
  1. 对每段关节空间路径生成 jerk 限幅轨迹，消除路点间速度突变；
  2. `update` 每拍从**实测位置**出发重算，天然吸收舵机跟踪滞后；
  3. Pi 上确定性：每拍计算是固定复杂度的解析解（ruckig 的 OTG 是闭式解，微秒级），满足确定性基线。
- 替代/补充：若不想加依赖，五次多项式（quintic）标量插值 30 行 Python 即可，但无在线重规划能力。Ruckig 是"够用且轻"的最优选。

**一手来源**：
- [Ruckig GitHub](https://github.com/pantor/ruckig) / [Ruckig 文档](https://docs.ruckig.com) / [PyPI ruckig](https://pypi.org/project/ruckig/)

---

### 2.7 树莓派确定性轻量实现方案（综合建议）

按依赖从零到重排序，推荐栈 = 第 1~3 层全部自研 + 第 4 层 Ruckig：

| 层 | 功能 | 实现 | 规模 |
|---|---|---|---|
| 1 | 角度解包 | `shortest_angular_distance = ((to−from+π) mod 2π) − π`（ROS angles 公式） | ~5 行 |
| 2 | 连续展开/折回 | 路点序列 unwrap → 连续角轨迹；发送前 fold 到舵机量程；误差判定一律用展开域 | ~20 行 |
| 3 | 姿态锁检验 | 每路点 FK 后断言 `|pitch − (−90°)| < tol`、`z ≥ z_min`（OMPL 约束思想的 O(1) 特例） | ~15 行 |
| 4 | 时间参数化 | Ruckig（pip，无 ROS）或五次样条 | 单一 wheel |
| 5 | 多段拆分 | 单段任一关节展开位移 > π → 插入中间点（JTC 规则）；与现有"超限翻倍加密"合并 | 改判据 |

**明确不适用本项目的**：MoveIt/MoveIt 2 全栈（ROS 依赖 + 非确定性规划器）、OMPL 采样规划（非确定性）、Drake（安装体积/启动时间）、完整 6D 四元数 SLERP（任务是标量 yaw）、Pinocchio 全 SE(3) CLIK（可用 4×5 简化 DLS 替代，且非必需）。

**修复 wrist_roll ±180° 发散的最小改动**（对照 2026-08-01 事故）：
1. 收敛误差计算改 `shortest_angular_distance(goal, present)`——−177°→+173° 的误差从 350° 变为 10°；
2. IK 求解后对 5 轴路点做 unwrap，保证相邻路点各轴差 ≤ π；
3. 多段路径按展开位移（而非端点差）做加密判据；
4. 保留现有 `max_joint_step_deg` 硬门禁不放宽（项目规则），加密重试即可。

---

## 3. 一手来源清单

**保留**：
- [MoveIt 2 Move Group 教程](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html) — Cartesian path 官方接口与真机警告
- [MoveIt OMPL 约束规划指南](https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html) — 硬姿态约束的官方做法
- [CartesianInterpolator API](https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1CartesianInterpolator.html) — MaxEEFStep/JumpThreshold 内部语义
- [PickNik Cartesian Planners 指南](https://picknik.ai/cartesian%20planners/moveit/motion%20planning/2021/01/07/guide-to-cartesian-planners-in-moveit.html) — 维护方对 legacy 插值器的否定结论
- [moveit issue #3327](https://github.com/ros-planning/moveit/issues/3327) — Noetic 兼容性与替代方案确认
- [ROS angles 文档](https://docs.ros.org/en/noetic/api/angles/html/index.html) — shortest_angular_distance 标准公式
- [joint_trajectory_controller 用户文档](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) — continuous joint wraparound 官方机制
- [ros2_controllers 迁移指南](https://control.ros.org/jazzy/doc/ros2_controllers/doc/migration.html) — URDF continuous 自动 wraparound
- [Eigen Quaternion](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html) / [tf2::slerp](https://docs.ros.org/en/jazzy/p/tf2/generated/function_namespacetf2_1afa98ebc8d8d08247c89f906267ea65f3.html) — SLERP 实现语义
- [Drake PiecewiseQuaternionSlerp](https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_quaternion_slerp.html) — 轨迹级符号累积修正样例
- [Pinocchio CLIK 教程](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html) — DLS 微分 IK 官方模板
- [Drake InverseKinematics](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html) / [DiffIK](https://drake.mit.edu/doxygen_cxx/differential__inverse__kinematics_8h.html) — 姿态硬约束与 QP 伺服
- [OMPL Constrained Planning](https://ompl.kavrakilab.org/constrainedPlanning.html) / [AtlasRRT](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1AtlasRRT.html) — 约束流形规划
- [Ruckig GitHub](https://github.com/pantor/ruckig) / [docs.ruckig.com](https://docs.ruckig.com) — jerk 限幅 OTG
- Shoemake 1985, "Animating Rotation with Quaternion Curves", SIGGRAPH — SLERP 原始论文
- Jaillet & Porta, "Path Planning Under Kinematic Constraints by Rapidly Exploring Manifolds", T-RO 2013 — AtlasRRT 原始论文

**丢弃**：CSDN/博客类二手转载（内容重复且非一手）、Yahboom 教程（厂商二手）、各大学课件 PDF（非规范来源）、Unity/Blender 论坛帖（非机器人上下文）。

## 4. 未能确认 / 后续步骤

- Ruckig 在 Pi 5 arm64 上的官方 wheel 可用性未在本机验证（仅确认 PyPI 有包）；引入前需在 Pi 上 `pip install ruckig` 并断言 import + 单次 `update` 耗时。
- 本项目 IK 是否已做 unwrap 需读 `workspace/src/control/` 现状确认（本研究只读不动代码）。
- 若未来需要 4×5 简化 DLS 笛卡尔伺服，Pinocchio 公式可直接移植（solve 一个 4×4 阻尼系统），无需引入库。
