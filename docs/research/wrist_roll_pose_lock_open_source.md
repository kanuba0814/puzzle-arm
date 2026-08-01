# wrist_roll 末端姿态锁：开源方案调研

> 日期：2026-08-01。只采用官方文档、官方源码或作者维护仓库；本报告用于选型，不代表已完成实机验收。

## 结论

本项目不需要引入 MoveIt、OMPL、Drake、Pinocchio、ROS 或 Ruckig。它的低空任务只有一个很窄的特例：固定 `x/y`，沿 `robot_base_mm` 的 Z 轴移动 30 mm，同时保持工具 pitch 和世界系 yaw。现有解析 IK 加 5 mm 笛卡尔路点、全路径预检和编码器读回足够。

开源方案的价值主要是提供正确语义：

1. MoveIt/Pilz 的 `LIN`：直线笛卡尔运动，起终姿态相同即保持姿态；本项目只借用语义。
2. MoveIt `OrientationConstraint`：姿态约束必须对整条路径成立，而不只是终点；本项目以逐路点 FK 断言实现。
3. ROS `angles`：任务空间 yaw 是周期量，可用最短角差比较；但本项目 J5 是**有界单圈舵机**，不能照搬 continuous joint 的 wraparound。
4. ROS `joint_trajectory_controller` 只对 URDF `continuous` 关节自动 wrap；这反向说明 J5 不应被当作 continuous joint。
5. Ruckig 能做同步 jerk-limited 时间参数化，但当前 30 mm/5 mm 的六段低速运动不值得增加新依赖。

## 对 J5 最关键的纠偏

### 周期姿态与有界关节必须分开

- 工具 yaw 是周期量：`+179°` 与 `-179°` 的任务空间误差是 `2°`。
- J5 是 STS3215 单圈位置舵机，软件限位 `[-180°, 180°]`；普通位置模式并无官方保证会跨 `0/4095` 边界走最短弧。
- 因此：
  - **任务空间 yaw 误差**可以做 wrap；
  - **J5 命令、步长和收敛误差**必须在预先选定的有界分支上做普通绝对差；
  - 禁止把 `177° -> -178°` 当成 5° 的可发送关节步长；应在全路径预检时换等价分支，找不到就零发送拒绝。

ROS 官方 `angles` 同时提供普通 `shortest_angular_distance` 和带关节限位的 `shortest_angular_distance_with_large_limits`，正是对这两类语义的区分。ROS 2 `joint_trajectory_controller` 也仅为 URDF `continuous` 关节启用自动 wraparound。

### LeRobot v0.6.0 的角度表示

LeRobot v0.6.0 `MotorNormMode.DEGREES` 使用电机全分辨率换算角度，并以 `(range_min + range_max) / 2` 作为零点。对 J5 的 `0..4095` 范围，接口自然给出约 `[-180°, 180°]` 的主值表示；这不是无限连续角，也不构成跨端点最短路保证。

### FEETECH 一手资料

STS3215 官方资料确认 12-bit、4096 counts/360°。FEETECH 官方 SDK 的 `WritePosEx()` 只是把目标位置写入寄存器，不在主机侧计算跨 0/4095 的最短路径。因此设计必须避免依赖未文档化的固件绕回行为。

## 可借鉴但不引入的方案

### MoveIt / Pilz

Pilz `LIN` 生成直线笛卡尔轨迹，并对姿态做 SLERP；起终姿态相同即保持姿态。MoveIt `OrientationConstraint` 定义目标姿态和各轴容差，要求路径点满足约束。它们适合完整 ROS/URDF/碰撞场景，但本项目的五轴解析模型和固定 30 mm 低空段无需整套依赖。

### OMPL、Pinocchio、Drake

- OMPL 约束流形规划适合复杂障碍物和高维约束，采样规划对本项目过重。
- Pinocchio/Drake 的 DLS 或 QP 微分 IK 适合通用 SE(3) 伺服；本项目只有解析可解的 `x/y/z/pitch/yaw` 五变量。
- 若未来引入任意曲线低空伺服，再评估 5×5 Jacobian/DLS；当前不做。

### Ruckig

Ruckig 提供逐自由度速度、加速度、jerk 限制和同步策略。它适合后续统一时间参数化，但当前已有速度档、5 mm 小路点和严格读回；在 arm64 wheel、周期稳定性完成实测前不进入确定性基线。

## 本项目采用的最小算法

1. 到达接触点正上方 30 mm。
2. 从实际五轴读回计算并锁存当前世界系 yaw 与 pitch。
3. 生成 6 个、每步 5 mm 的纯 Z 路点；每点 `x/y/yaw/pitch` 不变。
4. 解析 IK 后全路径检查关节限位、J5 15°限位余量、10°最大关节步长。
5. 逐点发送并读回；FK 验证 yaw/pitch 误差均不超过 ±5°。
6. 超差时停止深入，并沿已执行路点逆序退回入口。

## 一手来源

- MoveIt `OrientationConstraint`：<https://moveit.picknik.ai/main/api/html/classkinematic__constraints_1_1OrientationConstraint.html>
- MoveIt OMPL 约束规划指南：<https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html>
- Pilz Industrial Motion Planner `LIN`：<https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html>
- ROS `angles`：<https://docs.ros.org/en/jazzy/p/angles/angles.html>
- `shortest_angular_distance_with_large_limits`：<https://docs.ros.org/en/jazzy/p/angles/generated/function_angles_8h_1a0a260194cd877fe061195116274f1d85.html>
- ROS 2 Joint Trajectory Controller：<https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- LeRobot v0.6.0 `motors_bus.py`：<https://raw.githubusercontent.com/huggingface/lerobot/v0.6.0/src/lerobot/motors/motors_bus.py>
- FEETECH STS3215 官方产品资料：<https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html>
- FEETECH 官方 Arduino SDK `SMS_STS.cpp`：<https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.cpp>
- Ruckig：<https://github.com/pantor/ruckig>、<https://docs.ruckig.com/>
- Pinocchio CLIK 官方示例：<https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc-b-examples_i-inverse-kinematics.html>
- Drake Orientation Constraint：<https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html>

## 尚需实测确认

- 以安全高度、空载、低速执行 J5 `±10°`，确认模型中 J5 对世界系 yaw 的贡献符号。
- 30 mm 低空段实际 `x/y` 漂移门槛需由日志和现场观察冻结；设计初值为 2 mm 工程目标，不写成实测值。
- 不测试、不依赖 STS3215 跨 0/4095 边界的固件最短路行为。
