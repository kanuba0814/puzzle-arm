# 进度：wrist_roll 姿态锁死开源框架研究

- [x] 任务空间姿态约束 / Cartesian path（MoveIt computeCartesianPath + OrientationConstraint）
- [x] SO(3)/SE(3) 插值与角度解包（tf2/Eigen slerp、ROS angles）
- [x] 连续关节 winding 与反馈误差（joint_trajectory_controller wraparound）
- [x] 微分 IK / Jacobian 伺服与奇异性（Pinocchio DLS-CLIK、Drake DiffIK）
- [x] 约束规划（OMPL ConstrainedStateSpace / AtlasRRT、Drake AddOrientationConstraint）
- [x] 时间参数化（Ruckig OTG）
- [x] 树莓派轻量确定性实现建议
- [x] 撰写最终报告
