# robot_plugin

独立 LeRobot v0.6.0 五轴 robot 插件；不直接修改安装目录。

当前实际拓扑只有一台由原 SO-101 follower 改装的机械臂，因此只使用 `SO101Follower5D` robot adapter：舵机 ID 1–5，无 leader、无 teleoperator、无 `gripper.pos`。电磁铁不属于该插件动作字段。

五轴电机 calibration 可用 `workspace/tools/calibrate_single_arm.py` 交互生成；同一台 follower 的已核验历史文件使用 `workspace/tools/install_single_arm_motor_calibration.py` 严格五轴导入并回读。TCP/运动学 calibration 另存，不得混用。

真实运行优先使用 `connect_bumpless()`：它在断力矩状态先把五轴目标对齐当前读数，经回读一致后才使能力矩，避免继承旧 `Goal_Position` 引起跳变。

接口以 `docs/10_接口与数据契约.md`、`docs/23_单臂运动控制重构.md` 和 `data_contracts/` 为准。
