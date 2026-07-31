# control

单台原 SO-101 follower 改装五轴机械臂的确定性控制模块。实现配置驱动 FK/IK、全路径预检、软限位、实际位置收敛和 `MotionResult`；电磁铁是独立模块，不属于五轴动作。

接口以 `docs/10_接口与数据契约.md`、`docs/23_单臂运动控制重构.md` 和 `data_contracts/` 为准。

## 运动控制约束

- 唯一关节顺序：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`，舵机 ID 1–5。
- `MotionController.from_calibration_file()` 是真实运行构造入口；未通过质量门禁的标定不能执行。
- `MotionGoal` 只接受 `robot_base_mm`；table/image 坐标必须由上游显式转换。
- 所有路径点先完成 IK、软限位和关节步长预检，再允许发送第一个目标。
- 每个目标必须由实际五轴读回连续稳定收敛；写入 Goal Position 或 Moving=0 均不能单独判成功。
- 控制模块不调用电磁铁；Supervisor 根据成功的 MotionResult 决定吸放。

## 当前 accepted 标定

- 运动学：`workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json`
- SHA256：`c207bae180e187b6c639cc848320195dafce1f468920a0f2eba7f00c621eab38`
- 独立 holdout：XY RMS `2.480912 mm`、XY max `3.406456 mm`、|Z|max `2.458668 mm`。
- 纸面→基座：`workspace/calibration/contact_sheet_to_robot_base_20260731.json`；不得把该纸面 frame 静默当成正式 `table_mm`。
- 软限位采用同机机械端点内缩后的竞赛实用范围，J5 额外限制线缆扭转；它仍不是完整自碰包络。
- 实机冻结高度（相对 `calibration_sheet_mm`）：Z_SAFE `60 mm`、PREGRASP `20 mm`；`10 mm` 经现场观察过低或碰纸，PICK 接触高度仍为 `UNKNOWN`。
- 竞赛精确终点模式使用 P=24、Acceleration=30、沿径向远离机械臂 3 mm、命令 pitch `-87.5°`；D 点负载读回 `-90.03°` 且现场重复确认。快速巡航的 5°/单样本门槛不得用于最终抓放判定。

## 电磁铁控制验收条件

- 树莓派使用 `/dev/gpiochip0` 的 BCM GPIO17（line offset 17）。
- 继电器为低电平触发；请求 GPIO 后的初始状态必须为 3.3 V 高电平/断电。
- 对外只暴露 `MagnetController.set_enabled(bool)` 状态接口，不暴露裸 GPIO。
- 每次通电最多保持 10 秒，超时自动断电并报告 `timeout`。
- `close()`、急停和异常清理必须先断电，再释放 GPIO line。
- 同一进程独占 GPIO；请求失败时禁止降级为“假成功”。
- 额定电压、电流和温升仍为 `UNKNOWN`，10 秒是内部工程安全上限，不是实测连续工作能力。
