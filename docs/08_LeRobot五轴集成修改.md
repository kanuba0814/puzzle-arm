# 08 LeRobot 五轴集成修改

## 版本

固定使用 LeRobot v0.6.0。不要以 `main` 分支作为比赛环境，因为接口可能继续变化。

## 推荐方式：独立插件

创建独立安装包，名称以 `lerobot_robot_` 开头，例如：

```text
lerobot_robot_so101_5d/
  pyproject.toml
  lerobot_robot_so101_5d/
    __init__.py
    config_so101_5d.py
    so101_5d.py
```

官方 v0.6.0 BYOH 文档说明，`lerobot_robot_` 前缀插件可以被 CLI 自动发现，无需修改 LeRobot 本体。

## follower 修改点

基于官方 `SOFollower` 的逻辑实现自定义类：

- 电机表仅包含 ID 1-5。
- 删除 `gripper: Motor(6, ...)`。
- `_motors_ft` 自动生成 5 个 `.pos` 字段。
- `observation_features` = 五个关节位置 + 相机。
- `action_features` = 五个关节位置。
- `configure()` 删除或跳过 `motor == "gripper"` 的专用保护配置。
- `calibrate()` 只遍历五个电机；wrist_roll 仍按完整一圈处理。
- `send_action()` 只接受已声明的五个 `.pos` 键，并增加白名单检查。

官方 v0.6.0 源码中，motor feature 是从 `self.bus.motors` 自动生成，因此删除第六电机后特征会自然成为 5 维。

## leader/遥操作修改点

### 有五轴 leader

- 创建 `lerobot_teleoperator_so101_5d` 插件。
- 电机 ID 1-5，与 follower 键名完全一致。
- 重新标定。

### leader 仍有第六夹爪舵机

两种方式：

1. 首选：第六轴只作为人工磁铁触发输入，记录为独立事件，不进入 ACT action。
2. 不推荐：将其过滤掉后仅输出五个关节；必须保证 `action_features` 与实际 `get_action()` 完全一致。

## 数据维度

首选数据契约：

- `observation.state.shape = (5,)`
- `action.shape = (5,)`
- `magnet_state` 作为独立日志/观察字段，不是 action。

训练、回放、推理使用同一个 robot ID、字段名称、相机键名和顺序。

## 禁止混用

- 六维旧数据与五维新数据；
- 含 `gripper.pos` 的旧 checkpoint；
- 六轴标定文件；
- 不同相机键名；
- 角度制和 `RANGE_M100_100` 归一化模式混用。

## 验收

1. CLI 能发现自定义 robot type。
2. 未连接硬件时即可读取 feature contract。
3. 五电机 setup、calibrate、teleoperate、record、replay 均通过。
4. 数据集元数据中 action 维度为 5。
5. 训练生成的 ACT checkpoint 在五轴 follower 上加载无尺寸不匹配。
