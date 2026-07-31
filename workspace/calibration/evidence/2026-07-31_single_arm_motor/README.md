# 2026-07-31 单臂 LeRobot 电机标定证据

## 身份与来源

- 用户于 2026-07-31 明确确认：包内 `my_awesome_follower_arm` 就是当前连接树莓派的同一台改装 follower。
- 原始包来源：`/home/gxxl/nudec-ro101-E/calibration/lerobot_calibration_bundle_20260731.zip`
- 原始包 SHA256：`52c40e9cd1c6a366a512c35ae27d7e5733ca39df18b930da134dfa64319c948e`
- 包内全部 `SHA256SUMS` 已执行 `sha256sum -c`，全部通过。
- 包记录的历史软件为 LeRobot `0.4.4`、Seeed fork commit `0f392484458cb5ebca0310c0c4c47390a31c80ed`；比赛运行时仍固定 LeRobot `v0.6.0`。

原包同时保存 leader/follower 和 J6，仅作为不可变来源证据。运行时只允许使用从 follower 原件精确提取的 J1–J5 文件：

- `lerobot_follower_5d_candidate.json`
- SHA256：`ffde2ce4fd56515ad06691014ce714555fb731893daf6510dde05b1d3e343273`
- 固定字段：`shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`
- 不含 `gripper` / ID6。

## 同机交叉验证

在树莓派上把五轴 `Homing_Offset` 临时归零并保持断力矩后，人工记录 J1 两个真实机械端点：

- 实测 raw：`2751`、`1372`
- 候选标定反推写偏置前 raw：
  - `(797 + 1965) mod 4096 = 2762`
  - `(3518 + 1965) mod 4096 = 1387`
- 两端差值仅为 `11`、`15` counts。

J1 有效行程跨越编码器 `4095 -> 0` 周界；真实长弧跨度约 `2717` counts，与候选文件的 `2721` counts 一致。不能直接把数字较大值减较小值当作物理行程。该发现已用于修正 `control.motor_calibration.center_motor_range()` 的环绕处理和回归测试。

## Pi 导入与回读

- 目标：`~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d/epuzzle-single-arm.json`
- 目标 SHA256：`89ea5079eb9bb35012866d736d0438d419ee57c201e51e11a61530e3e6440ca8`
- 导入工具在力矩关闭状态写入 J1–J5 Homing_Offset/范围；舵机寄存器回读与来源逐字段一致。
- 随后的独立只读连接再次确认寄存器与文件一致，并读取五轴 raw/degree；见 `pi_read_only_verification.json`。
- 首次只读时 `shoulder_lift=716`，比范围下限 `723` 低 7 counts；用户在断力矩下把 J2 移离止挡后复读为 `761`，最终五轴全部位于导入范围内。
- 使用 rejected 运动学标定运行 preflight，结果为 `NOT_READY` / exit 2（预期）。

## 安全边界

- 以上仅证明电机归一化标定与同一实物高度一致。
- 这不是 TCP/运动学 `accepted` 标定，不能解除自动运动、低位运动或电磁铁门禁。
- 采集和核对期间未发送 `Goal_Position`，未开启电磁铁；Supervisor 保持 inactive/disabled。
