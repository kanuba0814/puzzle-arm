# 2026-07-31 单臂触点证据 v2

## 采集条件

- 同一台 `so101_follower_modified_single_arm`，使用已回读一致的精确五轴 LeRobot 电机标定。
- A4 纸以左下角为 `calibration_sheet_mm (0,0,0)`；用户按简图手绘目标并复核：方框 20 mm、A→B 70 mm、B→D 70 mm、A→E 80 mm。
- 电磁铁外径按单次卡尺读数 `20 mm`；目标为 20×20 mm 方框，磁铁工作面平放并使圆周与四边中点相切。
- GPIO17 fail-safe OFF 已在采集前重跑；Supervisor inactive；磁铁未激活。
- 五轴力矩关闭，全部姿态由用户手动托住摆放；程序只读 `Present_Position`，未发送 `Goal_Position`。

## 样本划分

- 拟合点（8）：A、B、D、E、F、G、H、I。
- 独立 holdout（4）：M、N、O、P。
- 每点 11 次 raw，固定显式五轴顺序；每轴样本跨度不超过 12 counts，且中值均在电机标定范围内。
- 首次 F 采样与 E 五轴 raw 完全相同，因此当场判为未实际移动并丢弃；重新摆放后的 F 才进入最终证据。
- 采集结束后用户确认机械臂已安全放稳，纸张保持未移动。

| 文件 | SHA256 |
| --- | --- |
| `single_arm_contacts_20260731.json` | `11bf8221865f96242963d58e5af839eb78249ed481c7e0ca829abb5cabb433d4` |
| `contact-capture-session.log` | `d46bff688ebdad8ff3850e273999a207ced3529d4b58c6da071db12a4879d3b0` |
| `single_arm_contact_sheet_v1.plan.json` | `4dde6b01477ffe87f308f8441a18220661d92f5e128f00b237e8b24efd1dd36f` |
| `single_arm_contact_sheet_hand_draw_v1.md` | `1f2da334234e361b446ba36a0fec9ac84263cc34a1afc780a03a0eff26cc53a7` |

## 拟合与接受结果

拟合程序只使用 8 个 fit 点优化 J2–J4 零偏和纸面刚体变换；M/N/O/P 未参与拟合或模型选择。生产 `PlanarArmKinematics` 独立重放得到：

- holdout XY RMS：`2.480912 mm`（门槛 5 mm）
- holdout XY max：`3.406456 mm`（门槛 8 mm）
- holdout |Z|max：`2.458668 mm`（门槛 3 mm）

三项门槛全部通过。用户随后明确要求“标定结果直接接受”，因此生成 `quality.status=accepted`；未删除或淡化以下风险。

| 输出 | SHA256 |
| --- | --- |
| `workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json` | `c207bae180e187b6c639cc848320195dafce1f468920a0f2eba7f00c621eab38` |
| `workspace/calibration/contact_sheet_to_robot_base_20260731.json` | `151d447b7a1ff144295aabcbb11e60ade643cbad908b5b65f06b4d8f06f8b271` |
| `fit_report.json` | `47cdef43fb20870c8444b77b109fe82942c9ea065b413ba72625b36a0b0482a8` |

## 保留风险

- 手绘定位误差未定量；
- 磁吸头轴向长度只有一次卡尺读数，重复性 `UNKNOWN`；
- 历史 URDF 原件 SHA256 仍为 `UNKNOWN`；
- 竞赛实用软限位来自同机机械端点内缩，J5 另有限制，但仍不是完整自碰包络。

首次真实高位无磁铁往返已完成；低位抓放仍未验收，并继续经过规划和安全过滤。
