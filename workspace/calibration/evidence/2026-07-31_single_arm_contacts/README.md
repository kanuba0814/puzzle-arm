# 2026-07-31 单臂触点证据

- 来源设备：`epuzzle-pi`，单台由原 SO-101 follower 改装的五轴机械臂。
- 来源目录：`/home/epuzzle/puzzle`。
- 坐标：触点 `world_mm` 为旧实现的 A4/table 坐标；`joint_deg` 为旧实现当时的五轴软件角度。
- 用途：只用于重放旧模型、设计新标定流程和建立失败基线；不得直接作为已验收生产标定。
- 限制：A–H 人工触点名义 Z=10 mm；I–L 彩色块顶面 Z=1 mm，记录声明 XY 人工不确定度 5 mm，工具面仅“近似平放”。

| 文件 | 角色 | SHA256 |
| --- | --- | --- |
| `fit_A-H.json` | 旧模型拟合样本 | `d2e7c4ff9605d3b00446c91f777d1f6d23e684e9b72ca41f1ad7276144abe1c2` |
| `holdout_I-L.json` | 未参与 A–H 拟合的独立重放点 | `ab0dab3850d2ee7e64175250def0658ce2e7658330e8d2b0d040e49e6fa55db8` |
| `legacy_arm_base_rejected.json` | 被拒绝的旧参数 | `f00bd841cf9bbcd7ffc165bad7fab94a6249542d1d2c4390b8d00fb317286c15` |

旧参数独立重放结果：XY RMS 9.011 mm、最大 11.421 mm、Z RMS 4.650 mm，因此状态固定为 `rejected`。
