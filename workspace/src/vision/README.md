# vision

视觉模块只输出观察，绝不驱动机械臂。

- 既有 `detect_red_pieces` 是冻结的 1280×720 红色基线。
- `observe_pieces(RectifiedFrame, VisionProfile)` 是通用白色/印刷碎片入口。它返回物理 `measured_contour`、求解虚拟 `polygon_table_mm`、毫米诊断和显式拒绝结果。
- `generic_vision_enabled=false`，且磁头 footprint 尚未冻结时抓取点为 `null`；不得据此自动执行。
