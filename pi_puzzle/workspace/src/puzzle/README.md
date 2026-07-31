# puzzle

`solve_puzzle(WorkspaceState, TargetRegion, PuzzleOptions)` 是纯几何求解入口：不读文件、不导入 vision、不发送动作。它接受 1--4 个带 `table_mm` 虚拟多边形的观察，构造完整/部分边候选并只输出确定性计划或显式拒绝/歧义。

纹理 evidence 是可选输入；默认配置 `texture_enabled=false`。最终执行仍必须经过 integration Supervisor 与控制安全过滤。
