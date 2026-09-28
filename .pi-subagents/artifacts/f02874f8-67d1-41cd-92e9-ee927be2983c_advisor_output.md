# Review: placement-gap implementation vs. "every pair ≥ 5 mm, seam pairs = 5 mm" requirement

## Inherited decisions (baseline contract)

- 求解几何回退为原版（`geometry_scale_factor=1.0`），不再 1.3× 放大。
- 间距只在**输出目标位**阶段加入，不改求解器匹配；朝向（每片的 R/Δyaw）必须保持求解结果。
- 原始 source/grip 视觉坐标保持不变；target 参考点是握点（adapter）或质心（generic solver）。
- 布局需居中于目标区中心。

## Findings

**F1 — blocker：`spread_target_point` 径向半间距数学上不满足"每对 ≥5mm / 接缝对 =5mm"**
`workspace/src/puzzle/solver.py:92-111` 与 `workspace/src/integration/_adapter_123.py:307-314`。每片沿"目标区中心→该片参考点"射线外移 2.5mm。失效模式：
- **同侧共线对**：两片在中心同一方向（如同一排的相邻两片），两者沿同一向量各移 2.5mm，**片间距不变，仍为 0**。
- **非共线对**：片间增量 = 两单位向量之差 × 2.5mm，有效接缝间距 ≈ 5·cos(θ/2)（θ 为两片射线夹角），从 5mm 到 0 甚至变近都有可能。
- **中心片**：`distance <= 1e-9` 时完全不动（solver.py:104），与它相邻的片单侧外移，间距只有 2.5mm。
- **非接缝对**：要求"every pair ≥5mm"，径向分散对非相邻片没有任何距离保证（放射状外移可能让原来够远的两片保持 <5mm）。

**F2 — high：两片接缝法向 ≠ 径向，间距不均且不可控**
即使两片分居中心两侧且共线，只有当接缝法向与径向一致时才恰好 5mm；实际拼图接缝方向任意，得到的间距是 0–5mm 的随机值，不满足"seam pairs at 5mm within tolerance"。

**F3 — high：gap 逻辑重复实现且语义不一致**
`solver.py:_plans_for_assembly`（spread 质心参考点）与 `_adapter_123.py`（spread 握点）是两份独立实现；同一 piece 在 generic solver 路径和 123 路径下 spread 的参考点不同（握点≠质心，piece_01 偏心 29.7mm），两条链路产出的间距行为不一致。gap 应收敛到一个共享模块，两个调用点都消费它。

**F4 — medium：spread 后未重新居中**
`solver.py:719-758` 先算 `shift` 使 bbox 居中，然后逐片外移；外移后 bbox 中心漂移（只有分布完全对称才不漂），最终布局不再居中于目标区。要求"keeping layout centered"未满足。

**F5 — medium：无验证/失败闭环**
spread 后没有重算任何片间距离；若结果不满足 5mm，没有任何 fail-closed 路径，计划照常下发执行。

**F6 — low：`_adapter_123.py:299` 残留 `geometry_scale_factor` 乘在局部握点上**
当前 factor=1.0 无实际影响，但接口上仍暗示缩放语义，与"回退原版"决定相矛盾，建议一并清理或注释冻结。

## Proposed deterministic algorithm（dependency-light, 仅 numpy/math）

**位置**：在 `solver.py` 新增一个深模块 `apply_placement_gap(placed_polygons, reference_points, gap_mm) -> (translations, achieved_min_distances)`，`_plans_for_assembly` 与 `_adapter_123` 都调用它（消除 F3）。

**步骤**（纯平移，朝向天然保留）：
1. 输入：求解后的 placed 多边形（assembly 坐标系）+ 每片参考点（质心或握点的 placed 位置）。
2. **接缝图**：对所有片对计算多边形最小距离（见 5），距离 < ε（如 0.5mm）判为接缝对，目标 = g；其余片对目标 = ≥g。
3. **确定性投影松弛**：固定迭代 N=50 轮；每轮按 canonical 顺序（排序后的 piece_id 对）处理所有片对：
   - 当前最小距离 d 与最近点对 witness（p_i, p_j），法向 n = normalize(p_j − p_i)；
   - 违例量 δ = g − d（接缝对双向投影到恰好 g：δ 可正可负；非接缝对只在 δ>0 时投影）；
   - 各移动 ½δ·n（片 i 移 −½δ·n，片 j 移 +½δ·n）。无随机性、顺序固定、轮数固定 → 完全确定。2–4 片规模下收敛极快。
4. **居中**：松弛后做一次全局平移，使所有 placed 顶点的 bbox 中心对齐目标区中心（平移不改变任何相对距离，修掉 F4）。
5. **距离验证（fail-closed）**：重算所有片对距离；任一对 < g − tol（建议 tol=0.5mm）则返回失败/拒绝出计划，不得静默下发（修掉 F5）。
6. 参考点（握点）随片平移同一向量，target 位姿 yaw 不变。

**多边形最小距离（无 shapely）**：求解输出保证多边形互不相交，因此片间距离 = 所有边对之间 segment-segment 距离的最小值；witness 点同时给出。4 边多边形 × ≤4 片 → ≤96 个边对，纯 numpy 几十行即可，对凸/非凸都精确（不相交前提下）。

**不推荐的替代**：绕质心均匀缩放（bisection 求 s）——接缝对间距变为 (s−1)·|c_i−c_j|，各对差异大且无法达到"恰好 5mm"，只满足 ≥5mm 的弱形式。

## Proposed test cases（`workspace/tests/test_puzzle_solver_synthetic.py` 或新文件）

1. **并排双矩形**：两片 50×60 共边 → 接缝距离 = 5±0.5mm；整体外尺寸在接缝方向 +5mm；yaw 与 gap=0 输出一致；bbox 中心 = 目标区中心。
2. **同侧共线三连片**（F1 回归）：中间片两侧的接缝都 = 5±0.5mm（径向法在此处必败）。
3. **T-offset 3 片夹具**：所有片对 ≥ 4.5mm；两个接缝对 = 5±0.5mm。
4. **中心片回归**（solver.py:104 早退分支）：参考点与装配中心重合的片仍能获得 5mm 间距。
5. **非相邻对角对**：原本 >0 但 <5mm 的非接缝对被推开到 ≥5mm。
6. **确定性**：同一输入连跑两次，输出坐标逐位相等。
7. **gap=0 回归**：输出与原版贴合计划完全一致。
8. **fail-closed**：注入无法收敛的构造（或迭代上限=0），结果必须是失败而非发出 <g−tol 的计划。
9. **朝向保持**：每片 target.yaw 与 gap=0 计划逐位相等。

## Risks / residual

- 非凸多边形 witness 计算在"不相交"假设下正确；若上游近似解（approximate/AMBIGUOUS 路径）允许微重叠，需先判重叠再算距离。
- 松弛在带环接缝图（4 片两两相触）上可能收敛到略偏离 5mm 的折中；N=50 + fail-closed 验证兜住，但需测试 4 片全接触夹具确认 tol=0.5mm 可达。
- 5mm 间距使外接矩形变大（约 +5mm/轴），需确认仍在目标区 90–120×50–90mm 范围内。
- generic solver 与 123 adapter 的参考点语义不同（质心 vs 握点），共享模块必须把参考点作为显式参数传入。

Need from main agent: 无阻塞；如需我进一步给出 `apply_placement_gap` 的具体接口签名/伪代码落地版，可再派一轮。Suggested execution prompt: 不建议本轮直接派 worker 改代码——先由主代理确认采用"投影松弛 + 接缝图"方案后再交 worker 实现上述模块与 9 个测试。