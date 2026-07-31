# 计划：吸收 puzzle-vision-simulator v2.2 的优秀算法

- **状态**：READY（分阶段执行；真机与扑克牌阶段受 Gate 约束）
- **计划目录**：`.taskman/plans/adopt-puzzle-vision-v2-2/`
- **外部参考**：`https://github.com/lvreng/puzzle-vision-simulator`
- **冻结参考版本**：tag `v2.2.0-vision-rounded-card`，commit `fbd9c1cbb9b621c0a54101cc421cc43d54926222`
- **原则**：独立重实现算法思想，不复制无明确许可证的源码和 Joker 素材。

## 1. 目标和非目标

### 目标

1. 在 `vision` 中实现真机可用的通用白色/印刷牌碎片观察：
   - 黑色工作面上的自适应分割；
   - 物理轮廓、凹凸多边形和虚拟直角；
   - 圆角恢复、毫米域边描述、最大净空抓取点；
   - 可解释置信度和完整诊断。
2. 在 `puzzle` 中实现最多 4 片的确定性求解（生产接口接受 1～4 片；1 片走合法矩形的平凡分支）：
   - 完整边、端点锚定部分边与可观测的内部部分边候选；
   - 不依赖 `cut_mode` 的统一拓扑枚举；
   - 目标矩形尺寸推断、闭环/覆盖/重叠/外边约束评分；
   - 等价解、次优差距和拒绝机制。
3. 在 Gate C 之后增加未知扑克牌的接缝纹理重排；已知牌面特征配准仅作为可选 adapter。
4. 通过离线回放、Pi 5 基准和 shadow 方式接入，不影响当前红色检测及机械控制基线。

### 非目标

- 不 vendor 或修改外部仓库；不复制其牌面素材。
- 不在本计划内修改 IK、轨迹、电磁铁或 ACT。
- 不在 PICK/PLACE 接触高度仍为 `UNKNOWN` 时触发自动抓放。
- 不用神经网络替代最多 4 片的确定性几何枚举。
- 不把仿真通过率写成真机实测值。

## 2. 取舍清单

| 外部做法 | 决策 | 本项目实现方式 |
| --- | --- | --- |
| 黑底白牌自适应亮度分割 | 吸收并增强 | 校正图上背景建模 + 多阈值稳定性，参数由 mm/px 比例推导；粘连证据不足时拒绝 |
| 短圆弧弦两侧直线外推为虚拟角 | 吸收 | 保留物理轮廓；毫米域拟合；仅在显式材料 profile 中启用，验证不会吞掉真实切割顶点 |
| 物理轮廓与虚拟多边形一一关联 | 吸收思想 | 从同一 Piece 实例派生，身份不靠独立排序；必要时才用 IoU/质心 assignment |
| 每对碎片保留多个边候选 | 吸收 | 按 piece pair 保留候选，避免全局 top-N 丢掉真实邻接 |
| 完整边 + T 形部分边匹配 | 吸收并泛化 | 统一候选模型，拓扑由求解器推断，不由 UI 指定 |
| 重叠、覆盖、矩形度、闭环联合评分 | 吸收 | 所有项归一到 mm/无量纲，记录分项和评分方向 |
| 固定 100×60 mm 目标 | 拒绝 | 在官方 90×50～120×90 mm 范围内从总面积和装配轮廓推断 |
| 普通模式先取凸包，凹形依赖 `cut_mode` | 拒绝 | 始终保留有向轮廓和凹顶点，多候选简化后由合法性评分选择 |
| “完整牌有 4 个圆角”硬排序 | 拒绝硬约束 | 仅可作为 printed-card profile 的弱诊断，不可强改顶点 |
| SIFT 对固定 Joker 图直接求每片位姿 | 限定使用 | 仅用于有合法自备参考图的 adapter；现场未知牌用几何候选后的接缝纹理评分 |
| 纹理失败静默退回几何 | 拒绝 | 返回 `texture_status` 与原因；花纹仍歧义时禁止动作 |
| 面积直接映射 confidence | 拒绝 | 由分割稳定、拟合残差、抓取净空、最佳/次佳差距共同校准 |
| 单体 `puzzle_sim.py` | 拒绝 | 两个深模块，公共 interface 小，内部实现私有 |

## 3. 目标模块与 interface

```text
RectifiedFrame(table_mm mapping)
        │
        ▼
vision.observe_pieces(frame, profile) -> VisionResult
        │   PieceObservation[]
        ▼
puzzle.solve_puzzle(workspace, target_region, options, texture_evidence=None)
    -> PuzzleSolveResult
        │   PlacementPlan[] | AMBIGUOUS | NO_SOLUTION
        ▼
shadow logger（本计划）
        │
        └── Supervisor 真机执行（Gate C 以后、另受 PICK/PLACE 安全门约束）
```

### `vision` 公共 interface

建议公开且只公开：

- `RectifiedFrame`
- `VisionProfile`
- `PieceObservation`
- `VisionResult`
- `observe_pieces(...)`
- 现有 `detect_red_pieces(...)` 在迁移前继续兼容。

内部实现可拆为 `_segmentation.py`、`_piece_geometry.py`，但不得把阈值、圆角启发式或 OpenCV 中间对象泄漏到公共 interface。

每个 `PieceObservation` 至少保存：

- `piece_id`（仅在该 source image 内唯一）、`timestamp_ns`、`source_image_id`；
- `measured_contour: {frame, points}`：实测物理轮廓，frame 显式为 `image_px` 或 `rectified_px`；
- `frame="table_mm"` 只管 `polygon_table_mm`、抓取点、中心和 yaw，不能暗示像素轮廓也是毫米；
- `polygon_table_mm`：求解用虚拟几何多边形；
- `pick_point_table_mm`、`center_table_mm`、`yaw_deg`；yaw 定义为从 table +x 逆时针、范围 `[-180, 180)` 的主轴初值，不是最终权威姿态；
- `equivalent_yaws_deg`、`confidence`；
- typed diagnostics：分割稳定度、多边形拟合残差、抓取净空、圆角候选数量、边界裁切状态；只有 `extra` 可保持自由结构。

`RectifiedFrame`/`VisionResult` 还必须记录 source image SHA256、单调时间戳、相机内参与 homography 的 ID/SHA256、`px_per_mm`、序列化 profile 及版本，使回放和毫米结果可审计。现有红色 adapter 在契约迁移时补充带 frame 的实测轮廓，但保持检测行为和原导出字段兼容；禁止为兼容而放宽 `additionalProperties: false`。

### `puzzle` 公共 interface

建议公开且只公开：

- `TargetRegion`
- `PuzzleOptions`
- `PuzzleSolveStatus`
- `PuzzleSolveResult`
- `TextureEvidence`（仅纹理阶段使用的校正图与逐片 mask，不包含生成/邻接真值）
- `solve_puzzle(...)`

`WorkspaceState` 显式区分 A4 上半源区、下半目标区和已锁定放置；白色求解不需要图像引用。可序列化状态只保存带 SHA256 的 `rectified_frame_ref`/mask refs，运行时 adapter 将其解析为只读 `TextureEvidence`；`puzzle` 不自行读文件、不重新分割、不访问 `vision` 私有实现。必须测试无 texture evidence 的白色路径。

`EdgeDescriptor`、图搜索、位姿传播、闭环优化、栅格评分和纹理条带采样均为私有实现。

`PuzzleSolveResult` 必须包含：

- `SOLVED / AMBIGUOUS / NO_SOLUTION`；
- 显式错误码；
- 推断目标矩形尺寸和 frame；
- 最终 `PlacementPlan[]`；
- 去重后的候选数、最佳/次佳分数和间隔；
- geometry/texture 各分项、评分方向（统一 penalty 越低越好）、是否实际执行纹理评分；
- solver/profile version、输入标定来源、分阶段耗时、扩展/剪枝候选数和 typed diagnostics。

## 4. 分阶段执行计划

> 每一阶段严格按“先写验收/失败测试，再实现，再验证”执行。阶段未通过不得打开下一层运行时开关。

### Phase 0 — 冻结来源、验收定义和兼容基线

**改动**

1. 新增 `docs/references/puzzle_vision_simulator_v2_2.md`：记录 URL、tag、commit、关键文件 SHA256、已验证测试和“无 LICENSE，不复制代码/素材”。
2. 新增 `docs/24_通用视觉与拼图求解器设计.md`：把本计划中的算法 seam、评分语义、错误码和 Gate 固化为项目设计文档。
3. 为当前 `red_piece_detector.py` 建立回放回归：使用现有 `red_pieces_frame.jpg` 与检测 JSON，先固定 4 片数量、中心、角度的容差，防止后续误伤。该 legacy 路径当前使用 `time.time_ns()`，Phase 0 不把其时间戳当作单调时钟证据；Phase 1 adapter 迁移时显式改为 `time.monotonic_ns()` 并增加时序测试。
4. 在 `docs/24_通用视觉与拼图求解器设计.md` 把“碎片是否保证纸面朝上”“是否保证可见间隙”“现场数量下限”“目标尺寸文字范围解释”登记为 `UNKNOWN`；未确认时分别走 `VISION_SURFACE_MISMATCH`、`VISION_MERGED_BLOBS`/拒绝路径，禁止镜像或猜测。
5. 同步 `docs/01_赛题需求追踪矩阵.md` 与 `docs/19_决策门与停止条件.md`：Gate C 统一为离线白色盲测 ≥18/20 且实机连续 5 次成功，不再保留 9/10 与 18/20 两套口径。
6. 在 `docs/12_测试与验收计划.md` 增加合成/开发机/Pi/真机四类证据标签，禁止混写。

**验收**

- 外部源码和 Joker 素材未进入仓库。
- 当前红色检测回放通过；现有控制/契约测试全绿。
- 文档明确区分官方要求、内部目标和未验证假设；纸面朝上/可见间隙未确认时仍有可测试的拒绝行为。
- Gate C 在 `docs/01`、`docs/12`、`docs/19` 和本计划中只有一套数字。

**建议 commit**

1. `docs: freeze external puzzle vision reference and adoption gates`
2. `test: lock current red piece detector replay baseline`

---

### Phase 1 — 先完成数据契约与独立测试夹具

**改动**

1. 更新 `data_contracts/piece_observation.schema.json`：新增嵌套 `measured_contour`（自身携带 `image_px|rectified_px` frame），明确顶层 `table_mm` 只适用于毫米字段，`polygon_table_mm` 是求解用虚拟几何而非实测物理轮廓；增加负例，拒绝把裸像素点放在 `frame=table_mm` 下。legacy 红色 adapter 的 `minAreaRect` 仍可作为保守虚拟几何，但必须在 diagnostics 标记 `geometry_source="legacy_min_area_rect"`，不能冒充精确轮廓。
2. 新增 `data_contracts/workspace_state.schema.json` 和 `data_contracts/puzzle_solve_result.schema.json`；扩充 `placement_plan.schema.json` 的纹理状态、等价姿态、来源时间戳和 authoritative target yaw。所有 yaw schema 统一为 `minimum: -180`、`exclusiveMaximum: 180`，避免 `-180/180` 双重表示。WorkspaceState 需表达 source/target region、可选 locked placements，以及只用于日志的图像/mask 引用和 SHA256。
3. 将 required diagnostics 固化为子 schema：视觉稳定性/拟合残差/净空/圆角/边界状态/等价 yaw，以及求解候选数/分阶段耗时/各 penalty 名称与方向；只允许 `extra` 作为扩展口。
4. 同步 `docs/10_接口与数据契约.md` 的错误码：至少加入 `VISION_MERGED_BLOBS`、`VISION_SURFACE_MISMATCH`、`SOLVER_AMBIGUOUS`、`SOLVER_TIMEOUT`、`TEXTURE_INSUFFICIENT`、`TEXTURE_AMBIGUOUS`；同步 `docs/06_拼图求解器.md` 中已经漂移的 PlacementPlan 字段名（以 schema 的 `source`/`target`/`plan_confidence` 为准），并同步 `workspace/src/vision/README.md`、`workspace/src/puzzle/README.md`。
5. 在 `configs/parameter_registry.yaml` 先登记而不捏造测量值：`px_per_mm`、几何绝对/相对容差、顶点容差、歧义分数间隔、接缝条带宽度/边缘排除宽度、通用视觉置信门槛均为 `UNKNOWN`，等待日志冻结；`generic_vision_enabled=false`、`texture_enabled=false`、白色 profile 的圆角虚拟化默认关闭。另设工程安全上限 `solver_time_budget_ms=10000` 与确定性的 `solver_max_expansions`（先为 `UNKNOWN`，启用前冻结）；超限不得返回部分最优解。
6. 新增本项目独立编写的固定种子夹具生成器，覆盖：
   - 中心放射、边界扇形、条带、等分矩形、T 形、角块、凹多边形；
   - 2/3/4 片；
   - 1 片合法矩形 smoke case，以及 2/3/4 片拓扑矩阵；
   - 纯白、程序生成花纹、圆角/直角、金属背面负例；
   - 旋转、照度梯度、模糊、噪声、阴影和 PNG 编解码；
   - A4 上下半分界线、碎片接触和 0～2 mm 间隙、异常大连通域。
7. 生成器真值只进入断言，生产函数只接收 PNG 重载像素或序列化 observations，建立“无生成器真值泄漏”测试。

**验收**

- 所有 JSON Schema 可解析，并有正/反例验证；`additionalProperties: false` 保持，不能靠放宽 schema 解决迁移。
- 测试夹具满足官方几何约束；非法夹具被生成器自身拒绝；完全粘连/背面朝上的预期是显式拒绝而非伪成功。
- 生产 interface 中不存在 `cut_mode`、生成多边形、真值邻接或真值变换参数。
- 程序生成素材有自己的来源说明，不引用外部 Joker 文件。

**建议 commit**

1. `contracts: define measured contour workspace and puzzle solve result`
2. `test: add independent pixel-isolated puzzle fixture corpus`

---

### Phase 2 — 通用碎片观察与圆角虚拟化

**实现顺序**

1. **分割**：在已校正 A4 上半源区估计工作面背景；对白色/印刷牌生成多个相邻阈值假设，做开闭运算和连通域过滤。核尺寸和面积门槛从 `px_per_mm`、最小合法边长与配置推导；A4 中间分界线不得进入碎片候选。
2. **粘连与表面异常**：利用多阈值分解一致性、异常面积/顶点数、轮廓颈部和可见接缝检测 merged blob；仅在证据一致时尝试距离变换/分水岭拆分并降置信，否则返回 `VISION_MERGED_BLOBS`。金属背面、强反光或不符合 profile 时返回 `VISION_SURFACE_MISMATCH`，不得镜像求解。
3. **物理轮廓**：始终保留原始有向外轮廓；内部牌面图案只作为孔洞，不参与外边界；接触 ROI 边界时显式降置信或拒绝。
4. **多边形候选**：每个多边形的顶点数只能为 3、4 或 5；用多组简化尺度和直线拟合产生多个假设并保留凹顶点。按轮廓残差、合法边长、简单多边形和阈值稳定性选择；近似并列但几何不同则拒绝，而不是任取一个。
5. **圆角虚拟化**：只在显式 `printed_card`/已冻结 profile 中检测“长直边—短弧—长直边”，拟合两侧直线并求交；白色金属片 profile 默认零替换。物理轮廓不变，只更新虚拟几何；每次替换记录 profile、弦深、拟合角和残差。
6. **抓取点**：对物理掩码做距离变换，输出最大净空点与 `pick_clearance_mm`；只有实测磁头 footprint 半径和安全余量已冻结时才能判定可抓。净空不足/参数为 `UNKNOWN` 时不以质心冒充安全点。
7. **姿态**：`minAreaRect/PCA` 只给按 table +x 逆时针的主轴初值；记录对称等价 yaw，最终 source/target yaw 由模板或拼图方案决定。
8. **置信度**：组合多阈值一致性、轮廓闭合、拟合残差、净空和帧边界状态；先输出分项，映射函数及接受阈值只能在有标签开发集上冻结，holdout 只验收一次。

**验收**

- 名义分离场景：7 类 × 2/3/4 片 × 至少 20 个固定种子的像素隔离合成回放，碎片数量正确率 100%；另对 1 片合法矩形做 smoke test。
- 接触/0～2 mm 间隙、背面朝上、分界线和异常连通域压力集：正确拆分或显式拒绝率 100%，高置信错误计数为 0；不宣称完全无缝接触必然可分。
- 扰动夹具的虚拟顶点误差 p95 ≤ 2 mm、最大 ≤ 3 mm；报告必须标注为合成结果。
- 凹片凹性保留率 100%；真实锐角不得被圆角规则替换；白色 profile 圆角替换数必须为 0。
- 各尺度/旋转 printed-card 圆角夹具的虚拟角误差 p95 ≤ 2 mm；不以“总数恰好 4”才能通过。
- 每个声称“可抓”的观察，其抓取点都位于按已冻结 footprint 腐蚀后的掩码内；footprint 未冻结或净空不足必须显式不可抓。
- 现有红色路径回归无变化。

**真机验证（Phase 2 退出条件）**

- 采集普通室内照明下白色与印刷牌原图、校正图、掩码和标注图；不得只用合成图宣布 DONE。
- 静态目标 30 帧中心坐标标准差 ≤ 1 mm；20 个网格点视觉绝对误差目标 ≤ 3 mm。
- 模拟正式场景 100 帧无持续漏检/误检；失败帧全部有诊断和错误码。

**建议 commit**

1. `vision: add scale-aware white card segmentation`
2. `vision: preserve concavity and recover virtual card corners`
3. `vision: compute safe pick points and calibrated diagnostics`
4. `test: validate generic observation on pixel-only replay corpus`

---

### Phase 3 — 毫米域确定性几何求解器

**实现顺序**

1. 从 `polygon_table_mm` 构造边描述：长度、方向、直线残差、相邻转角、凹凸性和端点。
2. 为每对碎片生成完整边候选；容差采用“Phase 2 实测绝对误差 + 相对误差”模型，而不是固定 12%，参数在开发集冻结。
3. 明确定义两类部分接触：
   - **端点锚定**：短边映射到长边前缀或后缀，offset 为 `0` 或 `L-S`；
   - **内部接触**：offset 是 `[min_residual, L-S-min_residual]` 内的连续有界变量。`min_residual` 保守取 `20 mm - vertex_tol_mm`，因为完整合法装配中的剩余接触应由合法边覆盖；若现场规则确认与此不同，再以证据修改。
4. 内部 offset 用确定性的毫米级粗网格和其他接触长度隐含断点生成初值，再在闭环优化中做有界连续细化；必须覆盖“90 mm 长边上 offset=37 mm”这类非端点、非中点夹具。`partial_offset_step_mm` 在 registry 中登记并经性能/召回日志冻结。
5. 候选按 piece pair 分组，不做全局 top-N 截断。完整匹配独占两条边；部分匹配在长边上登记占用区间，允许多个互不重叠区间共享该长边，禁止区间重叠，并验证最终内部边覆盖。
6. 回溯枚举连通邻接图并传播/优化 SE(2) 位姿；同一短边不得重复使用，长边按上一步的区间占用规则处理。
7. 在搜索过程中提前剪枝：面积、AABB、明显重叠、不可闭合、候选扩展上限、目标尺寸范围和外边可行性。每片至少一条目标外边是带测量容差的硬约束；若它清空候选，返回观察/几何诊断，不降级为软猜测。
8. 对带环和内部 offset 的候选用纯 NumPy 有界最小二乘细化，不新增 SciPy 运行时依赖；始终固定一个 gauge、显式规范化角度，避免 2π 分支漂移。
9. 对目标尺寸按 `width∈[90,120] mm`、`height∈[50,90] mm` 的 1 mm 网格及方向交换生成种子，用总面积/轮廓先剪枝，再连续细化；这是对题面文字范围的保守工程解释，须在设计文档标记待现场确认。目标落点由显式 `TargetRegion(table_mm)` 决定且不得越过 A4 下半区。
10. 评分全部可解释、统一为“penalty 越低越好”并保存分项：
    - 边匹配残差；
    - 闭环端点误差；
    - 碎片重叠面积；
    - 矩形缺口/越界面积；
    - 总面积守恒；
    - 外轮廓矩形度和周长；
    - 外边拟合残差（硬约束通过后的 tie-breaker）；
    - 复杂度/部分边惩罚。
11. 对旋转、镜像禁止、对称碎片和相同矩形做候选规范化与等价类去重。最佳与次佳比较发生在去重之后；非等价分数间隔不足时返回 `AMBIGUOUS`。
12. 生成确定性放置顺序：边界/角块优先、净空不足后置；同输入必须稳定输出同顺序。每份 plan 绑定 source image/timestamp，过期即拒绝。
13. 搜索同时受 `solver_max_expansions` 确定性上限和 `solver_time_budget_ms` 安全上限约束；任一超限返回 `SOLVER_TIMEOUT` 且不携带可执行的部分计划。所有候选使用全序 sort key、固定浮点序列化，不依赖 set/dict 遍历顺序或并行完成顺序。

**验收**

- 几何真值级夹具：7 类 × 2/3/4 片 × 30 seeds（至少 630 例）噪声前正确装配位于第一等价类，非法镜像解为 0；另含 1 片平凡矩形分支。
- “正确等价类”由独立 evaluator 判定：在允许的全局目标旋转/平移与碎片自身对称下比较 union 缺口、重叠、外矩形尺寸和每片 SE(2)，不能复用求解器自身 score 作为真值。
- 加入 Phase 2 观测误差后的合成回放正确/显式拒绝率 ≥ 99%，且高置信错误方案为 0。噪声 σ、离群率、漏检/粘连比例必须来自 Phase 2 合成与实拍残差日志，随 corpus 固化；未定义误差模型时该指标无效。
- 推断矩形两边尺寸在合成 holdout 上 p95 误差 ≤ 3 mm。
- 四块相同矩形返回合法等价类，不伪造不可观测身份。
- T 形夹具分别覆盖端点锚定与内部 offset；至少一例为 90 mm 长边、37 mm offset。多个短边可占用同一长边的互不重叠区间；其他普通夹具不得因部分边捷径产生更优错误解。
- 每个输出刚体矩阵满足 `RᵀR≈I`、`det(R)=+1`，禁止缩放和反射。
- 同一输入连续运行结果字节级稳定（耗时字段除外）。

**性能门槛**

- 开发机记录候选数、各阶段耗时和峰值内存。
- Pi 5 白色视觉+几何求解 p95 ≤ 2 s、单场最大 ≤ 5 s、RSS ≤ 250 MB；超限先优化剪枝和栅格尺度，不降低安全判据。

**建议 commit**

1. `puzzle: describe complete and partial edge candidates in millimetres`
2. `puzzle: enumerate connected rigid assemblies without topology hints`
3. `puzzle: infer legal rectangles and rank explainable candidates`
4. `puzzle: report equivalence ambiguity and deterministic placement plans`

---

### Phase 4 — 端到端离线流水线、真实白色盲测与 Gate C

**改动**

1. 新增 `workspace/tools/analyze_puzzle_frame.py`：读取已保存原图/校正参数，调用公共 interface，保存原图、校正图、掩码、物理/虚拟轮廓、候选摘要和最终 JSON。
2. 新增 `workspace/tools/benchmark_puzzle_pipeline.py`：固定 corpus、环境版本、CPU/RSS、每阶段延迟和结果哈希。
3. 将新流水线接到 GUI/运行页的 **shadow 按钮**：只显示和记录建议计划，不调用控制或电磁铁。
4. 冻结一份开发集与一份只用于最终验收的 holdout；所有图像和标注保存 SHA256。
5. 同步 `configs/parameter_registry.yaml`：冻结实测 OpenCV/Python 版本、校正尺度、profile、置信门槛、几何误差模型、搜索/歧义/接缝参数和 feature flags；尚未由日志得到的值继续为 `UNKNOWN`。`generic_vision_enabled`、`texture_enabled` 明确为布尔值且默认 `false`。
6. 盲测标注者只看到冻结输入与独立 evaluator，不看到参数调优过程；失败样本进入新开发集前，先关闭本轮 holdout 并另采下一轮 holdout，禁止反复调同一盲测集。

**Gate C 离线通过条件**

- 至少 20 套真实/实体模拟的不同白色 2～4 片盲测，第一等价类正确 ≥ 18 套；该口径同步替代旧 `9/10` 文档口径。
- 错误方案高置信接受次数为 0；拒绝/歧义计作未通过，不计作成功。
- 每次保存完整证据与错误码。
- Pi 5 性能满足 Phase 3 门槛。
- `generic_vision_enabled=false` 时当前红色检测和既有流程行为不变。

**停止条件**

- 未达到 18/20：保持 shadow，优先修轮廓简化、多阈值分割和候选剪枝；不得接真机动作，也不得通过反复调当前 holdout 宣布通过。
- 离线通过也不自动执行机械臂；还需 `docs/19_决策门与停止条件.md` 的 Gate C 实机条件，以及独立冻结 PICK/PLACE 接触高度。

**建议 commit**

 1. `tools: add replayable puzzle analysis and Pi benchmark`
 2. `integration: expose generic puzzle planning in shadow mode`
 3. `docs: record Gate C blind-set evidence and frozen parameters`

---

### Phase 5 — 未知扑克牌接缝纹理重排（Gate E）

**前置条件**

- Gate C 白色几何稳定；几何候选中必须包含真解。
- 采集合法且可再分发/内部使用的多牌面数据；不得采用外部仓库的 Joker 素材。

**实现顺序**

1. 只对几何 top-K 非等价候选计算纹理，不改变几何硬约束；`texture_top_k` 先登记为 `UNKNOWN`，由开发集的真解召回率/延迟曲线冻结。
2. 把每片原图按候选刚体位姿映射到目标坐标；沿内部接缝两侧采样等宽窄条，避开切割损伤和分割边缘。
3. 组合并记录独立分项：Lab 颜色差、梯度幅值/方向连续性、线条跨缝连续性、有效采样覆盖率。
4. 做亮度偏置归一化；有效纹理太少、遮挡或分数间隔不足时返回 `TEXTURE_INSUFFICIENT/AMBIGUOUS`。
5. 如自备题拥有合法完整参考图，再实现 `KnownCardRegistration` adapter（SIFT/AKAZE + RANSAC）；其状态、内点率、重投影误差和尺度检查必须单独输出，不能伪装成现场通用算法。
6. 添加专门回归，保证测试真正调用纹理 scorer，而不是只在带图案图像上运行几何求解。

**Gate E 验收**

- holdout 至少 40 个几何歧义场景，覆盖至少 4 种不同牌面和多种切割/旋转；构成需让同一 holdout 上的 geometry-only top-1 落在 30%～70%，避免“提升 20 个百分点”变成无意义或数学上不可能。
- 几何真等价类在冻结 top-K 中的召回率 ≥ 95%；不在 top-K 时单独记为几何召回失败，不允许纹理模块背锅或伪造解。
- 在同一完整 holdout 上，纹理最终 top-1 正确率 ≥ 90%，且相对 geometry-only 绝对提升 ≥ 20 个百分点；同时报告逐样本配对结果和 McNemar 检验作为诊断，不把 p 值另设为 Gate。
- 低纹理/重复纹理场景必须拒绝，错误方案高置信接受为 0。
- Pi 5 printed-card 视觉+求解 p95 ≤ 5 s，整体任务仍以 90 s 为内部目标、120 s 为官方上限。
- 未通过即按 Gate E 停止开发，正式版保留白色几何功能并关闭纹理。

**建议 commit**

 1. `puzzle: rank geometric candidates by card seam continuity`
 2. `test: prove texture ranking on geometry-ambiguous holdout`
 3. `docs: record Gate E result and printed-card fallback policy`

---

### Phase 6 — 安全启用和后续迁移

1. 默认配置保持：通用算法关闭、纹理关闭、ACT 关闭。
2. 启用顺序固定：离线 replay → Pi benchmark → GUI shadow → Supervisor 只读 plan → Gate C/E → 受监控实机。
3. Supervisor 每放一片后必须重新拍照和求解；不得沿用整局初始理想位姿。
4. `SOLVER_AMBIGUOUS`、`SOLVER_TIMEOUT`、纹理不足/歧义、低置信、视觉结果过期、标定失效均禁止动作。
5. 任何真机执行还必须冻结正式 `table_mm -> robot_base_mm` 变换及 SHA256、PICK/PLACE 接触高度、磁头 footprint/净空门槛；当前 contact-sheet 变换不得冒充正式视觉链路变换。
6. 现有 `detect_red_pieces` 只有在新通用观察器完成同数据回归并另有迁移审查后才能改为 adapter 或删除；本计划内不删除。

## 5. 测试矩阵

| 层级 | 输入 | 核心断言 | 证据标签 |
| --- | --- | --- | --- |
| 单元 | mm 多边形/轮廓 | 凹凸、圆角、边匹配、SE(2)、评分分项 | `synthetic-unit` |
| 像素隔离 | PNG 重载图像 | 不读取生成真值，检测与求解可复现 | `synthetic-pixel` |
| 压力像素集 | 接触/小间隙、背面、分界线、异常 blob | 正确拆分或显式拒绝；无高置信错误 | `synthetic-stress` |
| 回放 | 当前红色与新白色/牌面实拍 | 数量、顶点、抓取净空、错误码 | `recorded-hardware` |
| 盲测 | 冻结实体碎片 holdout | 18/20 白色；纹理 Gate E 指标 | `blind-hardware` |
| 性能 | Pi 5 正式环境 | p50/p95/max、RSS、候选数、版本 | `pi5-benchmark` |
| 集成 | shadow，不发动作 | schema、时效、feature flag、日志完整 | `shadow-integration` |

每次提交至少运行：

```bash
PYTHONPATH=workspace/src python -m unittest discover -s workspace/tests -v
python -m compileall -q workspace/src workspace/tools
ruff check workspace/src workspace/tools workspace/tests
```

修改 Python 文件后先跑 `lsp_diagnostics`，结束前跑 `lens_diagnostics mode=all`。Pi 专项基准使用背景任务执行并保存原始日志，不在开发机结果上推断 Pi 性能。

## 6. 文件影响范围

### 新增/修改（预期）

- `workspace/src/vision/`：通用观察模型与私有实现；保留 `red_piece_detector.py`。
- `workspace/src/puzzle/`：几何求解与可选纹理重排。
- `workspace/tests/`：契约、夹具、像素隔离、求解、纹理和回放测试。
- `workspace/tools/`：离线分析与基准工具。
- `data_contracts/`：PieceObservation、WorkspaceState、PuzzleSolveResult、PlacementPlan。
- `configs/parameter_registry.yaml`：版本、profile、尺度、门槛和 feature flags。
- `docs/05_*.md`、`docs/06_*.md`、`docs/10_*.md`、`docs/12_*.md`、`docs/15_*.md`、`docs/21_*.md`，以及新增来源/设计文档。

### 明确不改

- `workspace/src/control/`
- `workspace/src/robot_plugin/`
- `workspace/src/policy/`
- GPIO17、电磁铁超时与五轴字段顺序
- LeRobot v0.6.0 和已冻结 Pi 镜像依赖版本

## 7. 风险、回退与停止条件

| 风险 | 预防 | 回退/停止 |
| --- | --- | --- |
| 外部仓库无 LICENSE | 只记录思想/行为，独立实现、独立夹具、无外部素材 | 发现代码或素材混入立即移除并重写 |
| 合成过拟合 | 开发/holdout 分离，必须有实拍白色与多牌面 | 无真实证据不从 VERIFY 进入 DONE |
| 现场翻面/金属背面 | 保持题面程序假设为 UNKNOWN，profile 检查，不允许反射 | 返回 `VISION_SURFACE_MISMATCH`；确认现场摆放规则前不宣称覆盖 |
| 碎片接触或无可见缝 | 接触/0～2 mm 压力集，多阈值/分水岭仅在证据一致时拆分 | 返回 `VISION_MERGED_BLOBS`；不猜数量、不执行 |
| 圆角误吞真实切割顶点 | profile 门控、毫米域局部拟合、锐角/凹角负例、保留物理轮廓 | 白色 profile 默认关闭；关闭 printed-card 虚拟化可回退 |
| 搜索组合爆炸 | pair 分组候选、分支限界、候选统计和 Pi 门槛 | 超限拒绝，不靠放宽几何安全条件提速 |
| 等价解伪装唯一解 | 规范化、聚类、最佳/次佳非等价间隔 | 返回 AMBIGUOUS，禁止动作 |
| 扑克牌纹理稀疏/重复 | 接缝覆盖率与分数间隔门槛 | Gate E 未过即关闭 texture |
| 新算法破坏自备题 | side-by-side + feature flag + 红色回放 | 一键关闭通用算法，恢复原路径 |
| 真机安全条件未完成 | shadow 无动作，Supervisor 门禁 | PICK/PLACE 高度 UNKNOWN 时禁止接执行 |

## 8. 完成定义

本计划只有同时满足以下条件才算完成：

- 公共 interface 和数据契约冻结，文档/Schema/测试一致；
- 外部优秀思想已由独立实现覆盖，且没有复制无许可证源码/素材；
- 红色确定性基线无回归；
- 通用视觉完成合成、实拍重复性和失败诊断验收；
- 几何求解通过 630+ 夹具、20 套白色盲测和 Pi 性能门槛；
- 纹理只有在 Gate E 通过时启用，否则明确关闭且白色功能不受影响；
- 所有输出的每个坐标字段都带正确 frame/单位，并带单调时间戳、输入/标定 SHA256、置信度、错误码和可追溯日志；
- 任何真机动作仍由 Supervisor、安全过滤和现有控制门禁负责，正式 `table_mm -> robot_base_mm`、PICK/PLACE 高度和磁头 footprint 未冻结时只能 shadow。
