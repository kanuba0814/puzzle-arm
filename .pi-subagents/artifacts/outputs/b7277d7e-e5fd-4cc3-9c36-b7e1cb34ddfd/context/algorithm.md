# 代码级实现合同：通用视觉与 1–4 片确定性拼图求解（adopt-puzzle-vision-v2-2）

> 角色交接产物。读者是 writer 子代理。本文给出可直接实现的公共 interface、数据契约字段、算法取舍、精确文件/测试清单、Gate 划分与验证命令。只读分析，未修改任何项目文件。
> 依据：`.taskman/plans/adopt-puzzle-vision-v2-2/PLAN.md` + `context.md`（算法决策以这两份为准，本文将其落成代码级合同）、题面 `official_docs/competition/E题_拼图装置.txt`、当前仓库实测。

---

## 0. 现状盘点（证据）

| 事实 | 证据 |
| --- | --- |
| 红色检测基线已实现且被 GUI 链路使用 | `workspace/src/vision/red_piece_detector.py`（全文 181 行）；`workspace/src/vision/__init__.py:3-15` 导出 `EXPECTED_FRAME_SIZE/RedPieceObservation/detect_red_pieces/load_homography`；调用方 `workspace/tools/detect_red_pieces.py:14,53`、`workspace/apps/pick_place_page.py:42,167,180` |
| 红色检测对外行为 | `red_piece_detector.py:74` `detect_red_pieces(frame, homography, *, minimum_pixel_area=300.0)`；硬编码 1280×720 校验（`:83-89`）；HSV 双段红色阈值 + 5×5 椭圆开闭运算（`:91-105`）；table 范围裁剪 `-5..215 × -5..302 mm`（`:122-125`）；minAreaRect 作 `polygon_table_mm`，pick_point == center（`:136-143`）；yaw 归一到 `[-90,90)`（`:131-133`）；confidence = `area/2500` 截断（`:136`）；排序 key `(-table_y, table_x)`（`:127`），piece_id `red-{index}`；时间戳用 `time.time_ns()`（非单调时钟，PLAN Phase 0 明确：回放固定数量/中心/角度容差，Phase 1 迁移时才改 `monotonic_ns`） |
| 回放证据已存在 | `workspace/calibration/evidence/2026-07-31_gui_a4_workspace/red_pieces_frame.jpg` + `red_pieces_detection.json`（4 片，含 center_pixel/center_table_mm/angle_deg/pixel_area/size_table_mm）；另有 `latest_pieces.json`（`as_mapping()` 格式）可作 schema 正例 |
| puzzle 模块为空壳 | `workspace/src/puzzle/` 只有 README.md；无 `__init__.py`，无生产实现 |
| 现有契约 | `data_contracts/piece_observation.schema.json`：无 `measured_contour`/`equivalent_yaws_deg`；yaw `maximum: 180`（PLAN 要求全仓统一 `exclusiveMaximum: 180`，此处与 placement_plan 都要改）；`diagnostics` 为自由 object。`data_contracts/placement_plan.schema.json` 已是 `source`/`target` pose2 结构（与 `docs/06_拼图求解器.md` 末尾漂移的旧字段名 `pick_point_table_mm/source_yaw_deg/...` 不一致——以 schema 为准，改文档） |
| 契约测试模式 | `workspace/tests/test_control_data_contracts.py`：unittest + 纯 `json.loads` 解析，不断言 jsonschema 语义（开发机无 jsonschema 包，见 §9） |
| Gate C 口径漂移 | `docs/19_决策门与停止条件.md` Gate C 写"10 套盲测 9 套正确 + 实机连续 5 次"；`docs/01_赛题需求追踪矩阵.md:8` 写"20 套 >= 18 套"；PLAN 要求统一为"离线 ≥18/20 且实机连续 5 次" |
| 参数注册表 | `configs/parameter_registry.yaml`：无 `px_per_mm`、无几何容差、无 solver 预算、无 feature flag；`vision:` 段现有 `minimum_piece_confidence: 0.12`、`segmentation_profile: "red-hsv-dual-band-20260731"`；`frames.table_origin_definition: "A4 bottom-left; x right 0-210; y up 0-297"`（table_mm 原点=左下、y 向上，全仓统一沿用）；homography SHA256/RMS 0.257541 mm/max 1.029862 mm 已冻结 |
| 题面硬约束（E题 txt 行号） | 现场碎片：数量 ≤4（无下限）、每片 ≤5 边、每边 ≥20 mm、每片 ≥1 条目标外边、目标矩形 90×50 ~ 120×90 mm、120 s 上限（`:44-49`）；碎片=镀锌薄铁片+贴纸片总厚 ≤2 mm ≤30 g；测评"先遮挡摄像头，随机**不重叠**摆放"（`:74`）——官方只保证不重叠，**不保证间隙、不保证哪面朝上**；评分容忍"相邻碎片对应顶点距离 ≤2cm"（评分口径，≠求解容差）；上下半区有 ≤5 mm 实线分界线（`:41`） |
| 明确不改 | `workspace/src/control/`、`robot_plugin/`、`policy/`、GPIO17/电磁铁/五轴顺序、LeRobot v0.6.0 |
| 许可证约束 | 外部仓库 `lvreng/puzzle-vision-simulator` tag `v2.2.0-vision-rounded-card` commit `fbd9c1c…` 无 LICENSE → 独立重实现，禁止复制代码/Joker 素材；生成器素材必须程序生成并自带来源说明 |

---

## 1. 红色基线保留合同

1. `red_piece_detector.py` 本轮**不改算法行为**：不改阈值、排序、piece_id 规则、`as_mapping()` 导出键、`EXPECTED_FRAME_SIZE` 校验。允许的唯一改动是 Phase 1 契约迁移时新增字段（见 §3.4），且必须以不破坏现有调用方为前提。
2. 先建回放回归测试再动任何 vision 代码（PLAN Phase 0-3）：
   - 新文件 `workspace/tests/test_red_piece_replay.py`：加载 `workspace/calibration/evidence/2026-07-31_gui_a4_workspace/red_pieces_frame.jpg` + `table_homography_gui_20260731.npy`，调用 `detect_red_pieces`，断言：片数==4；每片 `center_table_mm` 与 `red_pieces_detection.json` 对应值容差 ≤1.0 mm；`angle_deg` 与 `yaw_deg` 容差 ≤2°（注意 JSON 的 angle_deg 与检测器 yaw 归一化区间不同，测试内需做 ±90° 等价换算后再比较，或仅比较中心+尺寸，具体以复跑实测为准——writer 先跑一次打印实际值再写容差）。
   - 该 JSON 是旧格式（`size_table_mm`/`angle_deg`），不是 `as_mapping()` 输出；测试以数值比对为准，不做键级相等。
3. `detect_red_pieces` 返回签名 `(observations, mask, annotated)` 不变；GUI `pick_place_page.py` 与 `tools/detect_red_pieces.py` 零改动。

---

## 2. vision 深模块 interface 合同

### 2.1 公共导出（`workspace/src/vision/__init__.py` 追加，旧导出保留）

```python
# __init__.py 新增导出（全部定义在 workspace/src/vision/generic_piece_observer.py）
RectifiedFrame
VisionProfile
PieceObservation
VisionResult
observe_pieces
```

**硬约束**：`observe_pieces` 是唯一入口函数；阈值、圆角启发式、OpenCV 中间对象（mask/contour/kernel）不得出现在公共签名或公共 dataclass 字段类型里（numpy 数组允许出现在 `RectifiedFrame` 输入与 `measured_contour.points` 中）。私有实现放 `workspace/src/vision/_segmentation.py`、`_piece_geometry.py`（下划线前缀=模块私有约定）。

### 2.2 数据结构（全部 frozen dataclass，与现有 `_Candidate`/`RedPieceObservation` 风格一致，4 空格缩进，中文注释）

```python
@dataclass(frozen=True)
class RectifiedFrame:
    image: np.ndarray                 # 已透视校正的俯视图，BGR 或灰度均可，shape 不固定
    px_per_mm: float                  # >0，有限
    table_mm_size: tuple[float, float]  # (210.0, 297.0)
    source_image_sha256: str          # 原图（未校正）SHA256
    source_image_id: str
    timestamp_ns: int                 # 单调时钟 time.monotonic_ns()
    homography_id: str                # 文件名或配置键
    homography_sha256: str
    camera_intrinsics_id: str | None  # 无内参时为 None，字段必须存在

@dataclass(frozen=True)
class VisionProfile:
    profile_id: str                   # 例 "white-galvanized-v1" / "printed-card-v1" / "legacy-red"
    material: str                     # "white_paper_on_metal" | "printed_card" | ...
    enable_rounded_corner_recovery: bool  # 白色金属片 profile 必须为 False
    # 分割/几何参数全部从 px_per_mm 推导或从 registry 读，这里不塞像素常数
    min_edge_length_mm: float = 20.0      # 题面硬约束
    max_piece_sides: int = 5              # 题面硬约束
    max_pieces: int = 4                   # 题面硬约束

@dataclass(frozen=True)
class MeasuredContour:
    frame: str                        # "image_px" | "rectified_px"（禁止 "table_mm"）
    points: tuple[tuple[float, float], ...]  # 有向外轮廓，含凹顶点

@dataclass(frozen=True)
class PieceObservation:               # 通用观察（与 legacy RedPieceObservation 并存，不改名旧类）
    piece_id: str                     # 仅 source_image_id 内唯一，如 "piece-0"
    timestamp_ns: int
    source_image_id: str
    measured_contour: MeasuredContour # 实测物理轮廓：抓取/纹理/诊断用
    polygon_table_mm: tuple[tuple[float, float], ...]  # 3–5 顶点，求解用虚拟几何
    pick_point_table_mm: tuple[float, float] | None    # None = 显式不可抓（禁止质心冒充）
    pick_clearance_mm: float | None
    center_table_mm: tuple[float, float]
    yaw_deg: float                    # [-180,180)，table +x 逆时针主轴初值，非最终权威姿态
    equivalent_yaws_deg: tuple[float, ...]             # 对称等价 yaw，可为空 tuple
    confidence: float
    diagnostics: "VisionDiagnostics"  # typed，见下；唯一自由扩展口是 diagnostics.extra

@dataclass(frozen=True)
class VisionDiagnostics:
    segmentation_stability: float     # 多阈值一致性 [0,1]
    polygon_fit_residual_mm: float
    pick_clearance_ok: bool | None    # None = footprint 未冻结，无法判定
    rounded_corner_candidates: int
    geometry_source: str              # "fitted_polygon" | "legacy_min_area_rect"
    touches_frame_border: bool
    error_code: str | None            # VISION_MERGED_BLOBS / VISION_SURFACE_MISMATCH / None
    extra: dict[str, object]

@dataclass(frozen=True)
class VisionResult:
    observations: tuple[PieceObservation, ...]
    status: str                       # "OK" | "REJECTED"
    error_code: str | None            # 结果级错误码
    profile: VisionProfile
    profile_version: str
    px_per_mm: float
    source_image_sha256: str
    timestamp_ns: int
    processing_time_ms: float

def observe_pieces(frame: RectifiedFrame, profile: VisionProfile) -> VisionResult: ...
```

**拒绝行为（不是异常，是返回值）**：`observe_pieces` 不因单帧坏图抛异常；表面不符/粘连无法拆分时返回 `VisionResult(status="REJECTED", error_code=..., observations=())`，或保留低置信 observation 并在 `diagnostics.error_code` 标记。只有输入结构性非法（px_per_mm≤0、非有限、维度错误）才 `raise ValueError`（沿用 `red_piece_detector._finite_float` 风格）。

### 2.3 分割与几何算法取舍（最小但优秀）

- **分割**（`_segmentation.py`）：在 A4 上半源区 ROI 内对灰度图做背景建模（大核中值/形态学开运算估计背景亮度场，核尺寸由 `px_per_mm × mm` 推导），生成 ≥3 个相邻绝对阈值假设；每个假设做开+闭运算（核尺寸 ≈ 1 mm 对应像素，奇数）、连通域过滤（面积上下限由 `px_per_mm`、20 mm 最小边长、`max_pieces` 推导）。跨阈值**存在性一致**的连通域才升为候选；`segmentation_stability` = 该连通域在多少比例阈值假设下稳定存在且面积变化 < 阈值。A4 中间分界线（题面 ≤5 mm 实线）通过 ROI 几何排除或细长条连通域剔除，**不得进入碎片候选**。
- **粘连/表面**：异常大面积（>单片理论上限）、异常顶点数、轮廓颈部（局部宽度 << 片宽）作为 merged-blob 证据；仅当多阈值分解一致给出 ≥2 个稳定子域时才尝试距离变换+分水岭拆分并降置信；否则 `VISION_MERGED_BLOBS` 整体拒绝。高反光/暗金属色块与 profile 亮度模型不符 → `VISION_SURFACE_MISMATCH`，**不做镜像/翻转求解**（题面未保证纸面朝上，保持 UNKNOWN）。
- **物理轮廓**：`cv2.findContours(RETR_EXTERNAL, CHAIN_APPROX_NONE)` 保留有向外轮廓原样存入 `measured_contour`；内部印刷图案轮廓（RETR 树）只当孔洞，不碰外边界。接触 ROI 边界 → `touches_frame_border=True` 且降置信或拒绝。
- **多边形候选**（`_piece_geometry.py`）：对轮廓做多组 `approxPolyDP` epsilon（mm 域：epsilon_px = epsilon_mm × px_per_mm，取 3–5 档），结合 Ramer-Douglas + 共线点合并，产生多个候选多边形；**顶点数只允许 3/4/5**；凹顶点必须保留（凸包禁用）。按（轮廓残差 mm、边长合法性 ≥20 mm−tol、简单多边形自相交检查、阈值稳定性）加权选择；两个候选残差近似并列但几何不同 → 该 piece 降置信并标 diagnostics，不任取。
- **圆角虚拟化**：仅 `enable_rounded_corner_recovery=True` 的 profile。检测"长直边—短弧（弧长 << 两侧直边，弦深 ≥ 可分辨阈值）—长直边"模式；对两侧直边段在**毫米域**做最小二乘直线拟合，求交得虚拟直角顶点替换弧段。物理轮廓不变；每次替换记录到 diagnostics（弦深、拟合角、残差进 `extra`）。白色 profile 下替换数必须为 0（测试断言）。锐角/凹角负例夹具必须证明不误替换。
- **抓取点**：对物理掩码 `cv2.distanceTransform`，最大净空点 → `pick_point_table_mm`，`pick_clearance_mm` = 距离值/px_per_mm。仅当 registry 中磁头 footprint 半径 + 安全余量已冻结（当前 `magnet.*` 多项为 UNKNOWN）才判 `pick_clearance_ok`；否则 `pick_point_table_mm=None` 或 `pick_clearance_ok=None`，**禁止以质心冒充可抓点**。
- **姿态**：`cv2.minAreaRect` 或 PCA 主轴，归一到 `[-180,180)`；对称形状把等价 yaw 写入 `equivalent_yaws_deg`（矩形 2 重、正方 4 重等）。最终 source/target yaw 由拼图方案决定，此值只是初值。
- **置信度**：分项先输出（stability、fit residual、clearance、border），组合映射函数在代码中留单点 seam（如 `_confidence_from_diagnostics()`），常数标 UNKNOWN/待定注释，由有标签开发集冻结——**不要在本轮捏造校准曲线**。

### 2.4 legacy adapter（Phase 1）

- 新文件 `workspace/src/vision/legacy_red_adapter.py`：`def red_observation_to_piece_observation(obs: RedPieceObservation, ...) -> PieceObservation`，`measured_contour` 用 minAreaRect 的 `pixel_box`（frame=`image_px`），`diagnostics.geometry_source="legacy_min_area_rect"`，时间戳改 `time.monotonic_ns()` 需同步加时序测试。不改 `detect_red_pieces` 本身。

---

## 3. 数据契约变更（data_contracts/，全部保持 `additionalProperties: false`）

### 3.1 修改 `piece_observation.schema.json`

- 新增嵌套对象 `measured_contour: {frame: enum["image_px","rectified_px"], points: array<point2> minItems 3}`（required 进顶层 required 列表？——决策：**加入 required**，legacy adapter 已能产出；writer 若发现回放 JSON 旧格式不兼容，注意 schema 校验对象是新生产的 mapping，不是历史 JSON）。
- 新增 `equivalent_yaws_deg: array<number>`、`pick_clearance_mm: number|null`、`source_image_id` 已有保留。
- yaw：改 `maximum: 180` → `exclusiveMaximum: 180`（minimum -180 保留）。
- `diagnostics` 改为子 schema：required `segmentation_stability / polygon_fit_residual_mm / pick_clearance_ok / rounded_corner_candidates / geometry_source / touches_frame_border`，可选 `error_code`，`extra` 为唯一自由 object。
- 注释字段语义：`polygon_table_mm` 是求解用虚拟几何；顶层 `frame="table_mm"` 只管毫米字段。
- **负例测试**：`frame="table_mm"` 下塞裸像素点、`measured_contour.frame="table_mm"`、多余顶层键 → 必须被拒绝（用轻量手写 validator 或 jsonschema，见 §9 环境约束）。

### 3.2 修改 `placement_plan.schema.json`

- 新增：`texture_status: enum["not_run","ok","insufficient","ambiguous"]`、`texture_status_reason: string`、`equivalent_yaws_deg`、`source_timestamp_ns: integer`、`target` pose 的 yaw 注明为 authoritative target yaw。yaw 同样统一 `exclusiveMaximum: 180`。旧 required 字段全部保留。

### 3.3 新增 `workspace_state.schema.json`

- required：`regions`（source: A4 上半、target: A4 下半，各含 `frame: "table_mm"` const 与矩形 bounds）、`pieces: array<PieceObservation ref>`；可选 `locked_placements: array<PlacementPlan ref>`、`rectified_frame_ref: {path, sha256}`、`mask_refs`（仅日志用，puzzle 运行时不读文件）。

### 3.4 新增 `puzzle_solve_result.schema.json`

required：`status: enum["SOLVED","AMBIGUOUS","NO_SOLUTION"]`、`error_code: (enum|null)`、`inferred_target: {width_mm,height_mm,frame:"table_mm"}`、`placements: array<placement_plan>`、`candidates_after_dedup: integer`、`best_score / second_best_score / score_gap: number|null`、`score_breakdown: object`（各 penalty 分项）、`score_direction: const "lower_is_better"`、`texture_executed: boolean`、`solver_version`、`profile_version`、`calibration_refs: {homography_sha256,...}`、`timing_ms: {segment: number}`、`expansions / pruned: integer`、`diagnostics`（typed 子 schema + extra 口）。

### 3.5 同步文档

- `docs/10_接口与数据契约.md`：错误码段增加 `VISION_MERGED_BLOBS / VISION_SURFACE_MISMATCH / SOLVER_AMBIGUOUS / SOLVER_TIMEOUT / TEXTURE_INSUFFICIENT / TEXTURE_AMBIGUOUS`；PieceObservation 字段清单同步。
- `docs/06_拼图求解器.md` 末尾"求解器输出"旧字段名（`pick_point_table_mm/source_yaw_deg/target_center_table_mm/target_yaw_deg/solver_confidence`）改为 schema 的 `source/target pose2 + plan_confidence`。
- `workspace/src/vision/README.md`、`workspace/src/puzzle/README.md` 同步新 interface。
- `docs/19` Gate C 改为"离线盲测 ≥18/20 且实机连续 5 次"，与 `docs/01:8` 唯一口径。
- 新增 `docs/references/puzzle_vision_simulator_v2_2.md`（URL/tag/commit/SHA256/无 LICENSE 声明）与 `docs/24_通用视觉与拼图求解器设计.md`（seam、错误码、Gate、UNKNOWN 登记：纸面朝上/可见间隙/数量下限/目标尺寸文字解释）。

### 3.6 `configs/parameter_registry.yaml` 追加（只登记 UNKNOWN，不捏造）

```yaml
vision:
  px_per_mm: "UNKNOWN"               # 候选 2–3，Pi 基准后冻结
  geometry_abs_tolerance_mm: "UNKNOWN"
  geometry_rel_tolerance: "UNKNOWN"
  vertex_tolerance_mm: "UNKNOWN"
  ambiguity_score_gap: "UNKNOWN"
  seam_strip_width_mm: "UNKNOWN"
  seam_edge_exclusion_mm: "UNKNOWN"
  generic_vision_confidence_threshold: "UNKNOWN"
  generic_vision_enabled: false
  texture_enabled: false
  rounded_corner_recovery_default: false
puzzle:
  solver_time_budget_ms: 10000       # 工程安全上限
  solver_max_expansions: "UNKNOWN"   # 启用前冻结
  partial_offset_step_mm: "UNKNOWN"
  texture_top_k: "UNKNOWN"
```

---

## 4. puzzle 深模块 interface 合同

### 4.1 公共导出（新 `workspace/src/puzzle/__init__.py`）

```python
TargetRegion
PuzzleOptions
PuzzleSolveStatus      # Enum: SOLVED / AMBIGUOUS / NO_SOLUTION
PuzzleSolveResult
TextureEvidence
solve_puzzle
```

```python
@dataclass(frozen=True)
class TargetRegion:      # A4 下半区，table_mm
    frame: str           # const "table_mm"
    min_x_mm / min_y_mm / max_x_mm / max_y_mm: float

@dataclass(frozen=True)
class PuzzleOptions:
    time_budget_ms: int = 10000        # 安全上限，超限 SOLVER_TIMEOUT
    max_expansions: int                # 确定性上限，来自 registry
    target_width_range_mm: tuple[float, float] = (90.0, 120.0)
    target_height_range_mm: tuple[float, float] = (50.0, 90.0)
    ambiguity_score_gap: float | None  # None=未冻结→分数间隔不足一律 AMBIGUOUS
    partial_offset_step_mm: float
    abs_tolerance_mm: float
    rel_tolerance: float
    vertex_tolerance_mm: float

@dataclass(frozen=True)
class TextureEvidence:               # 仅纹理阶段；白色路径必须为 None 且有无纹理路径的测试
    rectified_image: np.ndarray      # 只读
    piece_masks: dict[str, np.ndarray]   # piece_id -> mask，键必须 ⊆ 观察 piece_id

@dataclass(frozen=True)
class PlacementPlan:
    piece_id: str
    order: int
    source: Pose2   # {frame:"table_mm", center_mm, yaw_deg}
    target: Pose2   # yaw 为 authoritative target yaw
    geometry_score: float
    texture_score: float | None
    texture_status: str
    plan_confidence: float
    source_timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]
    solver_version: str

def solve_puzzle(
    workspace: WorkspaceState,        # pieces + source/target region + locked_placements
    target_region: TargetRegion,
    options: PuzzleOptions,
    texture_evidence: TextureEvidence | None = None,
) -> PuzzleSolveResult: ...
```

**边界**：`solve_puzzle` 纯函数——不读文件、不 import vision、不做 cv2 分割、不访问时钟（计时用注入的单调时钟或 `time.monotonic_ns` 仅用于预算检查）、同输入字节级稳定输出（耗时字段除外）。`WorkspaceState` 中图像/mask 引用仅日志用途，由 integration adapter 解析成 `TextureEvidence` 传入。生产签名里**不得出现** `cut_mode`、生成器真值、真值邻接、真值变换。

### 4.2 求解器算法合同（私有实现，文件建议）

```
workspace/src/puzzle/_edges.py        # EdgeDescriptor 构造与候选匹配
workspace/src/puzzle/_search.py       # 邻接图回溯 + SE(2) 传播 + 剪枝
workspace/src/puzzle/_refine.py       # 纯 NumPy 有界最小二乘闭环细化
workspace/src/puzzle/_target.py       # 目标矩形尺寸推断
workspace/src/puzzle/_scoring.py      # 栅格/解析评分分项
workspace/src/puzzle/_dedup.py        # 规范化、等价类聚类、放置顺序
workspace/src/puzzle/solver.py        # solve_puzzle 编排
```

**边描述**（`_edges.py`）：每片由 `polygon_table_mm` 生成有向边：长度、单位方向、直线残差、两端点、相邻内角/转角、凹凸性（叉积符号）。候选容差 = `abs_tolerance_mm + rel_tolerance × min(L1,L2)`（不用外部 12% 固定值）。

**三类匹配候选**：

1. 完整边匹配：|L1−L2| ≤ tol，独占两条边。
2. 端点锚定部分边：短边 S 映射到长边 L 的前缀或后缀，offset ∈ {0, L−S}。
3. 内部部分边（T 形）：offset 为连续有界变量 ∈ `[min_residual, L−S−min_residual]`，`min_residual = 20.0 − vertex_tolerance_mm`（题面最小边长推导；若现场规则证据不同再改）。初值 = 毫米级确定性粗网格（`partial_offset_step_mm`）∪ 其他接触长度隐含断点；细化阶段做有界连续优化。必须覆盖"90 mm 长边 offset=37 mm"夹具。

候选**按 piece pair 分组保留**，不做全局 top-N 截断（避免丢真实邻接）。完整匹配独占两条边；部分匹配在长边上登记占用区间，同一长边允许多个互不重叠区间，禁止区间重叠；装配完成后校验所有内部边被覆盖。

**枚举与位姿**（`_search.py`）：回溯枚举连通邻接图；每条候选边对齐生成 SE(2)（仅旋转+平移，`det(R)=+1`，禁止反射/缩放）；gauge 固定第一片。剪枝（在扩展前做）：面积守恒、AABB 越界、碎片对明显重叠、不可闭合、目标尺寸范围、外边可行性、扩展上限。**每片至少一条目标外边是硬约束**（带测量容差）；若清空候选 → `NO_SOLUTION` + 诊断，不降级为软猜测。

**闭环细化**（`_refine.py`）：纯 NumPy 实现有界最小二乘（法方程 + box 约束投影或自写 Levenberg 阻尼），**不新增 SciPy 依赖**；固定 gauge；角度显式 wrap 到 [-π,π) 避免 2π 分支漂移。

**目标尺寸推断**（`_target.py`）：`width ∈ [90,120]`、`height ∈ [50,90]` 按 1 mm 网格 × 方向交换生成种子；总面积约束 `|Σpiece_area − W·H|` 与轮廓/外边长度先剪枝，再连续细化。这是对题面"9cm×5cm~12cm×9cm"文字范围的保守工程解释，设计文档标注待现场确认。落点必须在 `TargetRegion` 内（A4 下半，y < 分界线）。

**评分**（`_scoring.py`）：统一 penalty 越低越好，分项全部记录：
边匹配残差；闭环端点误差；碎片重叠面积（栅格化或解析多边形交）；矩形缺口/越界面积；总面积守恒偏差；外轮廓矩形度与周长；外边拟合残差（硬约束通过后的 tie-breaker）；部分边/复杂度惩罚。栅格尺度由 px_per_mm 等效 mm 分辨率决定并记录。

**等价/歧义/超时**（`_dedup.py` + solver）：

- 候选规范化后按等价类去重：允许全局目标旋转/平移（在 TargetRegion 内）与碎片自身对称（`equivalent_yaws_deg`）；等分矩形/相同矩形必须返回合法等价类而非伪造身份。
- 最佳 vs 次佳比较发生在**去重之后**；非等价分数间隔 < `ambiguity_score_gap` → `AMBIGUOUS`（不携带可执行计划）；gap 未冻结时任何非唯一解都 AMBIGUOUS。
- `expansions > max_expansions` 或 `elapsed > time_budget_ms` → `SOLVER_TIMEOUT`，**不得携带部分最优计划**。
- 确定性：全序 sort key（长度→角度→piece_id 字典序）、固定浮点序列化（如 round 到 1e-6 或 repr）、禁止 set/dict 遍历顺序影响结果、无并行。同输入连续运行输出字节级相等（timing 字段除外）——回归测试断言。
- 放置顺序：边界/角块优先，净空不足后置，同输入稳定。每份 plan 绑定 `source_timestamp_ns`；过期由调用方拒绝。
- 1 片平凡分支：单片多边形本身是合法矩形且在目标尺寸范围内 → 直接 SOLVED（smoke 分支，不走过场搜索）。

---

## 5. 固定种子合成夹具合同

新文件 `workspace/tests/fixtures/puzzle_fixture_generator.py`（**本仓原创**，文件头注明程序生成、无外部素材）：

- **几何族**（7 类）：中心放射、边界扇形、条带、等分矩形、T 形（含端点锚定与 90mm/37mm 内部 offset 两例）、角块、凹多边形；片数 2/3/4 + 1 片合法矩形 smoke。
- **合法性自校验**：生成器自身拒绝违反题面的夹具（边数 >5、边长 <20 mm、无目标外边、自相交）；非法参数 → raise。
- **渲染**：程序生成纯白片与程序花纹（自绘线条/圆点，禁止外部 Joker/图片素材）；圆角/直角两版；金属背面负例（灰金属渐变）。扰动：随机旋转、照度梯度（线性/径向亮度场）、高斯模糊、传感器噪声、软阴影、PNG 编解码往返。`random.Random(seed)` 显式种子，禁止全局 random/np.random 隐式状态。
- **场景**：A4 上下半 + 分界线；碎片互不重叠（题面保证）但允许接触与 0–2 mm 间隙；异常大连通域。
- **真值隔离**：生成器输出 `(png_bytes, ground_truth)`；生产函数只接收 PNG 重载像素（`cv2.imdecode`）或序列化 observations；ground_truth 只进断言。专门测试：扫描生产 interface 的传参，断言无真值字段泄漏（如检查 fixture 测试里不把 ground_truth 传给 observe/solve）。
- 夹具落盘可选（`workspace/tests/fixtures/generated/`，内容哈希由 seed 决定，测试内重建即可，不入库大图）。

---

## 6. 精确文件清单

### 新增（代码）

| 文件 | 内容 |
| --- | --- |
| `workspace/src/vision/generic_piece_observer.py` | §2.2 全部公共类型 + `observe_pieces` 编排 |
| `workspace/src/vision/_segmentation.py` | 背景建模、多阈值、形态学、连通域、merged/surface 判定 |
| `workspace/src/vision/_piece_geometry.py` | 多边形候选、圆角虚拟化、距离变换抓取点、姿态/等价 yaw、置信度组合 |
| `workspace/src/vision/legacy_red_adapter.py` | RedPieceObservation → PieceObservation（geometry_source 标记） |
| `workspace/src/puzzle/__init__.py` | §4.1 公共导出 |
| `workspace/src/puzzle/solver.py` + `_edges/_search/_refine/_target/_scoring/_dedup.py` | §4.2 |
| `workspace/tools/analyze_puzzle_frame.py` | 离线回放：原图+校正参数 → 公共 interface → 保存掩码/轮廓/JSON |
| `workspace/tools/benchmark_puzzle_pipeline.py` | 固定 corpus 基准：延迟/RSS/结果哈希/版本 |

### 新增（测试）

| 文件 | 断言要点 |
| --- | --- |
| `workspace/tests/test_red_piece_replay.py` | §1.2：4 片、中心 ≤1 mm、角度容差（等价换算） |
| `workspace/tests/test_puzzle_data_contracts.py` | 新 schema 可解析；正/反例（§3.1 负例）；`additionalProperties:false`；yaw `exclusiveMaximum:180`；legacy `latest_pieces.json` 校验（如适用） |
| `workspace/tests/test_fixture_generator.py` | 7 族 × 2/3/4 片可生成；非法夹具被拒；同 seed 字节级一致；真值不泄漏进生产签名 |
| `workspace/tests/test_generic_observer_synthetic.py` | 名义分离：7 类×2/3/4 片×≥20 seeds 片数正确率 100%；1 片 smoke；顶点误差 p95 ≤2 mm / max ≤3 mm（报告标"合成"）；凹性保留 100%；白色 profile 圆角替换数==0；接触/0–2mm/背面/分界线/异常 blob 压力集：正确拆分或显式拒绝率 100%、高置信错误 0 |
| `workspace/tests/test_pick_clearance.py` | footprint 未冻结 → 不可抓（None）；冻结后抓取点在腐蚀掩码内 |
| `workspace/tests/test_puzzle_solver_unit.py` | 边描述/三类候选（含 90mm/37mm offset）；SE(2) `RᵀR≈I`、`det=+1`；评分分项方向统一；角度 wrap 无 2π 漂移 |
| `workspace/tests/test_puzzle_solver_synthetic.py` | 几何真值级 7 类×2/3/4×30 seeds（≥630 例）第一等价类正确、镜像解 0；1 片平凡分支；4 块相同矩形返回等价类；独立 evaluator（union 缺口/重叠/外矩形/逐片 SE(2)，不复用 solver score） |
| `workspace/tests/test_solver_equivalence.py` | 去重后最佳/次佳间隔不足 → AMBIGUOUS；超时/超 expansions → SOLVER_TIMEOUT 无部分计划；同输入字节级稳定（除耗时） |
| `workspace/tests/test_no_generator_truth_leak.py` | 生产路径只收 PNG 像素/序列化 observation |

### 修改

| 文件 | 改动 |
| --- | --- |
| `workspace/src/vision/__init__.py` | 追加 §2.1 导出（旧导出不动） |
| `data_contracts/piece_observation.schema.json` | §3.1 |
| `data_contracts/placement_plan.schema.json` | §3.2 |
| `data_contracts/workspace_state.schema.json` / `puzzle_solve_result.schema.json` | 新增，§3.3/3.4 |
| `configs/parameter_registry.yaml` | §3.6 |
| `docs/10_接口与数据契约.md`、`docs/06_拼图求解器.md`、`docs/19_决策门与停止条件.md`、`docs/01_赛题需求追踪矩阵.md`（如口径需对齐）、`docs/12_测试与验收计划.md`（证据标签） | §3.5 |
| `workspace/src/vision/README.md`、`workspace/src/puzzle/README.md` | 新 interface 说明 |
| 新增 `docs/references/puzzle_vision_simulator_v2_2.md`、`docs/24_通用视觉与拼图求解器设计.md` | 来源冻结 + 设计固化 |

### 不改

`workspace/src/control/`、`robot_plugin/`、`policy/`、`workspace/apps/`（shadow 按钮属 Phase 4 后续）、`red_piece_detector.py` 行为、`workspace/tools/detect_red_pieces.py`。

---

## 7. Gate 划分：现在能软件完成 vs 必须实拍/盲测

**本轮纯软件可完成（开发机+合成+已有回放）**：Phase 0 文档冻结与红色回放回归；Phase 1 全部契约/schema/夹具生成器/真值隔离；Phase 2 的合成部分（分割、轮廓、圆角、抓取点、诊断 + 合成压力集全部验收数字）；Phase 3 全部（630+ 夹具、等价/歧义/超时、确定性、推断矩形 p95 ≤3 mm 合成 holdout、开发机性能记录）；Phase 4 的工具与 shadow 接线代码、开发集/holdout 框架。

**必须实拍/Pi/盲测才能启用（本轮只能留 UNKNOWN + 拒绝路径）**：

- Phase 2 真机退出条件：实拍白色/印刷牌原图集；静态 30 帧中心 std ≤1 mm；20 网格点 ≤3 mm；100 帧无持续漏检。
- Pi 5 门槛：白色 p95 ≤2 s / max ≤5 s / RSS ≤250 MB（必须 Pi 实机日志，开发机数字不得外推）。
- Gate C：≥20 套真实/实体模拟白色盲测 ≥18/20 + 实机连续 5 次；`generic_vision_enabled` 此前保持 false。
- Gate E（纹理）：≥40 个几何歧义 holdout、top-K 召回 ≥95%、纹理 top-1 ≥90% 且绝对提升 ≥20pp；未过则 texture 永久关闭但白色功能保留。`texture_top_k`、接缝参数在召回/延迟曲线出来前一律 UNKNOWN。
- 真机动作冻结项：正式 `table_mm → robot_base_mm` 变换+SHA256、PICK/PLACE 接触高度（当前 UNKNOWN）、磁头 footprint/净空门槛。未冻结前 pick_point 判不了"可抓"，只能 shadow。
- 现场 UNKNOWN：纸面是否保证朝上、是否保证可见间隙、现场碎片数量下限、目标尺寸文字范围解释——设计文档登记，代码走拒绝路径，不猜。

---

## 8. 最小但优秀的算法取舍（给 writer 的定调）

1. **一个观察、两套几何**：物理轮廓（抓取/纹理/诊断）与虚拟多边形（求解）从同一 Piece 派生，不做跨实例 IoU 匹配。这是外部实现最值得吸收、成本最低的思想。
2. **多阈值稳定性 > 花哨分割**：3–5 个相邻阈值 + 一致性投票即可满足"正常室内照明"；HSV/Lab 多假设融合只在真实 holdout 证明需要时才加。
3. **圆角虚拟化默认关**：只对 printed-card profile 开；白色金属片 profile 替换数必须为 0（测试锁死）。物理轮廓永远保留，回退成本为零。
4. **部分边只建三类候选**（完整/端点锚定/内部有界 offset），内部 offset 用粗网格初值+有界连续细化，不做全连续优化——4 片规模下确定性足够。
5. **候选按 pair 分组，不做全局 top-N**：小改动、大召回收益。
6. **评分全部归一到 mm/无量纲、统一 penalty 方向、分项落盘**：可解释性比拟合精妙更重要；歧义宁可 AMBIGUOUS 不可伪造唯一解。
7. **1 mm 网格种子 + 连续细化推断目标尺寸**：题面范围小（31×41 格×2 方向），暴力网格即最优取舍。
8. **不新增 SciPy**：纯 NumPy 法方程/阻尼最小二乘；不引入神经网络。
9. **纹理只重排几何 top-K**：几何硬约束不动；纹理失败显式状态，不静默回退。
10. **置信度分项先行、映射后冻结**：本轮输出分项与 UNKNOWN 阈值，不写假校准曲线。

---

## 9. 验证环境与命令（实测）

- 开发机 `python3`（miniconda base）有 numpy 2.5.1，**无 cv2、无 jsonschema**；conda 环境 `yolo`、`mineru-client` 有 cv2 4.x + numpy（writer 视觉/夹具测试可用 `conda run -n yolo python -m unittest ...`，或建项目 venv 装 `opencv-python-headless`——注意不要把依赖写进 Pi 交付 wheelhouse 契约之外的地方）。
- 契约测试保持现有风格：纯 `json.loads` 结构断言即可在 base python 跑；实例级正/反例校验若用 jsonschema 需先在有该包的环境跑，或实现极简手写 validator（推荐后者，避免新依赖）。
- 基线测试命令（PLAN 指定）：

  ```bash
  PYTHONPATH=workspace/src python -m unittest discover -s workspace/tests -v
  python -m compileall -q workspace/src workspace/tools
  ruff check workspace/src workspace/tools workspace/tests   # ruff 0.15.22 在 ~/.local/bin
  ```

  现状基线：`python3` 无 PYTHONPATH 时 7 个 import error；加 PYTHONPATH 后 38 tests，仅 `test_so101_5d_contract` 因缺 `lerobot_robot_so101_5d` 失败（**预存在失败，与本轮无关，不要顺手修，也不要让它变得更糟**）。
- 视觉/求解新测试需 cv2：用 yolo 环境跑，命令形如 `export PATH="/home/gxxl/miniconda3/bin:$PATH" && PYTHONPATH=workspace/src conda run -n yolo python -m unittest workspace.tests.test_generic_observer_synthetic -v`（提交记录中注明所用环境）。

---

## 10. Meta-prompt 交接合同（给 writer）

- **Goal**：按 §6 文件清单落地 Phase 0–3 的软件部分与 Phase 4 工具代码；公共 interface 与契约冻结；全部新测试在合成/回放层通过；红色基线回归无变化。
- **Evidence**：本文件 §0–§9；PLAN.md 与 context.md（取舍冲突时以 PLAN 为准）；题面 txt 行号证据。
- **Success criteria**：§6 测试全部通过（视觉类在含 cv2 环境）；schema 正反例通过；`detect_red_pieces` 回放 4 片容差内；求解器 630+ 夹具第一等价类正确、镜像 0、字节级确定性；ruff/compileall 干净；预存在的 `test_so101_5d_contract` 失败不扩大。
- **Hard constraints**：不改 control/robot_plugin/policy/apps；不改红色检测行为；不复制外部无 LICENSE 代码/素材（夹具全部程序生成）；不新增 SciPy/神经网络依赖；`generic_vision_enabled=false`、`texture_enabled=false` 默认不动；所有跨模块字段带 frame/单位；UNKNOWN 不得捏造数值；schema 保持 `additionalProperties:false`。
- **Suggested approach**：顺序 = 契约+文档（Phase 0/1）→ 夹具生成器+真值隔离测试 → vision 观察器（先分割+多边形，再圆角+抓取点）→ solver（边候选→枚举→细化→评分→去重）→ 工具。每步先写失败测试。
- **Validation**：§9 命令；新视觉测试用 yolo conda 环境；结束前全量 unittest + ruff + compileall。
- **Stop/escalate**：题面解释与 PLAN 冲突、需要启用真机/改默认 flag、发现外部素材混入、或需在 dev 机永久安装新系统依赖时，停下来走 intercom 请示。
- **Resolved assumptions**：table_mm 原点=A4 左下、y 向上（registry 已冻结）；目标尺寸按 90–120 × 50–90 mm 1 mm 网格+方向交换；`min_residual = 20 − vertex_tol`；评分方向统一 lower-is-better；开发机无 cv2 用 conda yolo 环境验证。

---

## 11. 残余风险

1. 回放 JSON（`red_pieces_detection.json`）的 `angle_deg` 与检测器 `yaw_deg` 归一化区间不同，回放测试需先做等价换算并复跑打印实测值后定容差——容差未定前测试可能红。
2. 开发机缺 cv2/jsonschema，验证依赖 conda yolo 环境；若该环境被改动需备选（pip venv 装 opencv-python-headless）。
3. `min_residual=20−vertex_tol` 与目标尺寸文字解释均为工程保守解释，现场规则确认后可能需调整（已在设计文档登记 UNKNOWN）。
4. 1 片现场碎片是否可能出现未由题面保证；平凡分支已实现但盲测构成若全为 2–4 片，该分支只被 smoke 覆盖。
5. Phase 2 合成验收指标（顶点 p95 ≤2 mm 等）若分割 pipeline 取舍过简可能首轮不达标；按 PLAN 停止条件先修轮廓简化/多阈值，不得放宽指标或放宽 schema。
