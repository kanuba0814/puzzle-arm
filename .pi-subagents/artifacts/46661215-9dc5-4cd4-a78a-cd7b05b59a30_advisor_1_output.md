# Architecture & Algorithm Feasibility Critique — adopt-puzzle-vision-v2-2

Scope: read-only review of `.taskman/plans/adopt-puzzle-vision-v2-2/PLAN.md` + `context.md`, cross-checked against `docs/05`, `docs/06`, `docs/10`, `docs/12`, `docs/19`, `configs/parameter_registry.yaml`, `data_contracts/*.schema.json`, the current `workspace/src` tree, and the official `official_docs/competition/E题_拼图装置.pdf` (text extracted with `pdftotext`).

## Inherited decisions (baseline contract)

- Independent re-implementation only; external repo frozen at tag `v2.2.0-vision-rounded-card` / commit `fbd9c1c…`; no LICENSE → no copied code/assets.
- Two deep modules with minimal public seams: `vision.observe_pieces(frame, profile) -> VisionResult`, `puzzle.solve_puzzle(workspace, target_region, options) -> PuzzleSolveResult`.
- Physical contour AND virtual right-angle polygon both preserved; no unconditional convex hull; no `cut_mode` in production.
- Target rectangle inferred within official 90×50–120×90 mm, never fixed 100×60.
- Geometry first, texture only re-ranks geometric candidates; no silent fallback; ambiguity → `AMBIGUOUS`, no motion.
- Red HSV path (`detect_red_pieces`, `RedPieceObservation`, `workspace/tools/detect_red_pieces.py`, GUI caller at `workspace/apps/pick_place_page.py:167`) stays default; new path behind feature flag; shadow before any motion; PICK/PLACE heights UNKNOWN block real execution.
- Official constraints verified from the PDF: ≤4 pieces, ≤5 edges/piece, every edge ≥2 cm, each piece ≥1 edge on the target outer boundary, target 9×5 cm ~ 12×9 cm, vertex-pair error ≤2 cm, ≤2 min, A4 portrait with pieces random non-overlapping in the upper half, assembly in the lower half, normal indoor lighting.

## Overall verdict

The plan is architecturally sound: the deep-module seams match the AGENTS.md boundaries, the "reject list" (cut_mode, fixed target, convex hull, area→confidence, silent texture fallback) is correct, and the staged offline → Pi benchmark → shadow → Gate C/E integration honors the project's safety gating. Nothing below requires a pivot. However, there are 4 high-severity defects (one contract contradiction, one algorithmic ambiguity that implementers will likely get wrong, two unstated physical assumptions) and a series of medium issues in acceptance-criteria math, error-code sync, and contract details that must be amended before implementation agents start.

---

## Findings

### HIGH

**H1. Frame contradiction when adding `contour_rectified_px` to PieceObservation.**
`data_contracts/piece_observation.schema.json` declares a top-level `"frame": {"const": "table_mm"}` with `additionalProperties: false`. PLAN Phase 1 adds `contour_rectified_px` (rectified pixel coordinates) into this same object. That makes one object claim `frame=table_mm` while carrying a field in `rectified_px` — a direct violation of docs/10 ("所有接口必须写明坐标系") and an invitation for control-layer consumers to treat pixels as mm.
*Amendment:* in Phase 1, restructure to per-field frames, e.g. `contour: {frame: "rectified_px", points: [[x,y],…]}` and keep mm fields under the existing table_mm contract (or add an explicit `contour_frame` const). Add a negative schema test that a rectified_px contour at top level with `frame=table_mm` is rejected. Also note `additionalProperties:false` means the new field must be added to `properties` explicitly — trivial, but agents sometimes "fix" this by relaxing `additionalProperties`, which must be forbidden.

**H2. Partial-edge / T-junction candidate definition is ambiguous; the literal reading of "首段/末段" misses interior T-junctions.**
PLAN Phase 3 step 3: "支持一条长边对应短边的首段/末段和合法 T 形分割". A true T-junction has the short edge's endpoint landing in the *interior* of the long edge at an offset that is not derivable from any single piece's edge lengths (the split point is not a vertex of the long-edge piece). An implementer reading "首段/末段" will enumerate only endpoint-coincident sub-segment matches (offset = 0 or L−S), which are not T-junctions at all, and the acceptance line "T 形夹具确实使用部分边候选" can then pass while genuine interior T cases silently fail or produce wrong assemblies.
*Amendment:* define two distinct candidate classes in `docs/24`:
(a) endpoint-coincident sub-segment match (offset ∈ {0, L−S});
(b) interior T-junction with a continuous offset variable bounded by the ≥20 mm edge rule (both residual segments of the long edge ≥ 20 mm, or ≥ a frozen epsilon if the 20 mm rule applies only to true piece edges — decide and document which), the offset being fixed during pose propagation/loop closure and refined as a bounded free variable in the NumPy least-squares step.
Require a dedicated fixture whose T offset is an "awkward" value (e.g. 37 mm on a 90 mm edge, not L/2) to catch hardcoded or midpoint-only implementations, and a dedup rule so near-endpoint T candidates (offset < epsilon) don't duplicate class (a).

**H3. Unstated face-up / chirality assumption conflicts with reflection ban.**
The solver correctly forbids reflections (`det(R)=+1`, "非法镜像解为 0"), but this is only valid if fragments are always placed paper-side-up. The official text says only "随机不重叠摆放" — face orientation is not specified. A paper-side-down fragment presents (a) a mirrored silhouette that the reflection-banning solver can never assemble, and (b) a galvanized-metal surface that breaks the white-paper segmentation profile (docs/05 already mentions 镀锌反光 as a worst case in T1).
*Amendment:* record in `docs/24` and Phase 0 acceptance an explicit assumption "fragments are placed printed/paper side up; to be confirmed against competition procedure, marked UNKNOWN until confirmed". Add a metal-side-up fixture whose required behavior is explicit rejection with a diagnostic error code (e.g. `VISION_SURFACE_MISMATCH`), never a mirrored solution. Add this to the risk table.

**H4. Touching-piece segmentation gap makes "碎片数量正确率 100%" unfalsifiable on real scenes.**
Official placement is "随机不重叠" — non-overlapping but contact/near-contact is possible. Two touching white fragments merge into one connected component; the Phase 2 pipeline as specified (open/close + connected components + area filter) will then miscount, while the synthetic corpus (implicitly separated fixtures) still passes 100%.
*Amendment:* (1) add touching and 0–2 mm-gap fixtures to the Phase 1 generator; (2) require merged-blob detection using official bounds (blob area > max legal piece area derivable from 120×90 target and ≥2 cm edges, or fitted polygon vertex count > 5, or abnormal concavity), producing either an erosion/watershed split attempt with a confidence penalty or an explicit rejection with error code (e.g. `VISION_MERGED_BLOBS`) — never a silent wrong count; (3) state the non-contact assumption in `docs/24` and require violation detection rather than reliance.

### MEDIUM

**M1. Gate C numbers conflict with `docs/19`.** docs/19 Gate C: "至少 10 套盲测中 9 套几何解正确，实机连续 5 次成功". PLAN Phase 4: "20 套中 ≥18 套" offline, and defers real-machine to docs/19. Two different denominators for the same gate with no reconciliation.
*Amendment:* in Phase 0 (docs freeze) or Phase 4, edit `docs/19` Gate C to define both: offline blind-set ≥18/20 AND real-machine 5 consecutive; state explicitly that the plan supersedes the 10/9 figure. Update `docs/01_赛题需求追踪矩阵.md` accordingly.

**M2. Gate E "较 geometry-only 至少提升 20 个百分点" is either vacuous or impossible.** The holdout is defined as "几何歧义场景" → geometry-only top-1 is near chance by construction → +20pp is trivially met and proves little. Conversely, if geometry-only ≥80%, +20pp is mathematically impossible (ceiling 100%).
*Amendment:* criterion = texture top-1 ≥90% AND absolute improvement ≥20pp over the geometry-only baseline *measured on the same holdout*, AND holdout composition constrained so geometry-only top-1 ∈ [30%,70%] by design (mix of 2-way and k-way ambiguous cases); report both numbers. Add a paired significance note (e.g. McNemar) as a diagnostic, not a gate.

**M3. New error codes are not synced into docs/10, and the solver has no internal time budget.** PLAN introduces `SOLVER_AMBIGUOUS`, `TEXTURE_INSUFFICIENT`, `TEXTURE_AMBIGUOUS`, but docs/10's error-code list contains none of them. Separately, the only guard against combinatorial blowup is the 120 s global supervisor timeout — the risk table says "超限拒绝" but no mechanism/code is defined.
*Amendment:* Phase 1 must extend the docs/10 error-code list (also add `SOLVER_TIMEOUT`, `VISION_MERGED_BLOBS`, `VISION_SURFACE_MISMATCH` per H3/H4). Add config `solver_time_budget_ms` (well under 120 s, e.g. 10–20 s) returning `SOLVER_TIMEOUT` with candidate statistics; the global timeout remains the outer guard.

**M4. `yaw_deg` semantics are undefined and dangerous.** `minAreaRect/PCA` yaw is ambiguous mod 90°/180°; the schema bounds it to ±180 with no convention. `PlacementPlan.source` consumes a yaw; an implementer may treat the observation yaw as final despite Phase 2 step 6 saying it is initial-only.
*Amendment:* define the convention in `docs/24` + schema `description` (e.g. "first principal axis, CCW from table_mm +x, range (−180,180]"); require `equivalent_yaws_deg` in diagnostics; add a contract note that observation yaw is a hint and only the solver/template output is authoritative.

**M5. Rounded-corner virtualization is not explicitly profile-gated.** The 取舍 table correctly rejects the "4 rounded corners" hard rule, but Phase 2 implementation steps apply "长直边—短弧—长直边" replacement generically. White galvanized+paper fragments are shear/laser cut and likely have near-right corners with burrs; unconditional virtualization can eat real cut vertices — the plan's own risk table lists this.
*Amendment:* make corner recovery enabled only under an explicit profile flag (printed-card profile, or config `corner_virtualization: enabled|disabled`), default conservative for the white metal profile. Acceptance must include white-profile negative fixtures proving zero replacements, and the per-replacement diagnostics (chord depth, fit angle, residual — already required) must record the profile under which they ran.

**M6. Reproducibility fields missing from `RectifiedFrame`/`VisionResult`.** "字节级稳定" replay and auditable mm output require recording: homography file SHA256, `px_per_mm`, serialized `VisionProfile` + profile version, source image id and frame timestamp. PLAN shows `RectifiedFrame(table_mm mapping)` but never requires these fields.
*Amendment:* add them to the Phase 1 contract work (VisionResult.diagnostics sub-schema, see M8) and to `workspace_state.schema.json`.

**M7. `WorkspaceState` must optionally carry imagery for the texture stage, or the module seam will be violated.** Phase 5 seam sampling needs the rectified image plus per-piece masks mapped by candidate poses. If `workspace_state.schema.json` (Phase 1) lacks these, implementers will either re-segment inside `puzzle` or reach into `vision` internals — breaking the deep-module seam the plan mandates.
*Amendment:* WorkspaceState includes optional `rectified_frame_ref` (image reference + homography id + px_per_mm) and per-piece mask references, consumed only by the texture scorer; the white path must construct and solve without them (test this).

**M8. `diagnostics` is a free-form object in the schema.** The plan repeatedly requires recording sub-scores, scoring direction, corner-replacement details, and candidate counts; with an unconstrained `diagnostics` object these will drift per implementer and per phase.
*Amendment:* freeze a diagnostics sub-schema in Phase 1 (segmentation stability, polygon fit residual mm, pick clearance mm, corner candidate count, border-clipped flag, equivalent yaws, solver per-stage timings, candidate counts, score components + direction) with positive/negative schema tests; keep an `extra` escape hatch if needed, but typed for the required fields.

**M9. Tolerance/confidence constants must live in `parameter_registry.yaml`, not in implementers' heads.** "绝对测量误差 + 相对误差" (Phase 3 step 2), the AMBIGUOUS best/runner-up gap, seam band width/margin (Phase 5), and the new generic-vision confidence threshold are all unspecified numerically. Also `vision.minimum_piece_confidence: 0.12` in the registry is the red-path value and must not be silently reused.
*Amendment:* add registry keys (`edge_match_abs_tol_mm`, `edge_match_rel_tol`, `vertex_tol_mm`, `ambiguity_score_gap`, `seam_band_width_mm`, `seam_margin_mm`, `generic_vision_min_confidence`, `corner_virtualization`, `solver_time_budget_ms`, `px_per_mm`) initialized `UNKNOWN` and frozen from *logged* Phase 2/3 measurements — the context's "error model from Phase 2 residuals" principle must apply to the constants too.

**M10. Phase 3 "加入 Phase 2 观测误差后 ≥99%" lacks a defined error model.** Without specifying the injected error distribution, 99% is unfalsifiable.
*Amendment:* the error-injection parameters (vertex noise σ in mm, outlier rate, occasional merged/missed piece) must be derived from logged Phase 2 synthetic and real residuals, stored with the fixtures, and cited in the test evidence label.

### LOW

**L1. Target-rectangle inference: pin the official range interpretation and enumeration.** "9cm×5cm ~ 12cm×9cm 之间" is ambiguous between independent per-dimension ranges and corner points only. *Amendment:* enumerate a 1 mm grid, w∈[90,120] ∧ h∈[50,90] (1271 candidates, trivially cheap) pruned by total-area conservation ± measured tolerance and assembled-outline AABB; include w/h orientation swap since the target may be placed either way in the lower half; document the interpretation in `docs/24` and flag for confirmation against the PDF/judges in Phase 0.

**L2. "每片至少一条目标外边" should be a hard pruning constraint, not only a scoring term.** Phase 3 step 8 lists it among score items; the PDF states it as a cut property, so any correct assembly satisfies it. *Amendment:* use it as a hard filter during search (strong pruning) with an explicit diagnostic if it empties the candidate set (indicating observation error), and keep a score term only as tie-breaker.

**L3. A4 boundary-line fixture.** The ≤5 mm solid 分界线 sits between the piece region (upper half) and the target region (lower half); it can segment as a false piece or bridge touching pieces. *Amendment:* add a fixture with the boundary line; note in `docs/24` that detection ROI is the upper half and `TargetRegion` the lower half per the official procedure.

**L4. Determinism needs concrete rules.** "字节级稳定（耗时字段除外）" requires: total-order sort keys for all candidate lists (no set/dict iteration-order dependence), fixed float serialization, no wall-clock in result hashing, single-threaded or order-deterministic NumPy. *Amendment:* state these rules in `docs/24` Phase 3.

**L5. Wording traps for implementers.** (a) Phase 2 step 3 "生成 3～5 顶点候选" reads as both "polygons with 3–5 vertices" and "3–5 candidate polygons" — clarify: vertices per polygon ∈ {3,4,5} (official ≤5 edges) AND multiple hypotheses retained. (b) State score direction explicitly per component ("记录评分方向" is required but give the convention in `docs/24`). (c) Clarify that best/runner-up comparison happens *after* equivalence-class dedup.

**L6. Execution precondition list is incomplete.** Phase 6 gates on PICK/PLACE heights (good) but omits the still-unfrozen formal `table_mm → robot_base_mm` transform (docs/10: "正式视觉链路仍需独立冻结"; current registry only has the contact-sheet transform for 2026-07-31 evidence). *Amendment:* add it to the Phase 6 preconditions for any post-Gate-C real execution; shadow mode remains unaffected.

**L7. Registry sync scope.** Phase 4 step 5 syncs versions/profile/thresholds; add OpenCV/Python freeze values when measured (`opencv: UNKNOWN`, `python: UNKNOWN` today), the chosen `px_per_mm`, and the new feature flags (`generic_vision_enabled`, `texture_enabled`) explicitly as booleans defaulting false, matching Phase 6 item 1.

---

## Drift / contradiction check

- The plan does **not** drift from inherited constraints on module boundaries, five-axis order, magnet control, LeRobot pinning, or evidence labeling — it explicitly reinforces them.
- Silent assumption changes detected: (1) frame semantics of PieceObservation quietly become mixed-frame (H1); (2) "partial edge" quietly narrows to endpoint-coincident matching (H2); (3) face-up placement and non-contact placement are assumed but never stated (H3, H4); (4) Gate C acceptance silently re-denominates from docs/19 (M1).
- No conflict with the frozen red baseline: Phase 0 replay uses the existing evidence at `workspace/calibration/evidence/2026-07-31_gui_a4_workspace/red_pieces_frame.jpg` (verified present); `detect_red_pieces` callers (`workspace/tools/detect_red_pieces.py`, `workspace/apps/pick_place_page.py`) are untouched by the plan.

## Recommendation

Proceed with the plan after folding in amendments H1–H4 (contract + algorithm-definition level, cheap to fix now, expensive later) and M1–M10 (acceptance math, error codes, registry constants). All amendments are narrow edits to PLAN/context and the Phase 0/1 deliverables — no phase restructuring, no pivot. Priority order: fix H1/H2 in the Phase 1 contract-and-design-doc work *before* any solver code is written, since both define the seam semantics; H3/H4 belong in Phase 0 assumption logging plus Phase 1 fixtures.

## Risks / remaining uncertainties

- Face orientation of officially placed fragments (H3) is genuinely UNKNOWN until confirmed with the procedure/judges; the plan should carry it as an explicit assumption with a rejection path rather than resolve it by guesswork.
- Official target-range interpretation (L1) is ambiguous in the PDF text; the 1 mm full-grid enumeration makes the solver robust to either reading, so this is documentation-only risk.
- Pi 5 budgets (p95 ≤2 s white, ≤5 s printed, RSS ≤250 MB) are plausible but unverified; the plan correctly forbids inferring Pi performance from dev-machine results.
- The 18/20 and ≥99% numbers are attainable only if H4 touching cases and the M10 error model are defined; otherwise they are unfalsifiable rather than hard.

## Need from main agent

None blocking. One decision worth confirming when amending: does the ≥20 mm minimum-edge rule constrain the *residual segments* of a T-junction long edge (i.e., is the T offset bounded to [20 mm, L−20 mm]), or only true piece edges? Recommend the conservative reading (bounded residuals) and documenting it in `docs/24`.

## Suggested execution prompt

No implementation handoff is warranted from this review. The amendments above are edits to `PLAN.md`/`context.md` (and later Phase 0/1 docs), best applied by the planning agent itself before any `worker` is launched for Phase 1 contracts or Phase 3 solver code.