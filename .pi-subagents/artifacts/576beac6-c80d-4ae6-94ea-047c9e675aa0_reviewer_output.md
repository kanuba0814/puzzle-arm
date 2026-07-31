# Final Review: adopt-puzzle-vision-v2-2 PLAN.md + context.md

**Verdict: APPROVED** — no blockers, no major amendments required. Four minor notes below are resolvable in-flight during Phase 0/1 and do not gate plan start.

## Review

### Correct (verified against repo)

- **Priorities & safety preserved.** Plan §1 非目标 and Phase 6 keep control/IK/magnet/ACT untouched, new pipeline is feature-flagged off (`generic_vision_enabled=false`, `texture_enabled=false`), shadow-first enablement, and real-machine execution blocked while PICK/PLACE 接触高度 / 正式 `table_mm -> robot_base_mm` / 磁头 footprint remain `UNKNOWN`. Matches AGENTS.md priority list and docs/00_范围与冻结决策.md 关键决策门 4.
- **External strengths adopted, flaws excluded.** §2 取舍清单 absorbs adaptive black-surface segmentation, virtual corners from arc chords, per-piece-pair candidate retention, complete+partial edge matching, and joint overlap/coverage/closure scoring; rejects fixed 100×60 mm target, `cut_mode` runtime hint, convex-hull-first, "exactly 4 rounded corners" hard rule, silent texture fallback, area→confidence, and the monolith. Consistent with context.md Decisions/Discarded options; independent reimplementation with frozen tag/commit correctly handles the no-LICENSE risk.
- **Gate C dual-criteria conflict caught and resolved.** docs/19_决策门与停止条件.md currently says "至少 10 套盲测中 9 套" while docs/01_赛题需求追踪矩阵.md R4 says "20 套中 >= 18 套". Phase 0 item 5 explicitly unifies to one 18/20 + 5 实机连续成功 口径 across docs/01, docs/12, docs/19 and the plan. This was the largest pre-existing inconsistency and the plan handles it in the right phase.
- **Frame semantics coherent.** Nested `measured_contour` carrying its own `image_px|rectified_px` frame, with top-level `frame="table_mm"` scoped to mm fields only, fixes the real drift between docs/10 (which requires `contour_rectified_px`) and data_contracts/piece_observation.schema.json (which has no contour field at all). Negative schema examples for pixel-points-under-`table_mm` are specified. Frames used (`image_px`, `rectified_px`, `table_mm`) all exist in docs/10.
- **Phase dependencies executable.** P0 freeze+fixtures → P1 contracts → P2 vision → P3 solver explicitly consumes "Phase 2 实测绝对误差 + 相对误差" model and P2 logs for noise parameters (P3 验收 bullet 3 invalidates the ≥99% metric if the error model is undefined — correct sequencing) → P4 Gate C → P5 texture gated on Gate C → P6 enablement. No circular dependencies.
- **Acceptance criteria falsifiable.** Concrete thresholds throughout: p95 ≤ 2 mm / max ≤ 3 mm virtual-vertex error, 7 类 × 2/3/4 片 × 30 seeds (≥630), independent evaluator forbidden from reusing solver score as truth, 18/20 blind set, texture holdout construction constrained so geometry-only top-1 ∈ [30%,70%] making the "≥20pp absolute lift" criterion mathematically meaningful, byte-stable outputs, `RᵀR≈I, det(R)=+1`.
- **Phase 0 replay baseline is feasible.** `workspace/calibration/evidence/2026-07-31_gui_a4_workspace/red_pieces_frame.jpg` + `red_pieces_detection.json` exist; `red_piece_detector.py` exports `detect_red_pieces` as referenced.
- **UNKNOWN discipline.** parameter_registry.yaml additions are registered as `UNKNOWN` pending logs; no fabricated measurements; `solver_time_budget_ms=10000` is labelled an engineering safety cap. Matches AGENTS.md 数据真实性 rules.
- **docs/10 error-code sync planned** (Phase 1.4) adds VISION_MERGED_BLOBS, VISION_SURFACE_MISMATCH, SOLVER_AMBIGUOUS, SOLVER_TIMEOUT, TEXTURE_INSUFFICIENT, TEXTURE_AMBIGUOUS — consistent with existing code style (VISION_*, SOLVER_*).
- **Scope not overreaching.** §6 明确不改 lists control/robot_plugin/policy, GPIO17, five-axis order, LeRobot v0.6.0 — all verified present and untouched by the plan. New file targets (docs/references/, docs/24_*) do not yet exist, as expected for Phase 0 creation.

### Fixed

- None (read-only review; no edits applied).

### Blocker

- None.

### Note (minor, resolve during Phase 0/1 — none gate plan approval)

1. **Yaw range semantics.** Plan defines yaw as `[-180, 180)`, but both data_contracts/piece_observation.schema.json and placement_plan.schema.json currently allow `maximum: 180` inclusive. Phase 1 schema updates should tighten to `exclusiveMaximum: 180` (or explicitly document the normalization) so 180 and -180 cannot both represent the same angle in frozen data.
2. **Red adapter polygon semantics.** Phase 1 declares `polygon_table_mm` "是虚拟几何而非 min-area box", but red_piece_detector.py:130-136 currently exports the `cv2.minAreaRect` box as `polygon_table_mm`, and the plan keeps red detection behavior and export fields compatible. Phase 1 should either (a) state explicitly that the legacy red min-area box remains schema-valid as virtual geometry, or (b) word the schema description/negative examples so the frozen red path is not accidentally invalidated. One sentence in Phase 1 suffices.
3. **Red detector timestamp clock.** red_piece_detector.py uses `time.time_ns()` (wall clock) while docs/10 requires monotonic timestamps and the plan mandates monotonic for new interfaces. Since the red path's behavior is frozen, the Phase 0 replay baseline should not assert timestamp monotonicity for red output, and the red-adapter contract migration should note this known deviation (or switch to `time.monotonic_ns()` as an explicitly reviewed behavior change).
4. **docs/06 solver-output field list drift.** docs/06_拼图求解器.md lists PlacementPlan fields (`pick_point_table_mm`, `source_yaw_deg`, `target_center_table_mm`, `target_yaw_deg`, `solver_confidence`) that do not match the current schema (`source`/`target` pose objects, `plan_confidence`). Pre-existing drift; docs/06 is in §6 文件影响范围 — just ensure the Phase 1 doc sync rewrites this field list rather than only the error codes.

## Residual risks

- Gate E texture holdout (≥40 geometry-ambiguous scenes, ≥4 card faces) is not yet collected; correctly marked as blocking Gate E only, not white-geometry work.
- 碎片纸面朝上 / 可见间隙 / 现场数量下限 remain `UNKNOWN` against the official PDF; the plan correctly routes these to explicit rejection paths rather than assumptions.
- Pi 5 performance thresholds (p95 ≤ 2 s white / ≤ 5 s printed, RSS ≤ 250 MB) are unverified until Phase 3/4 benchmarks; plan forbids extrapolating dev-machine results to Pi.