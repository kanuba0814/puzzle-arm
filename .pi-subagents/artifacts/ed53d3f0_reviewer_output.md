## Review

- **Correct:** All three prior blockers are resolved in their reported forms:
  - Gaps below 5 mm are rejected by `PuzzleOptions`, the layout API, and CLI (`workspace/src/puzzle/solver.py:88-93`, `workspace/src/puzzle/discrete_layout.py:275-276`, `workspace/tools/live_detect_123.py:119-123`).
  - Single-piece plans now use discrete layout centering and bounds validation (`workspace/src/puzzle/solver.py:398-457`).
  - The adapter reads canonical `pick_point_table_mm`, supports the legacy fallback, and returns `INVALID_PICK_POINT` for malformed values (`workspace/src/integration/_adapter_123.py:41-51,238-259`).
- **Blocker (high):** Target reference transformation still assumes `piece.center_table_mm == mean(polygon vertices)`. At `workspace/src/integration/_adapter_123.py:342-354`, the transform translation is discarded and the target pick is computed from `placed_centroid + R @ (pick-center)`. The contract does not require `center_table_mm` to equal vertex mean. With valid finite canonical pieces whose centers differ from vertex mean, the solver reported verified 5 mm clearance but polygons reconstructed from emitted plans had **0 mm** actual clearance. The exact robust reference transform is `R @ reference + translation + layout_translation`, using pick or center as the reference.
- **Note (medium):** `workspace/tools/test_adapter_123.py:50-68` sets every center to `verts.mean(axis=0)`, so its non-centroid pick points do not expose the remaining assumption. It also still uses generated guillotine geometry rather than a captured measured fixture.

No files were edited and no Pi/hardware access occurred.