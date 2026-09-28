## Review

- **Correct:** The remaining high blocker is resolved. `workspace/src/integration/_adapter_123.py:341-355` now applies the exact solver transform to the selected source reference: `R @ source_reference + translation + layout_translation`.
- **Correct:** The integration fixture deliberately separates `center_table_mm` from vertex mean (`workspace/tools/test_adapter_123.py:50-69`) and all six seeds reconstruct polygons with 5 mm clearance and correct pick semantics.
- **Correct:** The stronger adversarial reproduction with different per-piece center offsets now reports identical internal and reconstructed minimum clearance.
- **Note:** No remaining high- or medium-severity correctness/safety issue was found in the scoped files.

The morphology implementation is outside this review scope; the supplied exact-frame replay evidence was not independently rerun. No files were edited and no Pi/hardware was accessed.