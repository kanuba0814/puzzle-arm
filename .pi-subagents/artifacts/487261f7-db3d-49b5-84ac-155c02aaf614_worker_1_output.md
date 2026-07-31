# Supervisor integration implementation handoff

## Implemented

- New `workspace/src/integration/` deep module:
  - `SupervisorConfig`, `SupervisorState`, `SupervisorErrorCode`, `SupervisorResult`, `ShadowPlanReport`, monotonic `VisionBatch`, and the single-owner `Supervisor` state machine.
  - Fail-closed future grasp FSM seam: UNKNOWN PICK/PLACE heights, disabled automatic mode, unofficial transform, unknown footprint clearance, stale vision, or mismatched plan all block before any `set_enabled(True)` call.
  - Vision-batch/placement-plan source image and monotonic-age checks; shadow accepts no hardware objects and rejects sweep.
  - JSONL runtime events use monotonic timestamps and schema-compatible keys.
  - ESTOP latches, invokes emergency-off if a magnet exists, and only permits explicit operator-confirmed reset after PARK verification.
- `JointSpaceParkMover` implements PARK↔HOME through an upright pose with explicit fixed five-axis ordering, ≤1° interpolation, and actual-readback tolerances.
- `plan_sweep_order` is deterministic and globally optimal for n≤4 via full permutation enumeration. Sweep constructs one continuous segment: first high point (60), all low (20) centers, last high (60); it performs full segment IK preflight before PARK transition or servo action, records each center actual readback, and explicitly commands magnet off when a magnet is available.
- Added strict Supervisor config/result schemas, registry keys, contract documentation, and an integration README.
- Added `workspace/tools/supervisor_run.py`. It intentionally exposes only safe `shadow-plan` on this image; all hardware modes fail closed. Legacy tools remain in place and unmodified.

## Changed files

- `workspace/src/integration/__init__.py`
- `workspace/src/integration/event_log.py`
- `workspace/src/integration/park_sequence.py`
- `workspace/src/integration/supervisor.py`
- `workspace/src/integration/sweep_planner.py`
- `workspace/src/integration/README.md`
- `workspace/tools/supervisor_run.py`
- `workspace/tests/test_sweep_planner.py`
- `workspace/tests/test_park_sequence.py`
- `workspace/tests/test_supervisor_gates.py`
- `workspace/tests/test_supervisor_event_log.py`
- `workspace/tests/test_supervisor_states.py`
- `workspace/tests/test_control_data_contracts.py`
- `data_contracts/supervisor_config.schema.json`
- `data_contracts/supervisor_result.schema.json`
- `configs/parameter_registry.yaml`
- `docs/10_接口与数据契约.md`

## Validation

Passed:

```text
PYTHONPATH=workspace/src python -m unittest \
  workspace/tests/test_sweep_planner.py \
  workspace/tests/test_park_sequence.py \
  workspace/tests/test_supervisor_gates.py \
  workspace/tests/test_supervisor_event_log.py \
  workspace/tests/test_supervisor_states.py -v
# 14 tests, OK

python -m compileall -q workspace/src workspace/tools
ruff check workspace/src workspace/tools workspace/tests
# both passed

PYTHONPATH=workspace/src python workspace/tools/supervisor_run.py shadow-plan ...
# emits successful shadow JSON; output JSON parses
```

Full `unittest discover` was run but cannot pass in this isolated checkout because baseline fixtures are absent: `workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json` and `contact_sheet_to_robot_base_20260731.json`; `test_so101_5d_contract` additionally requires the unavailable installed robot plugin. These failures are outside the changed integration scope and occur in existing calibration/plugin tests.

## Residual hardware gates

- Formal `table_mm -> robot_base_mm` transform plus manifest SHA256 is not frozen.
- PICK/PLACE contact heights, magnet footprint/clearance, settle time, retry counts, and PICK/PLACE verification sensor are UNKNOWN.
- Real robot/magnet/vision adapters and monitored PARK/HOME and continuous-sweep validation on Pi remain required.
- No repository `.git` metadata exists at the worker-tree root, so staged-file status cannot be independently queried here; no `git add` was run.