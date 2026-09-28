# Task for reviewer

Read-only review of the new 5 mm discrete puzzle layout implementation. Scope only: workspace/src/puzzle/discrete_layout.py, workspace/src/puzzle/solver.py, workspace/src/integration/_adapter_123.py, workspace/tests/test_discrete_layout.py, workspace/tests/test_puzzle_solver_synthetic.py, workspace/tools/live_detect_123.py, workspace/tools/test_adapter_123.py. Requirement: original measured geometry must still solve; after solving, preserve each piece orientation and source/grip semantics; every final polygon pair must have at least 5 mm Euclidean clearance, layout centered in TargetRegion, fail closed if clearance/bounds fail, deterministic, no new dependencies. Report concrete correctness/safety issues with severity and file/line; do not edit and do not access Pi/hardware.

## Acceptance Contract
Acceptance level: attested
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Return concrete findings with file paths and severity when applicable

Required evidence: review-findings, residual-risks

Finish with a fenced JSON block tagged `acceptance-report` in this shape:
Use empty arrays when no items apply; array fields contain strings unless object entries are shown.
`criteriaSatisfied[].status` must be exactly one of: satisfied, not-satisfied, not-applicable.
`commandsRun[].result` must be exactly one of: passed, failed, not-run.
`manualNotes` and `notes` are optional strings; an empty string means no note and does not satisfy `manual-notes` evidence.
```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "specific proof"
    }
  ],
  "changedFiles": [
    "src/file.ts"
  ],
  "testsAddedOrUpdated": [
    "test/file.test.ts"
  ],
  "commandsRun": [
    {
      "command": "command",
      "result": "passed",
      "summary": "short result"
    }
  ],
  "validationOutput": [
    "validation output or concise summary"
  ],
  "residualRisks": [
    "none"
  ],
  "noStagedFiles": true,
  "diffSummary": "short description of the diff",
  "reviewFindings": [
    "blocker: file.ts:12 - issue found, or no blockers"
  ],
  "manualNotes": "anything else the parent should know"
}
```