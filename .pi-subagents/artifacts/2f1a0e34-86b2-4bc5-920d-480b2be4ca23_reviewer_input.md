# Task for reviewer

Read-only safety/code review of the newly generated single-arm accepted kinematic calibration. Inspect: workspace/tools/fit_single_arm_kinematics.py, workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json, workspace/calibration/contact_sheet_to_robot_base_20260731.json, workspace/calibration/evidence/2026-07-31_single_arm_contacts_v2/{single_arm_contacts_20260731.json,fit_report.json,README.md}, workspace/src/control/{kinematics.py,calibration.py}, and relevant schemas/tests. Check especially: production FK formula matches fitter; holdouts are excluded from fitting/model selection; sign/zero-offset semantics; transform direction; accepted thresholds; hidden leakage/overfit; unsafe claims. Report concrete blockers or say none, with file/line evidence. Do not modify files.

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