# Task for oracle

只读做独立技术审计，不修改项目/source 文件。用户要求依据 `.taskman/plans/adopt-puzzle-vision-v2-2/{PLAN.md,context.md}` 和当前实机状态，一次性完成剩余软件并做优秀赛场 GUI。审计当前代码、计划与安全约束，找出最容易导致“看似完整、赛场出洋相”的问题。给出：本轮必须实现的 P0/P1；明确延后的实测 Gate；建议的深模块 seams；快速扫块的最短安全算法；GUI 用户流；可自动验证的 acceptance contract；树莓派部署/回退要求。不要泛泛而谈，要引用文件/符号。

---
Update progress at: /home/gxxl/nudec-re101/.pi-subagents/artifacts/progress/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/progress.md

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/context/oracle.md
This path is authoritative for this run.
Ignore any other output filename or output path mentioned elsewhere, including output destinations in the base agent prompt, system prompt, or task instructions.

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