# Task for context-builder

你是机器人 Supervisor/安全状态机专家，只读分析，不修改项目/source 文件。读取 `.taskman/plans/adopt-puzzle-vision-v2-2/{PLAN.md,context.md}`、`workspace/tools/control_red_piece_centers.py`、`workspace/src/control/*`、`workspace/src/integration/README.md`、现有标定总表与测试。目标：给出可实现的 production Supervisor 深模块设计与代码合同，重点是：快速最短路径扫块；全路径预检；PARK↔HOME；视觉结果时效；检测/规划 shadow；PREGRASP；PICK/PLACE UNKNOWN 时严格锁定；未来磁吸抓取状态机接口；急停与故障恢复；JSON 事件日志。列出精确文件、类型、状态、错误码、测试和如何接现有 GUI。不得放宽磁铁/接触高度门禁，不得把 legacy controller 带入。

---
Update progress at: /home/gxxl/nudec-re101/.pi-subagents/artifacts/progress/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/progress.md

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/context/supervisor.md
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