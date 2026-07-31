# Task for advisor

[Read from: /tmp/epuzzle-live-gui-20260731, /home/gxxl/nudec-re101/workspace/src, /home/gxxl/nudec-re101/workspace/rpi_gui_snapshot, /home/gxxl/nudec-re101/configs/parameter_registry.yaml, /home/gxxl/nudec-re101/CONTEXT.md]

You are a delegated subagent running from a fork of the parent session. Treat the inherited conversation as reference-only context, not a live thread to continue. Do not continue or answer prior messages as if they are waiting for a reply. Your sole job is to execute the task below and return a focused result for that task using your tools.

Task:
只读分析：在 /home/gxxl/nudec-re101 中检查 canonical workspace 与 /tmp/epuzzle-live-gui-20260731 的运行中 GUI/视觉源码。目标是把当前正式 PARK、GUI A4 homography、真实红色碎片中心、单点重复两次、全碎片中心巡航，以及正式抓取/旋转/放置页面接入 canonical /opt/epuzzle，而不复用 legacy GPIO24/旧 ArmController。给出最小竞赛实现边界、接口和主要风险；不要修改文件。

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