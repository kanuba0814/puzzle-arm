# Task for context-builder

你是比赛现场 7 英寸触屏 UX 与 pygame/fb0 工程专家，只读分析，不修改项目/source 文件。读取当前 `workspace/apps/pick_place_page.py`、`workspace/rpi_gui_snapshot/{gui_fb.py,gui_backend.py,test_gui_touch.py}`、当前截图/README、计划文档。给出一套赛场不出洋相的 GUI 实现合同：1024x600 信息架构、首页/运行页/标定与诊断页（必要时精简）、状态机驱动而不是抓 subprocess 最后一行、碎片选择和目标位姿、快速扫块、一键流程 shadow/安全锁、明确警告/错误/恢复、急停、触摸目标大小、中文字体、无串口/GPIO常驻、截图/交互测试、systemd 启动。要求视觉上正式、信息不拥挤、关键状态一眼可见。列出精确文件和测试。

---
Update progress at: /home/gxxl/nudec-re101/.pi-subagents/artifacts/progress/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/progress.md

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/context/gui.md
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