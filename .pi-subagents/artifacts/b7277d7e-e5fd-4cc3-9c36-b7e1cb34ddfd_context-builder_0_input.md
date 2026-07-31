# Task for context-builder

你是通用视觉与拼图算法架构专家，只读分析，不修改项目/source 文件。目标：基于 `.taskman/plans/adopt-puzzle-vision-v2-2/PLAN.md`、同目录 `context.md`、当前 `workspace/src/vision/red_piece_detector.py`、`workspace/src/puzzle/README.md`、data_contracts 和 tests，给出本轮可完整落地的代码级实现合同。必须覆盖：保留红色基线；通用白色/印刷碎片观察的深模块 interface；物理轮廓/虚拟几何/最大净空抓取点；1-4 片确定性矩形求解；等价/歧义/超时；固定种子合成夹具；需要新增/修改的精确文件和测试。禁止复制外部无许可证代码或素材。区分现在能软件完成、必须实拍/盲测才能启用的 Gate。输出应足够供一个 writer 直接实现，并指出最小但优秀的算法取舍。

---
Update progress at: /home/gxxl/nudec-re101/.pi-subagents/artifacts/progress/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/progress.md

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/b7277d7e-e5fd-4cc3-9c36-b7e1cb34ddfd/context/algorithm.md
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