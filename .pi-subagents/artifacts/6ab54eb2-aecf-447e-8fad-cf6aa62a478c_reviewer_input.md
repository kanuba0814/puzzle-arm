# Task for reviewer

只做独立工程推理，不修改文件、不调用工具。请分析以下问题并给出简短结论：一个五轴机械臂用相机识别不规则纸片，运动轨迹和编码器读回都显示成功，但电磁铁没有吸住纸片。现场看到磁头落点相对纸片上预期的带铁中心，向靠近机械臂方向偏约 10 mm。请从视觉参考点、工具中心偏置、平面坐标变换、径向补偿、相机 homography、磁铁安装偏心等角度列出 3 个最可能根因，给出一个最快且可回退的单变量验证，以及推荐的最小修复。不要假设已有方案，也不要建议关闭编码器反馈或无限制运动。

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