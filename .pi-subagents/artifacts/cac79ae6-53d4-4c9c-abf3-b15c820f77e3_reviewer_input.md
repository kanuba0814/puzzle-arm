# Task for reviewer

独立分析一个机器人抓取失败问题，不要假设或复述主会话已有方案，也不要修改文件、不要登录树莓派、不要执行真实硬件动作。只读检查仓库相关视觉/坐标变换/抓取执行代码和最近的成功运行日志。问题描述：四片拼图完整流程的运动轨迹与编码器读回均成功，但电磁铁没有吸附到碎片；现场观察磁头落点相对碎片卡纸上预期的带铁中心，向靠近机械臂方向偏约 10 mm。目标：给出一个简单、快速、可回退的诊断和修复方案，优先区分视觉参考点、工具中心/坐标变换、机械臂径向补偿、相机标定误差、磁铁安装偏心等根因。请明确：1) 你从代码和日志看到的关键证据；2) 3-5 个独立假设及各自最小验证；3) 推荐的最快一次性修复和为什么；4) 哪些改动不应做。不要使用或提及主会话可能提出的具体补偿方案，保持独立判断。

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