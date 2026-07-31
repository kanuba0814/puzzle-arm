# Task for researcher

为 /home/gxxl/nudec-re101 的 2026 电赛 E 题树莓派上位机镜像做官方资料研究。目标硬件按仓库冻结：Raspberry Pi 5 8GB、64GB SD、64位 Raspberry Pi OS，SO-101/SR-ARM101 去夹爪保留五轴，电磁铁 GPIO/MOSFET，顶置 UVC 或 Pi Camera，LeRobot 固定 v0.6.0 commit 30da8e687a6dfc617fcd94afc367ac7071c376ce，Python 要满足 >=3.12，ACT 默认关闭且仅可选。只用官方/一手来源：Raspberry Pi 官方 docs、Debian/Raspberry Pi OS 包资料、Hugging Face LeRobot v0.6.0 官方文档/源码/PyPI。研究并给出可引用的结论：推荐 OS/架构、Bookworm/Trixie/Python 兼容判断，headless 初启与 SSH 规则，Pi 5 config.txt/cmdline/GPIO/串口/相机/性能/散热/供电相关设置，systemd 服务与安全硬化，Python venv/apt/pip/torch/opencv/LeRobot 在 aarch64 的安装可行路径，离线缓存与镜像缩容/分区首次扩容/写盘校验。明确 UNKNOWN 和必须在实机验证项，不编造版本或性能。将研究报告写入指定文件，逐条附官方 URL 和检索日期；不要改其他文件。

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/f6f310b8-9015-43b1-9b43-3ea5adda298f/docs/22_树莓派5系统镜像官方依据.md
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