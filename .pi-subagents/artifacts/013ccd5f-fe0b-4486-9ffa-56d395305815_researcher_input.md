# Task for researcher

只读研究并写报告，不改其他文件。为 /home/gxxl/nudec-re101 的 2026 电赛 E 题树莓派上位机镜像核对官方资料。目标：Raspberry Pi 5 8GB、64GB SD、Raspberry Pi OS Lite 64-bit Trixie、SO-101/SR-ARM101 去夹爪五轴、电磁铁 GPIO/MOSFET、顶置 UVC 或 Pi Camera、LeRobot v0.6.0 commit 30da8e687a6dfc617fcd94afc367ac7071c376ce、Python >=3.12、ACT 默认关闭。只用官方一手来源：Raspberry Pi 官方 docs/下载站/pi-gen，Debian 官方资料，Hugging Face LeRobot v0.6.0 文档/源码/PyPI。覆盖：OS/架构和 Python 3.13 兼容判断；headless 用户/SSH/NetworkManager；Pi 5 供电散热、相机、GPIO、串口；systemd 与安全硬化；aarch64 上 torch 2.7.1、torchvision 0.22.1、OpenCV 4.13、LeRobot 0.6.0/Feetech wheel 可用性；离线 wheelhouse；镜像下载哈希、首次扩容和写盘校验。明确 UNKNOWN 与实机验证。逐条附官方 URL、访问日期 2026-07-29。写入指定 Markdown，末尾列残余风险。

---
**Output:**
Write your findings to exactly this path: /home/gxxl/nudec-re101/.pi-subagents/artifacts/outputs/013ccd5f-fe0b-4486-9ffa-56d395305815/docs/22_树莓派5系统镜像官方依据.md
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