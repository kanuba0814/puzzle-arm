# Task for worker

你是本任务唯一代码写入者。请完整实现 `docs/25_比赛GUI与维护控制设计.md`，把现有 `workspace/apps/pick_place_page.py` 升级为正式比赛 GUI，并补齐必要的后端深模块/Adapter、数据合同、测试与接口文档。

先读：根目录 `AGENTS.md`、`docs/25_比赛GUI与维护控制设计.md`、`workspace/apps/pick_place_page.py`、`workspace/tests/test_pick_place_page.py`、`workspace/tools/supervisor_run.py`、`docs/10_接口与数据契约.md`、相关 schema、`workspace/src/integration/supervisor.py`、`workspace/src/control/magnet_controller.py`。题面事实只准来自 `official_docs/competition/E题_拼图装置.pdf`。

实现目标：
- 正式 1024×600 四页：比赛、运行监视、手动、Debug；顶部全局状态/急停和底部消息常驻。
- 比赛页包含自备纯色 50 分、现场白色 20 分、扑克牌 30 分，均显示官方 120 秒；模式选择 + 唯一一键启动 + readiness 原因。未完成自动全流程绝不能假装可用，必须通过稳定能力接口显示 LOCKED/UNAVAILABLE 且按钮无副作用。
- 运行监视显示单调时钟计时、阶段、进度、预览、结构化结果/错误码，并保留安全中止/急停。
- 手动页有 joint/cartesian 页签、固定五轴顺序和单位、步长、读回/PARK/断力矩，以及 1/3/5 秒电磁铁脉冲和立即关闭。公共控件使用 `_tab=None` 语义，切页签不得消失。未完成的手动运动要保留精确命令接口但 fail-closed 禁用；不要为了点亮按钮绕过 IK/软限位/运动锁/稳定读回。
- 电磁铁如能在现有架构下安全完成，则用 Supervisor 唯一全局状态所有者 + MagnetController 实现有界脉冲（绝不超过 10 秒），异常、SIGTERM、急停、退出都请求 OFF；若架构证据不足则保持 LOCKED，但接口、UI、测试要完整。立即 OFF 应尽最大可能可用。不得提供无限保持入口。
- Debug 包含能力矩阵、自检、既有检测/红色基线/巡检/Shadow/PREGRASP/PARK 工具、本次会话显示/日志/证据开关，以及 generic vision/texture/ACT 的冻结 gate 状态；不得修改环境、参数注册表、标定文件或运动锁。
- 建立一个小而深的 GUI 后端接口（capabilities + build_request 或同等深度），集中 argv/超时/参数验证；保留既有 GUI 命令兼容。
- 急停继续锁存；同时终止动作、请求磁铁 OFF、请求五轴断力矩。任何未确认都保持 ESTOP。
- 同步更新跨模块合同：`data_contracts/`、`docs/10_接口与数据契约.md` 和测试；不要编辑 deployment build/root 快照或 cache。

硬约束：
- 不改官方文档；不把估计写成实测；UNKNOWN 保持 UNKNOWN。
- 不加入 `gripper.pos`，五轴字段显式固定。
- 不删除文件，不做 git 操作，不安装依赖，不启动真实硬件动作。
- Python 4 空格；中文 UI/注释优先。
- 当前目录不是 git 仓库，直接检查文件差异/时间即可。

验证合同：
1. 扩充 headless GUI 测试：四页截图、布局、三比赛模式/120秒、能力禁用与 fake-ready 请求、单调计时、手动解锁、固定五轴、`_tab=None`、磁铁时长、Debug gate 无副作用、急停磁铁 OFF。
2. 保持原有 GUI/result/heartbeat/offline/lock/ESTOP/font 测试通过。
3. 更新 supervisor/Schema 测试（若扩命令），运行相关 unittest；先跑 `lsp_diagnostics` 或 pyright/compileall，再跑聚焦测试。不要运行真实硬件。
4. 最后报告：修改文件、完成/保留接口功能、运行命令及退出码、未验证项和残余风险。

## Acceptance Contract
Acceptance level: checked
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Implement the requested change without widening scope
- criterion-2: Return evidence sufficient for an independent acceptance review

Required evidence: changed-files, tests-added, commands-run, residual-risks, no-staged-files

Review gate: required by reviewer.

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
    },
    {
      "id": "criterion-2",
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