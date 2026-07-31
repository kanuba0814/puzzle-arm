# HANDOFF — 正式赛场 7 英寸 GUI（pick_place_page 重构）

日期：2026-07-31。隔离副本：`/tmp/nudec-re101-terra-workers/gui`。

## 交付概要

按 `IMPLEMENTATION_CONTEXT.md` §3 合同完整重写 `workspace/apps/pick_place_page.py`，
并冻结了 GUI ↔ 未来 `workspace/tools/supervisor_run.py` 的命令/结果文件合同
（schema + docs/10），用测试 fake 验证。未触碰 control / robot_plugin / puzzle /
vision 算法；未启动子代理。

### 界面（1024×600，中文，深色设备风）

- **常驻顶栏（56px）**：页面名 + 4 个状态色块（页面状态 / 检测碎片 / 正式抓放 /
  电磁铁常关）+ 右上 120×48 红色急停（三页均可点，隔离区 ≥4px）。
- **常驻底栏（64px）**：24px 大字三级消息（INFO 灰 / WARN 橙 / ERROR 红）+
  冻结高度提示；RUNNING 时右侧显示命令、心跳阶段、耗时/超时与进度条；
  ESTOP 时右侧出现“回 PARK 并复位”红色按钮。
- **首页**：检测预览（选中碎片橙色高亮圈 + `◆ n` 徽标，其余碎片暗色小圈，
  映射自 pieces.json `diagnostics.center_pixel`）、可触摸选行的碎片列表
  （中心 mm + yaw）、选中碎片大字信息（中心/当前角→目标角/置信度）、
  4 个大入口（运行页 / 刷新检测 / 诊断 / 回 PARK）。
- **运行页**：上一块/下一块、目标角 ±15°（%180）、红色基线（检测+巡检）、
  快速最短巡检、通用拼图 Shadow、到选中碎片 20mm、回 PARK、上次结果面板；
  PICK/PLACE UNKNOWN 期间正式抓放按钮**物理隐藏**，锁定面板写明原因与解锁
  条件（接触高度标定 accepted 且单片成功率 ≥80%）。
- **诊断页**：8 项文件级自检（中文字体 / supervisor_run 工具 / 运动学标定
  accepted / 单应矩阵 / PARK 标定 / 检测数据新鲜度 / 电磁铁默认关 / 抓放锁）、
  重新自检、五轴断力矩、回 PARK、说明与日志尾部。不做手动 jog。

### 状态机与安全

- 显式 `PageState`：IDLE / RUNNING / FAULT / ESTOP；按钮可用性 = f(state)。
  RUNNING/ESTOP 中全部动作按钮置灰且触摸被忽略；FAULT 中动作恢复可用
  （用于“回 PARK → 刷新检测”恢复路径）。
- 结果驱动：`supervisor_run.py <command> --status-file … --result-file …`，
  只读结构化 JSON；PASS 且 returncode==0 才算成功；缺失/损坏/FAIL → FAULT 并
  原文上屏；stdout 只进日志。每命令超时 = 基线 + 按碎片数加码，超时杀进程。
- 急停：terminate→kill 子进程 + `tools/emergency_off_single_arm.py` 断力矩，
  **无 sudo、无 systemctl**；普通退出只在仍有子进程时做同样清理。ESTOP 锁存，
  只有 park 复位成功才回 IDLE；复位失败回到 ESTOP。
- 页面自身只持有 fb0 + evdev；串口/GPIO 仅由短命子进程持有。
- 中文字体缺失 → 屏幕上红色英文报错横幅（不再静默方块）；新增 `EP_GUI_FONT`
  显式字体覆盖（部署/预览用）；保留 `--screenshot PATH`，新增 `--page home|run|diag`。

## 变更文件

| 文件 | 变更 |
| --- | --- |
| `workspace/apps/pick_place_page.py` | 全量重写（约 1200 行） |
| `workspace/tests/test_pick_place_page.py` | 新增，22 项 headless 测试（内嵌 fake supervisor） |
| `data_contracts/supervisor_run_result.schema.json` | 新增结果文件合同 |
| `data_contracts/supervisor_run_status.schema.json` | 新增心跳文件合同 |
| `docs/10_接口与数据契约.md` | 新增“SupervisorRun 页面动作合同”一节 |
| `workspace/tests/README.md` | 补 GUI 测试前置条件 |
| `gui-preview.png`（副本根目录） | 首页预览截图（演示数据 + Noto CJK） |

## 验证证据

- `python3 workspace/tests/test_pick_place_page.py` → **22 tests OK**（SDL dummy）
- `PYTHONPATH=../src python3 -m unittest test_gui_touch`（rpi_gui_snapshot）→ **7 tests OK**
- `ruff check workspace/apps workspace/rpi_gui_snapshot workspace/tools workspace/tests` → **clean**
- `python3 -m compileall workspace/apps workspace/tests` → **OK**
- 三页离屏截图人工检查：布局不拥挤、色块/锁定面板/高亮均正确
- `unittest discover` 全套中 7 个 error 为隔离副本缺 accepted 标定/evidence/
  lerobot 插件的**既有**问题，与本次改动无关（已逐条核对 traceback）

## 合同要点（supervisor_run.py 实现方必读）

- 命令：`detect / red-baseline / sweep-shortest / pregrasp / park / puzzle-shadow /
  pick-place`；参数 `--status-file --result-file [--piece-index N] [--target-angle DEG]`。
- 心跳周期重写 status JSON（phase/progress/updated_at_ns）；结束必须写 result
  JSON（result/magnet_enabled/message/duration_s/events/final_joint_deg）。
- shadow/巡检类 `magnet_enabled` 必须 false；失败须抬升→回 PARK→FAIL+原文原因。
- `pick-place` 在解锁条件满足前工具侧也必须拒绝/保持 shadow。

## 残余风险 / 后续

1. `supervisor_run.py` 尚不存在（诊断页如实 WARN“未部署”）；实现后可先包一层
   现有 `detect_red_pieces.py` / `control_red_piece_centers.py`。
2. Pi 实机验收（fb0 非黑像素/颜色数 + 实屏触摸命中）未在宿主执行。
3. 底栏消息 28 字截断；长错误细节在诊断页日志尾部可见。
4. 合同 §3.8 的 `epuzzle-gui.service` 未新增（属部署任务，需决策）。
5. 建议镜像内固定 CJK 字体文件并把 `EP_GUI_FONT` 写进启动脚本，消除字体候选
   路径漂移。