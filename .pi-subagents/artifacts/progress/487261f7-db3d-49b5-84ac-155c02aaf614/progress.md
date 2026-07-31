# GUI 专项进度（2026-07-31）

## 状态：完成，待评审

## 已完成

- 重写 `workspace/apps/pick_place_page.py`：1024×600 中文三页赛场 GUI
  - 显式 `PageState`（IDLE/RUNNING/FAULT/ESTOP），按钮可用性由状态唯一决定
  - 常驻顶栏：页面名 + 4 个状态色块（页面状态/检测碎片/正式抓放/电磁铁）+ 120×48 全局急停
  - 常驻底栏：大字三级消息（INFO/WARN/ERROR）+ 安全冻结提示 + 运行中阶段/耗时/超时/进度条
  - 首页：检测预览（选中碎片橙色高亮圈 + 序号徽标）、可触摸碎片列表、4 个大入口
  - 运行页：选块/目标角 ±15°、红色基线、快速最短巡检、通用拼图 Shadow、20mm 预抓取、PARK；PICK/PLACE UNKNOWN 时正式抓放按钮隐藏，锁定面板给出原因与解锁条件
  - 诊断页：8 项文件级自检（字体/supervisor_run/标定/单应/PARK/检测数据/磁铁默认关/抓放锁）、断力矩、PARK、日志尾部
  - 动作全部调用未来 `workspace/tools/supervisor_run.py`；只读 status/result JSON，不解析 stdout；逐命令超时（基线+按片加码），超时杀进程进 FAULT
  - 急停无 sudo/systemctl：杀子进程 + `emergency_off_single_arm.py` 断力矩；ESTOP 锁存，仅“回 PARK 并复位”可解除
  - 中文字体缺失上屏红色英文报错横幅；新增 `EP_GUI_FONT` 显式字体覆盖；保留 `--screenshot`，新增 `--page`
- 新合同：`data_contracts/supervisor_run_result.schema.json` + `supervisor_run_status.schema.json`；`docs/10_接口与数据契约.md` 新增“SupervisorRun 页面动作合同”一节
- 新测试 `workspace/tests/test_pick_place_page.py`：22 项（截图/布局/结果JSON/急停/触摸/选块/心跳/超时/fake supervisor 全流程/字体缺失/自检）
- `workspace/tests/README.md` 补 GUI 测试前置（pygame-ce、SDL dummy、PYTHONPATH）
- 预览截图：`/tmp/nudec-re101-terra-workers/gui/gui-preview.png`（首页，含演示碎片数据）

## 验证

- `python3 workspace/tests/test_pick_place_page.py`：22 tests OK（SDL dummy headless）
- `test_gui_touch`（rpi_gui_snapshot）：7 tests OK
- `ruff check workspace/apps workspace/rpi_gui_snapshot workspace/tools workspace/tests`：clean
- `compileall workspace/apps workspace/tests`：OK
- 全套 discover 中 7 个 error 为隔离副本缺标定证据文件/lerobot 插件的既有问题，与本次改动无关（已逐条核对）

## 未做/风险

- `workspace/tools/supervisor_run.py` 尚不存在：合同已冻结（schema + docs/10），GUI 用测试 fake 验证；诊断页如实显示“未部署 WARN”
- 未改 control/robot_plugin/puzzle；未新增 systemd unit（合同 §3.8 建议留待部署任务）
- Pi 侧 fb0 非黑/实屏触摸验收需在实机执行
