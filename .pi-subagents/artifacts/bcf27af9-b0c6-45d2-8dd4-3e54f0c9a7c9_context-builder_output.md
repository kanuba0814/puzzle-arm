# GUI 上下文交付：树莓派 7 英寸 GUI（入口/后端接线/触摸/布局重叠根因）

只读分析，未修改任何文件，未访问 Pi，未执行硬件动作。日期：2026-08-01 之后快照。
正式页面为 `workspace/apps/pick_place_page.py`（1024×600 四页，已按 `docs/25` 重写）；
底层 fb0/evdev 原语来自 `workspace/rpi_gui_snapshot/gui_fb.py`。

---

## 1. 入口（Entry Points）

| 层 | 文件 | 要点 |
| --- | --- | --- |
| systemd | `deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-gui.service` | `ExecStart=/opt/epuzzle/workspace/tools/start_pick_place_page.sh`；`TTYPath=/dev/tty2`；`User=epuzzle`；`Wants=epuzzle-magnet-safe.service`；无 sudo；`WantedBy=multi-user.target`（不在 epuzzle.target，与 supervisor 解耦） |
| 启动脚本 | `workspace/tools/start_pick_place_page.sh` | `SDL_VIDEODRIVER=dummy` + `EP_GUI_FRAMEBUFFER=/dev/fb0` + `SDL_TOUCH_MOUSE_EVENTS=0`；非 tty/无 `EP_GUI_ON_VT=1` 时 return 2；**GUI 解释器默认 `/usr/bin/python3`（`EPUZZLE_PYTHON`），子进程解释器默认 `/opt/epuzzle/venv/bin/python`（`EP_GUI_PYTHON`）**；PYTHONPATH 插件目录在 venv site-packages 前 |
| 正式页面 | `workspace/apps/pick_place_page.py`（1926 行） | `main()` → `PickPlacePage.run()`；四页 competition/run/manual/debug；`PageState`(IDLE/RUNNING/FAULT/ESTOP)；**结果文件驱动**（`_finish_action` 只读 `--result-file` JSON，`PASS`+rc=0 才成功，不抓 stdout） |
| 底层原语 | `workspace/rpi_gui_snapshot/gui_fb.py` | `LinuxFramebuffer`（fb0 RGB565）、`EvdevTouchReader`（QDTECH MPI700 evdev→鼠标事件）、`touch_event_to_mouse_event`、`Button/Radio`、颜色常量 |
| Mock 后端 | `workspace/rpi_gui_snapshot/gui_backend.py` | `RobotBackend` 抽象 + `MockBackend`（机械臂模拟；仅 GPIO17 电磁铁可真实，经 `MagnetController`） |
| legacy（非部署真源） | `pi_puzzle/gui_fb.py`（1506 行） | 800×480 时代 kmsdrm 版；**不含** fb0/evdev 原语（`grep -c EvdevTouchReader` = 0）；`pi_puzzle/tools/start_gui_fb.sh` + `pi_puzzle/gui_app.py`(Tk) 已弃用。部署真源是 `workspace/rpi_gui_snapshot/`（2026-07-31 从 Pi 拉回的快照） |

## 2. 后端接线（Backend Wiring）

- GUI 唯一依赖：`workspace/src/integration/competition_backend.py` 的 `SupervisorRunBackend`：
  `capabilities()`（能力表）→ `build_request()`（argv+timeout）→ 子进程 `workspace/tools/supervisor_run.py <mode> --status-file --result-file`，`cwd=PROJECT_ROOT`（Pi 上 = `/opt/epuzzle`）。
- `supervisor_run.py` 已实现：`scan`(detect)、`sweep`(red-baseline/sweep-shortest)、`pregrasp`、`park`、`shadow-plan`(puzzle-shadow)、`pick-place`、`manual-magnet-pulse/off`、`commission-center`。
- **刻意 fail-closed（不要“强行接通”）**：
  - `competition-*`：`supervisor_run.py main()`（约 1266 行）直接 `SupervisorFailure(SUPERVISOR_LOCKED, "比赛全流程后端尚未接入")`；能力表 `competition_backend.py:321-332` 返回 `UNAVAILABLE`。
  - `manual-read-state/jog-*`：`run_manual_command()` 占位（`supervisor_run.py:936`）`SUPERVISOR_LOCKED`；能力表 `competition_backend.py:335-345` 返回 `UNAVAILABLE`。两者一致。
- PICK/PLACE 门：`pick_place_lock_reason()`（`competition_backend.py:140`）只校验 `quality.status==accepted` 且 `single_piece_success_rate>=0.8`；当前 `workspace/calibration/pick_place_contact_height.json` 已 accepted（`contact_heights_mm.pick=6.0/place=12.0`、10/10 现场报告）→ **门当前是开的**。
- 急停链路：`emergency_stop()`（`pick_place_page.py:955`）终止子进程 → `_request_magnet_off()`（`manual-magnet-off` 子进程）→ 调 `workspace/tools/emergency_off_single_arm.py` 断力矩（**无 sudo**）→ ESTOP 锁存；`reset_from_estop` 走 `park` 能力（allow_in_estop=True），PASS 才解除。
- **功能缺口（“GUI 功能接通”的真正可做项）**：run 页“安全中止”按钮（`safe_abort`，line 838）在 `RUNNING` 时被 `_refresh_button_states()`（line 1266-1281 `if busy: button.enabled=False`）禁用 → **运行中无法中止，只剩急停**。`safe_abort` 本体逻辑正确（terminate→kill→`_request_magnet_off`），只是按钮状态机把它锁死了。

## 3. 触摸/evdev 映射根因（`workspace/rpi_gui_snapshot/gui_fb.py`）

1. **候选主根因（触摸死/无反应）**：`EvdevTouchReader`（line 248）只处理 `_EV_ABS` X/Y/MT_X/MT_Y + `_EV_KEY BTN_TOUCH(330)`，**不处理 `ABS_MT_TRACKING_ID`(code 57)**。若 QDTECH MPI700 走 hid-multitouch protocol B 且不发 `BTN_TOUCH`（单指/多指驱动差异常见），`self.pressed` 永远 False → 永远不生成 `MOUSEBUTTONDOWN` → 点按无效。**硬件实际协议本机无法实证（UNKNOWN），需 Pi 端 `evtest`/`udevadm info` 确认**；代码层面该分支缺失是确定事实。
2. `_find_device()`（line 266）：名称必须同时含 `"QDTECH"`+`"MPI700"`，否则 `RuntimeError` → fb0 模式启动即崩；无 fallback、错误不上屏。
3. `scale_evdev_axis()`（line 179）：`maximum<=minimum` 抛 `ValueError`（设备未就绪时 ioctl 读回 0..0 即触发）；`poll()`（line 309）只捕获 `BlockingIOError`，设备拔出的 `OSError(EIO)` 直接冒泡 → 运行中 GUI 崩溃。
4. 无方向/交换处理：面板旋转、轴颠倒（X↔Y、正负向）无配置位；轴范围→屏幕是简单线性映射。
5. `_INPUT_EVENT = struct.Struct("llHHI")`（line 167）：64 位 timeval 假设；32 位系统错位（Pi 5 arm64 OK，需注明）。
6. `pointer_pos` 抬指不清除（`pick_place_page.py:1317-1320`；快照 run loop 同样）→ fb0 软光标残留在屏上。
7. 启动竞态：`start_pick_place_page.sh` 不做 udev settle 等待（legacy `start_gui_fb.sh` 有 trigger+settle，新脚本没有）；触摸节点未就绪时 2/3 直接崩溃。

## 4. 布局重叠根因（已用纯算术复现验证）

- **manual cartesian 页（P1，会误触）**：角度步长行 `(160+i*70, 456, 66, 44)`（`pick_place_page.py:491`）底部 500 > 导航 top 494 → deg1..deg4 共 5 个按钮与“比赛/运行”导航在 y=494..500 重叠；点击顺序步长按钮在前，导航该条带被拦截。
- **导航 vs 底栏（全部页，cosmetic）**：`_page_nav_buttons(y=494)`（line 602）底部 538 > `BOTTOM_BAR_TOP=536`（line 85）→ 底栏盖住导航底部 2px。
- **debug 页**：工具行 1（line 574，y=128..176）与能力面板 `(16,132,496,220)` 重叠 → 面板标题/首行能力文本被 3 个按钮盖住；日志面板 `(16,490,992,40)`（line 1824）与导航（y=494..538）重叠 → 日志行被导航盖住。
- **比赛页**：“◆ 已选择”（line 1545）与未就绪 reason（line 1548）同在 `(24, y+66)` → 选中且未就绪时文本叠字。
- **手动页**：锁状态文本 `(472,78)`（line 1676-1683）落在“读取状态”(540..660) 与“回 PARK”(672..792) 按钮之下，按钮绘制在后 → 文本被盖。
- **测试盲区**：`LayoutTest`（`workspace/tests/test_pick_place_page.py:274`）只检查 `manual_tab="joint"` 的可见按钮（cartesian 按钮 visible=False 被跳过），且只做按钮↔按钮 `colliderect`，不检查按钮 vs 面板/底栏 → cartesian 重叠与底栏重叠都测不到。

## 5. 测试现状（本机实测）

- `python3 workspace/tests/test_pick_place_page.py`：**40 项，1 失败** — `test_pick_place_locked_without_calibration`（line 635）：真实 workspace 标定文件已 accepted，`pick_place_lock_reason()` 返回 None，测试断言失效 → **测试非 hermetic**。
- `PYTHONPATH=workspace/src python3 workspace/rpi_gui_snapshot/test_gui_touch.py`：7 项全过（注意：必须带 PYTHONPATH，因为快照 `gui_backend.py` import `control.magnet_controller`；裸跑 ModuleNotFoundError）。
- `python3 workspace/tests/test_deployment_contracts.py`：16 项全过（含 GUI service、启动脚本、arm64 镜像 pygame-ce==2.5.5 断言）。
- 杂项：FakeSupervisorRunTest 泄漏一个子进程（ResourceWarning，测试基建小问题）。

## 6. 最小可交付实现清单

### A. 防重叠（`workspace/apps/pick_place_page.py`）
1. manual cartesian 角度步长行上移：`y 456→440`（或 h 40），保证 bottom ≤ 490；mm 行 `408→392`。
2. 导航：`_page_nav_buttons` 改为 `y=490, h=40`（bottom 530 < 536），或把 `BOTTOM_BAR_TOP` 提到 542。
3. debug：能力面板 `y 132→180`（或工具行 1 上移 8px）；日志面板 `y 490→482, h 36`。
4. 比赛页：reason 文本改 `(24, y+82)`。
5. 手动页：锁文本从 `(472,78)` 移出按钮区（如磁铁区 `(540,246)` 下方）。
6. 扩展 `LayoutTest`：遍历 `manual_tab` 两种取值；增加“按钮 vs 面板/底栏”断言。

### B. 触摸修复（`workspace/rpi_gui_snapshot/gui_fb.py` + `test_gui_touch.py`）
1. `EvdevTouchReader` 增加 `ABS_MT_TRACKING_ID`(57) 接触跟踪：`value>=0 → pressed=True`，`-1 → pressed=False`，与 `BTN_TOUCH` 取或。
2. `_find_device()`：名称匹配放宽（忽略大小写 + 候选子串 QDTECH/MPI700/goodix/ft5x0x/gslx680），失败返回 None 并在屏上显式报错而非崩溃。
3. `scale_evdev_axis()`：`max<=min` 时告警返回 0 不抛异常；`poll()` 捕获 `OSError` → 标记触摸离线、上屏提示，不崩 GUI。
4. `pointer_pos` 在 `MOUSEBUTTONUP` 时清除（`pick_place_page.py` 与快照 run loop 两处）。
5. 新增 `EP_GUI_TOUCH_SWAP/INVERT_X/INVERT_Y` 环境配置位（默认恒等；面板方向 UNKNOWN，实机确认后固化）。
6. 新测试：protocol B 无 BTN_TOUCH 序列能生成 down/up；名称 fallback；轴范围 0 不崩；抬指清光标。

### C. 功能接通（真正可交付项）
1. **run 页“安全中止”在 RUNNING 保持可点**：`_refresh_button_states` 特判该按钮（或按 `_tab`/text 放行），`safe_abort` 本体不改。
2. `test_pick_place_locked_without_calibration` 改 hermetic：mock `pick_place_lock_reason` 或指向临时缺失路径。
3. 文案修正：自检“电磁铁控制链：软件上限 10s”已过期（AGENTS 已废弃 10s 自动断电语义，改“调用方显式关闭”）；`run` 页阶段高亮是中文 vs 英文 phase 匹配不上（cosmetic）。
4. **不要动**：competition-*/manual-* 的 UNAVAILABLE/LOCKED 是刻意的 fail-closed 安全态，不是 bug。

### D. 验收条件
- `python3 workspace/tests/test_pick_place_page.py` 全绿（40+新布局回归）；
- `PYTHONPATH=workspace/src python3 workspace/rpi_gui_snapshot/test_gui_touch.py` 全绿（7+新 evdev 用例）；
- `python3 workspace/tests/test_deployment_contracts.py` 全绿；
- 四页离屏截图：按钮不重叠、不越界、急停热区每页可点（ScreenshotTest/LayoutTest 扩充后自动覆盖）；
- Pi 实机（现场授权后）：`evtest` 确认 QDTECH 事件协议（BTN_TOUCH 或 TRACKING_ID），触摸-光标同步目测，每页点按无误触。

## 7. 残留风险
- QDTECH MPI700 实际 evdev 协议未实证（本机无 Pi 访问）→ 触摸根因 3.1 是“代码缺失 + 候选根因”，落地前必须在 Pi 用 `evtest`/`/proc/bus/input/devices` 确认。
- 布局算术基于 1024×600 常量；若 fb0 实际分辨率不同（如 800×480 fallback）需重算。
- 镜像内 pygame 归属歧义：deployment 测试断言 `pygame-ce==2.5.5` 在 venv/镜像层，但 `start_pick_place_page.sh` 默认用 `/usr/bin/python3` 跑 GUI（依赖 dist-packages 或 PYTHONPATH 里的 venv site-packages）；需现场确认 `/etc/epuzzle/epuzzle.env` 是否应补 `EPUZZLE_PYTHON=/opt/epuzzle/venv/bin/python`。
- `_INPUT_EVENT` 64 位 timeval 打包假设（arm64 OK，32 位需改）。