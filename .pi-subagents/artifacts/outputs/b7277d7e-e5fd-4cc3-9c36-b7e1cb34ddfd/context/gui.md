# GUI 上下文交付：7 英寸赛场触屏 UX 与 pygame/fb0 工程

只读分析产出，未修改任何项目/source 文件。日期上下文：2026-07-31 已冻结单臂基线（PARK/PREGRASP=20mm/PICK=UNKNOWN）。

---

## 1. 现状盘点（证据：文件 + 行号）

### 1.1 正式页面 `workspace/apps/pick_place_page.py`

- 单页 1024×600 布局：左侧检测预览（16,72,594,430），右侧 2 列×5 行按钮（184×52，间距 8），右上急停按钮（宽 116×46），底部三行小字（碎片信息/状态/安全提示，18px）。截图证据：`workspace/calibration/evidence/2026-07-31_gui_a4_workspace/formal_pick_place_page.png`（部署前在 Pi 正式依赖环境生成）。
- fb0 路径：设 `EP_GUI_FRAMEBUFFER=/dev/fb0` 时用 `SDL_VIDEODRIVER=dummy` 离屏渲染，经 `LinuxFramebuffer` 转 RGB565 写 fb0；触摸经 `EvdevTouchReader`（QDTECH MPI700 evdev）。与 `gui_fb.py` 共享全部原语（第 20–37 行 import）。
- 动作执行模型（第 126–153 行 `_run`）：单 worker 线程 + `subprocess.Popen` + `queue.Queue` 回传；工具脚本为 `/opt/epuzzle/venv/bin/python workspace/tools/{detect_red_pieces.py,control_red_piece_centers.py}`，每次动作临时连串口、结束后回 PARK 并断力矩（**无常驻串口/GPIO，已满足**）。
- 模式：repeat（选中碎片重复 2 次）、sweep（巡检全部中心）、pregrasp（20mm 预抓取）、park；`pick_rotate_place` 被安全锁定（第 197–198 行，PICK 接触高度 UNKNOWN，符合冻结决策）。
- 急停（第 210–228 行）：terminate 当前子进程 → `sudo systemctl restart epuzzle-magnet-safe.service` → `emergency_off_single_arm.py`（五轴断力矩）。
- `--screenshot <path>` 支持离屏截图（第 339 行起），但无自动化断言。

### 1.2 可复用原语 `workspace/rpi_gui_snapshot/gui_fb.py`

- `Button`（按压态、碰撞、回调）、`Radio`、`LinuxFramebuffer`（校验 16bpp/stride，mmap 写 fb0）、`EvdevTouchReader`（evdev 协议解析、轴缩放钳制、SYN_REPORT 合成 DOWN/MOTION/UP）、`touch_event_to_mouse_event`、`force_opaque_kms_framebuffer`、`find_font/load_font`（FONT_CANDIDATES 第 86–93 行）、`is_widget_visible_on_manual_tab`（`_tab=None` 公共控件语义，AGENTS.md 要求有回归测试，已存在）。
- 旧二级菜单 GUI（FbGUI）：main/auto/manual/log 页 + 常驻状态栏（连接/模式/力矩/电磁铁/标定/任务 + 关节角）+ 全页面急停，是"状态栏一眼可见"的现成参考实现（`_draw_status` 第 ~520 行）。

### 1.3 后端契约 `workspace/rpi_gui_snapshot/gui_backend.py`

- `RobotState` 快照 + 四回调（on_log/on_state/on_progress/on_task_done），全部非阻塞、工作线程回调 → GUI queue 转主线程。`MockBackend` 可跑；`RealBackend` 未实现（`create_backend(prefer_real=True)` 直接 raise）。
- 电磁铁唯一接口 `MagnetController.set_enabled(bool)`，env 开关 `EPUZZLE_MAGNET_ENABLED=1`，单次通电上限 10s，GPIO17 高电平吸合（/dev/gpiochip0，已冻结）。

### 1.4 测试 `workspace/rpi_gui_snapshot/test_gui_touch.py`

- 7 项：KMS alpha 修正、RGB565 字节序、evdev 轴缩放钳制、`_tab=None` 公共控件、FINGER* 映射（DOWN/MOTION 钳制/UP）。
- **本机验证通过**：`PYTHONPATH=workspace/src python3 -m unittest test_gui_touch -v` → 7 tests OK（需先 `pip install pygame-ce`，宿主 base conda 原本没有；README 未记录 `PYTHONPATH=workspace/src` 这个前置条件——`gui_backend.py` 第 32 行 import `control.magnet_controller`）。

### 1.5 计划/部署文档关键约束

- `docs/11_运行状态机与故障恢复.md`：17 态状态机（INIT…RECOVERY/FAILED），只有 Supervisor 可切换状态，每态有最大持续时间，低高度动作前检查目标/时间戳/置信度。
- `docs/23_单臂运动控制重构.md`：五轴固定顺序、标定 accepted 门、PARK 门（工具脚本第 ~240 行：不在 PARK ±4° 拒绝页面动作）、PREGRASP=20mm 冻结、PICK=UNKNOWN、D 点终点补偿（径向 3mm、pitch -87.5°）。
- `docs/18_树莓派5部署.md`：启动自检 8 步、服务划分、离线依赖。
- systemd（`deployment/raspberry_pi/overlay/etc/systemd/system/`）：`epuzzle-magnet-safe.service`（oneshot fail-safe 关磁铁）、`epuzzle-preflight.service`、`epuzzle-supervisor.service`（均 `WantedBy=epuzzle.target`）。**没有任何 GUI unit**；GUI 仅由 `workspace/tools/start_pick_place_page.sh`（openvt 绑 VT2 + setpriv 降权 epuzzle + SDL dummy + fb0 环境变量）启动。

---

## 2. 发现与风险（按严重度）

| 级别 | 位置 | 问题 |
| --- | --- | --- |
| blocker | `pick_place_page.py:146` | 状态来自 `output.strip().splitlines()[-1]`——工具脚本最后一行 stdout 是整个 records JSON（`control_red_piece_centers.py:313` print 全量 JSON），截断到 80 字符后状态行是无意义片段。结构化结果已写入 `workspace/runtime/page_{mode}_result.json`（含 `result: PASS`、events、final_joint_deg）但页面从不读它。违反任务要求"状态机驱动而不是抓 subprocess 最后一行"。 |
| blocker | `pick_place_page.py:225-228` | 急停依赖 `sudo systemctl restart`：全仓库无 sudoers/NOPASSWD 配置（已 grep overlay、build-image.sh、README），epuzzle 用户下会卡在密码提示或直接失败；且 `run()` finally（第 330 行）在普通退出时也调用 `emergency_stop()` → 每次关 GUI 都重启系统服务。磁铁 fail-safe 本应由 magnet-safe service 在关机路径负责，GUI 不应持有 sudo。 |
| high | `pick_place_page.py` 整体 | 无页面状态机：`self.status` 是裸字符串；运行中按钮不置灰（只靠 `_run` 里一句"已有动作运行中"）；子进程无超时（挂死只能急停）；无耗时/进度显示。赛场上"卡没卡住"一眼看不出来。 |
| high | 缺失 | `pick_rotate_place` 安全锁定正确，但竞赛"一键流程"（检测→选块→预抓→抓放→PARK）尚无 shadow 模式与解锁条件定义；现场白色碎片/扑克牌流程没有入口。 |
| medium | 缺失 | 无 pick_place_page 自身的截图/交互回归测试（按钮矩形、急停热区、选中切换、结果文件驱动状态）；现有 7 项测试只覆盖 gui_fb 原语。 |
| medium | `rpi_gui_snapshot/fonts/` | 目录为空；`find_font()` 依赖系统字体（wqy-zenhei 等）。若镜像缺包中文变方块。合同应要求镜像内固定字体文件 + 启动自检。 |
| low | `pick_place_page.py:332` 等 | `pointer_pos` 抬指后不清除，软光标残留；fb0 模式下才画光标。 |
| low | README | 测试运行前置（pygame-ce、PYTHONPATH=workspace/src）未记录。 |
| info | 布局 | 按钮 184×52px（7 英寸 1024×600，约 155mm 物理高度 ⇒ 52px≈13mm）达标；但状态/安全提示 18px≈4.7mm，裁判距离下偏小，关键状态应升级为大号色块。 |

---

## 3. 赛场 GUI 实现合同（建议方案，供规划 agent 采用）

### 3.1 信息架构（1024×600 固定，三页 + 常驻区）

- **常驻顶栏（56px）**：页面名 + 全局状态色块（连接/力矩/磁铁/标定/PARK）+ 大字号当前阶段 + 右上红色急停（≥96×48，所有页面可点）。复用 `FbGUI._draw_status` 模式。
- **常驻底栏（~64px）**：一行大字状态（24px+，颜色编码：INFO 灰/WARN 橙/ERROR 红）+ 运行耗时与进度条。
- **首页**：检测预览图（左）+ 碎片列表/选中信息 + 4 个大入口（运行页 / 标定与诊断 / 刷新检测 / 回 PARK）。沿用现有截图的视觉语言（深色底、圆角面板、蓝绿主按钮）。
- **运行页**：碎片选择（上/下一块 + 预览高亮编号）、目标角度 ±15°（%180，显示当前角 vs 目标角）、快速扫块（sweep）、单块 repeat/pregrasp、一键流程按钮（见 3.4）。
- **标定与诊断页（精简）**：自检结果清单（舵机在线/相机/标定文件/homography 重投影/磁铁默认关）、回 PARK、断力矩、日志尾部 5 行。不做手动 jog（赛场不需要，避免误触）。

### 3.2 状态机驱动（替代抓最后一行）

- 页面内显式 `PageState`：IDLE / RUNNING(action) / ESTOP / FAULT，唯一写入口；按钮可用性 = f(state)。
- 子进程合同改为**结果文件驱动**：工具已写 `page_{mode}_result.json`（schema_version/mode/result/events/final_joint_deg）；页面只用它 + returncode 判定成败，stdout 仅进日志。给工具加 `--status-file` 周期写心跳（阶段名+时间戳）或在页面侧加子进程超时（sweep 按片数估算，如 20s/片上限）。
- 运行中禁用所有动作按钮（视觉置灰），仅急停/中止可点；超时自动判 FAULT 并提示恢复路径（回 PARK → 刷新检测）。

### 3.3 碎片选择与目标位姿

- 数据源：`workspace/runtime/red_piece_detection/pieces.json`（schema：piece_id/center_table_mm/yaw_deg/confidence/polygon，frame=table_mm 已带单位，符合 AGENTS.md 坐标契约）。
- 预览图叠加选中碎片高亮框 + 序号；信息行显示 `i/N 中心(x,y)mm 当前角→目标角`。
- 工具侧 PARK 门（±4°）拒绝时，页面必须把拒绝原因原文显示为 ERROR 并给出"回正式 PARK"引导按钮。

### 3.4 一键流程 shadow/安全锁

- 三段式：`SHADOW`（全流程空跑，磁铁强制关，结果文件标记 `magnet_enabled:false`，现有工具已是此语义）→ `ARMED`（PICK 高度标定冻结后由配置解锁，配置项缺失时按钮物理隐藏而非仅禁用）→ 正式。
- 解锁条件硬编码检查清单：PICK 高度标定文件存在且状态 accepted + 单片 20 次成功率记录 ≥80%（docs/13 硬门）。
- 任何一键流程步骤失败：抬升安全高度 → 回 PARK → 状态栏 ERROR + 恢复按钮（docs/11 规则）。

### 3.5 警告/错误/恢复 & 急停

- 三级消息：INFO（灰）/WARN（橙，可继续）/ERROR（红，必须处置）；ERROR 常驻直到用户确认或执行恢复动作。
- 急停：terminate/kill 子进程 + 调用 `emergency_off_single_arm.py` 断力矩；**去掉 `sudo systemctl restart`**（磁铁本就常关；若未来启用磁铁，用 `MagnetController.emergency_off()` 直接走 GPIO，GUI 进程在 gpio 组内无需 sudo）。普通退出路径不再触发 systemctl。
- 急停后进入 ESTOP 锁存态：只有"回 PARK 并复位"按钮可解除，防止误触继续。

### 3.6 触摸与字体

- 所有可点目标 ≥48px 高（当前 52 达标）；急停 ≥96×48 且四边留 8px 隔离区；相邻按钮间距 ≥8px（已达标）。
- 中文字体：镜像内放置固定 ttf（复用 FONT_CANDIDATES 首项 `fonts/NotoSansCJK-Regular.ttc`），启动自检找不到字体→ERROR 上屏而不是静默方块（gui_fb 目前只 print 到 stdout，fb0 用户看不见）。

### 3.7 无常驻资源

- 保持现状：页面只常驻 fb0 + evdev；串口/GPIO 由短命子进程持有，结束后回 PARK 断力矩（`control_red_piece_centers.py` finally 已保证）。GUI 自身不 import 串口/GPIO 库做常驻连接。

### 3.8 systemd 启动

- 新增 `epuzzle-gui.service`：`After=epuzzle-preflight.service`，`ExecStart=/opt/epuzzle/workspace/tools/start_pick_place_page.sh`，`Restart=on-failure`，挂到 `epuzzle.target`（Wants += epuzzle-gui.service）。保留手动 openvt 脚本路径作为 fallback。服务里不需要 sudo（fb0/input 经 epuzzle 的 video/input 组）。

### 3.9 测试清单（精确文件）

- 保留并运行 `workspace/rpi_gui_snapshot/test_gui_touch.py`（7 项，本机已验证 OK；README 补 `PYTHONPATH=workspace/src` + pygame-ce 前置）。
- 新增 `workspace/tests/test_pick_place_page.py`（headless，SDL dummy）：
  1. `--screenshot` 离屏渲染 1024×600 非黑、颜色数 >阈值（对应 AGENTS.md fb0 验收方法）；
  2. 按钮矩形不重叠、全部在屏内、急停热区 ≥96×48；
  3. 模拟 `page_sweep_result.json`（PASS/缺文件/JSON 损坏）驱动状态行与 PageState 转移；
  4. 运行中点击动作按钮被忽略（状态不变）；
  5. `select()` 在无碎片/有碎片/越界回绕行为；
  6. 急停函数不调用 `sudo`（静态断言或 mock subprocess.run 参数检查）；
  7. `_tab=None` 公共控件语义回归（若页面引入 tab）。
- 硬件在环（Pi）：fb0 非黑像素/颜色数 + 实屏触摸坐标命中（AGENTS.md 已规定的验收方式）。

---

## 4. Meta-prompt 交接（给规划/实现 agent）

- **目标**：产出 pick_place_page 的重构实施计划（或直接实现）：状态机 + 结果文件驱动 + 三页 IA + 一键流程 shadow 锁 + 急停去 sudo + 新测试文件。
- **证据**：本文件 §1（行号）、§2（风险表）、§3（合同）；截图 `workspace/calibration/evidence/2026-07-31_gui_a4_workspace/formal_pick_place_page.png`；结果 JSON schema 见 `control_red_piece_centers.py:290-313`。
- **硬约束**：只读基线不得破坏——五轴顺序、PARK ±4° 门、PREGRASP=20mm、PICK=UNKNOWN 期间抓放保持锁定、磁铁唯一接口 `MagnetController.set_enabled`、坐标带单位；ACT 可配置关闭；GUI 无常驻串口/GPIO。
- **验证**：`PYTHONPATH=workspace/src python3 -m unittest test_gui_touch`（rpi_gui_snapshot）；新增测试在宿主 SDL dummy 下可跑；Pi 侧 fb0/触摸验收按 AGENTS.md。
- **停止/升级**：PICK 高度标定数据不存在——不要尝试解锁抓放；sudoers 变更属系统配置，需要决策时通过 intercom 上报。

## 5. 已解决问题与假设

- 宿主运行 GUI 测试需 `pip install pygame-ce`（已装入 conda base）+ `PYTHONPATH=workspace/src`；7 项全过。
- `workspace/runtime/` 在开发机不存在，由工具运行时创建；页面读 pieces.json 失败时静默置空（行为正确，但应上屏提示"未检测"——已有）。
- 假设：MPI700 物理分辨率 1024×600 与 fb0 virtual_size 一致（`LinuxFramebuffer.read_size` 运行时读取，无需硬编码）。
