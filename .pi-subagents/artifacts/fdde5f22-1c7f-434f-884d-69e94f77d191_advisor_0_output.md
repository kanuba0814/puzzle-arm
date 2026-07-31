分析完成。以下是只读分析结果。

## 核心发现

### 1. Frame 对齐问题（关键，需先确认）
- Canonical `workspace/calibration/contact_sheet_to_robot_base_20260731.json`：`calibration_sheet_mm → robot_base_mm`，来源是手绘 A4 触点纸（原点左下，x 右 0–210，y 上 0–297）。
- GUI 新标定 `/home/epuzzle/puzzle/homography.npy`（SHA256 `d82e6514…`，1280×720，RMS 0.258 mm / max 1.03 mm，57 角点，时间戳 2026-07-31 14:29）：像素→世界 mm，**相同的原点/轴向约定**（`out/calib_summary.json` 明确 "origin=A4左下角, x右0-210, y上0-297"）。
- 若用户 GUI 标定的就是同一张未移动的触点纸，视觉 world mm 可直接复用现有 sheet→robot 变换，**不需要新的手眼标定**。若纸已移动/更换，必须用新 A4 位置重新锚定（最少 3 个触点）——这是最大的未决假设。
- 命名不一致（severity: medium）：`data_contracts/piece_observation.schema.json` 要求 `frame = "table_mm"`，而现有变换输出 frame 是 `calibration_sheet_mm`。建议把 PieceObservation 的 frame 扩展为允许 `calibration_sheet_mm`，或在 vision 输出侧显式重命名并记录等价关系。

### 2. Legacy 栈不可用，且运行中进程占用硬件（severity: blocker，操作性）
- 运行中的 legacy GUI（pid 5903, `/home/epuzzle/puzzle/gui_fb.py`）持有 `/dev/ttyACM0`（fd 31）和 `/dev/gpiochip0` line 17（fd 29/30）。**任何 canonical 运动或磁铁操作前必须先停止该进程**，否则串口/GPIO 独占冲突。
- Legacy `RealBackend`（`gui_backend.py:400-415`）走 `arm_controller.ArmController`；legacy `magnet_controller.py` 使用 `config.MAGNET_PIN = 24`（BCM GPIO24）且 gpiozero 失败时降级为"模拟模式返回 True"——违反"禁止假成功"原则，也与冻结的 GPIO17 冲突。不可复用。
- Legacy `config.py` 的 `PARK_SERVO_POSITIONS {1:1039,2:673,3:3270,4:2467,5:1115}` 与本次新冻结的 PARK raw `[1072,826,3261,2812,1115]` 不一致（J1/J2/J4 不同），必须替换，不能混用。

### 3. PARK 与软限位的矛盾（severity: high）
- 新 PARK joint_deg `[-95.43, -100.84, 96.13, 79.21, -81.98]`：J2=-100.84 超出软限位 [-100,100]，J3=96.13 超出 [-90,90]。这是合理的折叠停放位，但 `MotionController._plan` 对**当前位置**也做 FK 软限位校验，会拒绝从 PARK 出发的笛卡尔规划。
- 最小实现：保留独立的关节空间 PARK↔HOME 恢复路径（插值 direct joint move，即此前已验证的 `direct_move` 模式），不经过 `MotionController`；PARK 文件独立于 kinematic calibration 存放，例如 `workspace/calibration/park_pose_20260731.json`（含 raw、deg、采样数、SHA256、时间戳）。

### 4. Vision 最小边界
- 可用零件：`vision.py:583 detect_colored_pieces`（HSV 红色双区间、形态学、minAreaRect、`pixel_to_world`）+ `gui_calibration.py` 的 UVC 打开逻辑（`/dev/video0`，1280×720，MJPG fourcc）。
- 不要把 896 行的 legacy `vision.py` 整体搬进 canonical（它 import legacy `config`，含大量扑克牌/边缘检测死代码）。最小实现：在 `workspace/src/vision/` 新建约 100 行的 `red_piece_detector.py`：打开 UVC → 抓帧 → HSV 红检测 → 用 canonical 归档的 homography 副本做 `cv2.perspectiveTransform` → 输出符合 `PieceObservation` schema 的记录。把 `homography.npy` 拷贝进 `workspace/calibration/` 并记 SHA256（`d82e6514f9943d63f34649b1b6b228c1fda5a35be17ae6463d6b029fca9f3694`），禁止运行时从 legacy 目录隐式读取。
- 注意 `homography_1440.npy` 是 1440 宽版本；取帧分辨率必须与所用 homography 匹配（1280×720 ↔ homography.npy），或按 `gui_calibration._current_homography` 的方式做缩放——建议直接锁死 1280×720，拒绝分辨率不匹配。

### 5. 运动层已就绪，缺的是 integration
- `workspace/src/integration/` 只有 README。最小竞赛实现：一个 `PickPlaceSupervisor`（建议直接做成 `workspace/tools/pick_place_page.py` 或简单 GUI 页后端），状态机：`PARK →(joint-space)→ HOME → CAPTURE → SWEEP(20mm 过点) → PICK(精确终点模式) → LIFT(60mm) → ROTATE/PLACE → PARK`。
- 精确终点必须套用已冻结补偿：P=24、Accel=30、径向远离 3 mm、命令 pitch=-87.5°（在 `configs/parameter_registry.yaml` 已有字段，代码里尚未消费——需要实现读取，而不是再硬编码）。
- 巡航（sweep）用宽松 waypoint 门槛，且**各碎片中心顺序应按蛇形/最近邻排序**，不要按检测返回顺序。
- PICK 接触高度仍为 UNKNOWN：页面必须先做"降到接触高度"的人工确认子流程，不能把 20 mm 当抓取高度。

### 6. GUI 页面选项（建议 B）
- A. 给运行中的 legacy `gui_fb.py` 加页——违反"不把脏 legacy 树扩展为生产基线"，且其 RealBackend 绑定旧 ArmController，不推荐。
- B. 在 canonical 新建最小正式页（可复用 `workspace/rpi_gui_snapshot/gui_fb.py` 的 fb0/evdev 框架和 `test_gui_touch.py` 已验证的触摸路径），后端直连 `MotionController` + canonical `MagnetController`(GPIO17) + 新 vision 模块。工作量集中在"抓取/旋转/放置"一个页面 + 状态栏，不重做整套菜单。
- 无论哪个选项，启动前都要停止 pid 5903 的 legacy GUI（它也占用 framebuffer/触摸）。

### 7. 其他
- `workspace/rpi_gui_snapshot/README.md` 自述"机械臂动作当前仍为 Mock，RealBackend 尚未实现"——快照里的 backend 不能直接当真实后端，只借 UI 框架。
- 重复点验证：用检测到的某碎片中心，精确模式到位 2 次，记录每次 5 轴读回与 FK 重算 sheet 坐标，写入 evidence JSON；这是低成本且能直接喂给 T2 验收的测试。
- 相机占用：legacy GUI 未持有 /dev/video0（进程 fd 列表无 video 节点），但启动新 vision 前仍应确认。

## 建议的最小实现清单（按序）
1. 停止 legacy GUI 进程；归档 `homography.npy` + `calib_summary.json` 到 canonical 并记 SHA256。
2. 冻结 `park_pose_20260731.json`（raw + deg，5 样本均值，证据哈希）。
3. `workspace/src/vision/red_piece_detector.py` + 单测（合成帧/录制帧）。
4. 关节空间 PARK↔HOME 恢复函数（复用已验证插值模式）。
5. `PickPlaceSupervisor`：CAPTURE→重复点×2→蛇形 sweep→（人工确认接触高度后）pick/rotate/place；磁铁只经 canonical `MagnetController`。
6. 正式页 UI（选项 B）+ 全程 evidence JSON 记录。
7. 确认"GUI A4 == 触点纸"假设；不成立则先重锚定再跑 5。

## 主要风险
- **R1（高）**：GUI A4 与触点纸同一性未证实——若纸动过，所有 sheet→robot 结果失效，sweep 会系统性偏移。
- **R2（高）**：忘记停 legacy GUI 导致串口/GPIO 独占失败或双写磁铁。
- **R3（中）**：PICK 接触高度 UNKNOWN，页面上线后误用 20 mm 抓取会碰纸失败。
- **R4（中）**：PARK 在软限位外，任何"从当前位置直接笛卡尔规划"的调用都会 SAFETY_REJECTED——必须走关节空间恢复。
- **R5（中）**：取帧分辨率与 homography 不匹配导致世界坐标错误（锁死 1280×720 可消除）。
- **R6（低）**：HSV 红色阈值（`config.py:222`）是在旧光照下调的，现场光照变化可能漏检/误检，需现场确认 mask。