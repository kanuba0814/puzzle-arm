# Oracle 独立技术审计：adopt-puzzle-vision-v2-2 剩余软件 + 赛场 GUI

只读审计，未修改任何项目/source 文件。依据：`.taskman/plans/adopt-puzzle-vision-v2-2/{PLAN.md,context.md}`、仓库实测状态、同轮三份上下文产物（`algorithm.md`、`gui.md`、`supervisor.md`，已交叉核对，结论相互一致）。目标：找出最容易导致“看似完整、赛场出洋相”的问题。

---

## 1. Inherited decisions（基线合同，不可静默推翻）

- 优先级：安全/确定性基线 > 自备 50 分 > 白色 20 分 > 扑克 30 分 > ACT；ACT 默认关闭。
- 两个深模块：`vision.observe_pieces(frame, profile) -> VisionResult`、`puzzle.solve_puzzle(workspace, target_region, options, texture_evidence=None) -> PuzzleSolveResult`；内部实现私有。
- 独立重实现外部 v2.2 算法思想，无 LICENSE 不复制代码/Joker 素材；`cut_mode` 只进测试生成器。
- 红色基线 `detect_red_pieces` 本计划内不删不改行为；新功能默认 `generic_vision_enabled=false`、`texture_enabled=false`。
- 五轴顺序、GPIO17 高电平吸合/10s 软件上限、PREGRASP=20mm、Z_SAFE=60mm、PICK/PLACE=UNKNOWN 已冻结；PICK 高度冻结前禁止磁铁抓放。
- `contact_sheet_to_robot_base_20260731.json` 不得冒充正式 `table_mm -> robot_base_mm`（docs/10）。
- Gate C 统一为“离线白色盲测 ≥18/20 且实机连续 5 次”，取代 docs/19 的 9/10 口径。
- 每个跨模块字段带 frame/单位；证据分合成/开发机/Pi/真机四类标签；仿真数字不得写成实测。
- 目标矩形在官方 90×50~120×90 mm 内推断；SE(2) 禁止镜像；歧义返回 `SOLVER_AMBIGUOUS` 而不是伪造唯一解。

## 2. Diagnosis：当前真实状态

- `workspace/src/puzzle/`、`workspace/src/integration/` 只有 README——求解器和 Supervisor 完全不存在。
- `vision` 只有 `red_piece_detector.py`（205 行）；通用白色观察不存在。
- 正式 GUI 是 `workspace/apps/pick_place_page.py`（fb0+evdev，正确技术栈），但它是“按钮 spawn 短命工具脚本”的调试面板，不是状态机驱动的赛场界面。
- 镜像 auto-stack 是**指向不存在文件的空壳**（详见 §3.1）——当前真正工作的链路只有“SSH/VT 手动起 GUI → 按钮 → 子进程”。
- 实测已冻结：A4 homography（RMS 0.257541 mm）、运动学 accepted 基线、PARK、红色检测回放证据（`workspace/calibration/evidence/2026-07-31_gui_a4_workspace/`）。
- 所有“看起来接线好了”的部分（systemd supervisor、preflight、相机 udev）实际全部不闭合——这是“看似完整”的最大来源。

## 3. Drift / contradiction check（按严重度）

### 3.1 Blocker 级（不修则赛场直接出洋相）

1. **`epuzzle-supervisor.service` 指向不存在的目标**。`deployment/raspberry_pi/overlay/etc/systemd/system/epuzzle-supervisor.service` 有 `ConditionFileIsExecutable=/opt/epuzzle/workspace/src/integration/run_supervisor.py`，该文件不存在（integration 只有 README）。条件不满足时 systemd **静默跳过**——一键自动流程在镜像上是死的，且没有任何报错。
2. **preflight 自检必然 FAIL，且失败原因具有误导性**。`overlay/usr/local/sbin/epuzzle-selfcheck` 的 `calibration_files` 期望 `camera_intrinsics.json / table_homography.json / table_to_robot.json / tool_center_offset.json`，而实际冻结文件是 `table_homography_gui_20260731.npy`、`contact_sheet_to_robot_base_20260731.json` 等——文件名漂移；`camera_device` 检查依赖 `/dev/epuzzle-camera`，而 `70-epuzzle-devices.rules` 的相机规则**故意不匹配**（`idVendor==ffff` 占位）；`parameters_frozen` 要求 registry 中 `"UNKNOWN"` 计数为 0（当前 20+ 处）。三者叠加 ⇒ `epuzzle-preflight.service` 永远失败 ⇒ `Requires=` 的 supervisor 永远不起。**风险不在于“现在不起”，而在于有人会为了让它起来而放松检查或捏造 UNKNOWN 值**——必须明确：正确修法是冻结真实参数/补真实文件名，不是删检查。
3. **镜像里没有 pygame，正式 GUI 无法从冻结镜像启动**。`deployment/raspberry_pi/requirements-arm64.in` 无 pygame/pygame-ce；`build-image.sh` 只 rsync 该 venv（第 66 行）且不装任何 apt 包；而 `workspace/tools/start_pick_place_page.sh` 依赖 venv site-packages 或 `/usr/lib/python3/dist-packages` 提供 pygame。当前 Pi 上的 GUI 能跑说明pygame 是手工装的——**重刷镜像即丢失**，且 wheelhouse 须按 AGENTS.md 教训在 aarch64 环境解析并实机 import 验证。
4. **GUI 状态行抓子进程 stdout 最后一行**。`pick_place_page.py:146` `output.strip().splitlines()[-1]`，而 `control_red_piece_centers.py` 末尾 print 全量 records JSON——状态行是 JSON 截断片段，赛场上“成功/失败”显示为乱码片段。结构化结果已写入 `workspace/runtime/page_{mode}_result.json` 但页面从不读。
5. **急停按钮在无 sudoers 配置的系统上调 `sudo systemctl restart`**（`pick_place_page.py:225-228`）。全仓库 overlay/build 脚本均无 NOPASSWD 配置——epuzzle 用户下会卡密码提示或失败；且 `run()` 的 finally（约 330 行）使**正常退出 GUI 也会触发 systemctl**。磁铁 fail-safe 已由 root 级 `epuzzle-magnet-safe.service` 负责，GUI 不应持有 sudo。
6. **检测→控制链路无时效校验**。`control_red_piece_centers.py:load_pieces()` 读 `workspace/runtime/red_piece_detection/pieces.json` 不看 `timestamp_ns`；`red_piece_detector.py` 用的是 `time.time_ns()`（非单调）。检测后裁判/人员移动碎片再点“巡检”，机械臂按旧坐标走。Supervisor 必须有 `max_vision_age_ms` 门（registry 登记，未冻结前 fail-closed 于 shadow）。
7. **文档/契约三处硬漂移**（Phase 1 必须修，不是“注意一下”）：
   - `docs/19` Gate C 写“10 套 9 套”vs `docs/01` R4 与 PLAN 的“20 套 ≥18 套”；
   - `docs/10` PieceObservation 列 `contour_rectified_px` 为必要字段，但 `piece_observation.schema.json` 没有该字段且 `additionalProperties: false`——按文档输出必然过不了 schema；
   - `docs/06` 末尾求解器输出字段名（`pick_point_table_mm/source_yaw_deg/.../solver_confidence`）与 `placement_plan.schema.json`（`source/target/plan_confidence`）不一致——以 schema 为准改文档。
8. **yaw 双表示**：两个 schema 都是 `minimum:-180, maximum:180`（含两端），PLAN 要求统一 `exclusiveMaximum: 180`。不改则 -180/180 两个合法值破坏等价 yaw 去重的确定性。

### 3.2 High 级（看似能跑，实则有坑）

1. **legacy confidence = `area/2500`**（`red_piece_detector.py:136`）——正是 PLAN 明确拒绝的“面积映射置信度”。legacy 行为保留可以，但新 `observe_pieces` 严禁复制该模式；回放基线测试要把这个值钉死，防止有人“顺手改进”破坏回归。
2. **每次按钮动作都是 venv Python 冷启动 + `import lerobot`**（`control_red_piece_centers.py` 顶层 import）。Pi 5 上单次进程启动+导入可达数秒，一场 4 片×（检测+多段动作）很容易吃掉 120 s 官方上限的大头。常驻 Supervisor（`Nice=-5` 的 unit 已备好）不只是架构洁癖，是时间预算刚需。
3. **Tkinter `rpi_gui_snapshot/gui_app.py` 在交付镜像上不可运行**（Lite 镜像无 X/Wayland），`gui_backend.py` 的 `RealBackend` 未实现。它是开发期 mock——必须从赛场交付物里剔除或明确标注 dev-only，防止现场起错界面。
4. **镜像 `epuzzle.env` 里 `EPUZZLE_MAGNET_ENABLED=1`**，同文件注释却写“硬件参数未实测前不自动动作”。虽然 grasp 链路被 PICK=UNKNOWN 锁住、boot 时 magnet-off service 兜底，但全局 env 默认允许磁铁与冻结姿态矛盾。建议改 0，grasp gates 全过后再翻（见 §7 待决项）。
5. **`control_red_piece_centers.py` 实质把 contact-sheet 变换当 table→robot 用**（`TRANSFORM_PATH` + `target_pose()`）。这是当前已验收红色基线的既成事实（A4 与触点纸同台面），不要在本轮“顺手修掉”破坏基线；但**白色新链路不得继承此捷径**，正式 `table_mm→robot_base_mm` 冻结前白色只能 shadow（PLAN Phase 6.5 已有此门）。
6. **docs/10 桌面原点措辞“左上或左下，必须全项目统一”** vs registry 已冻结“bottom-left, y up 0-297”。措辞收敛为左下，避免新代码按左上实现。

### 3.3 Medium/Low

 1. `rpi_gui_snapshot/fonts/` 为空，`find_font()` 靠系统字体；镜像缺中文字体则满屏方块——镜像内固定 ttf + 自检上屏（gui.md §3.6）。
 2. 两个文档编号 23（`23_单臂运动控制重构.md`、`23_相机支架设计.md`）；新增 `docs/24_…` 无冲突，但应顺手把相机支架改为 25 或合并编号。
 3. `pick_place_page.py` 子进程无超时、运行中按钮不置灰、无耗时显示；`pointer_pos` 抬指不清除。
 4. `build-image.sh` 写 `display_auto_detect=0` 的 headless profile——AGENTS.md 表明实机 fb0 GUI 已工作（HDMI 不受影响），但重新构建镜像后必须在实机复核 fb0 非黑+触摸，不得想当然。
 5. `official_docs/competition/E题_拼图装置.txt` 是 PDF 的派生文本；algorithm.md 的行号引用可用，但事实判定以 PDF 为准（AGENTS.md）。

## 4. 本轮必须实现的 P0 / P1

### P0（软件可闭环、离线可验证；做完才谈“剩余软件完成”）

按 PLAN Phase 0–4 + Supervisor + GUI + 部署修补，六项缺一不可：

1. **Phase 0 冻结**（全是小改动，先做）：
   - `docs/references/puzzle_vision_simulator_v2_2.md`、`docs/24_通用视觉与拼图求解器设计.md`；
   - `workspace/tests/test_red_piece_replay.py`：用 `red_pieces_frame.jpg` + `table_homography_gui_20260731.npy` 钉死 4 片数量/中心(≤1mm)/角度（注意 JSON `angle_deg` 与检测器 `[-90,90)` yaw 的 ±90° 等价换算，先打印实测再定容差）；
   - Gate C 口径统一（docs/01、12、19、PLAN 四处只剩 18/20+实机 5 次一套数字）；
   - docs/12 增加 synthetic/dev-machine/pi5/hardware 四证据标签。
2. **Phase 1 契约**：`piece_observation.schema.json` 加嵌套 `measured_contour`（自带 `image_px|rectified_px` frame）+ 负例测试；新增 `workspace_state.schema.json`、`puzzle_solve_result.schema.json`；`placement_plan.schema.json` 加纹理状态/等价 yaw/来源时间戳；全部 yaw 改 `exclusiveMaximum: 180`；docs/10 补 `VISION_MERGED_BLOBS/VISION_SURFACE_MISMATCH/SOLVER_AMBIGUOUS/SOLVER_TIMEOUT/TEXTURE_INSUFFICIENT/TEXTURE_AMBIGUOUS` 并修 `contour_rectified_px` 条目；docs/06 字段名对齐 schema；registry 登记 `px_per_mm`、几何容差、歧义间隔、`solver_time_budget_ms=10000`、`solver_max_expansions`、`generic_vision_enabled=false`、`texture_enabled=false`（测量值一律 UNKNOWN）；固定种子夹具生成器 + “无生成器真值泄漏”测试。沿用 `workspace/tests/test_control_data_contracts.py` 的纯 json 校验模式（开发机无 jsonschema 包）。
3. **Phase 2–3 深模块实现**：`vision.observe_pieces`（多阈值分割、merged-blob/surface-mismatch 显式拒绝、凹性保留、profile 门控圆角虚拟化、距离变换抓取点、分项诊断）；`puzzle.solve_puzzle`（完整/端点锚定/内部 offset 部分边、pair 分组候选、SE(2) 回溯+NumPy 有界最小二乘、目标矩形 90–120×50–90 网格推断、统一 penalty 评分、等价类去重后最佳/次佳间隔、确定性 sort key）。验收按 PLAN 数字：合成回放数量正确率 100%、应力集显式拒绝且高置信错误 0、630+ 几何夹具第一等价类、字节级重复性。
4. **Supervisor（`integration/run_supervisor.py` 及同目录模块）**：按 `supervisor.md` §2 合同实现——`SupervisorConfig`、状态机（shadow 时 motion/magnet 注入 Stub，任何 `send_action`/`set_enabled` 直接 raise）、PARK↔HOME、`grasp_gates()` fail-closed、视觉时效门、JSONL 事件日志（符合 `runtime_event.schema.json`）、`sweep_planner`（见 §5）。它同时填上 §3.1-1 的空壳和 §3.2-10 的冷启动时间坑。
5. **GUI 重构**（`pick_place_page.py` 或新赛场页）：按 `gui.md` §3 合同——显式 `PageState` 状态机 + 结果文件驱动（读 `page_{mode}_result.json`，不再抓 stdout）、运行中按钮置灰+子进程超时、急停去 sudo + ESTOP 锁存态、一键流程 SHADOW→ARMED 两段锁（PICK 高度 accepted 前按钮物理隐藏）、字体自检上屏、新增 `workspace/tests/test_pick_place_page.py`（headless SDL dummy：截图非黑/颜色数、按钮矩形、结果文件驱动状态转移、急停不调 sudo 的静态断言）。
6. **部署修补**（见 §7）：pygame-ce 进 `requirements-arm64.in` 并重建 arm64 wheelhouse；preflight 的 calibration 文件名对齐已冻结文件；GUI systemd unit（`After=epuzzle-preflight.service`）；镜像重刷后实机 import+fb0+触摸验收。

### P1（本轮做但不阻塞 P0 验收）

- 纹理模块 seam（`TextureEvidence`、scorer 接口、专门回归证明 scorer 真被调用），功能默认关闭；
- `KnownCardRegistration` adapter 接口占位（不实现 SIFT 主流程）；
- `analyze_puzzle_frame.py` / `benchmark_puzzle_pipeline.py` 工具 + GUI shadow 按钮接 `ShadowPlanReport`；
- docs 05/15/21 同步、编号 23 冲突清理、Tkinter gui_app 标注 dev-only；
- `EPUZZLE_MAGNET_ENABLED` 默认 0（需主代理确认，见 §8）。

## 5. 明确延后的实测 Gate（本轮禁止用软件结果冒充）

| Gate | 阻塞项 | 未过前行为 |
| --- | --- | --- |
| Gate C 实机 | ≥20 套真实白色 2–4 片盲测 ≥18 套第一等价类 + 实机连续 5 次 | 白色只 shadow；不接动作 |
| PICK/PLACE 接触高度 | `z_pick_mm`/`z_place_mm` 实测标定（10s 停留确认法） | 抓放按钮物理隐藏，磁铁链路锁死 |
| 正式 `table_mm→robot_base_mm` | 独立标定+SHA256，替代 contact-sheet 代理 | 白色 plan 不得下发控制层 |
| 磁头 footprint/净空门槛、`on/off_settle_ms`、`max_pick_retries` | 实物测量 | “可抓”判定与重试 fail-closed |
| 相机型号/曝光/白平衡冻结 + udev 真实 id | 实机 `udevadm info` | `/dev/epuzzle-camera` 保持不匹配属预期 |
| Phase 2 真机退出 | 30 帧 std ≤1mm、20 网格点 ≤3mm、100 帧无持续误检 | 不得宣布视觉 DONE |
| Pi 5 性能 | 白色视觉+求解 p95 ≤2s、max ≤5s、RSS ≤250MB | 只优化剪枝，不降安全判据 |
| Gate E 纹理 | ≥40 个几何歧义 holdout、top-K 召回 ≥95%、top-1 ≥90% 且 +20pt | `texture_enabled=false`，白色功能不受影响 |

## 6. 深模块 seams（与同轮三份产物一致，直接采用）

1. `vision.observe_pieces(RectifiedFrame, VisionProfile) -> VisionResult`——公共面仅 `RectifiedFrame/VisionProfile/PieceObservation/VisionResult/observe_pieces`；分割/圆角/抓取点为 `_segmentation.py`/`_piece_geometry.py` 私有（algorithm.md §2）。
2. `puzzle.solve_puzzle(WorkspaceState, TargetRegion, PuzzleOptions, TextureEvidence|None) -> PuzzleSolveResult`——`EdgeDescriptor`、图搜索、位姿传播、栅格评分全私有；`puzzle` 不读文件、不碰 vision 私有实现（algorithm.md §4）。
3. `integration` Supervisor——公共面 `SupervisorConfig/SupervisorState/SupervisorResult/run_supervisor` + 注入式 `motion`/`magnet`/`clock` seam；shadow 用 Stub 注错（supervisor.md §2.1–2.5）。
4. GUI↔执行层 seam = **结果文件 + returncode**（`workspace/runtime/page_{mode}_result.json`），GUI 永不解析 stdout、永不常驻串口/GPIO（gui.md §3.2/3.7）。
5. 契约 seam = `data_contracts/*.schema.json` + registry；任何跨模块改动必须 schema/docs/测试三同步（AGENTS.md）。

## 7. 快速扫块的最短安全算法（采用 supervisor.md §2.6，补强两点）

- 输入：`PieceRef{piece_id, pick_point_table_mm}`（**用抓取点不用质心**）+ 起点 = HOME 上方 TCP 在 table_mm 的投影（FK 算一次冻结进 config 证据）。
- 排序：n≤4 直接**全排列穷举**（4!=24），成本 = 相邻点 table_mm 欧氏距离和（同一 Z_SAFE 平面巡航，水平距离是时间一阶代理）；n>4 退化最近邻并记录。tie-break 取 `piece_id` 字典序 ⇒ 字节级稳定。
- 执行骨架：每点 Z_SAFE(60mm) 巡航 → 目标上方 → 降 PREGRASP(20mm) → 停 `sweep_dwell_s`（legacy 1.0s）→ 回 Z_SAFE；全程经 `MotionController`（phase=SAFE/PREGRASP），**不降 PREGRASP 以下**。
- 整段预检：发送第一个目标前对所有点做 IK 干跑，任一不可达 ⇒ `IK_NO_SOLUTION` 整段拒绝（Supervisor 层对应 MotionController 的全路径预检）。
- 成本矩阵记入事件日志 payload；扫块前必须过视觉时效门（`max_vision_age_ms`，未冻结前该路径 fail-closed）。
- 补强：① 扫块期间 yaw 保持不变，不借用 `yaw_deg` 初值做姿态假设；② 把 dwell、巡航速度档登记 registry 而不是写死副本，避免与 legacy 工具参数漂移（supervisor.md §4-4 已警示）。

## 8. GUI 用户流（赛场，1024×600）

1. **开机**：`epuzzle-magnet-safe` → `epuzzle-preflight`（自检清单全绿才继续）→ GUI 首页（常驻顶栏：连接/力矩/磁铁/标定/PARK 色块 + 大红急停 ≥96×48）。
2. **首页**：自检结果 + 「刷新检测」「运行页」「标定诊断」「回 PARK」四大入口；自检 FAIL 项以 ERROR 色块常驻并给处置提示。
3. **检测**：一键抓帧 → 预览图叠加碎片编号框 → 碎片列表（i/N、中心 mm、当前角）。检测过期（超 `max_vision_age_ms`）时预览加盖“已过期”水印并禁用动作按钮。
4. **运行页**：上/下一块切换、目标角 ±15°、快速扫块、单块 repeat/pregrasp、**一键流程按钮**——SHADOW 模式全流程空跑（磁铁强制关、结果文件 `magnet_enabled:false`）；ARMED 需 PICK 高度 accepted + 单片 20 次 ≥80% 记录，未满足时按钮**物理隐藏**而非置灰。
5. **白色/扑克入口**：shadow 按钮 → 调 `solve_puzzle` → 展示 `ShadowPlanReport`（候选数、最佳/次佳间隔、推断矩形、错误码）；`AMBIGUOUS/NO_SOLUTION` 以 WARN 上屏，绝不显示“成功”。
6. **异常**：任何步骤失败 → 抬 Z_SAFE → 回 PARK → ERROR 常驻 + 恢复按钮；急停 = terminate 子进程 + `emergency_off_single_arm.py`（无 sudo）→ ESTOP 锁存，仅「回 PARK 并复位」可解。
7. **完成**：声/光指示区分成功/可恢复失败/终止失败（R8）；全程日志 JSONL。

## 9. 可自动验证的 acceptance contract

```bash
# 每次提交
PYTHONPATH=workspace/src python -m unittest discover -s workspace/tests -v
python -m compileall -q workspace/src workspace/tools workspace/apps
ruff check workspace/src workspace/tools workspace/tests workspace/apps
PYTHONPATH=workspace/src python -m unittest discover -s workspace/rpi_gui_snapshot -v   # GUI 原语 7 项
```

逐条可机检项：

1. 所有 `data_contracts/*.schema.json` 可 `json.loads`；正/负例测试存在且负例含“裸像素点挂 `frame=table_mm`”。
2. `test_red_piece_replay.py`：片数==4、中心 ≤1mm、角度等价容差内——红色基线零回归。
3. `grep -rn "cut_mode" workspace/src/vision workspace/src/puzzle` 为空（生产面零拓扑真值泄漏）。
4. 求解器确定性：同一夹具连跑两次输出字节级一致（耗时字段除外）的测试存在并通过。
5. 契约一致性静态检查：`docs/06` 不再出现 `solver_confidence`；`docs/19` 不再出现“9 套/9/10”；两个 schema 的 yaw 均为 `exclusiveMaximum: 180`。
6. GUI：`test_pick_place_page.py` 全过（截图非黑、按钮几何、结果文件驱动、急停源码无 `sudo`——对 `pick_place_page.py` 做 AST/grep 断言 `systemctl` 与 `sudo` 均不存在）。
7. Supervisor shadow 集成测试：Stub 注入下完整跑 SCAN→SHADOW_PLAN，任何 `send_action`/`set_enabled` 调用使测试失败。
8. 镜像验收（Pi 实机，`epuzzle-selfcheck --json`）：`/opt/epuzzle/venv/bin/python -c "import pygame"` 成功；`run_supervisor.py` 存在且可执行；fb0 非黑像素/颜色数达标；实屏触摸命中回归（AGENTS.md 既定方法）。
9. 证据标签：`docs/12` 四标签存在；所有性能/精度数字在日志中带来源标签，无合成数字标属实测（review 项，配 grep 辅助）。

## 10. 树莓派部署 / 回退要求

- **镜像内容**：`requirements-arm64.in` 增加 `pygame-ce==<冻结版本>`，wheelhouse 必须在 aarch64 环境重解析（AGENTS.md：x86 pip 会错拉 CUDA 包），构建后在 rootfs/实机逐项 import 验证；镜像内固定中文 ttf。
- **systemd**：新增 `epuzzle-gui.service`（`After=epuzzle-preflight.service`、`ExecStart=…/start_pick_place_page.sh`、`Restart=on-failure`，挂 `epuzzle.target`）；保留 openvt 手动脚本作 fallback。fb0/input 走 epuzzle 的 video/input 组，无需 sudo。
- **preflight 对齐**：`epuzzle-selfcheck` 的 calibration 文件名改为已冻结真实文件（或新增其期望文件）；相机 udev 用实机 `udevadm info` 属性冻结后替换 `ffff` 占位；`parameters_frozen` 检查**保留**，UNKNOWN 靠实测清零，禁止为绿灯而删检查。
- **不刷 working card**：当前卡作为已知良好基线保留；新镜像构建产物（img.zst + raw.sha256）写第二张卡，`flash-image.sh` 的 raw SHA256 回读必须一致；保留旧镜像文件可整卡回退。
- **运行时回退**：`generic_vision_enabled=false`+`texture_enabled=false` 即恢复纯红色基线；Supervisor unit 因 ConditionFileIsExecutable 天然可摘除；GUI 急停不依赖 systemd。
- **配置决策项**：`EPUZZLE_MAGNET_ENABLED` 建议默认 0，grasp gates 全过后翻 1——需主代理确认（当前镜像为 1，与“未实测不动作”注释矛盾）。
- **首启复核**：重刷后按 AGENTS.md 清单实机复核 fb0 非黑、触摸、磁铁默认关、`cloud-init status` 非 degraded。

## 11. Risks / 残留不确定性

- PICK/PLACE、footprint、桌→机变换三大 UNKNOWN 意味着**本轮交付物在赛场上只能 shadow + 红色基线演示**；若赛程要求本轮内真机白色，则不是软件问题而是实测排期问题，需主代理升级决策。
- 碎片是否纸面朝上/是否有可见间隙/数量下限仍 UNKNOWN（题面只保证不重叠）；完全无缝接触单目不可分——拒绝路径必须有测试，不得宣称覆盖。
- 120 s 预算在常驻 Supervisor 落地前无法准确测算；per-piece 重拍+重求解的时序需在 Pi 基准后回写 registry。
- `display_auto_detect=0` 与 MPI700 在新镜像上的组合需实机复核（旧证据表明 HDMI 不受影响）。
- algorithm.md/gui.md/supervisor.md 三份合同在实现期可能出现细节冲突；冲突时以 PLAN.md + schema 为唯一仲裁，三份产物降级为参考。

## 12. Need from main agent

1. `EPUZZLE_MAGNET_ENABLED` 默认值是否本轮翻 0（建议 0）。
2. 本轮范围确认：P0 六项全部落地≈Phase 0–4 + Supervisor + GUI + 部署修补，工作量很大；若必须裁剪，裁 P1 纹理 seam 与工具打磨，**不可裁**契约/红色回归/Supervisor 空壳/GUI 状态机与去 sudo/镜像 pygame——这五项正是“出洋相”清单本体。
3. 是否授权修改 deployment overlay（preflight 文件名、GUI unit）——属任务边界外模块，按 AGENTS.md 需确认。

## 13. Suggested execution prompt

建议按三份既有上下文产物直接 fan-out 实现（它们已是代码级合同，与本审计一致）：

- worker-algorithm：按 `context/algorithm.md` §1–§6 实现 Phase 0–3（契约→红色回归→vision→puzzle→夹具），验收命令见 §9。
- worker-supervisor：按 `context/supervisor.md` §2 实现 integration 模块 + `run_supervisor.py` + shadow Stub + 事件日志 + `sweep_planner`，测试清单见其 §2.12。
- worker-gui：按 `context/gui.md` §3 重构 `pick_place_page.py`（状态机、结果文件驱动、去 sudo、SHADOW/ARMED 锁、字体自检）+ `workspace/tests/test_pick_place_page.py`。
- worker-deploy（最后，依赖前三项的 `run_supervisor.py`）：pygame-ce 进 requirements、重建 wheelhouse、GUI unit、preflight 文件名对齐；镜像构建与刷写属破坏性/外部长任务，需用户确认后执行。

公共硬约束：不改 `control/`、`robot_plugin/`、五轴顺序、GPIO17、红色检测行为；UNKNOWN 不捏造；schema/docs/测试三同步；每 worker 完成后跑 §9 全量命令。
