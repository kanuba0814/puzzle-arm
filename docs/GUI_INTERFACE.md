# E题拼图装置 GUI 接口定义

更新时间：2026-07-31  
主要源码：`gui_backend.py`、`main.py`、`gui_calibration.py`、`gui_fb.py`

---

## 1. 分层结构

```text
gui_fb.py / gui_app.py
        │
        ▼
RobotBackend 抽象接口（gui_backend.py）
        ├── MockBackend：开发机模拟
        └── RealBackend：树莓派本地硬件
                    │
                    ▼
        ArmController / PuzzleRobot / LocalCalibration
```

GUI 不直接操作舵机、相机或运动规划。  
所有硬件动作都通过 `RobotBackend` 或 `LocalCalibration` 完成。

---

## 2. 通用约定

1. 控制方法默认非阻塞。
2. 同步可判断的错误立即返回：

```python
(False, "错误原因")
```

1. 异步执行结果通过回调返回。
2. 后端回调可能来自工作线程，GUI 必须通过 `queue.Queue` 转入主线程。
3. GUI 主线程不直接调用耗时的视觉或运动函数。
4. 真实硬件模式通过环境变量启用：

```bash
PUZZLE_REAL=1
```

---

## 3. 任务模式定义

定义位置：`gui_backend.py`

```python
TASK_PROFILES = {
    "task1": {
        "label": "任务1 自备纯色",
        "solver_mode": "task1",
        "expected_pieces": 4,
    },
    "task2_white": {
        "label": "任务2A 现场白色",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
    "task2_card": {
        "label": "任务2B 扑克牌",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
    "task2": {
        "label": "任务2 扑克牌",
        "solver_mode": "task2",
        "expected_pieces": 4,
    },
}
```

说明：

- `task1`：自备纯色碎片。
- `task2_white`：现场白色碎片。
- `task2_card`：现场扑克牌碎片。
- `task2`：兼容旧版 GUI 和命令行，当前按扑克牌任务处理。

辅助函数：

```python
def normalize_task_mode(task_mode: str) -> str
def task_profile(task_mode: str) -> dict
```

---

## 4. 数据结构

### 4.1 RobotState

定义位置：`gui_backend.py`

```python
@dataclass
class RobotState:
    connected: bool = False
    simulation: bool = True
    torque_enabled: bool = False
    magnet_on: bool = False
    calibrated: bool = False
    task_state: str = "IDLE"
    joints_deg: list[float] = field(default_factory=lambda: [0.0] * 5)
    servo_raw: dict[int, int] = field(default_factory=dict)
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `connected` | 舵机串口是否已连接 |
| `simulation` | 是否为模拟模式 |
| `torque_enabled` | 力矩是否使能 |
| `magnet_on` | 电磁铁是否通电 |
| `calibrated` | 工作区和基座标定是否均存在 |
| `task_state` | 当前任务状态 |
| `joints_deg` | J1–J5 当前角度 |
| `servo_raw` | 舵机原始位置值 |

允许的任务状态：

```text
IDLE
INIT
CALIBRATING
DETECTING
SOLVING
EXECUTING
DONE
ERROR
```

---

### 4.2 RunMetrics

定义位置：`gui_backend.py`

```python
@dataclass
class RunMetrics:
    elapsed_s: float = 0.0
    limit_s: float = 120.0
    stage: str = "IDLE"
    detail: str = "-"
    progress: float = 0.0
    detected_count: int = 0
    expected_count: int = 4
    placed_count: int = 0
    current_piece: int = 0
    success: bool | None = None
    message: str = ""
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `elapsed_s` | 当前已用时间 |
| `limit_s` | 比赛限制时间，默认 120 秒 |
| `stage` | 当前运行阶段 |
| `detail` | 当前阶段描述 |
| `progress` | 0.0–1.0 总进度 |
| `detected_count` | 已检测碎片数 |
| `expected_count` | 目标碎片数 |
| `placed_count` | 已放置碎片数 |
| `current_piece` | 当前搬运碎片编号 |
| `success` | `None` 运行中，`True` 成功，`False` 失败 |
| `message` | 最新结果或错误消息 |

---

## 5. 回调接口

定义位置：`gui_backend.py`

```python
LogCallback = Callable[[str, str], None]
StateCallback = Callable[[RobotState], None]
ProgressCallback = Callable[[str, float], None]
TaskDoneCallback = Callable[[bool, str], None]
MetricsCallback = Callable[[RunMetrics], None]
FrameCallback = Callable[[object], None]
```

### 5.1 日志回调

```python
on_log(level: str, message: str)
```

- `level`：`INFO` / `WARN` / `ERROR`
- `message`：日志内容

### 5.2 状态回调

```python
on_state(state: RobotState)
```

用于刷新顶部状态栏。

### 5.3 进度回调

```python
on_progress(stage: str, fraction: float)
```

- `stage`：当前阶段
- `fraction`：0.0–1.0

### 5.4 任务完成回调

```python
on_task_done(success: bool, message: str)
```

自动任务结束时调用一次。

### 5.5 比赛指标回调

```python
on_metrics(metrics: RunMetrics)
```

用于运行页显示计时、碎片数量和阶段详情。

注册方式：

```python
backend.set_metrics_callback(on_metrics)
```

### 5.6 相机帧回调

```python
on_frame(frame)
```

- `frame`：BGR 格式的最新相机帧。
- GUI 负责缩放和显示。
- 后端发送前应复制帧，避免工作线程覆盖。

注册方式：

```python
backend.set_frame_callback(on_frame)
```

---

## 6. RobotBackend 抽象接口

定义位置：`gui_backend.py`

### 6.1 回调注册

```python
backend.set_callbacks(
    on_log=...,
    on_state=...,
    on_progress=...,
    on_task_done=...,
)
backend.set_metrics_callback(...)
backend.set_frame_callback(...)
```

---

### 6.2 连接

```python
connect() -> tuple[bool, str]
disconnect() -> None
get_state() -> RobotState
```

说明：

- `connect()` 连接舵机串口和 GPIO。
- `disconnect()` 关闭电磁铁并释放资源。
- `get_state()` 返回当前状态快照。

---

### 6.3 力矩和电磁铁

```python
set_torque(enable: bool, servo_id: int | None = None) -> tuple[bool, str]
set_magnet(on: bool) -> tuple[bool, str]
```

说明：

- `servo_id=None` 表示全部舵机。
- `set_magnet(True)` 表示通电吸附。
- `set_magnet(False)` 表示断电释放。

---

### 6.4 手动控制

```python
home() -> None
jog_joint(joint_idx: int, delta_deg: float) -> tuple[bool, str]
move_joints(joints_deg: list[float]) -> None
jog_xyz(dx: float, dy: float, dz: float) -> tuple[bool, str]
move_to_xyz(x: float, y: float, z: float, wrist_pitch: float = -90) -> None
read_positions() -> dict[int, int]
```

说明：

- `joint_idx`：0–4，对应 J1–J5。
- `jog_joint()` 使用相对角度。
- `move_joints()` 使用 5 个关节的绝对角度。
- `jog_xyz()` 使用相对毫米位移。
- `move_to_xyz()` 使用机械臂坐标系绝对坐标。
- `read_positions()` 返回 `{舵机ID: 原始位置}`。

---

### 6.5 标定

```python
calibrate() -> None
mark_calibrated() -> None
```

说明：

- `calibrate()` 保留给旧 GUI 的一键标定接口。
- 当前树莓派 GUI 主要通过 `LocalCalibration` 完成可视标定。
- `mark_calibrated()` 重新检查持久化标定文件。

标定完成条件：

```text
homography.npy 存在
arm_base.json 存在
```

---

### 6.6 自动任务

```python
start_task(task_mode: str) -> tuple[bool, str]
abort() -> tuple[bool, str]
emergency_stop() -> None
```

#### `start_task`

合法 `task_mode`：

- `task1`
- `task2_white`
- `task2_card`
- `task2`（兼容模式）

同步返回：

- `(True, "比赛任务已启动")`
- `(False, "机械臂未连接")`
- `(False, "请先完成工作区和机械臂基座标定")`
- `(False, "已有任务在运行")`
- `(False, "未知任务模式")`

异步结果通过：

- `on_progress`
- `on_metrics`
- `on_frame`
- `on_task_done`

返回。

#### `abort`

- 设置中止标志。
- 尽量在当前运动结束后停止。
- 不主动释放力矩。

#### `emergency_stop`

- 设置中止标志。
- 关闭电磁铁。
- 释放全部舵机力矩。
- 状态改为 `IDLE`。

---

## 7. MockBackend 行为

定义位置：`gui_backend.py`

用途：开发机无硬件 GUI 联调。

特点：

- `connect()` 总是成功。
- 不访问串口、GPIO 或相机。
- 手动控制只更新模拟状态。
- `start_task()` 模拟：
  - 碎片检测
  - 拼图求解
  - 搬运执行
  - 声光提示
- 持续发送 `RunMetrics`。
- `abort()` 可以中止模拟任务。

---

## 8. RealBackend 行为

定义位置：`gui_backend.py`

用途：树莓派本地真实硬件。

### 8.1 连接

```python
self._arm = ArmController()
```

如果未发现串口：

```python
(False, "未发现本地舵机串口，真实后端不可用")
```

成功后：

- `connected=True`
- `simulation=False`
- `torque_enabled=True`
- 根据 `homography.npy` 和 `arm_base.json` 更新 `calibrated`
- 读取 J1–J5 当前角度

### 8.2 自动任务

真实后端创建可复用的 `PuzzleRobot`：

```python
PuzzleRobot(
    arm=self._arm,
    abort_event=self._abort_flag,
    progress_cb=progress,
    log_cb=robot_log,
    frame_cb=robot_frame,
    allow_interactive=False,
)
```

说明：

- 复用 GUI 已连接的 `ArmController`，避免重复打开串口。
- `allow_interactive=False`，比赛模式禁止交互式标定。
- `progress_cb` 转发阶段、进度和碎片计数。
- `frame_cb` 转发最新检测帧。
- `abort_event` 用于中止和急停。

---

## 9. PuzzleRobot 接口

定义位置：`main.py`

### 9.1 构造函数

```python
PuzzleRobot(
    arm=None,
    abort_event=None,
    progress_cb=None,
    log_cb=None,
    frame_cb=None,
    allow_interactive=True,
)
```

参数说明：

| 参数 | 含义 |
| --- | --- |
| `arm` | 可选，外部已连接的 `ArmController` |
| `abort_event` | `threading.Event`，用于中止任务 |
| `progress_cb` | 阶段进度回调 |
| `log_cb` | 日志回调 |
| `frame_cb` | 最新相机帧回调 |
| `allow_interactive` | 是否允许交互式视觉标定 |

当 `arm is None` 时，`PuzzleRobot` 自己创建并关闭 `ArmController`。  
当外部传入 `arm` 时，`cleanup()` 不关闭该机械臂。

### 9.2 进度回调签名

```python
progress_cb(
    stage: str,
    fraction: float,
    detail: str,
    detected_count: int,
    placed_count: int,
    current_piece: int,
)
```

### 9.3 日志回调签名

```python
log_cb(level: str, message: str)
```

### 9.4 相机帧回调签名

```python
frame_cb(frame)
```

### 9.5 运行方法

```python
run(task_mode: str = "task1") -> bool
```

合法底层求解模式：

- `task1`
- `task2`

`task2_white` 和 `task2_card` 由 `RealBackend` 转换为 `task2` 后传入。

运行结束后读取：

```python
robot.result_message
robot.detected_count
robot.placed_count
robot.current_piece
```

### 9.6 自动流程

```text
INIT
→ CALIBRATING
→ DETECTING
→ SOLVING
→ EXECUTING
→ DONE
```

失败时：

```python
robot.state == "ERROR"
robot.result_message == "失败原因"
```

---

## 10. LocalCalibration 接口

定义位置：`gui_calibration.py`

用途：树莓派本地系统标定，不创建 OpenCV 窗口，不使用网络。

### 10.1 构造

```python
LocalCalibration(arm, log)
```

参数：

- `arm`：本地 `ArmController` 或 `None`
- `log(level, message)`：日志回调

### 10.2 相机

```python
open_camera() -> tuple[bool, str]
close() -> None
update() -> frame | None
```

说明：

- `open_camera()` 打开本地 UVC 相机。
- `update()` 由 GUI 每帧调用，返回叠加标定信息后的 BGR 图像。

### 10.3 工作区标定

```python
calibrate_workspace() -> tuple[bool, str]
```

成功后保存：

- `homography.npy`
- `homography_1440.npy`
- `out/calib_summary.json`

### 10.4 触点标定

```python
set_target_from_view(view_pos, view_rect) -> tuple[bool, str]
release_torque() -> tuple[bool, str]
record_touch() -> tuple[bool, str]
undo_touch() -> tuple[bool, str]
solve_base() -> tuple[bool, str]
```

流程：

1. 在预览中点击目标点。
2. 释放力矩。
3. 手扶末端触碰目标点。
4. 记录触点。
5. 至少记录 3 个分散触点。
6. 解算基座。

成功后保存：

- `arm_base.json`
- `out/arm_base_world.json`

---

## 11. 持久化文件

| 文件 | 含义 |
| --- | --- |
| `homography.npy` | 当前工作区像素到毫米坐标单应矩阵 |
| `homography_1440.npy` | 兼容 2560×1440 工具链的单应矩阵 |
| `arm_base.json` | 机械臂基座到 A4 世界坐标外参 |
| `out/calib_summary.json` | 工作区标定摘要 |
| `out/arm_base_world.json` | 基座标定摘要 |
| `out/paths.json` | 运动/路径调试输出 |

---

## 12. 启动脚本接口

文件：`tools/start_gui_fb.sh`

脚本行为：

1. 优先使用 `venv/bin/python`。
2. 检查 `pygame`。
3. 检查中文字体。
4. 检查 OpenCV ArUco 和 NumPy。
5. 设置：

```bash
SDL_VIDEODRIVER=kmsdrm
PUZZLE_REAL=1
```

1. 启动：

```bash
python gui_fb.py
```

---

## 13. 线程安全要求

1. `RealBackend.start_task()` 必须在工作线程中运行。
2. `PuzzleRobot` 不直接操作 pygame 或 Tkinter 控件。
3. 后端回调只发送数据，不刷新 UI。
4. GUI 使用队列接收：

```text
("log", level, message)
("state", RobotState)
("progress", stage, fraction)
("metrics", RunMetrics)
("frame", frame)
("done", success, message)
```

1. 相机帧进入 GUI 队列前应复制。
2. 自动任务运行期间，GUI 不启动第二个后台任务。

---

## 14. 错误处理约定

常见错误消息：

- `机械臂未连接`
- `未发现本地舵机串口，真实后端不可用`
- `请先完成工作区和机械臂基座标定`
- `未知任务模式`
- `已有任务在运行`
- `未检测到可用碎片`
- `拼图求解失败`
- `时间不足，无法完成剩余搬运任务`
- `超过120秒限制`
- `任务被中止`
- `自动任务异常: ...`

GUI 应将 ERROR 显示在：

- 日志页
- 当前页面状态区
- 运行结果页
