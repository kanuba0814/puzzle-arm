# 2026-07-31 USB 恢复与磁铁关闭最短巡检

## 恢复前提

- 上一轮 `xhci-hcd.1` 死亡后，现场操作者重启 Raspberry Pi，并明确说明此前只插了 USB 控制板、未接舵机电源；随后接通舵机电源。
- 持久事故锁 `/etc/epuzzle/robot-motion.lock.json` 由现场操作者手动删除，视为现场明确批准解除该次事故锁；程序没有自动删除或绕过锁。
- 重启后 USB 身份恢复为 `1a86:55d3`、序列号 `5AB9067408`、`/dev/epuzzle-arm -> /dev/ttyACM0`。

## 只读恢复检查

第一次只读总线握手在舵机未供电时发现 ID 1–5 全部缺失，未发送写命令。舵机供电后：

- 五轴校准寄存器与冻结文件一致；
- 五轴最终自然收敛至正式 PARK 的 4° 容差内；
- 正式 `park` 生命周期返回 `PASS / NONE / PARKED / magnet_enabled=false`；
- 随后只读验收确认 Torque_Enable 五轴全 0。

## 本轮修复

1. `JointSpaceParkMover.home_to_park()` 原先只检查途中 8° 跟随误差，未在返回成功前重新检查正式 PARK 4° 容差。已添加终点实际读回门禁和回归测试。
2. 首次恢复后 `sweep-shortest` 结构化失败为 `GLOBAL_TIMEOUT: 目标在规划前已超过 deadline`，route 为空、磁铁关闭，异常清理后仍在 PARK 且断力矩。
3. 根因是时钟域混用：Supervisor 用 `time.monotonic_ns()` 生成 deadline，`MotionController` 默认用 `time.time_ns()` 比较。已统一为 `time.monotonic_ns()`，并添加生产默认构造路径回归测试。

## 成功巡检

修复后仅执行一次新的 `sweep-shortest`：

- 结果：`PASS / NONE`
- 路线：`red-1 -> red-2 -> red-0 -> red-3`
- 时长：`33.165 s`
- 四个中心均有 `sweep_center_actual_readback`，`success=true`、`error_code=NONE`
- 最终状态：`PARKED`
- `magnet_enabled=false`

最终只读验收：

- Torque_Enable：五轴全 0
- 正式 PARK 最大关节误差：`3.516483° <= 4°`
- 当前启动 USB/xHCI 故障计数：0
- 串口/GPIO holder：无
- `epuzzle-gui.service=active/enabled`
- `epuzzle-supervisor.service=inactive`
- `epuzzle-magnet-safe.service=active`
- 完整 `epuzzle-selfcheck --json`：`passed=true`；磁铁关闭巡检关键参数已冻结，37 个 deferred `UNKNOWN` 以 warning 保留并继续由对应功能门禁阻塞。
- 相机 udev identity 已冻结为 Sipeed MaixCAM `359f:2120` / `c9c28f4513595968` / capture index 0，`/dev/epuzzle-camera -> /dev/video0`；稳定路径检测再次得到 4 片、1280×720。

## 文件

- `pieces.json`、`annotated.jpg`、`mask.png`：成功巡检前同步刷新检测。
- `recovery_park_result.json`：恢复后的正式 PARK 生命周期结果。
- `deadline_failure_result.json`：时钟域修复前的 fail-closed 红信号。
- `sweep_result.json`、`sweep_status.json`、`sweep_output.json`：修复后成功结果。
- `sweep_events.jsonl`：包含四个中心实际五轴读回。
- `final_park_torque_state.json`：最终只读 PARK/力矩验收。
- `kernel_usb_recovery.log`：本次启动 USB 枚举日志。
- `final_runtime_state.txt`：巡检后服务、设备、holder 和部署文件哈希。
- `full_selfcheck.json`、`full_selfcheck.txt`：READY 与 deferred warning 的两种输出。
- `camera_udev_identity.txt`：稳定相机设备身份。
- `stable_camera_*`：使用 `/dev/epuzzle-camera` 的独立检测验证。
- `supervisor_detect_*`：完整 Supervisor `detect` 合同默认使用稳定相机路径，结构化 PASS、4 片、`magnet_enabled=false`。
- `final_runtime_state_after_camera.txt`：稳定相机部署后的最终服务、设备、holder、USB 与文件哈希。

所有数值均来自本轮 Pi 日志或寄存器读回，不把估计值写成实测值。
