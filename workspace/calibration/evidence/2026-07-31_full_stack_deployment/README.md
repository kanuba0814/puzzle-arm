# 2026-07-31 全栈部署与磁铁关闭巡检证据

## 已通过

- 部署前 Pi 备份：`/home/epuzzle/backups/full-stack-reviewed-pre-20260731T191057+0800.tar.gz`，SHA256 `4c8f73946545e7a87864ab39f4fcfe0ea9f2d9da6dbbe6a0d5011e66feff810a`。
- 部署包 SHA256：`41e0e7a02181f1f380fca2c0d0428b5663436b849095d19cefb998cb98321959`；Pi 端归档和逐文件哈希均通过。
- Pi `epuzzle-selfcheck --software-only --json`：`passed=true`，stdout 可严格解析为 JSON。
- Pi 完整 `epuzzle-selfcheck --json`：按预期 `passed=false`，明确包含 `robot_motion_lock: LOCKED / xHCI`；软件依赖通过不再能被误判为整机 READY。
- 五轴插件实际从 `/opt/epuzzle/workspace/src/robot_plugin/` 加载；`connect_bumpless` 合同 6/6 通过。部署时发现 venv 内同为 0.2.0 的旧安装缺少该方法，已通过源码优先 `PYTHONPATH` 和 selfcheck API 断言修复。
- GUI 三页 1024×600 离屏截图通过，正式服务 active，唯一占用 `/dev/fb0`，不常驻串口或 GPIO。
- 新鲜红片检测：4 片；检测结果 `PASS`，`magnet_enabled=false`。
- 继电器控制链路由操作者确认为 `/dev/gpiochip0`、BCM GPIO17、低电平触发；本次部署和巡检未吸合电磁铁。

## 巡检失败（不得写成通过）

执行一次 `sweep-shortest` 后，结果为：

- `result=FAIL`
- `error_code=SAFETY_REJECTED`（事后代码已修正为对明确 ENODEV 映射 `ROBOT_COMM_TIMEOUT`）
- 原始错误：`write failed: [Errno 19] No such device`
- `magnet_enabled=false`
- route 为空，没有任何碎片中心被报告为完成。

`kernel_usb_failure.log` 证明故障根因是 Raspberry Pi 的 `xhci-hcd.1` 整个 USB host controller 在 `19:18:46` 死亡：

- `xHCI host controller not responding, assume dead`
- `HC died; cleaning up`
- `cdc_acm ... usb_submit_urb(write bulk) failed: -19`
- 机械臂 `/dev/ttyACM0` 和 `/dev/epuzzle-arm` 随后消失。

`vcgencmd get_throttled` 为 `0x0`，因此没有 Pi 欠压/降频位证据；这不能排除 USB 控制器、电缆、机械臂控制板或外部供电瞬态。

## 持久事故锁

已由用户确认安装 root 管理的 `/etc/epuzzle/robot-motion.lock.json`（root:root `0644`）：

- 文件存在即锁定，不因 USB 节点重新出现、GUI 重启或 Pi 重启自动解除；
- GUI 禁用巡检、PREGRASP、PARK 和普通断力矩，保留检测、Shadow 和急停；
- CLI 在构造机械臂 adapter 前返回 `SUPERVISOR_LOCKED`；
- Pi GUI/CLI 锁门禁测试 36/36 通过；后续 GUI 单模块 24/24 及锁/CLI定向复验通过；
- 实测 `park` 结果为 `FAIL / SUPERVISOR_LOCKED / magnet_enabled=false`。

解除必须完成锁文件中的全部 `clear_requires`，并获得现场操作者明确批准；程序不提供自动解锁旁路。

## 安全状态与恢复前置条件

失败后确认：

- Supervisor/巡检进程已退出；
- 串口和 GPIO 无进程占用；
- `epuzzle-magnet-safe.service=active`；
- GUI 恢复为诊断模式；
- 机械臂设备节点仍缺失。

由于 USB 掉线可能发生在 bumpless 连接已经启用力矩之后，软件无法确认最终 PARK 或断力矩。恢复 USB、重启 Pi 或重试运动前，必须由现场操作者：

1. 人工支撑机械臂；
2. 切断机械臂舵机电源；
3. 确认机械结构处于安全支撑状态；
4. 再恢复 USB/重启；
5. 先做只读枚举和正式 PARK 状态检查，不直接重试巡检。

本目录 `SHA256SUMS` 覆盖首次归档文件；添加本 README 后需重新生成最终清单。
