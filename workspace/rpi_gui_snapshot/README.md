# 树莓派 7 英寸 GUI 快照

- 来源主机：`epuzzle@192.168.1.110`
- 来源目录：`/home/epuzzle/puzzle`
- 获取时间：`2026-07-31T02:50:49+09:00`
- 显示方式：pygame/SDL2 `kmsdrm`，无桌面环境全屏直显

## 文件

- `gui_fb.py`：7 英寸触摸屏主界面（二级菜单、常驻急停）。
- `gui_backend.py`：GUI 后端接口和当前 Mock 实现。
- `gui_app.py`：Tk 桌面调试界面。
- `tools/start_gui_fb.sh`：树莓派 tty 启动脚本，指定 `/dev/dri/card1`。

## 当前菜单

1. **自动控制**：一键标定、任务 1（纯色）、任务 2（扑克牌）、开始检测搬运、中止任务、进度显示。
2. **手动控制**：J1–J5 关节步进、XYZ 末端步进、力矩使能/失能、电磁铁吸/放、Home。
3. **系统日志**：日志查看和清空。
4. **常驻区域**：连接、模式、力矩、电磁铁、标定、任务和关节角状态；全页面急停按钮。

> 安全提示：机械臂动作当前仍为 Mock；仅 GPIO17 电磁铁可由环境配置启用真实硬件。机械臂 RealBackend 尚未实现，不能作为真实运动控制基线。

## GPIO17 电磁铁接口

- GUI 的“电磁铁 吸/放”和常驻急停通过 `control.MagnetController` 控制 `/dev/gpiochip0` line 17。
- 继电器低电平触发，连接和关闭时强制 GPIO17 输出 3.3 V 高电平。
- 唯一控制接口：`magnet_controller.set_enabled(bool)`。
- 单次吸合超过 10 秒自动断电；急停、GUI 退出和 GPIO 异常均走安全关闭路径。
- 线圈电压、电流、温升与连续通电能力仍为 `UNKNOWN`，没有执行真实吸合测试。

## 触摸与 tty 修复

- 从 SSH/pts 启动时，启动脚本通过 `openvt` 将 kmsdrm GUI 绑定到 `/dev/tty2`，避免仅显示硬件鼠标层。
- 启动时检查 QDTECH MPI700 的 `ID_INPUT_TOUCHSCREEN=1` udev 属性，缺失时重新触发设备规则。
- GUI 将 SDL `FINGER*` 事件显式映射为 `1024×600` 屏幕坐标和鼠标事件，使光标与触摸同步。
- Pi 5 上 SDL 2.32 的 EGL/KMSDRM 主平面持续黑屏，仅硬件光标可见；修改 pygame 软件 Surface 的 alpha 无法改变 SDL 随后生成的 GBM buffer。
- 正式 7 英寸屏改用 `SDL_VIDEODRIVER=dummy` 离屏绘图，再转换为 RGB565 直接写入 `/dev/fb0`；触摸直接读取 QDTECH MPI700 evdev，界面绘制软件光标。
- 手动页 Tab 过滤将 `_tab=None` 视为公共控件，保证“← 返回”等按钮在关节/末端页签均显示。
- 回归测试：`test_gui_touch.py`（触摸映射、evdev 坐标、RGB565 转换、公共返回按钮及 KMS 回退共 7 项）。
