# 树莓派上位机版本锁

冻结日期：2026-07-29  
镜像标识：`epuzzle-rpi5-2026-07-29-v2`

## 操作系统

- 硬件目标：Raspberry Pi 5 8GB
- 存储目标：64GB microSD；交付镜像为 8GiB，首启自动扩展
- OS：Raspberry Pi OS Lite 64-bit / Debian 13 (trixie)
- 官方基础镜像：`2026-06-18-raspios-trixie-arm64-lite.img.xz`
- 基础镜像 SHA256：`acff736ca7945e3b305f07cda4abdb870910e12634991da69783611756e381b3`
- Kernel：`6.18.34+rpt-rpi-2712`
- Architecture：`arm64` / `aarch64`
- Python：`3.13.5`

## Python 运行环境

- LeRobot：`0.6.0`
- LeRobot commit：`30da8e687a6dfc617fcd94afc367ac7071c376ce`
- 五轴插件：`lerobot_robot_so101_5d==0.2.0`
- PyTorch：`2.7.1`
- torchvision：`0.22.1`
- OpenCV headless：`4.13.0.90`
- NumPy：`2.2.6`
- Feetech dependency：`feetech-servo-sdk==1.0.0`
- 完整 wheel 清单及哈希：`deployment/raspberry_pi/cache/wheelhouse-cp313-arm64/SHA256SUMS`

## 运行默认

- ACT：关闭；没有最终 checkpoint，不可启用
- SSH：只允许构建时生成的 Ed25519 公钥；禁止 SSH 密码登录
- 本地 TTY：`epuzzle`，初始密码 `Epuzzle-2026!`，首次登录后必须修改
- Raspberry Pi `eth0`：`10.99.0.2/24`，无默认网关、无 DNS、禁用 DHCP/IPv6；`ipv4.dns-priority=0`，禁止负值
- 开发机 `enp0s31f6`：NetworkManager profile `epuzzle-direct`，`10.99.0.1/24`，`ipv4.never-default=yes`
- 入站防火墙：默认拒绝；允许 SSH、DHCP、ICMP、mDNS
- 电磁铁 GPIO：`UNKNOWN`；软件不请求 GPIO，依赖硬件栅极下拉保持默认关闭
- 主任务：不默认启用；硬件参数、udev、标定、Supervisor 未完成前由 preflight 拦截

## 尚未冻结

- ACT checkpoint SHA256：`UNKNOWN`
- Dataset version/SHA256：`UNKNOWN`
- Calibration bundle SHA256：`UNKNOWN`
- 相机型号、USB VID/PID/serial、分辨率、帧率：`UNKNOWN`
- 机械臂控制板 USB VID/PID/serial 和稳定设备路径：`UNKNOWN`
- 五个 STS3215 实物完整型号及供电：`UNKNOWN`
- 电磁铁 GPIO line、电压、电流、超时和温升：`UNKNOWN`

冻结版本不得直接在线升级。任何 OS、Python、LeRobot、torch、OpenCV 或五轴插件更新都必须生成新镜像 ID、重建 wheelhouse，并重新完成软件、断网恢复与实机回归测试。

> 实机状态说明（2026-07-29）：当前 Pi 曾由用户执行 `apt upgrade`，`/etc/debian_version` 已为 `13.6`，Python 仍报告 `3.13.5`，kernel 仍为 `6.18.34+rpt-rpi-2712`；LeRobot/PyTorch/OpenCV/五轴插件版本和导入回归全部通过。该状态是部署后实测，不改变上述基础镜像版本锁；下一次正式归档镜像应使用新镜像 ID。
