# Raspberry Pi 5 上位机镜像

目标：Raspberry Pi 5 8GB、64GB microSD、SO-101/SR-ARM101 去夹爪五轴、电磁吸头、单路顶置相机。

## 已固化基线

- Raspberry Pi OS Lite 64-bit，Debian 13 Trixie；
- 官方镜像：`2026-06-18-raspios-trixie-arm64-lite.img.xz`；
- Linux：`6.18.34+rpt-rpi-2712`；
- Python：`3.13.5`；
- LeRobot：`0.6.0`，源码 commit `30da8e687a6dfc617fcd94afc367ac7071c376ce`；
- PyTorch：`2.7.1`，torchvision：`0.22.1`；
- NumPy：`2.2.6`，OpenCV headless：`4.13.0.90`；
- 五轴插件：`lerobot_robot_so101_5d==0.2.0`；
- ACT：默认关闭；
- 以太网固定 `10.99.0.2/24`，无网关和 DNS；Wi-Fi 不预置凭据；
- SSH：仅 Ed25519 公钥，禁止 SSH 口令和 root 登录；
- 本地 TTY：用户 `epuzzle`，初始密码 `Epuzzle-2026!`，首次登录后必须修改；
- nftables：默认拒绝入站，仅开放 SSH、DHCP、ICMP 和 mDNS；
- 无桌面、VNC、NoMachine、Docker、ROS 等非必需组件。

镜像解压后约 8GiB，写入 64GB 卡后由 Raspberry Pi OS 首启机制自动扩展根分区。

## 交付文件

构建后位于 `deployment/raspberry_pi/output/`：

- `epuzzle-rpi5-2026-07-29-v2.img.zst`：可写盘镜像；
- `*.sha256`：压缩镜像 SHA256；
- `*.raw.sha256`、`*.raw.size`：写后校验依据；
- `epuzzle-image-access-ed25519`：SSH 私钥，权限必须保持 `0600`；
- `epuzzle-image-access-ed25519.pub`：镜像内授权公钥。

私钥不得放入公开仓库或发送给无关人员。需要更换密钥时，删除输出目录中的密钥并重新构建镜像。

## 构建

构建机需要 `libguestfs`、`guestfish`、`virt-resize`、Docker、QEMU user-static、xz、zstd。当前工作区已经缓存并校验基础镜像、arm64 wheelhouse 和 Python 环境。

```bash
chmod +x deployment/raspberry_pi/build-image.sh
deployment/raspberry_pi/build-image.sh
```

构建脚本会：

1. 校验官方基础镜像 SHA256；
2. 校验所有离线 wheel；
3. 扩展镜像到 8GiB；
4. 写入项目、五轴插件、离线依赖、systemd、SSH、nftables 与安全配置；
5. 生成压缩镜像及 raw/压缩两层哈希。

## 写盘

写盘是破坏性操作，脚本要求显式设备名和确认字符串，并拒绝系统盘、分区路径、已挂载设备和容量不足设备。

```bash
lsblk -o NAME,PATH,SIZE,MODEL,TRAN,TYPE,MOUNTPOINTS
chmod +x deployment/raspberry_pi/flash-image.sh
deployment/raspberry_pi/flash-image.sh \
    --device /dev/sdX \
    --confirm ERASE-/dev/sdX
```

脚本写完后会重新读取镜像覆盖范围并比对 raw SHA256。不要把 `/dev/sdX` 直接照抄成未经核对的设备。

## 首次启动与登录

1. 使用官方 5.1V/5A 27W 供电和主动散热；
2. 首次启动先只接网线、显示器/键盘（可选），不要给舵机和电磁铁上执行器电源；
3. 等待根分区扩容、SSH host key 生成和用户重命名完成；
4. 将开发机有线口固定为 `10.99.0.1/24`，不设置该接口的网关或 DNS；
5. 从开发机登录：

```bash
ssh -i deployment/raspberry_pi/output/epuzzle-image-access-ed25519 \
    epuzzle@10.99.0.2
```

树莓派 `eth0` 固定为 `10.99.0.2/24`，只用于与开发机直连；该接口没有默认网关和 DNS，不会抢占 Wi-Fi 的互联网默认路由。该 profile 的 DNS priority 必须保持默认 `0`，不能使用负值；NetworkManager 的负 DNS priority 会排除 Wi-Fi DHCP 下发的 DNS。

在使用 NetworkManager 的 Linux 开发机上，可创建专用有线连接：

```bash
sudo nmcli connection add type ethernet ifname '<开发机网卡名>' \
    con-name epuzzle-direct ipv4.method manual ipv4.addresses 10.99.0.1/24 \
    ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up epuzzle-direct
```

### 配置 Wi-Fi

镜像不保存赛场或个人 Wi-Fi 密码。触摸屏 TTY 登录凭据为 `epuzzle` / `Epuzzle-2026!`；登录后先执行 `passwd` 修改初始密码，再执行：

文本图形界面优先使用：

```bash
sudo nmtui
```

依次进入 **Activate a connection**、选择 SSID、输入密码并退出。也可以使用命令行：

```bash
sudo nmcli radio wifi on
sudo nmcli device wifi list
sudo nmcli device wifi connect 'SSID' password 'PASSWORD'
```

随后核对 `/etc/NetworkManager/system-connections/` 权限为 `0600`，并按网关、公网 IP、DNS、HTTPS 四层验证：

```bash
ip route
ping -c 2 1.1.1.1
getent hosts deb.debian.org
curl -4 -fsSIL https://deb.debian.org/
```

## 安全默认与启用顺序

`epuzzle-magnet-safe.service` 开机将 `/dev/gpiochip0` 的 BCM GPIO17 置为高电平，使低电平触发继电器保持释放。外部继电器输入仍必须硬件上拉到 3.3 V，软件不能替代上电最早期的硬件默认关闭。线圈电压、电流和温升仍为 `UNKNOWN`。

继电器控制链路已由操作者确认使用 `/dev/gpiochip0`、BCM GPIO17、低电平触发，部署环境允许 Supervisor 使用该控制器；这不代表线圈电气/热性能、吸取释放或耐久测试已通过。`z_pick_mm` 与 `z_place_mm` 仍为 `UNKNOWN`，所以自动 PICK/PLACE 保持 fail-closed。

常驻 Supervisor 服务**不会默认启用**；当前生产入口是 GUI 按动作启动的短命 `workspace/tools/supervisor_run.py`。GUI 可在全局 preflight 未通过时启动并显示诊断，但每个硬件动作仍执行自己的严格门禁。继续完成：

1. 逐个核对五个 STS3215 型号、ID、电压与供电；
2. 机械臂串口与 Sipeed MaixCAM 相机 udev 规则已按实机 VID/PID/serial 冻结；更换硬件时必须重新验收，禁止复用旧 identity；
3. 核对 `/etc/epuzzle/epuzzle.env` 中 GPIO17、低电平触发配置；吸合后由调用方显式断电；
4. 完成四个标定文件；
5. 磁铁关闭巡检所需参数必须全部冻结；PICK/PLACE、磁吸带载和 generic/texture 的保留 `UNKNOWN` 继续作为对应功能的 fail-closed 门禁；
6. 完成 PICK/PLACE 接触高度、吸取释放和带载验收；
7. 执行硬件自检，再显式解锁相应动作。

```bash
sudo /usr/local/sbin/epuzzle-selfcheck --software-only
sudo /usr/local/sbin/epuzzle-selfcheck
sudo systemctl enable --now epuzzle.target
```

硬件自检失败时 `epuzzle-preflight.service` 会阻止未来的常驻 Supervisor；当前短命动作由自身 preflight 和 Supervisor 门禁独立拒绝。GUI 仍可启动用于诊断。

若真实硬件事件导致最终 PARK 或断力矩无法确认，安装 root 管理的
`/etc/epuzzle/robot-motion.lock.json`。文件存在时 GUI/CLI 双层禁止巡检、PREGRASP、
PARK 和普通断力矩，即使 USB 节点重新出现也不自动解锁；检测、Shadow 和急停保留。
解除前必须完成锁文件 `clear_requires` 全部步骤并由现场操作者明确批准。

## 离线恢复 Python 环境

镜像内含完整 arm64/CPython 3.13 wheelhouse：

```bash
sudo /usr/local/sbin/epuzzle-offline-reinstall
```

该脚本先校验每个 wheel 的 SHA256，再断网重建 `/opt/epuzzle/venv` 并运行软件自检。

## 日常核对

```bash
cat /etc/epuzzle/image-release
systemctl status epuzzle-magnet-safe.service
sudo /usr/local/sbin/epuzzle-selfcheck --json
journalctl -u epuzzle-preflight.service -u epuzzle-supervisor.service
vcgencmd get_throttled
cat /sys/class/thermal/thermal_zone0/temp
```

`get_throttled=0x0` 才表示当前/历史欠压与降频位均未置位。正式测试仍须记录供电、温度、USB 枚举、相机参数和舵机通信数据。
