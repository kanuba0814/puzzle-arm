#!/bin/bash
# 树莓派 (OS Lite, 纯 tty 无桌面) 启动机械臂 GUI
# 用法: bash tools/start_gui_fb.sh
set -e
SCRIPT_PATH="$(readlink -f "$0")"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_PATH")")"
cd "$PROJECT_DIR"

# 从 SSH/pts 启动时切换到 tty2，固定 7 英寸屏所在的控制台，并避免其他
# console 输出覆盖直接写入 fb0 的界面。
CURRENT_TTY="$(tty 2>/dev/null || true)"
if [[ "${EP_GUI_ON_VT:-0}" != "1" && ! "$CURRENT_TTY" =~ ^/dev/tty[0-9]+$ ]]; then
	GUI_USER="$(stat -c '%U' "$PROJECT_DIR")"
	exec sudo openvt -s -f -w -c "${EP_GUI_VT:-2}" -- \
		setpriv --reuid="$GUI_USER" --regid="$GUI_USER" --init-groups \
		env HOME="/home/$GUI_USER" EP_GUI_ON_VT=1 \
		bash "$SCRIPT_PATH" "$@"
fi

# 触摸屏规则若在设备插入后才安装，现有 event 节点会保留旧属性。启动前只在
# 缺少 touchscreen 标记时重新触发对应 QDTECH MPI700 输入设备。
while IFS= read -r event_node; do
	if ! udevadm info --query=property --name="$event_node" |
		grep -qx 'ID_INPUT_TOUCHSCREEN=1'; then
		sudo udevadm trigger --action=change "$(udevadm info --query=path --name="$event_node")"
		sudo udevadm settle
	fi
done < <(grep -B 5 -A 5 'Name="QDTECH.*MPI700' /proc/bus/input/devices |
	grep -oE 'event[0-9]+' | sort -u | sed 's#^#/dev/input/#')

# 1) 解释器选择。pygame 仅负责离屏绘图，最终画面直接写入 RGB565 fb0。
PY=venv/bin/python
if ! $PY -c "import pygame" 2>/dev/null; then
	echo "[启动脚本] 安装 python3-pygame / libegl1 / libgles2 (需联网一次)"
	sudo apt-get install -y python3-pygame libegl1 libgles2
fi
# 中文字体
if ! ls /usr/share/fonts/truetype/wqy/wqy-zenhei.ttc >/dev/null 2>&1 &&
	[ ! -f fonts/NotoSansCJK-Regular.ttc ]; then
	echo "[启动脚本] 安装 fonts-wqy-zenhei (需联网一次)"
	sudo apt-get install -y fonts-wqy-zenhei
fi

# 3) 7 英寸屏直显: 避开 Pi 5 上 SDL/EGL KMSDRM 黑屏，使用 fb0 + evdev。
export SDL_VIDEODRIVER=dummy
export EP_GUI_FRAMEBUFFER=/dev/fb0
export SDL_TOUCH_MOUSE_EVENTS=0

# GPIO17 电磁铁配置由 root 管理，GUI 只通过 MagnetController 布尔接口访问。
if [[ -r /etc/epuzzle/epuzzle.env ]]; then
	set -a
	# shellcheck disable=SC1091
	source /etc/epuzzle/epuzzle.env
	set +a
fi
export PYTHONPATH="/opt/epuzzle/workspace/src${PYTHONPATH:+:$PYTHONPATH}"

exec "$PY" gui_fb.py
