#!/bin/bash
# 树莓派 (OS Lite, 纯 tty 无桌面) 启动机械臂 GUI
# 用法: bash tools/start_gui_fb.sh [gui_fb.py参数，如 --window 800x480]
set -e
cd "$(dirname "$0")/.."

# 1) 解释器选择: 优先项目venv（包含OpenCV ArUco、pygame、pyserial和舵机SDK）。
#    只有venv不存在时才回退系统Python。
if [ -x venv/bin/python ]; then
	PY=venv/bin/python
else
	PY=/usr/bin/python3
fi
if ! $PY -c "import pygame" 2>/dev/null; then
	echo "[启动脚本] 当前解释器缺少pygame: $PY"
	echo "  请先运行 bash tools/setup_pi.sh"
	exit 1
fi
# 中文字体
if ! ls /usr/share/fonts/truetype/wqy/wqy-zenhei.ttc >/dev/null 2>&1 &&
	[ ! -f fonts/NotoSansCJK-Regular.ttc ]; then
	echo "[启动脚本] 安装 fonts-wqy-zenhei (需联网一次)"
	sudo apt-get install -y fonts-wqy-zenhei
fi

# 2) 本地闭环依赖: GUI标定直接读取UVC并访问本地舵机，不再走SSH/MJPEG。
if ! $PY -c "import cv2, numpy; assert hasattr(cv2, 'aruco')" 2>/dev/null; then
	echo "[启动脚本] $PY 缺少 OpenCV ArUco/Numpy"
	echo "  请先运行 bash tools/setup_pi.sh"
	exit 1
fi

if ! $PY -c "import gpiozero" 2>/dev/null; then
	echo "[启动脚本] 警告: 缺少gpiozero，完成后蜂鸣器/LED将只记录日志"
fi

# 3) tty 直显 + 真实本地硬件后端 (不依赖X/桌面/SSH)
export SDL_VIDEODRIVER=kmsdrm
export PUZZLE_REAL=1

exec $PY gui_fb.py "$@"
