#!/bin/bash
# 正式 7 英寸抓取/旋转/放置页面。由 epuzzle-gui.service 绑定 tty2 启动。
set -euo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
PROJECT_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")"
PYTHON_BIN="${EPUZZLE_PYTHON:-/usr/bin/python3}"
CURRENT_TTY="$(tty 2>/dev/null || true)"

if [[ "${EP_GUI_ON_VT:-0}" != "1" && ! "$CURRENT_TTY" =~ ^/dev/tty[0-9]+$ ]]; then
	echo "GUI 必须由 epuzzle-gui.service（tty2）或本地虚拟终端启动；脚本不会提升权限。" >&2
	exit 2
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
	echo "GUI Python 不可执行: $PYTHON_BIN" >&2
	exit 2
fi

cd "$PROJECT_DIR"
export SDL_VIDEODRIVER=dummy
export SDL_AUDIODRIVER=dummy
export SDL_TOUCH_MOUSE_EVENTS=0
export EP_GUI_FRAMEBUFFER="${EP_GUI_FRAMEBUFFER:-/dev/fb0}"
# MPI700 的交互逻辑画布固定 1024×600；即使 fb0 临时协商为 1024×768，
# 页面与 evdev 坐标仍保持同一逻辑尺寸，由 RGB565 输出层显式缩放。
export EP_GUI_LOGICAL_SIZE="${EP_GUI_LOGICAL_SIZE:-1024x600}"
export EP_GUI_PYTHON="${EP_GUI_PYTHON:-/opt/epuzzle/venv/bin/python}"
export PYTHONPATH="$PROJECT_DIR/workspace/src/robot_plugin/lerobot_robot_so101_5d:$PROJECT_DIR/workspace/src:/opt/epuzzle/venv/lib/python3.13/site-packages:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

exec "$PYTHON_BIN" workspace/apps/pick_place_page.py "$@"
