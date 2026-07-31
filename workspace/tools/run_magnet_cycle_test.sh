#!/bin/bash
# 运行 60 秒 GPIO17 高低循环测试，并在退出时恢复高电平和 GUI。
set -euo pipefail

PUZZLE_DIR=/home/epuzzle/puzzle
TEST_SCRIPT=/opt/epuzzle/workspace/tools/magnet_cycle_test.py
GUI_PID=""

restore_safe_state() {
	# Active Low 继电器：高电平为关闭状态。
	sudo pinctrl set 17 op dh pu || true
	cd "$PUZZLE_DIR"
	if ! pgrep -f '^venv/bin/python gui_fb.py$' >/dev/null; then
		nohup setsid bash tools/start_gui_fb.sh </dev/null >out/gui_fb.log 2>&1 &
	fi
}
trap restore_safe_state EXIT INT TERM

GUI_PID="$(pgrep -n -f '^venv/bin/python gui_fb.py$' || true)"
if [[ -n "$GUI_PID" ]]; then
	kill "$GUI_PID"
	for _ in 1 2 3 4 5; do
		kill -0 "$GUI_PID" 2>/dev/null || break
		sleep 1
	done
fi

set -a
# shellcheck disable=SC1091
source /etc/epuzzle/epuzzle.env
set +a
export PYTHONDONTWRITEBYTECODE=1
cd /opt/epuzzle/workspace
/opt/epuzzle/venv/bin/python "$TEST_SCRIPT" "$@"
