#!/bin/bash
# 快速标定辅助工具启动脚本：抬臂到 HOME 并提供电磁铁手动控制。
set -euo pipefail

TOOL_SCRIPT=/opt/epuzzle/workspace/tools/quick_arm_magnet_control.py

# 加载系统环境（包括 PYTHONPATH 以访问系统 python3-gpiod）
set -a
# shellcheck disable=SC1091
source /etc/epuzzle/epuzzle.env
set +a

export PYTHONDONTWRITEBYTECODE=1

cd /opt/epuzzle/workspace

# 不使用 sudo：epuzzle 已在 gpio/dialout 组，/etc/epuzzle 可读。
# sudo 会无条件剥离 PYTHONPATH（即使带 -E），从而丢失系统 python3-libgpiod。
exec /opt/epuzzle/venv/bin/python "$TOOL_SCRIPT" "$@"
