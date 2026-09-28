#!/bin/bash
# USB 掉线自动恢复验证工具启动脚本（人工监督拔插）。
set -euo pipefail

TOOL_SCRIPT=/opt/epuzzle/workspace/tools/verify_usb_reconnect.py

# 加载系统环境（包括 PYTHONPATH 与 GPIO 权限组）
set -a
# shellcheck disable=SC1091
source /etc/epuzzle/epuzzle.env
set +a

export PYTHONDONTWRITEBYTECODE=1

cd /opt/epuzzle/workspace

exec /opt/epuzzle/venv/bin/python "$TOOL_SCRIPT" "$@"
