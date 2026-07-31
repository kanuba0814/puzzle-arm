#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /opt/astra_ws/devel/setup.bash
exec "$@"
