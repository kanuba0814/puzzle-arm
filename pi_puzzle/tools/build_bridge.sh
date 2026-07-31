#!/usr/bin/env bash
# 编译 astra_bridge (Astra S 取帧桥接程序)
# 用法: bash tools/build_bridge.sh
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

# 自动发现 OpenNI_SDK 目录
SDK_DIR=$(ls -d sdks/OpenNI_SDK/OpenNI_*_linux 2>/dev/null | head -1)
if [ -z "$SDK_DIR" ]; then
    echo "[Error] 未找到 OpenNI_SDK, 请先解压到 sdks/OpenNI_SDK/"
    exit 1
fi

INCLUDE_DIR="$SDK_DIR/sdk/Include"
LIB_DIR="$PROJECT_ROOT/$SDK_DIR/sdk/libs"

if [ ! -f "$INCLUDE_DIR/OpenNI.h" ]; then
    echo "[Error] 找不到头文件: $INCLUDE_DIR/OpenNI.h"
    exit 1
fi
if [ ! -f "$LIB_DIR/libOpenNI2.so" ]; then
    echo "[Error] 找不到库: $LIB_DIR/libOpenNI2.so"
    exit 1
fi

echo "[Build] SDK: $SDK_DIR"
g++ -O2 -Wall -o tools/astra_bridge tools/astra_bridge.cpp \
    -I"$INCLUDE_DIR" \
    -L"$LIB_DIR" -lOpenNI2 \
    -Wl,-rpath,"$LIB_DIR"

echo "[Build] 完成: tools/astra_bridge"
