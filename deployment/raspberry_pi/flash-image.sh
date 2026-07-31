#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ARCHIVE="$SCRIPT_DIR/output/epuzzle-rpi5-2026-07-29-v2.img.zst"
DEVICE=""
CONFIRMATION=""

usage() {
	cat <<'EOF'
用法：
  ./flash-image.sh --device /dev/sdX --confirm ERASE-/dev/sdX

该命令会不可逆覆盖目标块设备。请先用 lsblk 核对设备，不接受分区路径。
EOF
}

while (($#)); do
	case "$1" in
	--device)
		DEVICE=${2:-}
		shift 2
		;;
	--confirm)
		CONFIRMATION=${2:-}
		shift 2
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		usage >&2
		exit 2
		;;
	esac
done

[[ -b "$DEVICE" ]] || {
	echo "不是块设备: $DEVICE" >&2
	exit 2
}
[[ "$(lsblk -dnro TYPE "$DEVICE")" == "disk" ]] || {
	echo "必须指定整盘，不能指定分区: $DEVICE" >&2
	exit 2
}
[[ "$CONFIRMATION" == "ERASE-$DEVICE" ]] || {
	echo "确认字符串不匹配；未执行写盘。" >&2
	exit 2
}
[[ -f "$ARCHIVE" && -f "$ARCHIVE.sha256" && -f "$ARCHIVE.raw.sha256" && -f "$ARCHIVE.raw.size" ]] || {
	echo "镜像或校验文件不完整: $ARCHIVE" >&2
	exit 2
}

while read -r mountpoint; do
	[[ -z "$mountpoint" ]] && continue
	echo "目标盘仍有挂载点 $mountpoint；请先卸载，未执行写盘。" >&2
	exit 2
done < <(lsblk -nro MOUNTPOINTS "$DEVICE")

ROOT_SOURCE=$(findmnt -nro SOURCE /)
ROOT_DISK="/dev/$(lsblk -no PKNAME "$ROOT_SOURCE" 2>/dev/null || true)"
[[ "$DEVICE" != "$ROOT_SOURCE" && "$DEVICE" != "$ROOT_DISK" ]] || {
	echo "拒绝覆盖当前系统盘: $DEVICE" >&2
	exit 2
}

(cd "$(dirname "$ARCHIVE")" && sha256sum -c "$(basename "$ARCHIVE").sha256")
RAW_SIZE=$(<"$ARCHIVE.raw.size")
DEVICE_SIZE=$(lsblk -bdnro SIZE "$DEVICE")
((DEVICE_SIZE >= RAW_SIZE)) || {
	echo "目标设备太小: device=$DEVICE_SIZE image=$RAW_SIZE" >&2
	exit 2
}

lsblk -o NAME,PATH,SIZE,MODEL,TRAN,TYPE,MOUNTPOINTS "$DEVICE"
echo "开始覆盖 $DEVICE；该动作不可撤销。"
zstd -dc "$ARCHIVE" | sudo dd of="$DEVICE" bs=16M iflag=fullblock oflag=direct conv=fsync status=progress
sync

echo "读取写入区域并校验 SHA256..."
EXPECTED_RAW_HASH=$(cut -d' ' -f1 "$ARCHIVE.raw.sha256")
ACTUAL_RAW_HASH=$(sudo dd if="$DEVICE" bs=1M count=$((RAW_SIZE / 1048576)) iflag=fullblock status=progress | sha256sum | cut -d' ' -f1)
[[ "$ACTUAL_RAW_HASH" == "$EXPECTED_RAW_HASH" ]] || {
	echo "写后校验失败: expected=$EXPECTED_RAW_HASH actual=$ACTUAL_RAW_HASH" >&2
	exit 1
}
echo "写盘及逐字节范围校验通过。"
