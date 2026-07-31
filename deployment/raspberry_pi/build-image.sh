#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
BASE_DIR="$SCRIPT_DIR/cache/base"
BASE_XZ="$BASE_DIR/2026-06-18-raspios-trixie-arm64-lite.img.xz"
BASE_IMAGE="$BASE_DIR/2026-06-18-raspios-trixie-arm64-lite.img"
BASE_SHA256=acff736ca7945e3b305f07cda4abdb870910e12634991da69783611756e381b3
WHEELHOUSE="$SCRIPT_DIR/cache/wheelhouse-cp313-arm64"
VENV="$SCRIPT_DIR/cache/venv-cp313-arm64"
BUILD_DIR="$SCRIPT_DIR/build"
OUTPUT_DIR="$SCRIPT_DIR/output"
OUTPUT_IMAGE="$OUTPUT_DIR/epuzzle-rpi5-2026-07-29-v2.img"
OUTPUT_ARCHIVE="$OUTPUT_IMAGE.zst"
IMAGE_SIZE=${EPUZZLE_IMAGE_SIZE:-8G}
INITIAL_PASSWORD=${EPUZZLE_INITIAL_PASSWORD:-Epuzzle-2026!}

required_commands=(guestfish virt-resize xz zstd sha256sum ssh-keygen tar rsync truncate openssl)
for command_name in "${required_commands[@]}"; do
	command -v "$command_name" >/dev/null || {
		echo "缺少构建命令: $command_name" >&2
		exit 1
	}
done

[[ -f "$BASE_XZ" ]] || {
	echo "缺少官方基础镜像: $BASE_XZ" >&2
	exit 1
}
echo "$BASE_SHA256  $BASE_XZ" | sha256sum -c -
[[ -f "$WHEELHOUSE/SHA256SUMS" ]] || {
	echo "缺少离线 wheelhouse 或 SHA256SUMS" >&2
	exit 1
}
(cd "$WHEELHOUSE" && sha256sum -c SHA256SUMS)
shopt -s nullglob
pygame_wheels=("$WHEELHOUSE"/pygame_ce-2.5.5-*-aarch64.whl)
shopt -u nullglob
[[ ${#pygame_wheels[@]} -eq 1 ]] || {
	echo "缺少唯一的 pygame-ce 2.5.5 CPython arm64 wheel；请在 aarch64 环境重建 wheelhouse。" >&2
	exit 1
}
grep -Fq "$(basename "${pygame_wheels[0]}")" "$WHEELHOUSE/SHA256SUMS" || {
	echo "pygame-ce wheel 未由 SHA256SUMS 覆盖。" >&2
	exit 1
}
[[ -f "$VENV/pyvenv.cfg" ]] || {
	echo "缺少已构建的 arm64 Python 环境: $VENV" >&2
	exit 1
}

mkdir -p "$BASE_DIR" "$OUTPUT_DIR"
if [[ ! -f "$BASE_IMAGE" ]]; then
	xz -dk "$BASE_XZ"
fi

rm -rf "$BUILD_DIR"
mkdir -p \
	"$BUILD_DIR/root" \
	"$BUILD_DIR/boot" \
	"$BUILD_DIR/root/opt/epuzzle/offline/wheelhouse" \
	"$BUILD_DIR/root/opt/epuzzle/venv" \
	"$BUILD_DIR/root/home/pi/.ssh"

rsync -a --exclude='__pycache__/' "$SCRIPT_DIR/overlay/" "$BUILD_DIR/root/"
# 当前基础镜像的 cloud.cfg 引用了未随 cloud-init 安装的模块，首启会显示 degraded。
mkdir -p "$BUILD_DIR/root/etc/cloud"
guestfish --ro -a "$BASE_IMAGE" -m /dev/sda2 \
	download /etc/cloud/cloud.cfg "$BUILD_DIR/root/etc/cloud/cloud.cfg"
sed -i '/^[[:space:]]*- netplan_nm_patch[[:space:]]*$/d' \
	"$BUILD_DIR/root/etc/cloud/cloud.cfg"
if grep -q 'netplan_nm_patch' "$BUILD_DIR/root/etc/cloud/cloud.cfg"; then
	echo "未能移除无效 cloud-init 模块 netplan_nm_patch" >&2
	exit 1
fi
rsync -a "$VENV/" "$BUILD_DIR/root/opt/epuzzle/venv/"
rsync -a "$WHEELHOUSE/" "$BUILD_DIR/root/opt/epuzzle/offline/wheelhouse/"
install -m 0644 "$SCRIPT_DIR/requirements-arm64.in" \
	"$BUILD_DIR/root/opt/epuzzle/offline/requirements-arm64.in"

project_entries=(
	AGENTS.md
	README.md
	START_HERE.md
	configs
	data_contracts
	docs
	official_docs
	templates
	workspace
)
for project_entry in "${project_entries[@]}"; do
	rsync -a \
		--exclude='__pycache__/' \
		--exclude='build/' \
		--exclude='*.egg-info/' \
		"$REPO_ROOT/$project_entry" \
		"$BUILD_DIR/root/opt/epuzzle/"
done

SSH_KEY="$OUTPUT_DIR/epuzzle-image-access-ed25519"
if [[ ! -f "$SSH_KEY" ]]; then
	ssh-keygen -q -t ed25519 -N '' -C 'epuzzle-image-2026-07-29' -f "$SSH_KEY"
fi
install -m 0600 "$SSH_KEY.pub" "$BUILD_DIR/root/home/pi/.ssh/authorized_keys"

PASSWORD_HASH=$(printf '%s' "$INITIAL_PASSWORD" | openssl passwd -6 -stdin)
printf 'epuzzle:%s\n' "$PASSWORD_HASH" >"$BUILD_DIR/boot/userconf.txt"
: >"$BUILD_DIR/boot/ssh"
cat >"$BUILD_DIR/boot/user-data" <<'EOF'
#cloud-config
hostname: epuzzle-pi
preserve_hostname: false
timezone: Asia/Shanghai
locale: C.UTF-8
ssh_pwauth: false
package_update: false
package_upgrade: false
EOF
cat >"$BUILD_DIR/boot/meta-data" <<'EOF'
dsmode: local
instance_id: epuzzle-rpi5-2026-07-29-v2
local-hostname: epuzzle-pi
EOF
cat >"$BUILD_DIR/boot/network-config" <<'EOF'
network:
  version: 2
  ethernets:
    wired:
      match:
        name: "e*"
      dhcp4: false
      dhcp6: false
      addresses:
        - 10.99.0.2/24
      link-local: []
      optional: true
EOF

guestfish --ro -a "$BASE_IMAGE" -m /dev/sda1 download /config.txt "$BUILD_DIR/boot/config.txt"
guestfish --ro -a "$BASE_IMAGE" -m /dev/sda1 download /cmdline.txt "$BUILD_DIR/boot/cmdline.txt"
python3 - "$BUILD_DIR/boot/config.txt" "$BUILD_DIR/boot/cmdline.txt" <<'PY'
from pathlib import Path
import sys

config_path = Path(sys.argv[1])
cmdline_path = Path(sys.argv[2])
config = config_path.read_text(encoding="utf-8")
config = config.replace("dtparam=audio=on", "dtparam=audio=off")
config += (
    "\n# E puzzle headless upper-computer profile\n"
    "display_auto_detect=0\n"
    "# Active-low relay fail-safe: BCM GPIO17 / physical pin 11 defaults high.\n"
    "gpio=17=op,dh,pu\n"
)
config_path.write_text(config, encoding="utf-8")

arguments = cmdline_path.read_text(encoding="utf-8").strip().split()
arguments = [argument for argument in arguments if not argument.startswith("console=serial")]
for required_argument in ("usbcore.autosuspend=-1", "cfg80211.ieee80211_regdom=CN"):
    if required_argument not in arguments:
        arguments.append(required_argument)
cmdline_path.write_text(" ".join(arguments) + "\n", encoding="utf-8")
PY

mkdir -p \
	"$BUILD_DIR/root/etc/systemd/system/multi-user.target.wants" \
	"$BUILD_DIR/root/etc/systemd/system/getty.target.wants" \
	"$BUILD_DIR/root/var/log/journal" \
	"$BUILD_DIR/root/opt/epuzzle/workspace/logs" \
	"$BUILD_DIR/root/opt/epuzzle/workspace/calibration"
ln -sfn ../epuzzle-magnet-safe.service \
	"$BUILD_DIR/root/etc/systemd/system/multi-user.target.wants/epuzzle-magnet-safe.service"
ln -sfn /usr/lib/systemd/system/nftables.service \
	"$BUILD_DIR/root/etc/systemd/system/multi-user.target.wants/nftables.service"
ln -sfn /usr/lib/systemd/system/getty@.service \
	"$BUILD_DIR/root/etc/systemd/system/getty.target.wants/getty@tty1.service"
ln -sfn /usr/lib/systemd/system/multi-user.target \
	"$BUILD_DIR/root/etc/systemd/system/default.target"
chmod 0700 "$BUILD_DIR/root/home/pi/.ssh"
chmod 0600 "$BUILD_DIR/root/home/pi/.ssh/authorized_keys"
chmod 0755 \
	"$BUILD_DIR/root/usr/local/lib/epuzzle/magnet-off.py" \
	"$BUILD_DIR/root/usr/local/sbin/epuzzle-selfcheck" \
	"$BUILD_DIR/root/usr/local/sbin/epuzzle-offline-reinstall"
chmod 0640 "$BUILD_DIR/root/etc/epuzzle/epuzzle.env"
chmod 0600 "$BUILD_DIR/root/etc/NetworkManager/system-connections/epuzzle-direct.nmconnection"
if grep -Eq '^dns-priority=-' \
	"$BUILD_DIR/root/etc/NetworkManager/system-connections/epuzzle-direct.nmconnection"; then
	echo "有线维护连接不得使用负 DNS 优先级，否则会排除 Wi-Fi DNS" >&2
	exit 1
fi

cat >"$BUILD_DIR/root/etc/epuzzle/image-release" <<EOF
IMAGE_ID=epuzzle-rpi5
IMAGE_VERSION=2026-07-29-v2
BASE_IMAGE=2026-06-18-raspios-trixie-arm64-lite.img.xz
BASE_SHA256=$BASE_SHA256
ARCHITECTURE=arm64
PYTHON=3.13.5
LEROBOT=0.6.0
LEROBOT_COMMIT=30da8e687a6dfc617fcd94afc367ac7071c376ce
TORCH=2.7.1
TORCHVISION=0.22.1
OPENCV=4.13.0.90
ACT_DEFAULT=disabled
EOF

ROOT_TAR="$BUILD_DIR/root.tar"
BOOT_TAR="$BUILD_DIR/boot.tar"
tar --numeric-owner --owner=0 --group=0 -cpf "$ROOT_TAR" -C "$BUILD_DIR/root" .
tar --numeric-owner --owner=0 --group=0 -cpf "$BOOT_TAR" -C "$BUILD_DIR/boot" .

rm -f "$OUTPUT_IMAGE" "$OUTPUT_ARCHIVE" "$OUTPUT_ARCHIVE.sha256" "$OUTPUT_IMAGE.bmap"
truncate -s "$IMAGE_SIZE" "$OUTPUT_IMAGE"
virt-resize --expand /dev/sda2 "$BASE_IMAGE" "$OUTPUT_IMAGE"

guestfish --rw -a "$OUTPUT_IMAGE" <<EOF
run
mount /dev/sda2 /
mkdir-p /boot/firmware
mount /dev/sda1 /boot/firmware
tar-in $ROOT_TAR /
tar-in $BOOT_TAR /boot/firmware
chown 1000 1000 /home/pi/.ssh
chown 1000 1000 /home/pi/.ssh/authorized_keys
chown 1000 1000 /opt/epuzzle/workspace/logs
chown 1000 1000 /opt/epuzzle/workspace/calibration
chown 0 1000 /etc/epuzzle/epuzzle.env
chmod 0700 /home/pi/.ssh
chmod 0600 /home/pi/.ssh/authorized_keys
chmod 0640 /etc/epuzzle/epuzzle.env
umount-all
EOF

stat -c '%s' "$OUTPUT_IMAGE" >"$OUTPUT_ARCHIVE.raw.size"
sha256sum "$OUTPUT_IMAGE" >"$OUTPUT_ARCHIVE.raw.sha256"
zstd -T2 -12 --long=27 --force "$OUTPUT_IMAGE" -o "$OUTPUT_ARCHIVE"
sha256sum "$OUTPUT_ARCHIVE" >"$OUTPUT_ARCHIVE.sha256"
sha256sum "$SSH_KEY.pub" >"$SSH_KEY.pub.sha256"
if [[ ${EPUZZLE_KEEP_RAW:-0} != 1 ]]; then
	rm -f "$OUTPUT_IMAGE"
fi

cat <<EOF
镜像构建完成：
  $OUTPUT_ARCHIVE
  $OUTPUT_ARCHIVE.sha256
SSH 私钥（仅保留在构建机，不在镜像内）：
  $SSH_KEY
默认登录：ssh -i $SSH_KEY epuzzle@epuzzle-pi.local
EOF
