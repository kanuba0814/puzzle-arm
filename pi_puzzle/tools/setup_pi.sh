#!/usr/bin/env bash
# ============================================================
# 树莓派 5 离线环境安装脚本 (舵机部署, 阶段1)
# 在树莓派上、项目根目录执行:  bash tools/setup_pi.sh
# 全程离线: 所有依赖来自 sdks/offline/ (开发机上有网时备好)
# ============================================================
set -e
cd "$(dirname "$0")/.."   # 切到项目根目录

echo "=============================================="
echo "  拼图装置 - 树莓派舵机环境离线安装"
echo "=============================================="

# ---------- 0. 环境信息 ----------
echo ""
echo "[0] 环境信息:"
echo "    OS:     $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "    内核:   $(uname -m)"
echo "    Python: $(python3 --version)"
PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
CPTAG="cp$(python3 -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")"
if ! ls sdks/offline/wheels/numpy-*${CPTAG}*.whl >/dev/null 2>&1; then
    echo "    ⚠️  没有匹配 Python ${PYVER} (${CPTAG}) 的 numpy wheel!"
    echo "       见 sdks/offline/README.md 在开发机重新下载后再运行本脚本"
fi

# ---------- 1. 创建 venv (Bookworm 默认 PEP 668, 禁止 pip 直装系统环境) ----------
echo ""
echo "[1] 创建虚拟环境 venv/ (--system-site-packages: 继承系统 lgpio/opencv)"
if [ ! -d venv ]; then
    if ! python3 -m venv --system-site-packages venv 2>/dev/null; then
        echo "    ❌ venv 创建失败: 缺 python3-venv 包"
        echo "       ⛔ 人工步骤(需联网一次): sudo apt install python3-venv"
        exit 1
    fi
fi
VPY=venv/bin/python
VPIP=venv/bin/pip
echo "    venv 就绪: $($VPY --version)"

# ---------- 2. 离线安装 wheels ----------
echo ""
echo "[2] 离线安装 Python wheels"
WHEELS=sdks/offline/wheels
# 格式: "PyPI包名:import名:必需(y/n)"
PKGS="pyserial:serial:y numpy:numpy:y gpiozero:gpiozero:y opencv-python-headless:cv2:n"
for entry in $PKGS; do
    pkg="${entry%%:*}"; rest="${entry#*:}"
    imp="${rest%%:*}"; need="${rest##*:}"
    if $VPY -c "import $imp" 2>/dev/null; then
        echo "    ✓ $pkg 已存在, 跳过"
    elif $VPIP install --no-index --find-links "$WHEELS" "$pkg" --quiet 2>/dev/null \
         && $VPY -c "import $imp" 2>/dev/null; then
        echo "    ✓ $pkg 安装成功"
    elif [ "$need" = "y" ]; then
        echo "    ❌ $pkg 安装失败 (必需项, 中止)"
        echo "       检查 wheels 目录是否有匹配 Python 版本的包: ls $WHEELS"
        exit 1
    else
        echo "    ⚠️  $pkg 安装失败 (可选项, 跳过; 视觉联调前需补齐)"
    fi
done

# ---------- 3. 安装 scservo_sdk (纯Python, 直接拷入 venv site-packages) ----------
echo ""
echo "[3] 安装 scservo_sdk (Feetech 官方舵机SDK)"
if $VPY -c "import scservo_sdk" 2>/dev/null; then
    echo "    ✓ scservo_sdk 已存在, 跳过"
else
    SP=$($VPY -c "import site; print(site.getsitepackages()[0])")
    cp -r sdks/offline/scservo_sdk "$SP/"
    echo "    ✓ 已拷贝到 $SP/scservo_sdk"
fi

# ---------- 4. 系统级检查 ----------
echo ""
echo "[4] 系统级检查"

# lgpio: Pi 5 GPIO 后端, 只能来自系统 (apt 预装), 无法用 pip 离线装
if $VPY -c "import lgpio" 2>/dev/null; then
    echo "    ✓ lgpio 可用 (Pi 5 GPIO 后端)"
else
    echo "    ⚠️  lgpio 缺失! gpiozero 将无法驱动引脚"
    echo "       ⛔ 人工步骤(需联网一次): sudo apt install python3-lgpio"
fi

# dialout 组: 串口访问权限
if groups | grep -q dialout; then
    echo "    ✓ 用户 $USER 在 dialout 组"
else
    echo "    ⚠️  用户 $USER 不在 dialout 组, 串口将无权限访问"
    echo "       执行: sudo usermod -aG dialout $USER  (然后【重新登录】生效)"
    read -p "       现在自动执行? [y/N] " -r
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo usermod -aG dialout "$USER"
        echo "       已加入 dialout 组, ⚠️ 请重新登录后再继续!"
    fi
fi

# ---------- 5. 自检 ----------
echo ""
echo "[5] 安装自检 (venv python)"
$VPY - <<'EOF'
import sys
ok = True
for mod, hint in [("scservo_sdk", "舵机SDK"), ("serial", "pyserial"),
                  ("numpy", "numpy"), ("gpiozero", "gpiozero")]:
    try:
        __import__(mod)
        print(f"    ✓ {mod:14s} ({hint})")
    except ImportError as e:
        print(f"    ❌ {mod:14s} 导入失败: {e}")
        ok = False
try:
    import lgpio
    print(f"    ✓ {'lgpio':14s} (Pi 5 GPIO后端)")
except ImportError:
    print(f"    ⚠️  {'lgpio':14s} 缺失 (见上方人工步骤)")
try:
    import cv2
    print(f"    ✓ {'cv2':14s} (opencv {cv2.__version__}, 视觉联调用)")
except ImportError:
    print(f"    ⚠️  {'cv2':14s} 缺失 (舵机部署不需要, 视觉联调前补齐)")
print("")
print("自检" + ("通过 ✅" if ok else "有失败项 ❌"))
sys.exit(0 if ok else 1)
EOF

echo ""
echo "=============================================="
echo "  安装完成! 使用方法:"
echo "    source venv/bin/activate     # 激活环境 (每次新开终端都要)"
echo "    python3 tools/servo_tool.py scan"
echo "  下一步: 按 docs/DEPLOY_PI.md 阶段2 接线"
echo "=============================================="
