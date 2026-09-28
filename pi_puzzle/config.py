"""
SO-ARM101 拼图装置 - 全局配置
所有参数集中管理，方便调试和修改
"""

import glob as _glob
import json
from pathlib import Path

import numpy as np

# ===================== 机械臂参数 =====================
# SO-ARM101 连杆长度 (mm)
# 2026-07-31 采用 TheRobotStudio 官方 URDF (so101_new_calib.urdf) 关节轴距解算:
#   L1 = base_link原点(桌面安装平面) → 肩J2轴高 = 116.6
#   L2 = J2→J3轴距 = 116.0  (肘轴侧偏18.3mm, 平面模型忽略误差<1mm)
#   L3 = J3→J4轴距 = 135.0
#   L4 = J4→磁铁面 = J4→J5轴61.1(URDF腕结构) + 电磁铁总长37(实测) = 98.1
# 旧标称值 67/100/100/80 全部作废; 平面臂展 L2+L3+L4 ≈ 349mm
L1 = 116.6  # 桌面 -> 肩关节轴心高度（物理测量值）
L2 = 116.0  # 大臂 (肩轴 -> 肘轴)
L3 = 135.0  # 小臂 (肘轴 -> 腕轴)
L4 = 98.1  # 腕轴 -> 磁铁面

# A4 上 A-H 八个手动触点拟合出的简化平面模型修正。
# L1_EFFECTIVE 吸收安装倾斜/工具中心高度；角度修正只用于 FK/IK 模型，
# 不改舵机 raw<->软件角度映射，也不会影响 raw 停车位。
L1_EFFECTIVE = 104.1573
KINEMATIC_JOINT_OFFSETS_DEG = (0.0, 0.9248, 3.5343, 28.6032)
# E 点三次实机探测得到：无补偿时向基座偏约5mm，+5mm时越过约3mm；
# 线性插值得到命令目标应沿远离基座的径向偏置约3.125mm。
ARM_TARGET_RADIAL_COMP_MM = 3.125

# 舵机参数 (Feetech STS3215)
SERVO_BAUDRATE = 1000000  # 串口波特率
SERVO_PORT = "auto"  # 'auto'=自动探测 /dev/ttyUSB*,ttyACM*; 多设备时改为具体路径

# 舵机ID (5舵机+电磁铁方案: 夹爪舵机ID 6已物理拆除, 电磁铁走GPIO)
ID_BASE = 1  # 底座旋转 J1
ID_SHOULDER = 2  # 肩关节 J2
ID_ELBOW = 3  # 肘关节 J3
ID_WRIST_PITCH = 4  # 腕俯仰 J4
ID_WRIST_ROLL = 5  # 腕旋转 J5
SERVO_IDS = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST_PITCH, ID_WRIST_ROLL]

# 舵机角度范围 (单位: 度)
# 2026-07-30 实测标定 (LeRobot流程, 数据见 calibration_result.json):
#   方法: 零位=整条手臂水平向前伸直(FK零度); 各关节手掰到机械极限记录范围
#   数值 = 实测极限向内侧留 ~3° 安全余量
# 注意: J2 上抬端 +82 为实测极限(非全程); J3 几乎不能反弓(伸直=上限)
SERVO_LIMITS = {
    ID_BASE: (-103, 103),  # raw [922,3347] 对称
    ID_SHOULDER: (-21, 82),  # raw [3237↓,1995↑], SIGN=-1
    ID_ELBOW: (-177, 2),  # raw [3302折叠,1201伸直], 肘朝下分支
    ID_WRIST_PITCH: (-98, 100),  # raw [3086,745] 对称, -90°竖直朝下载范围内
    ID_WRIST_ROLL: (-254, 88),  # raw [4011,109], 近全周
}

# 舵机值范围 (STS3215: 0-4095 对应 0-360度)
SERVO_MIN_VALUE = 0
SERVO_MAX_VALUE = 4095
SERVO_RANGE_DEG = 360.0

# 装配标定 (2026-07-30 实测, 见 calibration_result.json)
# SERVO_SIGNS: raw增大方向与软件正方向关系 (-1=反向; 本臂5个关节全为-1)
#   J2 曾误记+1, 经"上抬→raw减小(2962→1995)"实测修正为-1
# SERVO_OFFSETS: 零偏 = 一字形零位(全臂水平前伸)实测raw - 2048
#   零位raw: {1:2131, 2:2962, 3:1243, 4:1930, 5:1116}
SERVO_SIGNS = {1: -1, 2: -1, 3: -1, 4: -1, 5: -1}
SERVO_OFFSETS = {1: 83, 2: 914, 3: -805, 4: -118, 5: -932}

# 运动速度
SERVO_SPEED = 2000  # 舵机运动速度 (0-4095, 越大越快)
DEFAULT_MOVE_TIME = 800  # 默认运动时间 (ms)

# 纸外驻车位。优先使用用户手动摆放并原位使能验证过的舵机原始位置；
# 不把它换算成关节角插值，因为该折叠姿态的 J2 跨越了软件角度零点。
PARK_SERVO_POSITIONS = {1: 1039, 2: 673, 3: 3270, 4: 2467, 5: 1115}
# 旧 P3 关节位保留作诊断/回退，不再作为 go_home() 默认目标。
PARK_JOINT_ANGLES = [-88.0, 61.0, -86.0, -66.0, 0.0]
PARK_SERVO_SPEED = 180

# ===================== 电磁铁 (替代夹爪) =====================
# 树莓派 GPIO -> MOSFET -> 电磁铁 (高电平=通电吸附)
MAGNET_PIN = 24  # BCM编号 (避开 LED=18, BUZZER=23)
MAGNET_RELEASE_MS = 300  # 断电后释放等待 (配合消磁电路, 残磁粘连则调大)

# ===================== 摄像头参数 =====================
# 相机后端类型: 'usb' / 'astra' / 'rtsp' / 'mjpeg'
# 当前主用: MaixCam UVC gadget 直连树莓派 (/dev/video0, MJPG 1280x720@30)
# (2026-08-01 确认: 已从 MJPEG 快照切换为 UVC 直连)
CAMERA_TYPE = "usb"

# --- MaixCam MJPEG 快照参数 (CAMERA_TYPE='mjpeg' 时使用) ---
# /shot.jpg=1280x720(ISP降采样,噪声低), /shot1440.jpg=2560x1440(全分辨率)
MJPEG_SHOT_URL = "http://10.161.167.1:8080/shot.jpg"

# --- MaixCam Pro RTSP 参数 (CAMERA_TYPE='rtsp' 时使用) ---
# MaixCam 上运行 /maixapp/apps/rtsp_stream 应用, 640x480
RTSP_URL = "rtsp://10.161.167.1:8554/live"
RTSP_TRANSPORT = "tcp"  # 'tcp' 稳定; 'udp' 延迟低但可能丢包
RTSP_OPEN_TIMEOUT_MS = 5000
RTSP_READ_TIMEOUT_MS = 5000

# --- USB 摄像头参数 (CAMERA_TYPE='usb' 时使用) ---
CAMERA_INDEX = 0  # /dev/video0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# --- MaixCam UVC gadget 参数 (内置通道, 标定/生产推荐) ---
# MaixCam USB线直连时同时呈现: RNDIS网卡(10.161.167.x) + UVC设备(/dev/video0)
# UVC能力: MJPG 1280x720@30 / YUYV 640x480@30 (无1440p, 高分仅MJPEG快照通道)
# 优势: OpenCV内置VideoCapture直读, 不依赖设备端mjpeg_server应用
MAIXCAM_UVC_INDEX = 0
MAIXCAM_UVC_SIZE = (1280, 720)  # UVC最高分辨率

# --- Orbbec Astra S 参数 (CAMERA_TYPE='astra' 时使用) ---
# Astra S 通过 C++ 桥接程序 tools/astra_bridge 取流 (OpenNI2),
# RGB/Depth 分辨率由桥接程序以相机默认值为准 (通常 640x480@30)。
# 注意: PyPI 的 openni Python 包在 Python 3.11+ 下有内存崩溃 bug, 不可用。

# OpenNI2 SDK 路径 (编译桥接程序时使用, 见 tools/build_bridge.sh)
# 不同平台请下载对应二进制: x64 / arm32 / arm64
_OPENNI_SDK_DIRS = _glob.glob("sdks/OpenNI_SDK/OpenNI_*_linux/sdk/libs")
_OPENNI_SDK_DIRS += _glob.glob("sdks/OpenNI_SDK/OpenNI_*_arm64/sdk/libs")
_OPENNI_SDK_DIRS += _glob.glob("sdks/OpenNI_SDK/OpenNI_*_arm32/sdk/libs")
OPENNI2_REDIST_PATH = (
    _OPENNI_SDK_DIRS[0] if _OPENNI_SDK_DIRS else "sdks/OpenNI_SDK/Linux/OpenNI2"
)
del _glob, _OPENNI_SDK_DIRS

# 摄像头预热帧数 (部分相机首帧很暗, 需要丢弃前几帧)
WARMUP_FRAMES = 5

# ===================== 相机标定参数 (calibrate_camera.py) =====================
# 标定用高分辨率通道 (生产检测用720p, 标定用1440p再按比例缩放派生)
MJPEG_SHOT1440_URL = "http://10.161.167.1:8080/shot1440.jpg"
CALIB_IMAGE_SIZE = (2560, 1440)  # 标定分辨率 (宽,高)
PROD_IMAGE_SIZE = (1280, 720)  # 生产检测分辨率 (宽,高)

# ChArUco 标定板 (celibrater/ 下两个PDF, 均为 8x11格 DICT_4X4_50, 70个内角点)
# 打印有缩放误差, 以下为实测方格边长; marker边长按PDF标称比例折算
CALIB_BOARD_SQUARES = (8, 11)
CALIB_SQUARE_BIG_MM = 21.12  # 大板实测 (22mm文件, 2026-07-31用户实测)
CALIB_MARKER_BIG_MM = CALIB_SQUARE_BIG_MM * 16.0 / 22.0  # ≈15.36
CALIB_SQUARE_SMALL_MM = 15.6  # 小板实测 (16mm文件)
CALIB_MARKER_SMALL_MM = CALIB_SQUARE_SMALL_MM * 11.5 / 16.0  # ≈11.21

# PDF版面几何 (100%缩放下的设计值, mm): 网格外框在A4页面中的位置
# (left/top = 网格左/上外缘到页面左/上边缘的距离, y从页面顶向下; 页头文字朝上)
# 由 pdftoppm 300dpi 渲染实测: 大板 17/15 小板 41/30
CALIB_PDF_GRID_BIG = {"left": 17.0, "top": 15.0, "w": 176.0, "h": 242.0}  # 8x22 x 11x22
CALIB_PDF_GRID_SMALL = {
    "left": 41.0,
    "top": 30.0,
    "w": 128.0,
    "h": 176.0,
}  # 8x16 x 11x16
# 打印缩放假设: s = 实测格边/标称格边 (本批 21.12/22 = 0.96)
# 页边距(网格外缘->纸边)如何随缩放变化, 两种模式:
#   'center'      : 内容绕页面中心缩放 (Adobe/Chrome"适应可打印区域"的标准行为)
#                   左=105*(1-s)+17s≈20.52  上=148.5*(1-s)+15s≈20.34
#   'proportional': 页边距随内容同比缩放 (按打印前比例, 相当于绕角落缩放)
#                   左=17s≈16.32  上=15s≈14.40
# 两者原点相差~4-6mm; 标定预览会同时画出两种框供肉眼比对
CALIB_MARGIN_MODE = "center"
# 最可靠: 用尺量"网格左/上外缘到纸左/上边的距离"(mm)填这里, 直接覆盖一切假设:
# 2026-07-30 已从锚点帧S/V剖面实测: 左19 上19 下47 右22 (闭合校验209.96x298.3±1.3)
CALIB_PAPER_MARGIN_OVERRIDE = (19.0, 18.3)

# 世界坐标系约定 (calibrate_camera.py 建立, 与 vision.set_paper_homography 一致):
#   原点 = 打印纸左下角 (页头文字朝上摆放), x向右 0-210 (短边), y向上 0-297 (长边)
#   摆放要求: 纸的下边缘(y=0)朝向机械臂基座

# 双板自动识别: 平放帧用已知桌面距离(复用下方CAMERA_HEIGHT_MM)反推焦距,
# 两种板假设只有一个自洽; 手持帧靠隐含距离落在此区间的合理性+相邻帧标签传递
HAND_Z_RANGE = (100.0, 450.0)  # 手持板距相机的合理距离区间 (mm)

# 引导式交替采集流程
CALIB_MIN_DESK = 4  # 平放帧最少数量 (第1帧需大板对齐A4左下角)
CALIB_MIN_HAND = 10  # 手持倾斜帧最少数量 (需覆盖画面四角/边缘)
CALIB_STABLE_PX = 0.5  # 角点抖动小于此像素值视为稳定
CALIB_STABLE_SEC = 0.8  # 持续稳定此秒数后自动拍摄
CALIB_MIN_CORNERS_DESK = 50  # 平放帧最少检测角点数 (板大部分入镜)
CALIB_MIN_CORNERS_HAND = 24  # 手持帧最少检测角点数
SAVE_CALIB_FRAMES = True  # 采集帧存到 debug_imgs/calib_<时间戳>/ 供 --images 复用

# 摄像头标定矩阵 (透视变换: 像素坐标 -> 世界坐标)
# 通过4个标定点计算，初始为None，运行calibration后填充
HOMOGRAPHY_MATRIX = None

# A4纸尺寸 (纵向放置，单位: mm)
A4_WIDTH = 210.0  # 水平方向
A4_HEIGHT = 297.0  # 垂直方向
DIVIDER_LINE_Y = 148.5  # 上下区域分界线 (A4高度的一半)

# 摄像头安装高度 (俯视)
# calibrate_camera.py 平放帧的双板识别依赖此值 (桌面距离反推焦距), 务必实测填准
CAMERA_HEIGHT_MM = 350  # 摄像头距桌面高度，需实测

# 深度有效范围 (单位: mm, 用于 Astra 深度流的截断与背景差分)
DEPTH_MIN_MM = 300
DEPTH_MAX_MM = 1000
TABLE_DEPTH_MM = 500  # 桌面参考深度, 实测后填入

# ===================== 任务参数 =====================
# 工作平面高度 (桌面高度，相对于机械臂底座)
TABLE_Z = 0.0  # 桌面Z坐标
APPROACH_Z = 80.0  # 接近高度 (拾取/放置前的悬停高度)
PICK_Z = 5.0  # 拾取高度 (夹爪接触碎片的高度)

# 目标矩形区域 (下半区域的中心区域)
# 目标矩形放置起始位置 (A4纸下半区域中心)
TARGET_CENTER_X = 105.0  # A4宽度中心
TARGET_CENTER_Y = 223.0  # 下半区域中心

# 拼图碎片参数 (第1项: 选手自备碎片)
# 图2碎片尺寸: 总宽8cm, 高2cm, 带1cm和3cm切口
PIECE_LENGTH = 80.0  # 碎片长度
PIECE_WIDTH = 20.0  # 碎片宽度

# 碎片颜色 (HSV范围，用于颜色检测)
PIECE_COLOR_HSV = {
    "lower": np.array([0, 80, 80]),  # 需根据实际颜色标定
    "upper": np.array([10, 255, 255]),
}

# 第2项: 现场碎片参数
TARGET_RECT_MIN = (90, 50)  # 目标矩形最小尺寸 (宽,高) mm
TARGET_RECT_MAX = (120, 90)  # 目标矩形最大尺寸 (宽,高) mm
MAX_PIECES = 4  # 最大碎片数量

# ===================== 完成指示 =====================
LED_PIN = 18  # GPIO引脚 (指示灯)
BUZZER_PIN = 23  # GPIO引脚 (蜂鸣器)

# ===================== 坐标系定义 =====================
# 世界坐标系: 原点在A4纸左下角(页头朝上), X向右0-210, Y向上0-297, Z出纸面
# 机械臂坐标系: 原点在底座中心, FK零位=全臂水平前伸方向为臂系+X
# 基座在世界系中的位置+朝向: 由 tools/calibrate_base_world.py (触点法) 标定
ARM_BASE_X = 105.0  # 底座X坐标 (待触点标定回填)
ARM_BASE_Y = -30.0  # 底座Y坐标 (待触点标定回填)
ARM_BASE_Z = 0.0  # 底座Z坐标 (桌面高度)
ARM_BASE_ROT_DEG = 0.0  # 臂系->世界 绕Z旋转角 (度)

# GUI本地触点标定产物优先于源码占位值；重启程序后自动生效。
_ARM_BASE_FILE = Path(__file__).resolve().with_name("arm_base.json")
if _ARM_BASE_FILE.exists():
    try:
        _arm_base = json.loads(_ARM_BASE_FILE.read_text(encoding="utf-8"))
        ARM_BASE_X = float(_arm_base["base_x"])
        ARM_BASE_Y = float(_arm_base["base_y"])
        ARM_BASE_ROT_DEG = float(_arm_base["rot_deg"])
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        print(f"[Config] 忽略无效的 {_ARM_BASE_FILE}: {exc}")

# ===================== 调试参数 =====================
DEBUG = True  # 开启调试输出
SHOW_IMAGE = True  # 显示OpenCV图像窗口
SAVE_LOG = True  # 保存运行日志

# ===================== 任意形状拼图求解参数 =====================
# (当前阶段: 随机任意碎片 -> 反推未知矩形 -> 输出直线路径)
EDGE_MATCH_TOL_MM = 3.0  # 边长匹配容差 (相机约1.4px/mm, 检测误差~2mm)
EDGE_MATCH_TOL_RELAX = 5.0  # 严格匹配无解时的放宽容差
POLY_APPROX_EPS_MM = 1.2  # 多边形近似误差 (mm, 在mm坐标系做approxPolyDP)
MIN_EDGE_LEN_MM = 8.0  # 小于此长度的边视为噪声并合并 (题目最小边20mm)
OVERLAP_TOL_RATIO = 0.03  # 允许重叠面积比例 (相对最小碎片面积)
RECT_ANGLE_TOL_DEG = 12.0  # 矩形角90度容差
RECT_SIDE_TOL_MM = 8.0  # 矩形对边等长容差
RECT_AREA_TOL = 0.05  # 矩形面积 vs 碎片总面积 容差比例
RECT_DIM_RANGE = (40.0, 135.0)  # 矩形边长合理范围 (题目: 9x5~12x9cm)
SOLVER_MAX_STATES = 20000  # DFS搜索状态上限
RASTER_PX_PER_MM = 3  # 重叠检查光栅化分辨率

# 目标矩形放置位置: 下半区中心 (世界坐标y向上, 上半区=碎片源区)
ASSEMBLY_TARGET_X = A4_WIDTH / 2  # 105
ASSEMBLY_TARGET_Y = DIVIDER_LINE_Y / 2  # 74.25
