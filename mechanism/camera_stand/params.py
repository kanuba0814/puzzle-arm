"""相机支架全局参数（唯一真源）。

对应设计文档 docs/23_相机支架设计.md。
约定：全部尺寸单位 mm，质量单位 g，力单位 N。
坐标系：零件各自局部坐标，装配时统一到 stand 坐标系（桌面 Z=0，底座圆心为 XY 原点，
X 指向 A4 方向，Z 竖直向上）。
"""

# ============ 用户冻结需求 ============
BASE_DIAMETER_MM = 166.2  # 底座圆盘直径（用户指定）
YAW_CONTACT_Z_MM = 60.0  # 偏航接触点高度（用户要求 50~70，取中值）
LENS_HEIGHT_NOMINAL_MM = 364.0  # 镜头光心目标高度 = A4 对角线
LENS_HEIGHT_TOLERANCE_MM = 20.0  # 允许范围 ±20
REACH_MM = 230.0  # 底座圆心到 A4 中心的水平前伸
ELBOW_TILT_RANGE_DEG = 10.0  # 顶端俯仰微调范围 ±10°

# ============ 打印/材料 ============
PRINTER_BED_MM = 256.0  # P1S 成型尺寸
NOZZLE_MM = 0.4
WALL_MM = 2.0  # 主壁厚（5 层墙）
PLA_DENSITY_G_CM3 = 1.24
PLA_ELASTIC_MODULUS_MPA = 2200.0  # 打印件保守弹性模量（工程假设）

# ============ 紧固件（用户限定：仅 M3×6 短螺丝 + M3 螺母/铜柱） ============
M3_CLEARANCE_MM = 3.2  # M3 通孔
M3_NUT_AF_MM = 5.5  # M3 螺母对边
M3_NUT_TRAP_AF_MM = 5.8  # 螺母孔对边（含装配间隙）
M3_NUT_THICK_MM = 2.4
M3_SCREW_LEN_MM = 6.0
CLAMP_EAR_THICK_MM = 2.8  # 夹紧耳（含螺母侧）厚度
CLAMP_EAR_THIN_MM = 2.0  # 夹紧耳（通孔侧）厚度，保证螺丝穿耳+缝后全牙进螺母
SELF_TAP_PILOT_MM = 2.5  # M3 自攻底孔（固定连接用）
STANDOFF_LEN_MM = 8.0  # M3 双通铜柱长度（夹紧副 barrel）

# ============ 底座 P1 ============
BASE_TUB_HEIGHT_MM = 32.0  # P1a 底座桶高度
BASE_TOP_THICK_MM = 3.0  # P1b 顶板厚度
CAVITY_DIAMETER_MM = 150.0
CAVITY_DEPTH_MM = 30.0  # 配重腔深度（容积约 530 cm3）
BASE_WALL_MM = 3.0
BASE_RIB_COUNT = 8
BASE_RIB_THICK_MM = 2.0
TOWER_OUTER_D_MM = 56.0  # 偏航塔柱外径
TOWER_CLAMP_SLIT_MM = 2.0  # 开口夹环缝隙
YAW_SPIGOT_D_MM = 40.0  # 转子止口直径
YAW_SPIGOT_CLEARANCE_MM = 0.4  # 止口直径间隙
YAW_SPIGOT_LEN_MM = 18.0
YAW_SOCKET_EXTRA_DEPTH_MM = 2.0

# 配重材料密度（g/cm3，工程估计，实测后更新）
SAND_DENSITY_G_CM3 = 1.5
STEEL_GRIT_DENSITY_G_CM3 = 4.5

# ============ 偏航转子 P3 ============
ROTOR_FLANGE_D_MM = 60.0
ROTOR_FLANGE_THICK_MM = 4.0
ROTOR_BOLT_COUNT = 4
ROTOR_BOLT_PCD_MM = 44.0  # 与斜梁根部连接孔节圆直径

# ============ 斜梁 P4 ============
BOOM_RUN_MM = REACH_MM  # 水平投影
BOOM_RISE_MM = 63.0  # 根部(66)到顶端销孔(129)的升高
BOOM_ROOT_W_MM = 30.0  # 根部箱形截面宽
BOOM_ROOT_H_MM = 20.0
BOOM_TIP_W_MM = 20.0
BOOM_TIP_H_MM = 15.0
BOOM_ROOT_PLATE_THICK_MM = 2.0
BOOM_RIB_PITCH_MM = 30.0  # 内部隔板间距

# ============ 顶端肘节 P5 + boardCam 接口（实测值，勿随意改） ============
FORK_SLOT_WIDTH_MM = 12.0  # boardCam 叉口宽（实测）
FORK_BLOCK_W_MM = 35.0  # 叉耳块宽 X（实测 73~108）
FORK_BLOCK_D_MM = 46.0  # 叉耳块深 Y（实测）
FORK_BLOCK_H_MM = 55.0  # 叉耳块高 Z（实测）
FORK_PRONG_Y_SPAN_MM = 12.0  # 叉口 Y 向跨度（实测估计 34~46，试片验证）
FORK_HOLE_DIAMETER_MM = 9.6  # 叉耳底部 Y 向盲孔（实测，v1 不使用）
FORK_COUNTERBORE_D_MM = 17.0  # 后段沉孔（实测，v1 不使用）

LUG_THICK_MM = 11.8  # 插榫厚度 X（叉口 12.0 - 0.2）
LUG_DEPTH_MM = 11.6  # 插榫深度 Y（叉口跨度 12 - 0.4，试片验证）
LUG_HEIGHT_MM = 38.0  # 插榫插入高度
ELBOW_CLAMP_BAND_W_MM = 10.0  # C 形夹箍带宽（沿 Z）

# 肘节铰链
ELBOW_HINGE_PIN_D_MM = 8.0
ELBOW_ARC_R_MM = 25.0  # 圆弧长孔半径
ELBOW_CLAMP_BOLTS = 2

# ============ 装配链（stand 坐标系） ============
# 底座顶板顶面 -> 转子法兰 -> 斜梁根部
ROTOR_FLANGE_TOP_Z_MM = YAW_CONTACT_Z_MM + ROTOR_FLANGE_THICK_MM
BOOM_BASE_Z_MM = ROTOR_FLANGE_TOP_Z_MM + BOOM_ROOT_PLATE_THICK_MM
PIN_HOLE_Z_MM = 129.0  # 顶端销孔标称高度
PIN_HOLE_X_MM = REACH_MM  # 顶端销孔水平位置
L_LENS_ASSUMED_MM = 235.0  # 叉耳孔心到镜头光心距离（假设，实测后更新）

# ============ 验收载荷/阈值 ============
TIP_TEST_FORCE_N = 1.0  # 倾覆验收水平扰动力
BOOM_PROOF_LOAD_N = 2.5  # 梁端验收竖直载荷
BOOM_DEFLECTION_LIMIT_MM = 3.0
BASE_MASS_MIN_G = 1000.0  # 底座总重下限
SAFETY_FACTOR_TIP = 2.0
