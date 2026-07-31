"""
SO-ARM101 逆运动学模块 (5舵机+电磁铁方案: 5个关节 J1-J5)

SO-ARM101 原为6自由度, 本方案拆除夹爪舵机(ID 6), 末端为电磁铁:
  J1 底座旋转   - 绕Z轴
  J2 肩关节俯仰 - 绕水平轴
  J3 肘关节俯仰 - 绕水平轴
  J4 腕俯仰     - 绕水平轴
  J5 腕旋转     - 绕末端轴线

对于桌面拼图任务，简化策略:
  - J4 (腕俯仰) 固定，使末端垂直向下吸取
  - J1 控制水平朝向
  - J2+J3 二连杆IK控制到达(r, z)
  - J5 控制末端旋转角度(对齐碎片方向)
"""
import numpy as np

from config import (
    KINEMATIC_JOINT_OFFSETS_DEG,
    L1,
    L1_EFFECTIVE,
    L2,
    L3,
    L4,
    SERVO_IDS,
    SERVO_LIMITS,
    SERVO_MAX_VALUE,
    SERVO_MIN_VALUE,
    SERVO_OFFSETS,
    SERVO_RANGE_DEG,
    SERVO_SIGNS,
)


def _counts_per_deg():
    """舵机物理分辨率: 4096个位置 / 360度 (线性映射, 与限位无关)"""
    return (SERVO_MAX_VALUE - SERVO_MIN_VALUE + 1) / SERVO_RANGE_DEG


def deg_to_servo(deg, servo_id):
    """
    关节角(度) -> 舵机值 (0-4095)
    线性物理映射: value = 2048 + OFFSETS + SIGNS*deg*4096/360
      deg=0 = 标定基准零位 (零位时读数=2048+OFFSETS)
      SERVO_LIMITS 仅作安全钳位, 不参与缩放
    """
    min_deg, max_deg = SERVO_LIMITS[servo_id]
    deg = np.clip(deg, min_deg, max_deg)
    center = (SERVO_MIN_VALUE + SERVO_MAX_VALUE) / 2
    servo_value = int(center + SERVO_OFFSETS[servo_id]
                      + SERVO_SIGNS[servo_id] * deg * _counts_per_deg())
    return np.clip(servo_value, SERVO_MIN_VALUE, SERVO_MAX_VALUE)


def servo_to_deg(value, servo_id):
    """舵机值 -> 关节角(度) (deg_to_servo 的逆变换; sign=±1 自逆)"""
    center = (SERVO_MIN_VALUE + SERVO_MAX_VALUE) / 2
    return SERVO_SIGNS[servo_id] * \
        (value - center - SERVO_OFFSETS[servo_id]) / _counts_per_deg()


def forward_kinematics(joint_angles):
    """
    正运动学: 关节角 -> 末端坐标 (世界坐标系中相对于底座)
    返回 (x, y, z) 末端位置 (mm)
    """
    j1, j2, j3, j4 = joint_angles[:4]
    d1, d2, d3, d4 = KINEMATIC_JOINT_OFFSETS_DEG
    j1_rad = np.radians(j1 + d1)
    j2_rad = np.radians(j2 + d2)
    j3_rad = np.radians(j3 + d3)
    j4_rad = np.radians(j4 + d4)

    # 在臂平面内的投影 (r-z平面)
    # 大臂端点
    p2_r = L2 * np.cos(j2_rad)
    p2_z = L1_EFFECTIVE + L2 * np.sin(j2_rad)

    # 小臂端点
    p3_r = p2_r + L3 * np.cos(j2_rad + j3_rad)
    p3_z = p2_z + L3 * np.sin(j2_rad + j3_rad)

    # 末端 (考虑腕部，腕朝下)
    end_r = p3_r + L4 * np.cos(j2_rad + j3_rad + j4_rad)
    end_z = p3_z + L4 * np.sin(j2_rad + j3_rad + j4_rad)

    # 转换到3D (底座旋转)
    x = end_r * np.cos(j1_rad)
    y = end_r * np.sin(j1_rad)
    z = end_z

    return np.array([x, y, z])


def inverse_kinematics(x, y, z, wrist_pitch_deg=-90, gripper_roll_deg=0):
    """
    逆运动学: 目标坐标 -> 关节角度

    参数:
        x, y, z: 目标位置 (mm, 相对于机械臂底座坐标系)
        wrist_pitch_deg: 末端绝对俯仰角 (默认-90度=垂直向下)
        gripper_roll_deg: 腕旋转角 (用于对齐碎片方向)

    返回:
        joint_angles: [J1, J2, J3, J4, J5] 或 None(不可达)
    """
    d1, d2, d3, d4 = KINEMATIC_JOINT_OFFSETS_DEG

    # J1: 底座旋转；减去模型零位修正后得到舵机软件角度。
    j1 = np.degrees(np.arctan2(y, x)) - d1

    # 水平距离
    r = np.sqrt(x**2 + y**2)

    # theta = 末端绝对角度 (j2 + j3 + j4 的总和)
    # wrist_pitch_deg 是绝对角度: -90 = 垂直向下, 0 = 水平
    # wrist_pitch_deg 保持旧调用语义：它是软件关节角 J2+J3+J4 的和。
    # 物理模型中的绝对俯仰还需加上三个关节的零位修正。
    theta = wrist_pitch_deg + d2 + d3 + d4
    theta_rad = np.radians(theta)

    # 修正: 减去腕部长度的投影 (用绝对角度, 不是相对j4)
    r_adj = r - L4 * np.cos(theta_rad)
    z_adj = z - L1_EFFECTIVE - L4 * np.sin(theta_rad)

    # 二连杆IK (肩+肘)
    # D = sqrt(r_adj^2 + z_adj^2)
    D = np.sqrt(r_adj**2 + z_adj**2)

    # 可达性检查
    if D > (L2 + L3) - 1 or D < abs(L2 - L3) + 1:
        print(f"[IK] 不可达: r={r:.1f}, z={z:.1f}, D={D:.1f}, "
              f"范围=[{abs(L2-L3):.1f}, {L2+L3:.1f}]")
        return None

    # 肘关节角 (余弦定理)
    cos_j3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_j3 = np.clip(cos_j3, -1.0, 1.0)
    # 肘朝下解 (j3<0): 实测肘关节机械范围主要在负方向(折叠方向),
    # 桌面抓取的自然姿态为"肩抬+小臂下折", 见 calibration_result.json
    j3_model = -np.degrees(np.arccos(cos_j3))

    # 肩关节角（先解物理模型角度，再换回软件关节角）。
    alpha = np.arctan2(z_adj, r_adj)
    beta = np.arctan2(L3 * np.sin(np.radians(j3_model)),
                      L2 + L3 * np.cos(np.radians(j3_model)))
    j2_model = np.degrees(alpha - beta)
    j2 = j2_model - d2
    j3 = j3_model - d3

    # 先求物理 J4，再减去模型零位修正。
    j4_model = theta - j2_model - j3_model
    j4 = j4_model - d4

    # J5: 腕旋转
    j5 = gripper_roll_deg

    # 限位检查 (5个关节)
    angles = [j1, j2, j3, j4, j5]
    for i, (sid, angle) in enumerate(zip(SERVO_IDS, angles)):
        lo, hi = SERVO_LIMITS[sid]
        if angle < lo or angle > hi:
            print(f"[IK] 关节{i+1}超限: {angle:.1f}° (范围 {lo}~{hi}°)")
            return None

    return angles


def ik_to_servo_values(joint_angles):
    """
    将关节角度转换为舵机值
    返回 dict {servo_id: value} (5个舵机)
    """
    if joint_angles is None:
        return None

    result = {}
    for sid, angle in zip(SERVO_IDS, joint_angles):
        result[sid] = deg_to_servo(angle, sid)
    return result


def check_workspace(x, y, z, wrist_pitch_deg=-90):
    """检查目标点是否在工作空间内"""
    angles = inverse_kinematics(x, y, z, wrist_pitch_deg)
    return angles is not None


def get_workspace_bounds():
    """
    返回工作空间范围 (r_min, r_max, z_min, z_max)
    用于验证路径可行性
    """
    r_min = abs(L2 - L3) + L4 * 0.7  # 近似最小水平到达
    r_max = L2 + L3 + L4 * 0.7       # 近似最大水平到达
    z_min = -L1 + 20                  # 最低高度 (避免撞底座)
    z_max = L1 + L2 + L3              # 最高高度
    return (r_min, r_max, z_min, z_max)


if __name__ == '__main__':
    # 测试IK
    print("=== 逆运动学测试 ===")
    print(f"连杆参数: L1={L1}, L2={L2}, L3={L3}, L4={L4}")

    test_points = [
        (150, 0, 50, "正前方中等距离"),
        (120, 80, 50, "右前方"),
        (120, -80, 50, "左前方"),
        (140, 0, 20, "桌面拾取高度"),
        (160, 0, 30, "正前方低位置"),
    ]

    r_min, r_max, z_min, z_max = get_workspace_bounds()
    print(f"工作空间: r=[{r_min:.0f}, {r_max:.0f}], z=[{z_min:.0f}, {z_max:.0f}]")
    print()

    for x, y, z, desc in test_points:
        angles = inverse_kinematics(x, y, z)
        if angles:
            # 验证: 正运动学回算
            fk = forward_kinematics(angles)
            error = np.sqrt(np.sum((fk - np.array([x, y, z]))**2))
            servo_vals = ik_to_servo_values(angles)
            print(f"[OK] {desc}: ({x},{y},{z})")
            print(f"     关节角: [{', '.join(f'{a:.1f}' for a in angles)}]°")
            print(f"     FK验证: ({fk[0]:.1f},{fk[1]:.1f},{fk[2]:.1f}) 误差={error:.2f}mm")
            print(f"     舵机值: {servo_vals}")
        else:
            print(f"[FAIL] {desc}: ({x},{y},{z}) 不可达")
        print()
