"""
轨迹规划模块

功能:
1. 点到点运动规划 (关节空间插值)
2. 笛卡尔空间直线插值
3. 拾取-放置动作序列生成
4. 碰撞 avoidance (Z轴抬升策略)
"""

import math
import time

from config import (
    APPROACH_Z,
    ARM_BASE_ROT_DEG,
    ARM_BASE_X,
    ARM_BASE_Y,
    ARM_TARGET_RADIAL_COMP_MM,
    ID_BASE,
    ID_ELBOW,
    ID_SHOULDER,
    ID_WRIST_PITCH,
    ID_WRIST_ROLL,
    PICK_Z,
    SERVO_SPEED,
    TARGET_CENTER_X,
    TARGET_CENTER_Y,
)
from kinematics import inverse_kinematics


def world_to_arm(wx, wy):
    """世界目标 -> 机械臂目标，含八点标定后的实机径向命令补偿。"""
    dx, dy = wx - ARM_BASE_X, wy - ARM_BASE_Y
    radius = math.hypot(dx, dy)
    if radius > 1e-6:
        scale = (radius + ARM_TARGET_RADIAL_COMP_MM) / radius
        dx, dy = dx * scale, dy * scale
    th = math.radians(-ARM_BASE_ROT_DEG)
    c, s = math.cos(th), math.sin(th)
    return (c * dx - s * dy, s * dx + c * dy)


def arm_to_world(ax, ay):
    """机械臂坐标 -> 世界坐标 (world_to_arm 的逆变换)"""
    th = math.radians(ARM_BASE_ROT_DEG)
    c, s = math.cos(th), math.sin(th)
    return (ARM_BASE_X + c * ax - s * ay, ARM_BASE_Y + s * ax + c * ay)


def world_angle_to_arm(deg):
    """世界系中的朝向角 -> 臂系末端J5角 (随基座旋转移项)"""
    return deg - ARM_BASE_ROT_DEG


class MotionPlanner:
    def __init__(self, arm_controller, abort_event=None):
        self.arm = arm_controller
        self.abort_event = abort_event

    def _aborted(self):
        return self.abort_event is not None and self.abort_event.is_set()

    def _sleep(self, seconds):
        if self._aborted():
            return False
        time.sleep(seconds)
        return not self._aborted()

    # ==================== 关节空间插值 ====================

    def move_joints_smooth(self, target_angles, steps=50, dt=0.02):
        """
        关节空间平滑运动 (线性插值)
        target_angles: [J1..J6] 目标角度
        steps: 插值步数
        dt: 步间间隔(秒)
        """
        # 读取当前角度 (从缓存)
        current = self._get_current_angles()

        for i in range(1, steps + 1):
            if self._aborted():
                print("[Plan] 关节插值已中止")
                return False
            alpha = i / steps
            # 线性插值 (可改为S曲线平滑)
            alpha = self._smooth_step(alpha)
            interp = []
            for c, t in zip(current, target_angles, strict=True):
                interp.append(c + (t - c) * alpha)

            self.arm.move_joints(interp, speed=SERVO_SPEED)
            if not self._sleep(dt):
                return False
        return True

    def _smooth_step(self, t):
        """S曲线平滑: 避免起停冲击"""
        return t * t * (3 - 2 * t)

    def _get_current_angles(self):
        """获取当前关节角度"""
        from kinematics import servo_to_deg

        angles = []
        servo_ids = [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_WRIST_PITCH, ID_WRIST_ROLL]
        for sid in servo_ids:
            val = self.arm.current_positions.get(sid, 2048)
            angles.append(servo_to_deg(val, sid))
        return angles

    # ==================== 笛卡尔空间运动 ====================

    def move_line(
        self, x1, y1, z1, x2, y2, z2, steps=30, dt=0.03, wrist_pitch=-90, gripper_roll=0
    ):
        """
        笛卡尔空间直线插值
        从 (x1,y1,z1) 直线运动到 (x2,y2,z2)
        """
        for i in range(1, steps + 1):
            if self._aborted():
                print("[Plan] 直线路径已中止")
                return False
            alpha = self._smooth_step(i / steps)
            x = x1 + (x2 - x1) * alpha
            y = y1 + (y2 - y1) * alpha
            z = z1 + (z2 - z1) * alpha

            ok = self.arm.move_to_xyz(x, y, z, wrist_pitch, gripper_roll)
            if not ok:
                print(f"[Plan] 路径点不可达: ({x:.1f},{y:.1f},{z:.1f})")
                return False
            if not self._sleep(dt):
                return False
        return True

    def move_arc(self, x1, y1, z1, x2, y2, z2, lift_z=APPROACH_Z, steps=40, dt=0.03):
        """
        弧形运动: 抬起 -> 平移 -> 下降
        避免碰到其他碎片
        """
        # 阶段1: 抬升
        if not self.move_line(x1, y1, z1, x1, y1, lift_z, steps=10, dt=dt):
            return False
        # 阶段2: 水平移动
        if not self.move_line(x1, y1, lift_z, x2, y2, lift_z, steps=steps, dt=dt):
            return False
        # 阶段3: 下降
        return self.move_line(x2, y2, lift_z, x2, y2, z2, steps=10, dt=dt)

    # ==================== 拾取-放置序列 ====================

    def pick_and_place(self, pick_pos, pick_angle, place_pos, place_angle):
        """
        完整的拾取-放置动作序列 (电磁铁版: open_gripper=断电释放,
        close_gripper=通电吸附, 见 arm_controller 兼容别名)

        参数:
            pick_pos: (x, y) 拾取位置 (世界坐标mm)
            pick_angle: 拾取时末端旋转角度 (度)
            place_pos: (x, y) 放置位置 (世界坐标mm)
            place_angle: 放置时末端旋转角度 (度)
        """
        px, py = pick_pos
        tx, ty = place_pos

        # 世界坐标 -> 机械臂坐标 (含基座旋转)
        pick_x, pick_y = world_to_arm(px, py)
        place_x, place_y = world_to_arm(tx, ty)
        pick_roll = world_angle_to_arm(pick_angle)
        place_roll = world_angle_to_arm(place_angle)

        print(
            f"[Pick] ({pick_x:.0f},{pick_y:.0f}) -> "
            f"[Place] ({place_x:.0f},{place_y:.0f})"
        )

        # Step 1: 移动到拾取位置上方
        print("  1. 移动到拾取上方")
        if self._aborted():
            return False
        if not self.arm.move_to_xyz(
            pick_x, pick_y, APPROACH_Z, wrist_pitch=-90, gripper_roll=pick_roll
        ):
            return False
        self.arm.wait_for_stop()

        # Step 2: 确保电磁铁断电释放
        print("  2. 电磁铁释放准备")
        if self._aborted() or not self.arm.open_gripper():
            return False
        if not self._sleep(0.3):
            return False

        # Step 3: 下降到碎片
        print("  3. 下降拾取")
        if not self.move_line(
            pick_x, pick_y, APPROACH_Z, pick_x, pick_y, PICK_Z, steps=10, dt=0.03
        ):
            return False

        # Step 4: 电磁铁通电吸附
        print("  4. 电磁铁吸附")
        if self._aborted() or not self.arm.close_gripper():
            return False
        if not self._sleep(0.5):
            return False

        # Step 5: 抬升
        print("  5. 抬升")
        if not self.move_line(
            pick_x, pick_y, PICK_Z, pick_x, pick_y, APPROACH_Z, steps=10, dt=0.03
        ):
            return False

        # Step 6: 弧形移动到放置位置上方
        print("  6. 移动到放置上方")
        if not self.move_arc(
            pick_x,
            pick_y,
            APPROACH_Z,
            place_x,
            place_y,
            APPROACH_Z,
            lift_z=APPROACH_Z + 20,
        ):
            return False

        # Step 7: 旋转夹爪对齐角度
        if self._aborted():
            return False
        if not self.arm.move_to_xyz(
            place_x, place_y, APPROACH_Z, wrist_pitch=-90, gripper_roll=place_roll
        ):
            return False
        self.arm.wait_for_stop()

        # Step 8: 下降放置
        print("  7. 下降放置")
        if not self.move_line(
            place_x, place_y, APPROACH_Z, place_x, place_y, PICK_Z, steps=10, dt=0.03
        ):
            return False

        # Step 9: 电磁铁断电释放
        print("  8. 电磁铁释放碎片")
        if self._aborted() or not self.arm.open_gripper():
            return False
        if not self._sleep(0.3):
            return False

        # Step 10: 抬升离开
        print("  9. 抬升离开")
        if not self.move_line(
            place_x, place_y, PICK_Z, place_x, place_y, APPROACH_Z, steps=10, dt=0.03
        ):
            return False

        return not self._aborted()

    # ==================== 预设动作 ====================

    def go_home(self):
        """回到初始位置"""
        self.arm.go_home()
        self.arm.wait_for_stop()

    def calibrate_pose(self):
        """
        进入标定姿态: 末端指向工作区中心上方
        """
        center_x = TARGET_CENTER_X - ARM_BASE_X
        center_y = TARGET_CENTER_Y - ARM_BASE_Y
        self.arm.move_to_xyz(center_x, center_y, APPROACH_Z + 50)
        self.arm.wait_for_stop()

    # ==================== 路径验证 ====================

    def validate_path(self, waypoints):
        """
        验证路径中所有点是否可达
        waypoints: [(x,y,z), ...]
        """
        for i, (x, y, z) in enumerate(waypoints):
            if not inverse_kinematics(x, y, z):
                print(f"[Plan] 路径点{i}不可达: ({x:.0f},{y:.0f},{z:.0f})")
                return False
        return True
