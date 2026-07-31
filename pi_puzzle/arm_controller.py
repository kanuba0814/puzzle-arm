"""
舵机控制模块 (SO-ARM101, 5舵机 + 电磁铁方案)

功能:
1. 与 Feetech STS3215 舵机通信 (串口半双工, 波特率1M)
2. 单舵机/多舵机位置控制 (5个关节舵机 ID 1-5)
3. 读取舵机反馈 (位置、负载、电压)
4. 电磁铁通断 (替代夹爪, 委托 magnet_controller)
5. 平滑运动控制 (加减速)

硬件说明:
- 夹爪舵机(ID 6)已物理拆除, 本模块只控制 ID 1-5
- 电磁铁由树莓派 GPIO 驱动 (见 magnet_controller.py)
- 兼容别名: close_gripper()=magnet_on() (闭合夹爪≈通电吸附),
           open_gripper()=magnet_off() (张开夹爪≈断电释放)
  使 motion_planner.py 的 pick_and_place 流程无需改动

依赖: scservo_sdk (Feetech官方SDK, 纯Python)
  树莓派离线安装: bash tools/setup_pi.sh
  (未安装时自动尝试 sdks/offline/ 内置拷贝; 都没有则进入模拟模式)
"""

import glob
import sys
import time
from pathlib import Path

from config import *

# scservo_sdk 优先用系统已安装的; 否则回退到项目内置离线拷贝
# (scservo_sdk 依赖 pyserial 的 import serial, 一并回退)
_offline = Path(__file__).resolve().parent / "sdks" / "offline"
try:
    import serial  # noqa: F401
except ImportError:
    if (_offline / "pyserial_fallback").is_dir():
        sys.path.insert(0, str(_offline / "pyserial_fallback"))
try:
    from scservo_sdk import *

    SCS_LIBRARY_OK = True
except ImportError:
    if _offline.is_dir():
        sys.path.insert(0, str(_offline))
    try:
        from scservo_sdk import *

        SCS_LIBRARY_OK = True
    except ImportError:
        SCS_LIBRARY_OK = False
        print("[警告] scservo_sdk 未安装, 使用模拟模式")
        print("  树莓派上安装: bash tools/setup_pi.sh")

from magnet_controller import MagnetController

# STS3215 寄存器 (对照 Feetech 官方控制表, 与 tools/servo_tool.py 一致)
ADDR_TORQUE_ENABLE = 40  # 0x28
ADDR_GOAL_POSITION = 42  # 0x2A
ADDR_GOAL_VELOCITY = 46  # 0x2E
ADDR_PRESENT_POSITION = 56  # 0x38
ADDR_PRESENT_LOAD = 60  # 0x3C (注意: 0x3E=62 是电压, 不是负载!)
ADDR_MOVING = 66  # 0x42

MAGNET_SETTLE_MS = 200  # 通电吸附等待时间


def _detect_port():
    """自动探测舵机驱动板串口"""
    cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if len(cands) == 1:
        return cands[0]
    if not cands:
        print("[Arm] 未发现 /dev/ttyUSB*|ttyACM*, 进入模拟模式")
    else:
        print(f"[Arm] 多个串口候选 {cands}, 请在 config.py 明确 SERVO_PORT")
    return None


class ArmController:
    def __init__(self, port=SERVO_PORT, baudrate=SERVO_BAUDRATE):
        self.port = _detect_port() if port in (None, "auto") else port
        self.baudrate = baudrate
        self.simulation = not SCS_LIBRARY_OK or self.port is None
        self.initialized = False
        self.current_positions = {}  # 当前各舵机角度
        self.magnet = MagnetController(MAGNET_PIN)

        if not self.simulation:
            self._init_hardware()

    def _init_hardware(self):
        """初始化硬件通信"""
        self.packet_handler = PacketHandler(0)  # STS/SMS系列 protocol_end=0
        self.port_handler = PortHandler(self.port)

        if not self.port_handler.openPort():
            print(f"[Arm] 串口打开失败: {self.port} (检查dialout组权限)")
            self.simulation = True
            return

        if not self.port_handler.setBaudRate(self.baudrate):
            print(f"[Arm] 波特率设置失败: {self.baudrate}")
            self.simulation = True
            return

        self.initialized = True
        print(f"[Arm] 串口已连接: {self.port} @ {self.baudrate}")

    # ==================== 基础控制 ====================

    def write_position(self, servo_id, value, speed=SERVO_SPEED):
        """
        设置舵机位置
        servo_id: 舵机ID (1-5)
        value: 目标位置 (0-4095)
        speed: 运动速度 (0-4095)
        """
        value = int(max(SERVO_MIN_VALUE, min(SERVO_MAX_VALUE, value)))

        if self.simulation:
            print(f"[Sim] 舵机{servo_id} -> {value} (speed={speed})")
            self.current_positions[servo_id] = value
            return True

        # 先限速再写位置, 保证第一次运动也是设定速度 (带读回验证)
        if not self._verified_write2B(servo_id, ADDR_GOAL_VELOCITY, speed):
            return False
        return self._verified_write2B(servo_id, ADDR_GOAL_POSITION, value)

    def write_positions_sync(self, positions, speed=SERVO_SPEED):
        """
        写多个舵机位置 (逐个写入 + 读回目标寄存器验证, 失败自动重试)
        不用 GroupSyncWrite 广播: 广播无单舵机应答, 丢包完全不可见
        (阶段6实测出现 J1 静默丢包原地不动); 逐写启动偏差<50ms 可忽略
        positions: {servo_id: value, ...}
        """
        positions = {
            sid: int(max(SERVO_MIN_VALUE, min(SERVO_MAX_VALUE, val)))
            for sid, val in positions.items()
        }

        if self.simulation:
            for sid, val in positions.items():
                print(f"[Sim] 舵机{sid} -> {val}")
            self.current_positions.update(positions)
            return True

        ok = True
        for sid, val in positions.items():
            # 先限速再写位置, 保证第一次运动也是设定速度
            if not self._verified_write2B(sid, ADDR_GOAL_VELOCITY, speed):
                ok = False
            if not self._verified_write2B(sid, ADDR_GOAL_POSITION, val):
                ok = False
        return ok

    def _verified_write2B(self, servo_id, addr, value, retries=2):
        """2字节写 + 读回验证 (防静默丢包), 失败重试 retries 次"""
        readback = None
        for _ in range(retries + 1):
            self.packet_handler.write2ByteTxRx(self.port_handler, servo_id, addr, value)
            time.sleep(0.002)  # 等舵机寄存器更新
            readback, result, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, addr
            )
            if result == 0 and readback == value:
                return True
        print(
            f"[Arm] 舵机{servo_id} 写@{addr}={value} 验证失败 "
            f"(读回{readback}), 已重试{retries}次"
        )
        return False

    def read_position(self, servo_id):
        """读取舵机当前位置"""
        if self.simulation:
            return self.current_positions.get(servo_id, 2048)

        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_PRESENT_POSITION
        )
        if result != 0:
            print(
                f"[Arm] 舵机{servo_id}读取失败: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
            return None
        return value

    def read_load(self, servo_id):
        """读取舵机负载 (判断是否卡住); 低10位幅值+bit10方向"""
        if self.simulation:
            return 0

        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_PRESENT_LOAD
        )
        if result != 0:
            return None
        magnitude = value & 0x3FF
        return -magnitude if value & 0x400 else magnitude

    # ==================== 高层控制 ====================

    def move_joints(self, joint_angles, speed=SERVO_SPEED):
        """
        通过关节角度控制机械臂
        joint_angles: [J1, J2, J3, J4, J5] (度, 5个关节)
        """
        from kinematics import deg_to_servo

        if len(joint_angles) != len(SERVO_IDS):
            print(f"[Arm] 关节角数量错误: {len(joint_angles)}, 应为{len(SERVO_IDS)}")
            return False

        positions = {}
        for sid, angle in zip(SERVO_IDS, joint_angles):
            positions[sid] = deg_to_servo(angle, sid)
            self.current_positions[sid] = positions[sid]

        return self.write_positions_sync(positions, speed)

    def move_to_xyz(self, x, y, z, wrist_pitch=-90, gripper_roll=0, speed=SERVO_SPEED):
        """
        控制末端到指定空间位置
        x, y, z: 目标坐标 (mm, 相对于机械臂底座)
        """
        from kinematics import ik_to_servo_values, inverse_kinematics

        angles = inverse_kinematics(x, y, z, wrist_pitch, gripper_roll)
        if angles is None:
            print(f"[Arm] 无法到达 ({x:.0f},{y:.0f},{z:.0f})")
            return False

        servo_vals = ik_to_servo_values(angles)
        if servo_vals is None:
            return False

        for sid, val in servo_vals.items():
            self.current_positions[sid] = val

        return self.write_positions_sync(servo_vals, speed)

    # ==================== 电磁铁 (替代夹爪) ====================

    def magnet_on(self):
        """电磁铁通电 (吸附碎片), 阻塞等待吸合"""
        ok = self.magnet.on()
        time.sleep(MAGNET_SETTLE_MS / 1000)
        return ok

    def magnet_off(self):
        """电磁铁断电 (释放碎片), 内含释放延时(配合消磁电路)"""
        return self.magnet.off()  # off() 内含 MAGNET_RELEASE_MS 延时

    # ---- 夹爪兼容别名: motion_planner/main 无需改动 ----
    # 语义映射: close_gripper=抓取=通电吸附, open_gripper=释放=断电
    def close_gripper(self):
        """[兼容别名] 原'闭合夹爪' -> 电磁铁通电吸附"""
        return self.magnet_on()

    def open_gripper(self):
        """[兼容别名] 原'张开夹爪' -> 电磁铁断电释放"""
        return self.magnet_off()

    # ==================== 状态管理 ====================

    def go_home(self):
        """回到用户手动确认的纸外驻车位，不经过歧义的关节角换算。"""
        print(f"[Arm] 回纸外驻车位(raw): {PARK_SERVO_POSITIONS}")
        self.current_positions.update(PARK_SERVO_POSITIONS)
        return self.write_positions_sync(
            PARK_SERVO_POSITIONS, speed=PARK_SERVO_SPEED
        )

    def get_current_position(self):
        """读取所有舵机当前位置"""
        positions = {}
        for sid in SERVO_IDS:
            pos = self.read_position(sid)
            if pos is not None:
                positions[sid] = pos
        return positions

    def is_moving(self, servo_id):
        """检查舵机是否在运动"""
        if self.simulation:
            return False

        value, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, servo_id, ADDR_MOVING
        )
        if result != 0:
            return False
        return bool(value)

    def wait_for_stop(self, timeout_ms=10000):
        """等待所有舵机停止运动 (4.9V低压供电下运动很慢, 留足10s)"""
        if self.simulation:
            time.sleep(0.5)
            return

        start = time.time()
        while time.time() - start < timeout_ms / 1000:
            moving = False
            for sid in SERVO_IDS:
                if self.is_moving(sid):
                    moving = True
                    break
            if not moving:
                return
            time.sleep(0.05)
        print("[Arm] 等待超时")

    def disable_torque(self, servo_id=None):
        """关闭力矩 (可手动摆动机械臂, 调试用)"""
        if self.simulation:
            return
        ids = [servo_id] if servo_id else SERVO_IDS
        for sid in ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, sid, ADDR_TORQUE_ENABLE, 0
            )

    def enable_torque(self, servo_id=None):
        """使能力矩"""
        if self.simulation:
            return
        ids = [servo_id] if servo_id else SERVO_IDS
        for sid in ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, sid, ADDR_TORQUE_ENABLE, 1
            )

    def close(self):
        """关闭连接"""
        self.magnet.close()
        if not self.simulation and self.port_handler:
            self.port_handler.closePort()
            print("[Arm] 串口已关闭")


if __name__ == "__main__":
    arm = ArmController()

    print("=== 机械臂控制测试 (5舵机+电磁铁) ===")
    print("命令: h=home, pos=读位置, t=释放力矩, e=使能力矩")
    print("      o=电磁铁OFF(释放), c=电磁铁ON(吸附), q=quit")
    print("      m <x> <y> <z> = 移动到坐标")

    arm.go_home()
    time.sleep(1)

    while True:
        cmd = input("> ").strip()
        if cmd == "q":
            break
        elif cmd == "h":
            arm.go_home()
            arm.wait_for_stop()
        elif cmd == "o":
            arm.magnet_off()
        elif cmd == "c":
            arm.magnet_on()
        elif cmd == "t":
            arm.disable_torque()
            print("力矩已释放, 可手动摆臂 (用 pos 读各关节值)")
        elif cmd == "e":
            arm.enable_torque()
        elif cmd.startswith("m "):
            parts = cmd.split()
            if len(parts) == 4:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                arm.move_to_xyz(x, y, z)
                arm.wait_for_stop()
        elif cmd == "pos":
            pos = arm.get_current_position()
            print(f"当前位置: {pos}")

    arm.close()
