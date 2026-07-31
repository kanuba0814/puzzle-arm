# pyright: reportMissingImports=false

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.types import RobotAction

from .config_so101_5d import SO101Follower5DConfig

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
JOINT_FEATURES = tuple(f"{joint_name}.pos" for joint_name in JOINT_NAMES)


class SO101Follower5D(SOFollower):
    """去掉 ID 6 夹爪后的 SO-101 五轴 follower。"""

    config_class = SO101Follower5DConfig
    name = "so101_follower_5d"

    def __init__(self, config: SO101Follower5DConfig):
        # 不调用 SOFollower.__init__，避免创建 ID 6 的 gripper。
        Robot.__init__(self, config)
        self.config = config
        if self.calibration and set(self.calibration) != set(JOINT_NAMES):
            missing = sorted(set(JOINT_NAMES) - set(self.calibration))
            extra = sorted(set(self.calibration) - set(JOINT_NAMES))
            raise ValueError(
                f"LeRobot 标定必须是当前单臂精确五轴； missing={missing}, extra={extra}"
            )
        norm_mode = (
            MotorNormMode.DEGREES
            if config.use_degrees
            else MotorNormMode.RANGE_M100_100
        )
        self.bus = FeetechMotorsBus(
            port=config.port,
            motors={
                joint_name: Motor(motor_id, "sts3215", norm_mode)
                for motor_id, joint_name in enumerate(JOINT_NAMES, start=1)
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    def connect_bumpless(
        self,
        *,
        p_coefficient: int = 16,
        acceleration: int = 10,
    ) -> dict[str, float]:
        """先把目标对齐当前位置，再使能力矩，避免连接瞬间跳变。"""
        if self.bus.is_connected:
            raise RuntimeError("机械臂总线已经连接")
        try:
            self.bus.connect()
            self.bus.disable_torque()
            if not self.is_calibrated:
                raise RuntimeError("LeRobot 五轴电机标定寄存器不一致")
            self.bus.configure_motors(
                return_delay_time=0,
                maximum_acceleration=acceleration,
                acceleration=acceleration,
            )
            for motor in JOINT_NAMES:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, p_coefficient)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)
                self.bus.write("Acceleration", motor, acceleration)
            present_raw = self.bus.sync_read(
                "Present_Position", normalize=False, num_retry=2
            )
            self.bus.sync_write(
                "Goal_Position", present_raw, normalize=False, num_retry=2
            )
            goal_raw = self.bus.sync_read("Goal_Position", normalize=False, num_retry=2)
            mismatch = {
                motor: int(goal_raw[motor]) - int(present_raw[motor])
                for motor in JOINT_NAMES
            }
            if any(abs(delta) > 1 for delta in mismatch.values()):
                raise RuntimeError(f"当前位置保持目标回读不一致: {mismatch}")
            self.bus.enable_torque(num_retry=2)
            for camera in self.cameras.values():
                camera.connect()
            present_deg = self.bus.sync_read(
                "Present_Position", normalize=True, num_retry=2
            )
            return {f"{motor}.pos": float(present_deg[motor]) for motor in JOINT_NAMES}
        except BaseException:
            if self.bus.is_connected:
                try:
                    self.bus.disable_torque(num_retry=2)
                finally:
                    self.bus.disconnect(disable_torque=True)
            raise

    def send_action(self, action: RobotAction) -> RobotAction:
        """拒绝缺失、额外或六轴旧字段，再交由官方安全步长过滤发送。"""
        actual_keys = set(action)
        expected_keys = set(JOINT_FEATURES)
        if actual_keys != expected_keys:
            missing_keys = sorted(expected_keys - actual_keys)
            extra_keys = sorted(actual_keys - expected_keys)
            raise ValueError(
                "五轴动作字段必须精确匹配固定契约；"
                f" missing={missing_keys}, extra={extra_keys}"
            )
        return super().send_action(action)
