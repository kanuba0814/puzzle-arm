# pyright: reportMissingImports=false

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("so101_follower_5d")
@dataclass
class SO101Follower5DConfig(RobotConfig):
    """五轴 SO-101 follower 配置；不包含夹爪舵机。"""

    port: str
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = True
