"""华馨京 / FashionStar 舵机版 FeelForce 模块包。

该包将原 FeelForce 的模块化机械臂思想迁移到舵机控制场景中。通信和协议
由 `FashionStarServoBus` 融合封装；上层通过 `HuaXinjingArm`、发布器和
重力补偿控制器完成 ROS 话题发布与阻尼模式控制。
"""

from .arm import HuaXinjingArm
from .config import HuaXinjingConfig
from .dynamics import DampingMap, RobotDynamics
from .gravity_compensation import GravityCompensator
from .servo_bus import FashionStarServoBus
from .state_types import ArmState, ServoState

__all__ = [
    "ArmState",
    "DampingMap",
    "FashionStarServoBus",
    "GravityCompensator",
    "HuaXinjingArm",
    "HuaXinjingConfig",
    "RobotDynamics",
    "ServoState",
]
