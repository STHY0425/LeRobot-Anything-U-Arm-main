from __future__ import annotations

from typing import Optional, Sequence

import rospy
from std_msgs.msg import Float64MultiArray

from .arm import HuaXinjingArm
from .state_types import ArmState


class ServoStatePublisher:
    """舵机状态 ROS 发布器。

    发布器只消费 `HuaXinjingArm` 的状态快照/数组接口，不直接读取硬件。硬件
    读取节奏由节点循环控制，这样控制器和发布器可以使用同一帧 `ArmState`。

    Args:
        arm: 机械臂语义入口。
    """

    def __init__(self, arm: HuaXinjingArm) -> None:
        self.arm = arm
        self.angle_pub = rospy.Publisher(arm.config.topic, Float64MultiArray, queue_size=10)
        self.current_pub = None
        if arm.config.read_current:
            self.current_pub = rospy.Publisher(
                arm.config.current_topic, Float64MultiArray, queue_size=10
            )

    def publish_angles(self, state: Optional[ArmState] = None) -> None:
        """发布舵机角度数组。

        Args:
            state: 可选状态快照。为空时使用 `arm.latest_state`。
        """

        self.angle_pub.publish(Float64MultiArray(data=self.arm.angle_message_data(state)))

    def publish_currents(self, state: Optional[ArmState] = None) -> None:
        """发布舵机电流数组。

        Args:
            state: 可选状态快照。为空时使用 `arm.latest_state`。
        """

        if self.current_pub is not None:
            self.current_pub.publish(Float64MultiArray(data=self.arm.current_message_data(state)))

    def publish_all(self, state: Optional[ArmState] = None) -> None:
        """发布已启用的全部舵机状态话题。

        Args:
            state: 可选状态快照。为空时使用 `arm.latest_state`。
        """

        self.publish_angles(state)
        self.publish_currents(state)


class GravityDebugPublisher:
    """重力补偿调试话题发布器。

    Args:
        power_topic: 阻尼功率话题名。
        torque_topic: 补偿力矩话题名。
    """

    def __init__(
        self,
        power_topic: str = "/servo_damping_powers",
        torque_topic: str = "/gravity_torques",
    ) -> None:
        self.power_pub = rospy.Publisher(power_topic, Float64MultiArray, queue_size=10)
        self.torque_pub = rospy.Publisher(torque_topic, Float64MultiArray, queue_size=10)

    def publish(self, powers: Sequence[float], torques: Sequence[float]) -> None:
        """发布阻尼功率和补偿力矩。

        Args:
            powers: 阻尼功率数组，单位 mW。
            torques: 补偿力矩数组，单位 Nm。
        """

        self.power_pub.publish(Float64MultiArray(data=powers))
        self.torque_pub.publish(Float64MultiArray(data=torques))
