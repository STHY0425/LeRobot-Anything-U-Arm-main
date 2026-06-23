from __future__ import annotations

import rospy

from .arm import HuaXinjingArm
from .config import HuaXinjingConfig
from .dynamics import DampingMap, RobotDynamics
from .gravity_compensation import GravityCompensator
from .publisher import GravityDebugPublisher, ServoStatePublisher
from .ros_params import load_arm_config_from_ros


class ServoReaderNode:
    """只读舵机 ROS 节点装配器。

    该节点负责读取舵机角度/电流并发布到 ROS 话题，不下发控制命令。

    Args:
        config: 舵机节点配置。
    """

    def __init__(self, config: HuaXinjingConfig) -> None:
        self.config = config
        self.arm = HuaXinjingArm(config)
        self.publisher = ServoStatePublisher(self.arm)
        self.rate = rospy.Rate(config.rate_hz)

    @classmethod
    def from_ros(cls) -> "ServoReaderNode":
        """从 ROS 参数服务器构造只读节点。

        Returns:
            已装配但尚未连接硬件的节点对象。
        """

        return cls(load_arm_config_from_ros())

    def connect(self) -> None:
        """连接舵机并打印发布话题信息。"""

        self.arm.connect()
        rospy.loginfo(
            "HuaXinjing reader connected: %s @ %d, ids=%s",
            self.config.port,
            self.config.baudrate,
            list(self.config.servo_ids),
        )
        rospy.loginfo("Publishing angles to %s", self.config.topic)
        if self.config.read_current:
            rospy.loginfo("Publishing currents to %s", self.config.current_topic)

    def run(self) -> None:
        """运行只读发布循环。

        每个周期只读取一次硬件状态，然后发布同一帧快照。这样保持发布数据
        内部一致，也避免发布器直接访问硬件。
        """

        self.connect()
        while not rospy.is_shutdown():
            state = self.arm.read_state()
            self.publisher.publish_all(state)
            self.rate.sleep()

    def close(self) -> None:
        """关闭节点持有的硬件资源。"""

        self.arm.close()


class GravityCompensationNode:
    """重力补偿 ROS 节点装配器。

    该节点负责装配机械臂、动力学模型、阻尼映射、补偿控制器和调试发布器。
    入口脚本只负责 `rospy.init_node()`，实际对象生命周期集中在这里。

    Args:
        config: 舵机节点配置。
        dynamics: 动力学模型。
        damping: 阻尼映射配置。
        mode: 重力补偿模式，支持 `auto`、`dynamics`、`adaptive`。
        release_on_shutdown: 退出时是否释放舵机。False 时恢复基础阻尼。
        power_topic: 阻尼功率调试话题名。
        torque_topic: 补偿力矩调试话题名。
    """

    def __init__(
        self,
        config: HuaXinjingConfig,
        dynamics: RobotDynamics,
        damping: DampingMap,
        mode: str,
        release_on_shutdown: bool = False,
        power_topic: str = "/servo_damping_powers",
        torque_topic: str = "/gravity_torques",
    ) -> None:
        self.config = config
        self.arm = HuaXinjingArm(config)
        self.state_publisher = ServoStatePublisher(self.arm)
        self.debug_publisher = GravityDebugPublisher(power_topic, torque_topic)
        self.compensator = GravityCompensator(
            self.arm, dynamics=dynamics, damping=damping, mode=mode
        )
        self.release_on_shutdown = release_on_shutdown
        self.rate = rospy.Rate(config.rate_hz)

    @classmethod
    def from_ros(cls) -> "GravityCompensationNode":
        """从 ROS 参数服务器构造重力补偿节点。

        Returns:
            已装配但尚未连接硬件的节点对象。
        """

        config = load_arm_config_from_ros(
            angle_topic_param="~angle_topic",
            default_angle_topic="/servo_angles",
            publish_absolute=False,
            read_current_default=False,
        )
        damping = DampingMap(
            base_power_mw=int(rospy.get_param("~base_power", 100)),
            max_power_mw=int(rospy.get_param("~max_power", 3000)),
            torque_gain_mw_per_nm=float(rospy.get_param("~torque_gain", 100.0)),
            angle_gain_mw_per_deg=float(rospy.get_param("~angle_gain", 20.0)),
            release_delta_deg=float(rospy.get_param("~release_delta", -2.0)),
        )
        dynamics = RobotDynamics(
            dof=len(config.servo_ids),
            urdf_path=rospy.get_param("~urdf_path", ""),
            compensation_gain=float(rospy.get_param("~compensation_gain", 1.0)),
        )
        return cls(
            config=config,
            dynamics=dynamics,
            damping=damping,
            mode=rospy.get_param("~mode", "auto"),
            release_on_shutdown=bool(rospy.get_param("~release_on_shutdown", False)),
            power_topic=rospy.get_param("~power_topic", "/servo_damping_powers"),
            torque_topic=rospy.get_param("~torque_topic", "/gravity_torques"),
        )

    def connect(self) -> None:
        """连接舵机并初始化重力补偿控制器。"""

        self.arm.connect()
        self.compensator.initialize()
        rospy.loginfo("HuaXinjing gravity compensation started")
        rospy.loginfo(
            "mode=%s dynamics_available=%s",
            self.compensator.mode,
            self.compensator.dynamics is not None and self.compensator.dynamics.available,
        )
        if self.compensator.dynamics is not None and self.compensator.dynamics.load_error:
            rospy.logwarn(
                "Dynamics model unavailable, fallback may be used: %s",
                self.compensator.dynamics.load_error,
            )

    def run(self) -> None:
        """运行重力补偿循环。

        每个周期的顺序固定为：

        1. 读取一帧舵机状态。
        2. 使用同一帧状态计算并下发阻尼。
        3. 使用同一帧状态发布角度和调试话题。
        """

        self.connect()
        while not rospy.is_shutdown():
            state = self.arm.read_state()
            self.compensator.step(state)
            self.state_publisher.publish_angles(state)
            self.debug_publisher.publish(
                self.compensator.power_array(), self.compensator.torque_array()
            )
            self.rate.sleep()

    def close(self) -> None:
        """停止补偿并关闭硬件资源。"""

        self.compensator.stop(release=self.release_on_shutdown)
        self.arm.close()
