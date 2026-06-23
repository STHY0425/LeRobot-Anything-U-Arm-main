from __future__ import annotations

from typing import Optional

import rospy

from .config import HuaXinjingConfig
from .state_types import normalize_servo_ids, parse_float_list, parse_servo_ids


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
DEFAULT_NUM_SERVOS = 7
DEFAULT_RATE_HZ = 50.0
DEFAULT_TIMEOUT = 0.0


def load_arm_config_from_ros(
    *,
    angle_topic_param: str = "~topic",
    default_angle_topic: str = "/servo_angles",
    publish_absolute: Optional[bool] = None,
    read_current_default: bool = False,
) -> HuaXinjingConfig:
    """从 ROS 参数服务器读取舵机节点配置。

    支持的主要私有参数包括：

    - `~port` / `~baudrate` / `~timeout`
    - `~servo_ids` / `~num_servos` / `~rate`
    - `~zero_on_start` / `~publish_absolute`
    - `~step_size` / `~jump_threshold`
    - `~read_current` / `~current_topic`
    - `~joint_servo_ids` / `~joint_signs` / `~joint_offsets_deg`

    Args:
        angle_topic_param: 角度话题参数名。只读节点使用 `~topic`，重力补偿节点
            使用 `~angle_topic`，因此这里做成可配置。
        default_angle_topic: 角度话题默认值。
        publish_absolute: 强制指定是否发布绝对角度。为 None 时读取 ROS 参数。
        read_current_default: `~read_current` 未配置时的默认值。

    Returns:
        标准化后的 `HuaXinjingConfig`。

    Raises:
        ValueError: `~servo_ids` 为空或全部越界时抛出。
    """

    num_servos = int(rospy.get_param("~num_servos", DEFAULT_NUM_SERVOS))
    raw_servo_ids = parse_servo_ids(rospy.get_param("~servo_ids", DEFAULT_SERVO_IDS))
    servo_ids = normalize_servo_ids(raw_servo_ids, num_servos)
    if not servo_ids:
        raise ValueError("~servo_ids is empty after validation")
    if len(servo_ids) != len(raw_servo_ids):
        rospy.logwarn("Some servo ids were ignored because they are outside the output array")

    if publish_absolute is None:
        publish_absolute = bool(rospy.get_param("~publish_absolute", False))

    raw_joint_servo_ids = parse_servo_ids(rospy.get_param("~joint_servo_ids", servo_ids))
    joint_servo_ids = normalize_servo_ids(raw_joint_servo_ids, num_servos)
    if len(joint_servo_ids) != len(raw_joint_servo_ids):
        rospy.logwarn(
            "Some joint servo ids were ignored because they are outside the output array"
        )

    return HuaXinjingConfig.from_values(
        port=rospy.get_param("~port", DEFAULT_PORT),
        baudrate=int(rospy.get_param("~baudrate", DEFAULT_BAUDRATE)),
        servo_ids=servo_ids,
        num_servos=num_servos,
        rate_hz=float(rospy.get_param("~rate", DEFAULT_RATE_HZ)),
        timeout=float(rospy.get_param("~timeout", DEFAULT_TIMEOUT)),
        topic=rospy.get_param(angle_topic_param, default_angle_topic),
        zero_on_start=bool(rospy.get_param("~zero_on_start", True)),
        publish_absolute=publish_absolute,
        use_last_valid=bool(rospy.get_param("~use_last_valid", True)),
        step_size_deg=float(rospy.get_param("~step_size", 0.0)),
        jump_threshold_deg=float(rospy.get_param("~jump_threshold", 90.0)),
        read_current=bool(rospy.get_param("~read_current", read_current_default)),
        current_topic=rospy.get_param("~current_topic", "/servo_currents"),
        joint_servo_ids=joint_servo_ids,
        joint_signs=parse_float_list(rospy.get_param("~joint_signs", [])),
        joint_offsets_deg=parse_float_list(rospy.get_param("~joint_offsets_deg", [])),
    )
