from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass(frozen=True)
class ServoState:
    """单个舵机的一帧状态。

    Attributes:
        servo_id: 舵机 ID，同时也是 ROS 输出数组中的默认索引。
        angle_deg: 舵机角度，单位 degree。读取失败时为 None。
        current_a: 舵机电流，单位 A。未启用电流读取或读取失败时为 None。
        online: 当前帧是否来自有效硬件反馈。使用上一帧缓存补值时为 False。
    """

    servo_id: int
    angle_deg: Optional[float]
    current_a: Optional[float] = None
    online: bool = True

    @classmethod
    def offline(cls, servo_id: int) -> "ServoState":
        """创建离线舵机状态。

        Args:
            servo_id: 离线舵机 ID。

        Returns:
            angle/current 为空、online 为 False 的状态对象。
        """

        return cls(servo_id=servo_id, angle_deg=None, current_a=None, online=False)

    @property
    def angle_rad(self) -> Optional[float]:
        """舵机角度的弧度表示。

        Returns:
            当 `angle_deg` 有效时返回 rad，否则返回 None。
        """

        if self.angle_deg is None:
            return None
        return self.angle_deg * math.pi / 180.0


@dataclass(frozen=True)
class ArmState:
    """机械臂的一帧舵机状态快照。

    `states` 使用舵机 ID 作为 key，便于处理非连续 ID；输出给 ROS 时再
    转换为固定长度数组，数组索引仍然对应舵机 ID。

    Attributes:
        states: 舵机 ID 到 `ServoState` 的映射。
        num_servos: ROS 输出数组长度。
    """

    states: Dict[int, ServoState]
    num_servos: int

    def angle_array_deg(self, zero_angles: Optional[Dict[int, float]] = None) -> List[float]:
        """生成 ROS 角度数组。

        Args:
            zero_angles: 每个舵机的零点角度。为空时发布绝对角度。

        Returns:
            长度为 `num_servos` 的 degree 数组。离线或越界舵机填 0。
        """

        zero_angles = zero_angles or {}
        data = [0.0] * self.num_servos
        for servo_id, state in self.states.items():
            if 0 <= servo_id < self.num_servos and state.angle_deg is not None:
                data[servo_id] = state.angle_deg - zero_angles.get(servo_id, 0.0)
        return data

    def current_array_a(self) -> List[float]:
        """生成 ROS 电流数组。

        Returns:
            长度为 `num_servos` 的 A 数组。没有电流数据的位置填 0。
        """

        data = [0.0] * self.num_servos
        for servo_id, state in self.states.items():
            if 0 <= servo_id < self.num_servos and state.current_a is not None:
                data[servo_id] = state.current_a
        return data

    @property
    def online_ids(self) -> List[int]:
        """当前帧中在线的舵机 ID 列表。"""

        return [servo_id for servo_id, state in self.states.items() if state.online]


def parse_servo_ids(raw_ids) -> List[int]:
    """解析 ROS 参数中的舵机 ID 列表。

    Args:
        raw_ids: 单个整数、Python 列表/元组，或形如 "[0,1,2]" 的字符串。

    Returns:
        解析后的整数 ID 列表。
    """

    if isinstance(raw_ids, int):
        return [raw_ids]
    if isinstance(raw_ids, str):
        text = raw_ids.strip().strip("[]")
        if not text:
            return []
        return [int(item.strip()) for item in text.split(",") if item.strip()]
    return [int(raw_id) for raw_id in raw_ids]


def parse_float_list(raw_values) -> List[float]:
    """解析 ROS 参数中的浮点数组。

    Args:
        raw_values: 单个数字、Python 列表/元组，或形如 "[1,-1,0]" 的字符串。

    Returns:
        解析后的浮点数列表。
    """

    if raw_values is None:
        return []
    if isinstance(raw_values, (int, float)):
        return [float(raw_values)]
    if isinstance(raw_values, str):
        text = raw_values.strip().strip("[]")
        if not text:
            return []
        return [float(item.strip()) for item in text.split(",") if item.strip()]
    return [float(raw_value) for raw_value in raw_values]


def normalize_servo_ids(raw_ids: Iterable[int], num_servos: int) -> List[int]:
    """过滤输出数组范围之外的舵机 ID。

    Args:
        raw_ids: 原始舵机 ID 序列。
        num_servos: ROS 输出数组长度。

    Returns:
        满足 `0 <= id < num_servos` 的舵机 ID 列表。
    """

    return [int(servo_id) for servo_id in raw_ids if 0 <= int(servo_id) < num_servos]
