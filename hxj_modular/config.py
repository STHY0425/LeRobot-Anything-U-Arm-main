from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class HuaXinjingConfig:
    """华馨京舵机节点运行配置。

    该配置同时服务只读发布节点和重力补偿节点。舵机 ID 默认也作为 ROS
    数组下标，因此 `num_servos` 应覆盖最大的舵机 ID。

    Attributes:
        port: 串口设备路径，例如 `/dev/ttyUSB0`。
        baudrate: 串口波特率。
        servo_ids: 需要读取/控制的舵机 ID。
        num_servos: ROS 输出数组长度。
        rate_hz: 节点循环频率。
        timeout: 串口读取超时时间，单位秒。
        topic: 角度发布话题名。
        zero_on_start: 启动时是否把当前角度记为零点。
        publish_absolute: 是否发布舵机绝对角度；False 时发布相对启动零点。
        use_last_valid: 读取失败时是否沿用上一帧有效角度/电流。
        step_size_deg: 角度变化小于该阈值时保持上一帧，0 表示不启用。
        jump_threshold_deg: 单帧跳变超过该阈值时保持上一帧，0 表示不启用。
        read_current: 是否读取并发布电流。
        current_topic: 电流发布话题名。
        joint_servo_ids: 参与动力学模型的舵机 ID 顺序。为空时使用 `servo_ids`。
        joint_signs: 舵机角度到 URDF 关节角的方向系数，常用 1 或 -1。
        joint_offsets_deg: 舵机角度进入 URDF 前叠加的偏置，单位 degree。
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    servo_ids: Tuple[int, ...] = (0, 1, 2, 3, 4, 5, 6)
    num_servos: int = 7
    rate_hz: float = 50.0
    timeout: float = 0.0
    topic: str = "/servo_angles"
    zero_on_start: bool = True
    publish_absolute: bool = False
    use_last_valid: bool = True
    step_size_deg: float = 0.0
    jump_threshold_deg: float = 90.0
    read_current: bool = False
    current_topic: str = "/servo_currents"
    joint_servo_ids: Tuple[int, ...] = ()
    joint_signs: Tuple[float, ...] = ()
    joint_offsets_deg: Tuple[float, ...] = ()

    @classmethod
    def from_values(
        cls,
        *,
        port: str,
        baudrate: int,
        servo_ids: Iterable[int],
        num_servos: int,
        rate_hz: float,
        timeout: float,
        topic: str,
        zero_on_start: bool,
        publish_absolute: bool,
        use_last_valid: bool,
        step_size_deg: float,
        jump_threshold_deg: float,
        read_current: bool = False,
        current_topic: str = "/servo_currents",
        joint_servo_ids: Iterable[int] = (),
        joint_signs: Iterable[float] = (),
        joint_offsets_deg: Iterable[float] = (),
    ) -> "HuaXinjingConfig":
        """从宽松入参构造标准配置对象。

        ROS 参数读取出来的值可能是列表、元组或数字字符串，本工厂函数负责把它们
        统一转换为不可变元组和明确的数值类型。

        Args:
            port: 串口设备路径。
            baudrate: 串口波特率。
            servo_ids: 需要读取/控制的舵机 ID。
            num_servos: ROS 输出数组长度。
            rate_hz: 节点循环频率。
            timeout: 串口读取超时时间。
            topic: 角度发布话题名。
            zero_on_start: 是否启动归零。
            publish_absolute: 是否发布绝对角度。
            use_last_valid: 是否使用上一帧有效值补偿离线舵机。
            step_size_deg: 小幅变化抑制阈值。
            jump_threshold_deg: 跳变抑制阈值。
            read_current: 是否读取电流。
            current_topic: 电流发布话题名。
            joint_servo_ids: 动力学关节对应的舵机 ID 顺序。
            joint_signs: 动力学关节方向系数。
            joint_offsets_deg: 动力学关节角度偏置。

        Returns:
            类型标准化后的配置对象。
        """

        return cls(
            port=port,
            baudrate=int(baudrate),
            servo_ids=tuple(int(x) for x in servo_ids),
            num_servos=int(num_servos),
            rate_hz=float(rate_hz),
            timeout=float(timeout),
            topic=topic,
            zero_on_start=bool(zero_on_start),
            publish_absolute=bool(publish_absolute),
            use_last_valid=bool(use_last_valid),
            step_size_deg=float(step_size_deg),
            jump_threshold_deg=float(jump_threshold_deg),
            read_current=bool(read_current),
            current_topic=current_topic,
            joint_servo_ids=tuple(int(x) for x in joint_servo_ids),
            joint_signs=tuple(float(x) for x in joint_signs),
            joint_offsets_deg=tuple(float(x) for x in joint_offsets_deg),
        )

    @property
    def active_joint_servo_ids(self) -> Tuple[int, ...]:
        """实际参与动力学映射的舵机 ID 顺序。

        Returns:
            `joint_servo_ids` 非空时返回它，否则返回完整 `servo_ids`。
        """

        if self.joint_servo_ids:
            return self.joint_servo_ids
        return self.servo_ids

    def joint_sign(self, index: int) -> float:
        """获取指定动力学关节的方向系数。

        Args:
            index: 动力学关节序号。

        Returns:
            配置中的方向系数；未配置时返回 1.0。
        """

        if index < len(self.joint_signs):
            return self.joint_signs[index]
        return 1.0

    def joint_offset_deg(self, index: int) -> float:
        """获取指定动力学关节的角度偏置。

        Args:
            index: 动力学关节序号。

        Returns:
            配置中的角度偏置，单位 degree；未配置时返回 0。
        """

        if index < len(self.joint_offsets_deg):
            return self.joint_offsets_deg[index]
        return 0.0
