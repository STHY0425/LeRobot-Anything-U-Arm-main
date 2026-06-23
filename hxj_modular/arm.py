from __future__ import annotations

import math
import threading
from typing import Dict, List, Optional, Sequence

from .config import HuaXinjingConfig
from .servo_bus import FashionStarServoBus
from .state_types import ArmState, ServoState


class HuaXinjingArm:
    """模块化机械臂门面。

    `HuaXinjingArm` 是上层控制器和发布器访问舵机的唯一语义入口。它不暴露
    官方 SDK 对象，而是把硬件反馈转换为机械臂状态快照，并负责启动零点、
    上一帧缓存、ROS 输出数组和动力学关节角映射。

    Args:
        config: 舵机节点运行配置。
        bus: 可选舵机总线。测试时可注入 fake bus；生产环境默认创建
            `FashionStarServoBus`。
    """

    def __init__(
        self, config: HuaXinjingConfig, bus: Optional[FashionStarServoBus] = None
    ) -> None:
        self.config = config
        self.bus = bus or FashionStarServoBus(
            config.port, config.baudrate, timeout=config.timeout
        )
        self.zero_angles: Dict[int, float] = {servo_id: 0.0 for servo_id in config.servo_ids}
        self.last_raw_angles_deg: List[float] = [0.0] * config.num_servos
        self.last_angle_message: List[float] = [0.0] * config.num_servos
        self.last_currents_a: List[float] = [0.0] * config.num_servos
        self._angle_message_initialized = False
        self._state_lock = threading.Lock()
        self._latest_state = ArmState(
            states={servo_id: ServoState.offline(servo_id) for servo_id in config.servo_ids},
            num_servos=config.num_servos,
        )

    @property
    def connected(self) -> bool:
        """底层舵机总线是否已连接。"""

        return self.bus.connected

    @property
    def latest_state(self) -> ArmState:
        """最近一次硬件读取生成的机械臂状态快照。

        Returns:
            不可变的 `ArmState` 快照。发布器应读取这个快照，而不是直接触发
            硬件读取。
        """

        with self._state_lock:
            return self._latest_state

    def connect(self) -> None:
        """连接舵机总线，并按配置执行启动零点校准。"""

        self.bus.connect()
        if self.config.zero_on_start and not self.config.publish_absolute:
            self.calibrate_zero()

    def close(self) -> None:
        """关闭底层舵机总线。"""

        self.bus.close()

    def calibrate_zero(self) -> Dict[int, float]:
        """把当前舵机角度记录为相对零点。

        Returns:
            舵机 ID 到零点角度的映射，单位 degree。
        """

        state = self.read_state(use_last_valid=False)
        with self._state_lock:
            for servo_id, servo_state in state.states.items():
                if servo_state.angle_deg is not None:
                    self.zero_angles[servo_id] = servo_state.angle_deg
            self._angle_message_initialized = False
            return dict(self.zero_angles)

    def read_state(self, use_last_valid: Optional[bool] = None) -> ArmState:
        """读取所有配置舵机，并更新最新状态快照。

        Args:
            use_last_valid: 是否在单个舵机读取失败时使用上一帧有效值。为 None 时
                使用 `config.use_last_valid`。

        Returns:
            本次读取后的 `ArmState` 快照。
        """

        if use_last_valid is None:
            use_last_valid = self.config.use_last_valid
        states = self.bus.read_all_states(
            self.config.servo_ids, read_current=self.config.read_current
        )

        with self._state_lock:
            if use_last_valid:
                for servo_id, servo_state in list(states.items()):
                    if servo_state.angle_deg is None and 0 <= servo_id < self.config.num_servos:
                        states[servo_id] = type(servo_state)(
                            servo_id=servo_id,
                            angle_deg=self.last_raw_angles_deg[servo_id],
                            current_a=self.last_currents_a[servo_id],
                            online=False,
                        )

            arm_state = ArmState(states=states, num_servos=self.config.num_servos)
            self._latest_state = arm_state
            self._update_last_valid_unlocked(arm_state)
            return arm_state

    read_and_update = read_state

    def angle_message_data(self, state: Optional[ArmState] = None) -> List[float]:
        """生成 `/servo_angles` 的消息数据。

        Args:
            state: 可选状态快照。为空时使用 `latest_state`。

        Returns:
            长度为 `config.num_servos` 的角度数组，单位 degree。
        """

        state = state or self.latest_state
        with self._state_lock:
            zero = {} if self.config.publish_absolute else self.zero_angles
            data = state.angle_array_deg(zero)

            if not self._angle_message_initialized:
                # 第一帧直接接受，避免绝对角度或未归零场景被跳变阈值误过滤。
                self.last_angle_message = data[:]
                self._angle_message_initialized = True
                return data

            for servo_id in self.config.servo_ids:
                if not 0 <= servo_id < self.config.num_servos:
                    continue
                previous = self.last_angle_message[servo_id]
                value = data[servo_id]
                if (
                    self.config.jump_threshold_deg > 0.0
                    and abs(value - previous) > self.config.jump_threshold_deg
                ):
                    data[servo_id] = previous
                    continue
                if (
                    self.config.step_size_deg > 0.0
                    and abs(value - previous) < self.config.step_size_deg
                ):
                    data[servo_id] = previous

            self.last_angle_message = data[:]
            return data

    def current_message_data(self, state: Optional[ArmState] = None) -> List[float]:
        """生成 `/servo_currents` 的消息数据。

        Args:
            state: 可选状态快照。为空时使用 `latest_state`。

        Returns:
            长度为 `config.num_servos` 的电流数组，单位 A。
        """

        state = state or self.latest_state
        with self._state_lock:
            data = state.current_array_a()
            self.last_currents_a = data[:]
            return data

    def joint_positions_rad(
        self,
        state: Optional[ArmState] = None,
        joint_servo_ids: Optional[Sequence[int]] = None,
        dof: Optional[int] = None,
    ) -> List[float]:
        """把舵机角度映射成动力学模型使用的关节角。

        映射公式为：

        ```text
        q_i = radians((servo_angle_i - zero_i) * joint_sign_i + joint_offset_i)
        ```

        Args:
            state: 可选状态快照。为空时使用 `latest_state`。
            joint_servo_ids: 动力学关节对应的舵机 ID 顺序。为空时使用配置。
            dof: 动力学模型自由度。提供时会截断或补零到该长度。

        Returns:
            动力学模型使用的关节角数组，单位 rad。
        """

        state = state or self.latest_state
        servo_ids = list(joint_servo_ids or self.config.active_joint_servo_ids)
        if dof is not None:
            servo_ids = servo_ids[:dof]

        q_rad: List[float] = []
        with self._state_lock:
            zero_angles = dict(self.zero_angles)
            last_angles = list(self.last_raw_angles_deg)

        for index, servo_id in enumerate(servo_ids):
            servo_state = state.states.get(servo_id)
            if servo_state is not None and servo_state.angle_deg is not None:
                angle_deg = servo_state.angle_deg
            elif 0 <= servo_id < self.config.num_servos:
                angle_deg = last_angles[servo_id]
            else:
                angle_deg = 0.0

            relative_deg = angle_deg - zero_angles.get(servo_id, 0.0)
            joint_deg = (
                relative_deg * self.config.joint_sign(index)
                + self.config.joint_offset_deg(index)
            )
            q_rad.append(math.radians(joint_deg))

        if dof is not None:
            while len(q_rad) < dof:
                q_rad.append(0.0)
        return q_rad

    def _update_last_valid_unlocked(self, state: ArmState) -> None:
        """更新上一帧有效角度/电流缓存。

        调用者必须已经持有 `_state_lock`。

        Args:
            state: 用于更新缓存的状态快照。
        """

        for servo_id, servo_state in state.states.items():
            if not 0 <= servo_id < self.config.num_servos:
                continue
            if servo_state.angle_deg is not None:
                self.last_raw_angles_deg[servo_id] = servo_state.angle_deg
            if servo_state.current_a is not None:
                self.last_currents_a[servo_id] = servo_state.current_a
