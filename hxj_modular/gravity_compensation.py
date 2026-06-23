from __future__ import annotations

from typing import Dict, List, Optional, Sequence

from .arm import HuaXinjingArm
from .dynamics import DampingMap, RobotDynamics
from .state_types import ArmState


class GravityCompensator:
    """将 FeelForce 重力补偿力矩映射到华馨京舵机阻尼模式。

    原 FeelForce 的理想输出是关节前馈力矩 `-tau_g(q)`。华馨京舵机没有直接
    力矩接口，因此本类将补偿力矩转换为阻尼功率，并通过 `set_damping()`
    下发。

    Args:
        arm: 机械臂语义入口。
        dynamics: 可选动力学模型。`auto`/`dynamics` 模式下使用。
        damping: 阻尼映射配置。为空时使用默认值。
        mode: 控制模式，支持 `auto`、`dynamics`、`adaptive`。
    """

    def __init__(
        self,
        arm: HuaXinjingArm,
        dynamics: Optional[RobotDynamics] = None,
        damping: Optional[DampingMap] = None,
        mode: str = "auto",
    ) -> None:
        self.arm = arm
        self.dynamics = dynamics
        self.damping = damping or DampingMap()
        self.mode = mode.lower()
        self.last_angles_deg: Dict[int, float] = {servo_id: 0.0 for servo_id in arm.config.servo_ids}
        self.current_powers_mw: Dict[int, int] = {
            servo_id: self.damping.base_power_mw for servo_id in arm.config.servo_ids
        }
        self.last_torques_nm: Dict[int, float] = {servo_id: 0.0 for servo_id in arm.config.servo_ids}

    def initialize(self) -> None:
        """初始化阻尼控制器。

        初始化时会设置所有舵机为基础阻尼，并读取一帧角度作为自适应模式的
        起始参考。

        Raises:
            RuntimeError: 强制 `dynamics` 模式但动力学模型不可用时抛出。
        """

        self._ensure_dynamics_mode_available()
        self.arm.bus.set_all_damping(self.arm.config.servo_ids, self.damping.base_power_mw)
        state = self.arm.read_state()
        for servo_id, servo_state in state.states.items():
            if servo_state.angle_deg is not None:
                self.last_angles_deg[servo_id] = servo_state.angle_deg

    @property
    def use_dynamics(self) -> bool:
        """当前循环是否使用动力学重力补偿。"""

        if self.mode == "dynamics":
            return True
        if self.mode == "adaptive":
            return False
        return self.dynamics is not None and self.dynamics.available

    def step(self, state: Optional[ArmState] = None) -> Dict[int, int]:
        """执行一次补偿控制周期。

        Args:
            state: 可选状态快照。传入时可避免控制器重复读取硬件。

        Returns:
            舵机 ID 到当前阻尼功率的映射，单位 mW。

        Raises:
            RuntimeError: 强制 `dynamics` 模式但动力学模型不可用时抛出。
        """

        self._ensure_dynamics_mode_available()
        state = state or self.arm.read_state()
        if self.use_dynamics and self.dynamics is not None and self.dynamics.available:
            powers = self._step_dynamics(state)
        else:
            powers = self._step_adaptive_angle(state)

        for servo_id, power in powers.items():
            self.arm.bus.set_damping(servo_id, power)
        self.current_powers_mw.update(powers)
        return dict(self.current_powers_mw)

    def _step_dynamics(self, state: ArmState) -> Dict[int, int]:
        """使用 URDF/Pinocchio 计算重力补偿阻尼功率。

        Args:
            state: 当前舵机状态快照。

        Returns:
            参与动力学补偿的舵机 ID 到阻尼功率的映射。
        """

        dof = self.dynamics.dof if self.dynamics is not None else len(self.arm.config.servo_ids)
        ordered_ids = self._controlled_servo_ids(dof)
        q_rad = self.arm.joint_positions_rad(state, ordered_ids, dof=dof)

        gravity_tau = self.dynamics.compute_gravity_torque(q_rad)
        powers = {}
        for idx, servo_id in enumerate(ordered_ids):
            torque = gravity_tau[idx] if idx < len(gravity_tau) else 0.0
            # 原 FeelForce 前馈力矩是 -tau_g(q)。乘 joint_sign 是为了把 URDF
            # 关节方向映射回舵机方向，便于调试话题显示真实补偿方向。
            compensation_torque = -torque * self.arm.config.joint_sign(idx)
            self.last_torques_nm[servo_id] = compensation_torque
            powers[servo_id] = self.damping.torque_to_power(compensation_torque)
        return powers

    def _step_adaptive_angle(self, state: ArmState) -> Dict[int, int]:
        """无动力学模型时的角度变化自适应阻尼。

        Args:
            state: 当前舵机状态快照。

        Returns:
            舵机 ID 到阻尼功率的映射。
        """

        powers = {}
        for servo_id in self.arm.config.servo_ids:
            servo_state = state.states.get(servo_id)
            if servo_state is None or servo_state.angle_deg is None:
                powers[servo_id] = self.current_powers_mw[servo_id]
                continue

            previous_angle = self.last_angles_deg.get(servo_id, servo_state.angle_deg)
            delta_deg = servo_state.angle_deg - previous_angle
            previous_power = self.current_powers_mw[servo_id]
            powers[servo_id] = self.damping.angle_delta_to_power(delta_deg, previous_power)
            self.last_angles_deg[servo_id] = servo_state.angle_deg
            self.last_torques_nm[servo_id] = 0.0
        return powers

    def _controlled_servo_ids(self, dof: int) -> Sequence[int]:
        """获取参与动力学补偿的舵机 ID。

        Args:
            dof: 动力学模型自由度。

        Returns:
            截断到 `dof` 长度的舵机 ID 序列。
        """

        return tuple(self.arm.config.active_joint_servo_ids[:dof])

    def _ensure_dynamics_mode_available(self) -> None:
        """校验强制动力学模式是否可用。

        Raises:
            RuntimeError: `mode == "dynamics"` 且动力学模型不可用时抛出。
        """

        if self.mode != "dynamics":
            return
        if self.dynamics is not None and self.dynamics.available:
            return
        detail = ""
        if self.dynamics is not None and self.dynamics.load_error:
            detail = f": {self.dynamics.load_error}"
        raise RuntimeError(f"强制动力学模式不可用，请检查 URDF 和 Pinocchio{detail}")

    def torque_array(self) -> List[float]:
        """生成 `/gravity_torques` 调试话题数据。

        Returns:
            长度为 `num_servos` 的补偿力矩数组，单位 Nm。
        """

        data = [0.0] * self.arm.config.num_servos
        for servo_id, torque in self.last_torques_nm.items():
            if 0 <= servo_id < self.arm.config.num_servos:
                data[servo_id] = torque
        return data

    def power_array(self) -> List[float]:
        """生成 `/servo_damping_powers` 调试话题数据。

        Returns:
            长度为 `num_servos` 的阻尼功率数组，单位 mW。
        """

        data = [0.0] * self.arm.config.num_servos
        for servo_id, power in self.current_powers_mw.items():
            if 0 <= servo_id < self.arm.config.num_servos:
                data[servo_id] = float(power)
        return data

    def stop(self, release: bool = False) -> None:
        """停止补偿控制。

        Args:
            release: True 时释放舵机；False 时恢复基础阻尼。
        """

        if not self.arm.connected:
            return
        if release:
            self.arm.bus.release_all(self.arm.config.servo_ids)
        else:
            self.arm.bus.set_all_damping(self.arm.config.servo_ids, self.damping.base_power_mw)
