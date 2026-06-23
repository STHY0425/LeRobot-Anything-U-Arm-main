from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


class RobotDynamics:
    """FeelForce 风格的重力力矩计算器。

    该类只做动力学计算，不依赖舵机、ROS 或控制器。存在 URDF 和 Pinocchio 时，
    使用 `computeGeneralizedGravity()` 计算广义重力项；不可用时保留错误信息，
    由上层决定是否退化为自适应阻尼。

    Args:
        dof: 默认自由度。成功加载 URDF 后会被 URDF 模型的 `nq` 覆盖。
        urdf_path: URDF 文件路径。为空时不尝试加载动力学模型。
        compensation_gain: 重力补偿增益。1.0 表示完整补偿，0.5 表示半补偿。

    Attributes:
        dof: 当前动力学模型自由度。
        compensation_gain: 重力补偿增益。
        urdf_path: 当前加载的 URDF 路径。
        load_error: 最近一次 URDF/Pinocchio 加载失败原因；成功时为 None。
    """

    def __init__(
        self,
        dof: int,
        urdf_path: str = "",
        compensation_gain: float = 1.0,
    ) -> None:
        self.dof = int(dof)
        self.compensation_gain = float(compensation_gain)
        self.urdf_path = urdf_path
        self.load_error: Optional[str] = None
        self._pin = None
        self._model = None
        self._data = None

        if urdf_path:
            self._try_load_urdf(urdf_path)

    def _try_load_urdf(self, urdf_path: str) -> None:
        """尝试从 URDF 构建 Pinocchio 固定基座模型。

        Args:
            urdf_path: URDF 文件路径。
        """

        try:
            import pinocchio as pin

            self._model = pin.buildModelFromUrdf(urdf_path)
            self._data = self._model.createData()
            self._pin = pin
            self.dof = int(self._model.nq)
            self.load_error = None
        except Exception as exc:
            self._pin = None
            self._model = None
            self._data = None
            self.load_error = str(exc)

    @property
    def available(self) -> bool:
        """动力学模型是否可用。"""

        return self._pin is not None and self._model is not None and self._data is not None

    def compute_gravity_torque(self, q_rad: List[float]) -> List[float]:
        """计算当前关节角下的广义重力力矩。

        Args:
            q_rad: 关节角数组，长度必须等于 `dof`，单位 rad。

        Returns:
            每个关节的重力力矩，单位 Nm。如果动力学模型不可用，返回全 0。

        Raises:
            ValueError: `q_rad` 长度与模型自由度不一致时抛出。
        """

        if not self.available:
            return [0.0] * self.dof
        if len(q_rad) != self.dof:
            raise ValueError("q_rad length does not match dynamics dof")

        import numpy as np

        tau = self._pin.computeGeneralizedGravity(
            self._model, self._data, np.array(q_rad, dtype=float)
        )
        return [float(value) * self.compensation_gain for value in tau]


@dataclass(frozen=True)
class DampingMap:
    """把力矩或角度变化映射为舵机阻尼功率。

    华馨京舵机没有直接力矩控制接口，因此动力学补偿力矩只能近似映射为
    `set_damping()` 的功率值。该映射不是闭环力矩控制，第一次上硬件应使用
    保守的 `base_power_mw` 和 `torque_gain_mw_per_nm`。

    Attributes:
        base_power_mw: 基础阻尼功率，单位 mW。
        max_power_mw: 最大阻尼功率，单位 mW。
        torque_gain_mw_per_nm: 每 1 Nm 补偿力矩增加的阻尼功率。
        angle_gain_mw_per_deg: 自适应模式下每 1 degree 角度变化增加的阻尼功率。
        release_delta_deg: 自适应模式下释放阻尼的反向角度变化阈值。
    """

    base_power_mw: int = 100
    max_power_mw: int = 3000
    torque_gain_mw_per_nm: float = 100.0
    angle_gain_mw_per_deg: float = 20.0
    release_delta_deg: float = -2.0

    def clamp(self, power_mw: float) -> int:
        """把功率限制在 `[0, max_power_mw]`。

        Args:
            power_mw: 待限制的功率，单位 mW。

        Returns:
            限幅后的整数功率。
        """

        return int(max(0, min(round(power_mw), self.max_power_mw)))

    def torque_to_power(self, torque_nm: float) -> int:
        """把补偿力矩映射为阻尼功率。

        Args:
            torque_nm: 补偿力矩，单位 Nm。正负号只表示方向，阻尼功率取绝对值。

        Returns:
            阻尼功率，单位 mW。
        """

        return self.clamp(self.base_power_mw + abs(torque_nm) * self.torque_gain_mw_per_nm)

    def angle_delta_to_power(self, delta_deg: float, previous_power_mw: int) -> int:
        """自适应阻尼模式下，根据角度变化更新阻尼功率。

        Args:
            delta_deg: 当前角度相对上一帧的变化，单位 degree。
            previous_power_mw: 上一帧阻尼功率，单位 mW。

        Returns:
            新阻尼功率，单位 mW。
        """

        if delta_deg > 0.0:
            target = self.base_power_mw + delta_deg * self.angle_gain_mw_per_deg
            return max(previous_power_mw, self.clamp(target))
        if delta_deg <= self.release_delta_deg:
            return self.clamp(self.base_power_mw)
        return self.clamp(previous_power_mw)
