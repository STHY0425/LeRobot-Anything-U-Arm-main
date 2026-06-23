from __future__ import annotations

import threading
from typing import Dict, Iterable, Sequence

from .state_types import ServoState


class FashionStarServoBus:
    """通信和协议融合后的华馨京 / FashionStar 舵机总线。

    华馨京官方 SDK 已经封装串口收发和协议帧细节，所以本类不再拆分
    `Communication` 和 `ProtocolDecoder`。上层只看到舵机语义接口：

    - `read_state()` / `read_all_states()` 读取舵机反馈。
    - `set_damping()` / `set_all_damping()` 下发阻尼模式功率。
    - `release_all()` 释放所有舵机。

    Attributes:
        port: 串口设备路径。
        baudrate: 串口波特率。
        timeout: 串口读取超时时间，单位秒。
    """

    def __init__(self, port: str, baudrate: int, timeout: float = 0.0) -> None:
        """初始化舵机总线对象，但不立刻打开串口。

        Args:
            port: 串口设备路径，例如 `/dev/ttyUSB0`。
            baudrate: 串口波特率。
            timeout: 串口读取超时时间，单位秒。
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._uart = None
        self._manager = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        """总线是否已经连接到官方 SDK 管理器。"""

        return self._manager is not None

    def connect(self) -> None:
        """打开串口并创建官方 SDK 管理器。

        SDK 依赖在这里延迟导入，目的是让无硬件/无 SDK 的环境也能导入算法
        模块并运行单元测试。真正连接硬件时仍然直接使用官方 API。

        Raises:
            RuntimeError: 缺少 `pyserial` 或 `fashionstar_uart_sdk` 时抛出。
        """

        if self.connected:
            return
        try:
            import serial
            from fashionstar_uart_sdk import UartServoManager
        except ImportError as exc:
            raise RuntimeError(
                "缺少华馨京/FashionStar 舵机 SDK 或 pyserial，无法连接舵机"
            ) from exc

        self._uart = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=self.timeout,
        )
        self._manager = UartServoManager(self._uart)

    def close(self) -> None:
        """关闭串口并清空 SDK 管理器。

        关闭过程吞掉底层串口异常，避免节点退出时因为二次关闭或设备断开而
        掩盖真正的业务异常。
        """

        with self._lock:
            uart = self._uart
            self._manager = None
            self._uart = None
            if uart is not None:
                try:
                    uart.close()
                except Exception:
                    pass

    def _require_manager(self):
        """获取官方 SDK 管理器。

        Returns:
            已连接的 `UartServoManager` 实例。

        Raises:
            RuntimeError: 总线尚未连接时抛出。
        """

        if self._manager is None:
            raise RuntimeError("FashionStarServoBus is not connected")
        return self._manager

    def read_angle_deg(self, servo_id: int):
        """读取单个舵机角度。

        Args:
            servo_id: 舵机 ID。

        Returns:
            官方 SDK 返回的角度值，单位 degree；读取失败时可能为 None。
        """

        with self._lock:
            return self._require_manager().query_servo_angle(int(servo_id))

    def read_current_a(self, servo_id: int):
        """读取单个舵机电流。

        Args:
            servo_id: 舵机 ID。

        Returns:
            官方 SDK 返回的电流值，单位 A；读取失败时可能为 None。
        """

        with self._lock:
            return self._require_manager().query_current(int(servo_id))

    def read_state(self, servo_id: int, read_current: bool = False) -> ServoState:
        """读取单个舵机状态。

        Args:
            servo_id: 舵机 ID。
            read_current: 是否同步读取电流。

        Returns:
            读取成功时返回在线状态；任何异常或空角度都转换为离线状态。
        """

        try:
            angle = self.read_angle_deg(servo_id)
            if angle is None:
                return ServoState.offline(servo_id)
            current = self.read_current_a(servo_id) if read_current else None
            return ServoState(
                servo_id=int(servo_id),
                angle_deg=float(angle),
                current_a=None if current is None else float(current),
                online=True,
            )
        except Exception:
            return ServoState.offline(servo_id)

    def read_all_states(
        self, servo_ids: Sequence[int], read_current: bool = False
    ) -> Dict[int, ServoState]:
        """批量读取多个舵机状态。

        Args:
            servo_ids: 需要读取的舵机 ID 序列。
            read_current: 是否同步读取电流。

        Returns:
            舵机 ID 到状态对象的映射。
        """

        return {
            int(servo_id): self.read_state(int(servo_id), read_current=read_current)
            for servo_id in servo_ids
        }

    def set_damping(self, servo_id: int, power_mw: int) -> None:
        """设置单个舵机阻尼功率。

        Args:
            servo_id: 舵机 ID。
            power_mw: 阻尼功率，单位 mW。有效范围以官方 SDK/舵机文档为准。
        """

        with self._lock:
            self._require_manager().set_damping(int(servo_id), int(power_mw))

    def set_all_damping(self, servo_ids: Iterable[int], power_mw: int) -> None:
        """对多个舵机设置相同阻尼功率。

        Args:
            servo_ids: 需要设置的舵机 ID。
            power_mw: 阻尼功率，单位 mW。
        """

        for servo_id in servo_ids:
            self.set_damping(int(servo_id), int(power_mw))

    def stop_on_control_mode(self, servo_id: int, method: int = 0x10, power: int = 0) -> None:
        """调用官方停止/控制模式接口。

        Args:
            servo_id: 舵机 ID。
            method: 官方 SDK 的停止方式参数。
            power: 官方 SDK 的功率参数。
        """

        with self._lock:
            self._require_manager().stop_on_control_mode(
                int(servo_id), method=method, power=int(power)
            )

    def release_all(self, servo_ids: Iterable[int]) -> None:
        """释放多个舵机。

        Args:
            servo_ids: 需要释放的舵机 ID。
        """

        for servo_id in servo_ids:
            try:
                self.stop_on_control_mode(int(servo_id), method=0x10, power=0)
            except Exception:
                pass
