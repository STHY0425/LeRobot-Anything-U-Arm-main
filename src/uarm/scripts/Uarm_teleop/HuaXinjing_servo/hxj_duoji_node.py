#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机单文件 ROS 节点框架。

本文件仍然只保留一个 py 文件，但内部按职责分成三个类：

1. ServoConfig：整理手动配置和读取 config/example.json。
2. RosPublisher：ROS 发布线程，只读共享状态，不访问舵机 SDK。
3. Controller：控制线程，唯一访问串口和 UartServoManager。

核心功能：基于 3D DH 运动学的非对称三档动态阻尼控制。
- 所有关节（含轴向）参与 3D 运动学，确保末端位置和速度准确。
- 轴向关节固定 base_damping_power，不参与末端阻尼分发。
- 切向关节按雅可比列范数权重分发三档末端合阻尼预算。
- 三档预算（end_damping_low/mid/high）为固定值，调试时改 DEFAULTS 字典即可。
- 根据末端竖直速度 vz 判断顺逆重力，自动切换三档。
"""

import json
import math
import os
import threading
import time

import numpy as np
import rospy # type: ignore
import serial
from std_msgs.msg import Float64MultiArray # type: ignore
import fashionstar_uart_sdk as uservo # type: ignore

from key_hold_thread import KeyHoldThread


# 控制状态机的状态名。
STATE_START = "START"      # 启动：同步回零 + 终端交互，等待用户选择目标状态
STATE_IDLE = "IDLE"        # 空闲：不主动下发运动命令
STATE_HOLD = "HOLD"        # 阻尼保持：3D 三档动态阻尼控制（核心）
STATE_LOCKED = "LOCKED"    # 锁死：所有舵机保持锁力，完全固定
STATE_ERROR = "ERROR"      # 错误：预留给停机、释放、等待人工处理


# 全局默认配置，
DEFAULTS = {
    # 三档末端合阻尼预算（mW），切向关节按雅可比列范数权重分发这个预算。
    "end_damping": 1000,       # 末端合阻尼预算（兼容旧字段，用作是否启用阻尼的开关）
    "end_damping_low": 600,    # 低档：逆重力（上抬）时用，值小，用户推起来不费力。
    "end_damping_mid": 1000,   # 中档：水平/静止时用，正常手感。
    "end_damping_high": 1500,  # 高档：顺重力（下坠）时用，值大，托住机械臂。
    # 轴向关节固定阻尼功率（mW），不参与末端阻尼分发。
    "base_damping_power": 500,
    # 单个舵机阻尼功率的硬上限（mW），舵机硬件限制。
    "max_damping_power": 1000,
    # 顺逆重力判定阈值（m/s），末端竖直速度超过此值才切换档位。
    "vz_threshold": 0.01,
}


# 机械臂整体参数。
class Arm:
    # 初始化机械臂参数。
    # joint_types 是每个关节的类型字符串列表，取值为 "axial"（轴向）或 "tangential"（切向）。
    # 轴向关节的旋转轴沿连杆纵轴，改变下游切向关节所在平面的朝向。
    # 切向关节的旋转轴垂直于连杆，构成平面 N-R 串联机构。
    # joints_twist 是 DH 参数 α（扭转角，弧度），控制旋转轴方向变化：
    #   切向 α=0（同平面），轴向 α=π/2（平面翻转 90°）。
    # joints_offset 是 DH 参数 d（z 轴偏移），通常为 0。
    def __init__(self, arm_name="", dof=0, joints_length=None,joints_mass=None,actuator_mass=None, joint_types=None, joints_twist=None, joints_offset=None):
        self.arm_name = arm_name
        self.dof = int(dof)
        if joints_length is None:
            joints_length = []
        self.joints_length = joints_length
        if joints_mass is None:
            joints_mass = []
        self.joints_mass = joints_mass
        if actuator_mass is None:
            actuator_mass = []
        self.actuator_mass = actuator_mass
        # 关节类型列表，长度等于 dof，元素为 "axial" 或 "tangential"。
        # 阻尼分发逻辑根据这个字段把关节分成两组分别处理。
        if joint_types is None:
            joint_types = []
        self.joint_types = joint_types
        # DH 参数 α（扭转角，弧度）。
        # 切向 α=0（同平面），轴向 α=π/2（平面翻转 90°）。
        # 3D 运动学需要这个参数来跟踪轴向旋转导致的平面偏转。
        if joints_twist is None:
            joints_twist = []
        self.joints_twist = joints_twist
        # DH 参数 d（z 轴偏移），通常为 0。
        if joints_offset is None:
            joints_offset = []
        self.joints_offset = joints_offset

class ServoConfig:
    # 初始化配置读取类。
    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=0.0,
        servo_ids=None,
        num_servos=None,
        rate=50.0,
        angle_topic="/servo_angles",
        current_topic="/servo_currents",
        release_on_shutdown=False,
        arm_config=None,
        end_damping=DEFAULTS["end_damping"],
        end_damping_low=DEFAULTS["end_damping_low"],
        end_damping_mid=DEFAULTS["end_damping_mid"],
        end_damping_high=DEFAULTS["end_damping_high"],
        base_damping_power=DEFAULTS["base_damping_power"],
        max_damping_power=DEFAULTS["max_damping_power"],
        vz_threshold=DEFAULTS["vz_threshold"],
    ):
        # servo_ids 和 num_servos 默认不传，由 JSON 的 dof 动态决定。
        # 如果用户手动传了值，则以用户值为准（覆盖 JSON）。
        user_servo_ids = servo_ids
        user_num_servos = num_servos

        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.rate = float(rate)
        if self.rate <= 0.0:
            raise ValueError("rate 必须大于 0")
        self.angle_topic = angle_topic
        self.current_topic = current_topic
        self.release_on_shutdown = self.parse_bool(release_on_shutdown)
        self.arm_config = self.default_arm_config() if arm_config is None else arm_config
        self.arm_params = None
        self.end_damping = float(end_damping)
        # 三档末端合阻尼预算（mW），切向关节按雅可比权重分发。
        self.end_damping_low = int(end_damping_low)
        self.end_damping_mid = int(end_damping_mid)
        self.end_damping_high = int(end_damping_high)
        # 轴向关节固定阻尼功率（mW），不参与末端阻尼分发。
        self.base_damping_power = int(base_damping_power)
        # 阻尼功率安全上限（mW），所有关节的阻尼功率都会被限制在此范围内。
        self.max_damping_power = int(max_damping_power)
        # 顺逆重力判定阈值（m/s）。
        self.vz_threshold = float(vz_threshold)

        # 先加载 JSON 机械臂参数，拿到 dof 后再派生 servo_ids 和 num_servos。
        self.load_arm_params()

        if user_servo_ids is not None:
            # 用户手动传了 servo_ids，以用户值为准。
            self.servo_ids = self.parse_int_list(user_servo_ids)
        elif self.arm_params is not None:
            # 没传 servo_ids，从 JSON 的 dof 自动生成 [0, 1, ..., dof-1]。
            self.servo_ids = list(range(self.arm_params.dof))
        else:
            # JSON 也没加载到，空列表兜底。
            self.servo_ids = []

        if user_num_servos is not None:
            # 用户手动传了 num_servos，以用户值为准。
            self.num_servos = int(user_num_servos)
        else:
            # 没传，跟 servo_ids 长度一致。
            self.num_servos = len(self.servo_ids)

        self.values = self.to_dict()

    # 返回默认机械臂 JSON 配置路径。
    @staticmethod
    def default_arm_config():
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "example.json")

    # 读取机械臂 JSON 参数文件。
    @staticmethod
    def load_arm_params_file(path):

        # with open(...) as file_obj 会在读取结束后自动关闭文件。
        # encoding="utf-8" 用来避免 Windows 上读取中文注释或中文字段时乱码。
        with open(path, "r", encoding="utf-8") as file_obj:
            return ServoConfig.build_arm_from_dict(json.load(file_obj))

    @staticmethod
    def build_arm_from_dict(raw_params):
        arm_name = str(ServoConfig.require_key(raw_params, "arm_name", "arm"))
        dof = int(ServoConfig.require_key(raw_params, "dof", "arm"))
        joints_data = ServoConfig.require_key(raw_params, "joints", "arm")

        joints_length = []
        joints_mass = []
        actuator_mass = []
        # 关节类型列表，从 JSON 的 "type" 字段读取。
        # 如果某个 joint 没有写 type 字段，默认当作 "tangential"（切向）处理。
        joint_types = []
        # DH 参数 α（扭转角，弧度）和 d（z 轴偏移）。
        # 3D 运动学需要这两个参数来跟踪轴向旋转导致的平面偏转。
        joints_twist = []
        joints_offset = []
        for joint_index in range(dof):
            joint_name = "joint%d" % (joint_index + 1)
            joint_data = ServoConfig.require_key(joints_data, joint_name, "joints")
            link_data = ServoConfig.require_key(joint_data, "link", joint_name)

            joints_length.append(float(ServoConfig.require_key(link_data, "length", joint_name + ".link")))
            joints_mass.append(float(link_data.get("mass", 0.0)))
            actuator_mass.append(float(joint_data.get("actuator_mass", 0.0)))
            # 读取关节类型，缺省时默认 "tangential"。
            jtype = str(joint_data.get("type", "tangential"))
            joint_types.append(jtype)

            # 读取 α 扭转角：优先读 JSON 的 twist 字段，缺省时按关节类型自动填充。
            # 切向 α=0（同平面），轴向 α=π/2（旋转轴翻转 90°，平面偏转）。
            twist = link_data.get("twist", None)
            if twist is None:
                twist = math.pi / 2 if jtype == "axial" else 0.0
            joints_twist.append(float(twist))

            # 读取 d 偏移：优先读 JSON 的 offset 字段，缺省 0。
            offset = link_data.get("offset", 0.0)
            joints_offset.append(float(offset))

        return Arm(
            arm_name=arm_name,
            dof=dof,
            joints_length=joints_length,
            joints_mass=joints_mass,
            actuator_mass=actuator_mass,
            joint_types=joint_types,
            joints_twist=joints_twist,
            joints_offset=joints_offset,
        )

    # 读取 JSON 必填字段，缺字段时直接报错中断。
    @staticmethod
    def require_key(data, key, owner):
        if not isinstance(data, dict):
            raise ValueError("%s 必须是 JSON 对象" % owner)
        if key not in data:
            raise ValueError("%s 缺少必填字段: %s" % (owner, key))
        return data[key]

    # 解析手动配置里的整数列表，支持数字、列表和 "[0,1,2]" 字符串。
    @staticmethod
    def parse_int_list(raw_value):
        if raw_value is None:
            return []
        if isinstance(raw_value, int):
            return [raw_value]
        if isinstance(raw_value, str):
            text = raw_value.strip()
            text = text.strip("[")
            text = text.strip("]")
            if text == "":
                return []
            result = []
            for item in text.split(","):
                item = item.strip()
                if item != "":
                    result.append(int(item))
            return result

        result = []
        for item in raw_value:
            result.append(int(item))
        return result

    # 解析手动配置里的布尔值，支持 bool、数字和常见字符串。
    @staticmethod
    def parse_bool(raw_value):
        if isinstance(raw_value, bool):
            return raw_value
        if isinstance(raw_value, int) or isinstance(raw_value, float):
            return raw_value != 0
        if isinstance(raw_value, str):
            text = raw_value.strip().lower()
            if text in ("true", "1", "yes", "y", "on"):
                return True
            if text in ("false", "0", "no", "n", "off", ""):
                return False
            raise ValueError("无法解析布尔参数: %s" % raw_value)
        return bool(raw_value)

    # 按手动设置整理成普通配置字典。
    # 透传所有参数给 ServoConfig，阻尼参数默认值由 DEFAULTS 统一管理。
    @staticmethod
    def read_config(**kwargs):
        return ServoConfig(**kwargs).values

    # 按 arm_config 路径读取机械臂 JSON 参数。
    def load_arm_params(self):
        if self.arm_config != "":
            self.arm_params = self.load_arm_params_file(self.arm_config)

    # 导出普通字典，兼容 RosPublisher 和 Controller。
    def to_dict(self):
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": self.timeout,
            "servo_ids": list(self.servo_ids),
            "num_servos": self.num_servos,
            "rate": self.rate,
            "angle_topic": self.angle_topic,
            "current_topic": self.current_topic,
            "release_on_shutdown": self.release_on_shutdown,
            "arm_config": self.arm_config,
            "arm_params": self.arm_params,
            "end_damping": self.end_damping,
            # 三档末端合阻尼预算（mW）。
            "end_damping_low": self.end_damping_low,
            "end_damping_mid": self.end_damping_mid,
            "end_damping_high": self.end_damping_high,
            # 轴向关节固定阻尼功率（mW）。
            "base_damping_power": self.base_damping_power,
            # 阻尼功率安全上限（mW）。
            "max_damping_power": self.max_damping_power,
            # 顺逆重力判定阈值（m/s）。
            "vz_threshold": self.vz_threshold,
        }

    # 根据舵机 ID 和舵机数量创建共享状态。
    @staticmethod
    def create_shared_state_from_values(servo_ids, num_servos):
        servos = {}
        for servo_id in servo_ids:
            servos[int(servo_id)] = {
                "id": int(servo_id),
                "angle": None,
                "current": 0.0,
                "voltage": 0.0,
                "power": 0.0,
                "temp": 0.0,
                "status": 0,
                "turn": 0.0,
                "online": False,
                "stamp": 0.0,
            }

        return {
            "servo_ids": list(servo_ids),
            "num_servos": int(num_servos),
            "servos": servos,
            "control_state": STATE_START,
            "last_error": "",
            "last_update": 0.0,
            # 遥测字段：由 Controller 写入，RosPublisher 读取发布。
            "end_velocity": [0.0, 0.0, 0.0],  # [vx, vy, vz] m/s
            "damping_mode": 0,                 # 0=非HOLD, 1=low, 2=mid, 3=high
            "damping_powers": {},              # {servo_id: power_mW}
        }

    # 根据当前配置创建线程共享状态。
    def create_shared_state(self):
        return self.create_shared_state_from_values(self.servo_ids, self.num_servos)

# ROS 线程类：只负责发布 ROS 话题，不访问串口和舵机 SDK。
class RosPublisher:
    STATE_TOPICS = (
        ("angle", "angle_topic", "/servo_angles"),
        ("current", "current_topic", "/servo_currents"),
        ("voltage", "voltage_topic", "/servo_voltages"),
        ("power", "power_topic", "/servo_powers"),
        ("temp", "temp_topic", "/servo_temps"),
        ("status", "status_topic", "/servo_statuses"),
        ("turn", "turn_topic", "/servo_turns"),
        ("online", "online_topic", "/servo_online"),
    )

    # 遥测话题：数据不在 servos 子表里，独立发布。
    TELEMETRY_TOPICS = (
        ("end_velocity", "/servo_end_velocity"),    # [vx, vy, vz] m/s
        ("damping_mode", "/servo_damping_mode"),    # [mode] 0/1/2/3
        ("damping_powers", "/servo_damping_powers"), # [pw0, pw1, ...] mW
    )

    # 保存 ROS 线程需要用到的对象。
    def __init__(self, config, shared_state, state_lock, stop_event, rospy):
        self.config = config
        self.shared_state = shared_state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.rospy = rospy

    # 生成某个状态字段的话题数组。
    @staticmethod
    def make_state_array(shared_state, field_name):
        data = []
        for _ in range(shared_state["num_servos"]):
            data.append(0.0)

        for servo_id, servo_state in shared_state["servos"].items():
            if servo_id >= 0 and servo_id < len(data):
                value = servo_state.get(field_name)
                if value is None:
                    value = 0.0
                if isinstance(value, bool):
                    value = 1.0 if value else 0.0
                data[servo_id] = float(value)
        return data

    # 复制一份共享状态给 ROS 线程读取。
    @staticmethod
    def copy_shared_state(shared_state, state_lock):
        state_lock.acquire()
        try:
            copied = {
                "servo_ids": list(shared_state["servo_ids"]),
                "num_servos": shared_state["num_servos"],
                "servos": {},
                "control_state": shared_state["control_state"],
                "last_error": shared_state["last_error"],
                "last_update": shared_state["last_update"],
                # 遥测字段一并拷贝，避免 ROS 线程读到脏数据。
                "end_velocity": list(shared_state.get("end_velocity", [0.0, 0.0, 0.0])),
                "damping_mode": shared_state.get("damping_mode", 0),
                "damping_powers": dict(shared_state.get("damping_powers", {})),
            }
            for servo_id, servo_state in shared_state["servos"].items():
                copied["servos"][servo_id] = dict(servo_state)
            return copied
        finally:
            state_lock.release()

    # 创建每类舵机状态对应的 ROS 数组发布器。
    @staticmethod
    def create_publishers(rospy, config):
        publishers = {}
        for field_name, config_key, default_topic in RosPublisher.STATE_TOPICS:
            topic = config.get(config_key, default_topic)
            publishers[field_name] = rospy.Publisher(topic, Float64MultiArray, queue_size=10)
        return publishers

    # 把所有舵机状态按字段分别发布成数组。
    @staticmethod
    def publish_state_arrays(publishers, shared_state):
        for field_name in publishers:
            publishers[field_name].publish(Float64MultiArray(data=RosPublisher.make_state_array(shared_state, field_name)))

    # 创建遥测话题对应的 ROS 发布器。
    @staticmethod
    def create_telemetry_publishers(rospy):
        publishers = {}
        for field_name, default_topic in RosPublisher.TELEMETRY_TOPICS:
            publishers[field_name] = rospy.Publisher(default_topic, Float64MultiArray, queue_size=10)
        return publishers

    # 发布遥测数据（末端速度、阻尼档位、阻尼功率）。
    @staticmethod
    def publish_telemetry(telemetry_pubs, shared_state):
        # 末端速度 [vx, vy, vz]
        telemetry_pubs["end_velocity"].publish(
            Float64MultiArray(data=list(shared_state.get("end_velocity", [0.0, 0.0, 0.0])))
        )
        # 阻尼档位 [mode]
        telemetry_pubs["damping_mode"].publish(
            Float64MultiArray(data=[float(shared_state.get("damping_mode", 0))])
        )
        # 阻尼功率，按 servo_id 索引排列
        num = shared_state["num_servos"]
        pw_data = [0.0] * num
        for sid, pw in shared_state.get("damping_powers", {}).items():
            if 0 <= sid < num:
                pw_data[sid] = float(pw)
        telemetry_pubs["damping_powers"].publish(
            Float64MultiArray(data=pw_data)
        )

    # ROS 发布线程的主循环。
    def run(self):
        publishers = self.create_publishers(self.rospy, self.config)
        telemetry_pubs = self.create_telemetry_publishers(self.rospy)
        rate = self.rospy.Rate(self.config["rate"])

        while not self.stop_event.is_set() and not self.rospy.is_shutdown():
            state_copy = self.copy_shared_state(self.shared_state, self.state_lock)
            self.publish_state_arrays(publishers, state_copy)
            self.publish_telemetry(telemetry_pubs, state_copy)
            rate.sleep()

# 控制线程类：唯一访问串口和 UartServoManager 的地方。
class Controller:
    # 保存控制线程需要用到的对象。
    def __init__(self, config, shared_state, state_lock, stop_event, rospy):
        self.config = config
        self.shared_state = shared_state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.rospy = rospy
        self.uart = None
        self.manager = None

        # 上一周期的所有关节角度（弧度），用于差分计算角速度。
        # 第一周期为 None，此时速度为 0，走中档。
        self.prev_angles = None
        # 上一周期的时间戳，用于计算实际 dt。
        self.prev_timestamp = None

        # LOCKED 状态触发标志，本期默认 False。
        # 后续接入外部传感器（如按钮等）后由外部设置。
        self.lock_requested = False
        # 从 LOCKED 解锁标志，同理后续外部触发。
        self.unlock_requested = False

        # START 状态是否已完成回零和交互。
        # 进入 START 后只执行一次回零+交互，之后等待用户选择目标状态。
        self._start_done = False

    # 安全读取对象属性，同时兼容对象和字典。
    # None 当作缺失处理：getattr 在属性存在但值为 None 时返回 None 而非 default，
    # dict.get 同理，所以取到 None 后手动替换为 default_value。
    @staticmethod
    def get_attr(obj, name, default_value):
        if isinstance(obj, dict):
            value = obj.get(name, default_value)
        else:
            value = getattr(obj, name, default_value)
        return default_value if value is None else value

    # 把官方同步监控数据转换成普通字典，供共享状态和状态机使用。
    @staticmethod
    def build_servo_state(servo_id, monitor_data):
        angle = Controller.get_attr(monitor_data, "angle_monitor", None)
        if angle is None:
            angle = Controller.get_attr(monitor_data, "angle", None)

        return {
            "id": int(servo_id),
            "angle": None if angle is None else float(angle),
            "current": float(Controller.get_attr(monitor_data, "current", 0.0)),
            "voltage": float(Controller.get_attr(monitor_data, "voltage", 0.0)),
            "power": float(Controller.get_attr(monitor_data, "power", 0.0)),
            "temp": float(Controller.get_attr(monitor_data, "temp", 0.0)),
            "status": int(Controller.get_attr(monitor_data, "status", 0)),
            "turn": float(Controller.get_attr(monitor_data, "turn", 0.0)),
            "online": angle is not None,
            "stamp": time.time(),
        }

    # 从官方 SDK 管理器缓存里取单个舵机对象。
    @staticmethod
    def get_servo_from_manager(manager, servo_id):
        if not hasattr(manager, "servos"):
            return None

        servos = manager.servos
        if hasattr(servos, "get"):
            return servos.get(servo_id)

        try:
            return servos[servo_id]
        except Exception:
            return None

    # 打开串口并创建官方 SDK 管理器。
    @staticmethod
    def open_servo_manager(config):
        uart = serial.Serial(
            port=config["port"],
            baudrate=config["baudrate"],
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=config["timeout"],
        )
        manager = uservo.UartServoManager(uart)
        return uart, manager

    # 使用官方同步读取 API 读取一组舵机状态。
    @staticmethod
    def read_sync_monitor(manager, servo_ids):
        result = manager.send_sync_servo_monitor(servo_ids)
        states = {}

        for servo_id in servo_ids:
            monitor_data = None
            if isinstance(result, dict):
                monitor_data = result.get(servo_id)
                if monitor_data is None:
                    raise RuntimeError("舵机 %d 同步监控读取失败" % int(servo_id))
            elif result is not None and hasattr(result, "__iter__"):
                for item in result:
                    item_id = Controller.get_attr(item, "servo_id", None)
                    if item_id is None:
                        item_id = Controller.get_attr(item, "id", None)
                    if item_id == servo_id:
                        monitor_data = item
                        break

            if monitor_data is None:
                monitor_data = Controller.get_servo_from_manager(manager, servo_id)

            if monitor_data is None:
                raise RuntimeError("舵机 %d 同步监控读取失败" % int(servo_id))

            states[int(servo_id)] = Controller.build_servo_state(servo_id, monitor_data)

        return states

    # 把控制线程读到的舵机状态写入共享状态。
    @staticmethod
    def update_shared_servo_state(shared_state, state_lock, new_states):
        # shared_state 是 ROS 线程和控制线程共用的字典。
        # 只要要写它，就先 acquire() 加锁，写完 finally 里 release() 解锁。
        state_lock.acquire()
        try:
            for servo_id, servo_state in new_states.items():
                shared_state["servos"][int(servo_id)] = servo_state
            shared_state["last_update"] = time.time()
        finally:
            state_lock.release()

    # 切换控制状态机状态。
    @staticmethod
    def set_control_state(shared_state, state_lock, new_state):
        state_lock.acquire()
        try:
            shared_state["control_state"] = new_state
        finally:
            state_lock.release()

    # 启动状态：同步回零 + 终端交互。
    # 进入后只执行一次：
    # 1. 用 set_servo_angle 把所有舵机转到 0°，wait() 阻塞等待到位。
    # 2. 通过终端 input() 与用户交互，等待用户选择目标状态。
    # 3. 切换到用户选择的状态。
    # 控制线程在 START 期间被 input() 阻塞是合理的——
    # START 本身就是等用户指令，不需要 50Hz 轮询。
    def handle_start(self, manager):
        if self._start_done:
            return

        servo_ids = self.config["servo_ids"]

        # 1. 同步回零：逐个下发 0° 目标，interval=2000ms 给足转向时间。
        if self.rospy is not None:
            self.rospy.loginfo("STATE_START: 开始同步回零（%d 个舵机 → 0°）", len(servo_ids))
        for servo_id in servo_ids:
            try:
                manager.set_servo_angle(int(servo_id), 0.0, interval=2000)
            except Exception as exc:
                if self.rospy is not None:
                    self.rospy.logerr("舵机 %d 回零失败: %s", servo_id, exc)

        # 2. 等待舵机转到目标位置（SDK 没有 wait()，用 sleep 替代）。
        if self.rospy is not None:
            self.rospy.loginfo("STATE_START: 等待舵机到位（2.5s）")
        time.sleep(2.5)

        if self.rospy is not None:
            self.rospy.loginfo("STATE_START: 回零完成，等待用户选择目标状态")

        # 3. 终端交互：用户选择下一个状态。
        next_state = self._prompt_for_target_state()

        # 4. 切换状态，标记 START 已完成。
        self.set_control_state(self.shared_state, self.state_lock, next_state)
        self._start_done = False  # 下次再进 START 可以重新执行

        if self.rospy is not None:
            self.rospy.loginfo("STATE_START: 已切换到 %s", next_state)

    # 终端交互：提示用户选择目标状态，返回状态名字符串。
    def _prompt_for_target_state(self):
        choices = {
            "1": (STATE_HOLD, "阻尼保持（3D 动态阻尼）"),
            "2": (STATE_LOCKED, "锁死（所有舵机固定）"),
            "3": (STATE_IDLE, "空闲（不下发命令）"),
        }
        prompt_lines = [
            "",
            "========================================",
            "  请选择目标状态：",
        ]
        for key, (state, desc) in choices.items():
            prompt_lines.append("  %s. %s — %s" % (key, state, desc))
        prompt_lines.append("========================================")
        prompt_text = "\n".join(prompt_lines) + "\n请输入选项编号: "

        while True:
            try:
                choice = input(prompt_text).strip()
            except EOFError:
                # 非交互环境（如管道/后台）默认进 IDLE，避免卡死。
                print("非交互环境，默认进入 IDLE")
                return STATE_IDLE
            if choice in choices:
                return choices[choice][0]
            print("无效选项 '%s'，请重新输入" % choice)

    # 空闲状态处理：当前不下发舵机命令。
    def handle_idle(self, manager):
        return

    # 阻尼保持状态：3D 三档动态阻尼控制（核心）。
    # 每周期根据末端 3D 速度的 z 分量（vz）判断顺逆重力，
    # 切向关节分低/中/高三档阻尼，轴向关节固定基础阻尼。
    def handle_hold(self, manager):
        self.apply_dynamic_damping(manager)

    # 锁死状态：所有舵机保持锁力，完全固定.
    # 利用 SDK 的 stop_on_control_mode(method=0x11) 保持锁力。
    # 每周期重复调用是幂等的，舵机收到相同指令不会报错。
    def handle_locked(self, manager):
        for servo_id in self.config["servo_ids"]:
            try:
                manager.stop_on_control_mode(int(servo_id), method=0x11, power=500)
            except Exception:
                pass

    # 错误保护状态处理：预留给停机、释放或等待人工复位。
    def handle_error(self, manager):
        return

    # 执行一次控制状态机分发。
    def run_state_machine_once(self, manager):
        # 这里只拿锁读取 control_state，读完马上释放。
        # 不要拿着锁去执行 handle_xxx()，否则 ROS 线程可能等太久。
        self.state_lock.acquire()
        try:
            current_state = self.shared_state["control_state"]
        finally:
            self.state_lock.release()

        # 检查 lock/unlock 请求（本期 flag 默认 False，后续接外部传感器）。
        if self.lock_requested and current_state == STATE_HOLD:
            self.set_control_state(self.shared_state, self.state_lock, STATE_LOCKED)
            self.lock_requested = False
            return
        if self.unlock_requested and current_state == STATE_LOCKED:
            self.set_control_state(self.shared_state, self.state_lock, STATE_HOLD)
            self.unlock_requested = False
            return

        if current_state == STATE_START:
            self.handle_start(manager)
            self._clear_telemetry()
        elif current_state == STATE_IDLE:
            self.handle_idle(manager)
            self._clear_telemetry()
        elif current_state == STATE_HOLD:
            self.handle_hold(manager)
        elif current_state == STATE_LOCKED:
            self.handle_locked(manager)
            self._clear_telemetry()
        elif current_state == STATE_ERROR:
            self.handle_error(manager)
            self._clear_telemetry()
        else:
            self.set_control_state(self.shared_state, self.state_lock, STATE_ERROR)
            self._clear_telemetry()

    # 释放所有舵机。
    def release_servos(self, manager):
        for servo_id in self.config["servo_ids"]:
            try:
                manager.stop_on_control_mode(int(servo_id), method=0x10, power=0)
            except Exception:
                pass

    # ===== 末端阻尼分发方法 =====
    # 以下方法实现从 JSON 机械臂参数到舵机阻尼功率的完整解算链路：
    #
    # JSON → classify_joints（分类 axial/tangential）
    #     → build_dh_table（构建切向 DH 表）
    #     → forward_kinematics（正运动学，累积角）
    #     → compute_jacobian（2×N 雅可比矩阵）
    #     → distribute_damping（雅可比列范数权重分发）
    #     → apply_damping（下发 set_damping）
    #
    # 核心设计原则：
    # 1. 阻尼功率直接是 mW，不经过力矩转换。这款舵机的 set_damping
    #    只能延缓角度变化，不是输出力矩。
    # 2. 模块化可塑性：3-7 轴任意 axial/tangential 构型自动适配，
    #    DH 表行数 = 切向关节数，雅可比列数 = 切向关节数。
    # 3. 轴向关节不参与分发，固定 base_damping_power。
    # 4. 切向关节按雅可比列范数分配 end_damping 预算，预算守恒。

    # 根据 arm_params.joint_types 分类舵机 ID。
    # 遍历每个关节，读取类型字符串，把轴向和切向分别收集。
    #
    # 3D 运动学已包含所有关节（含轴向），DH 参数 a 直接取 joints_length[i]，
    # 中间轴向杆长自然包含在 3D DH 表里，不再需要单独算"切向有效杆长"。
    #
    # 返回三元组 (axial_ids, tangential_ids, tangential_indices)：
    # - axial_ids: 轴向关节对应的舵机 ID 列表
    # - tangential_ids: 切向关节对应的舵机 ID 列表
    # - tangential_indices: 切向关节在原始关节序列中的下标，用于从 3D 雅可比中提取切向列
    # servo_ids 的第 i 个元素对应 arm_params 的第 i 个关节。
    @staticmethod
    def classify_joints(arm_params, servo_ids):
        axial_ids = []
        tangential_ids = []
        tangential_indices = []

        for i in range(arm_params.dof):
            # 关节类型，如果 joint_types 不够长则默认 "tangential"。
            jtype = arm_params.joint_types[i] if i < len(arm_params.joint_types) else "tangential"
            # 舵机 ID，如果 servo_ids 不够长则用索引值兜底。
            servo_id = servo_ids[i] if i < len(servo_ids) else i

            if jtype == "axial":
                # 轴向关节：旋转轴沿连杆纵轴，不参与阻尼分发。
                axial_ids.append(servo_id)
            else:
                # 切向关节：旋转轴垂直于连杆，参与阻尼分发。
                tangential_ids.append(servo_id)
                tangential_indices.append(i)

        return axial_ids, tangential_ids, tangential_indices

    # ===== 3D 运动学方法 =====
    # 以下方法实现 3D DH 运动学，包含所有关节（含轴向）。
    # 轴向关节的 α=π/2（旋转轴翻转 90°，平面偏转），
    # 切向关节的 α=0（同平面）。
    # 3D 运动学确保底座/中间 axial 旋转后末端位置和速度准确，
    # 从而正确判断顺逆重力。

    # 单个 DH 关节的 4×4 齐次变换矩阵。
    # T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
    @staticmethod
    def dh_transform(a, alpha, d, theta):
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        return np.array([
            [ct,       -st * ca,  st * sa,  a * ct],
            [st,        ct * ca, -ct * sa,  a * st],
            [0.0,       sa,       ca,       d     ],
            [0.0,       0.0,      0.0,      1.0   ],
        ])

    # 构建 3D DH 参数表（所有关节，含轴向）。
    # 返回 list of dict: [{"a":, "alpha":, "d":, "theta":}, ...]
    # 长度 = arm_params.dof（3-7），由 JSON 决定。
    @staticmethod
    def build_dh_table_3d(arm_params):
        dh_table = []
        for i in range(arm_params.dof):
            a = arm_params.joints_length[i] if i < len(arm_params.joints_length) else 0.0
            alpha = arm_params.joints_twist[i] if i < len(arm_params.joints_twist) else 0.0
            d = arm_params.joints_offset[i] if i < len(arm_params.joints_offset) else 0.0
            dh_table.append({"a": a, "alpha": alpha, "d": d, "theta": 0.0})
        return dh_table

    # 3D 正运动学。
    # 所有关节的 4×4 变换矩阵连乘，得到末端位姿。
    # 返回:
    # - T_total: 4×4 齐次变换矩阵（末端位姿）
    # - joint_transforms: 每个关节的累积变换矩阵列表（用于雅可比求 z 轴和关节原点）
    @staticmethod
    def forward_kinematics_3d(dh_table):
        T_total = np.eye(4)
        joint_transforms = []

        for row in dh_table:
            T_i = Controller.dh_transform(row["a"], row["alpha"], row["d"], row["theta"])
            T_total = T_total @ T_i
            # 保存到当前关节为止的累积变换（用于雅可比求 z 轴和关节原点）。
            joint_transforms.append(T_total.copy())

        return T_total, joint_transforms

    # 计算 3×N 雅可比矩阵。
    # J[:,j] = z_j × (p_end - p_j)
    # 其中 z_j 是第 j 个关节的旋转轴方向，p_j 是第 j 个关节原点位置。
    # 返回 np.array，形状 3×N。
    @staticmethod
    def compute_jacobian_3d(joint_transforms, p_end):
        n = len(joint_transforms)
        J = np.zeros((3, n))

        px_end, py_end, pz_end = p_end

        for j in range(n):
            # 第 j 个关节的旋转轴方向 z_j = T_j 的第三列前三行。
            zx = joint_transforms[j][0, 2]
            zy = joint_transforms[j][1, 2]
            zz = joint_transforms[j][2, 2]

            # 第 j 个关节的原点位置 p_j = T_{j-1} 的平移部分。
            # j=0 时原点在世界原点 (0,0,0)。
            if j == 0:
                px_j, py_j, pz_j = 0.0, 0.0, 0.0
            else:
                px_j = joint_transforms[j - 1][0, 3]
                py_j = joint_transforms[j - 1][1, 3]
                pz_j = joint_transforms[j - 1][2, 3]

            # J[:,j] = z_j × (p_end - p_j)
            J[0, j] = zy * (pz_end - pz_j) - zz * (py_end - py_j)
            J[1, j] = zz * (px_end - px_j) - zx * (pz_end - pz_j)
            J[2, j] = zx * (py_end - py_j) - zy * (px_end - px_j)

        return J

    # 从 3D 雅可比矩阵中提取切向列。
    # J_3d 是 3×N_total，提取 tangential_indices 对应的列 → 3×N_tan。
    # 确保阻尼分发只使用切向列，轴向列不参与。
    @staticmethod
    def extract_tangential_jacobian(J_3d, tangential_indices):
        return J_3d[:, tangential_indices]

    # 阻尼分发。
    # 列范数 s_j = sqrt(Jx² + Jy² + Jz²)
    # 权重公式不变：w_j = s_j / Σ(s_k)，power_j = end_damping × w_j。
    # 返回 np.array，单位 mW，只分给切向关节。
    @staticmethod
    def distribute_damping_3d(J_tan, end_damping):
        n = J_tan.shape[1]
        if n == 0:
            return np.array([])

        # 计算每列的 3D 范数。
        norms = np.sqrt(J_tan[0, :] ** 2 + J_tan[1, :] ** 2 + J_tan[2, :] ** 2)

        total = np.sum(norms)
        if total < 1e-9:
            # 所有列范数接近零（完全奇异），均匀分配避免除零。
            return np.full(n, end_damping / n)

        # 归一化权重 × 预算。
        return end_damping * (norms / total)

    # 把遥测数据写入共享状态（末端速度、阻尼档位、阻尼功率）。
    # 一次锁写入三个字段，减少锁竞争。
    def _write_telemetry(self, v_end, mode_code, damping_powers):
        self.state_lock.acquire()
        try:
            self.shared_state["end_velocity"] = [float(v_end[0]), float(v_end[1]), float(v_end[2])]
            self.shared_state["damping_mode"] = int(mode_code)
            self.shared_state["damping_powers"] = dict(damping_powers)
        finally:
            self.state_lock.release()

    # 非 HOLD 状态时清零遥测，避免脏数据残留。
    def _clear_telemetry(self):
        self.state_lock.acquire()
        try:
            self.shared_state["end_velocity"] = [0.0, 0.0, 0.0]
            self.shared_state["damping_mode"] = 0
            self.shared_state["damping_powers"] = {}
        finally:
            self.state_lock.release()

    # ===== 三档动态阻尼 =====
    # 执行一次完整的 3D 动态阻尼解算和下发。
    # 这是控制线程在 HOLD 状态下每周期调用的主方法。
    # 流程：
    # 1. 分类关节 → 2. 构建 3D DH → 3. 读角度 → 4. 3D FK → 5. 3D Jacobian
    # 6. 角度差分 → 末端速度 vz → 判断顺逆重力 → 三档切换 → 下发 → 写遥测
    def apply_dynamic_damping(self, manager):
        arm = self.config.get("arm_params")
        if arm is None:
            return

        # 1. 分类关节：轴向 / 切向。
        axial_ids, tangential_ids, tangential_indices = \
            self.classify_joints(arm, self.config["servo_ids"])

        # 2. 构建 3D DH 表（所有关节，含轴向 α=π/2）。
        dh_table = self.build_dh_table_3d(arm)

        # 3. 从共享状态读取所有舵机角度，转弧度后填入 DH 表。
        #    加锁读取，读完立即释放，不持锁做后续运算。
        self.state_lock.acquire()
        try:
            for i in range(len(dh_table)):
                servo_id = self.config["servo_ids"][i] if i < len(self.config["servo_ids"]) else i
                servo_state = self.shared_state["servos"].get(servo_id, {})
                angle_deg = servo_state.get("angle")
                if angle_deg is None:
                    # 角度数据尚未就绪，跳过本轮不下发，等下一周期再试。
                    return
                dh_table[i]["theta"] = math.radians(angle_deg)
        finally:
            self.state_lock.release()

        # 4. 3D 正运动学 → 末端位置 + 各关节累积变换。
        T_total, joint_transforms = self.forward_kinematics_3d(dh_table)
        p_end = np.array([T_total[0, 3], T_total[1, 3], T_total[2, 3]])

        # 5. 3D 雅可比矩阵（3×N_total，含所有关节列）。
        J_3d = self.compute_jacobian_3d(joint_transforms, p_end)

        # 6. 角度差分 → 关节角速度 → 末端速度。
        current_angles = np.array([row["theta"] for row in dh_table])
        current_time = time.time()

        if self.prev_angles is not None and self.prev_timestamp is not None:
            dt = current_time - self.prev_timestamp
            if dt > 1e-6:
                omegas = (current_angles - self.prev_angles) / dt
            else:
                omegas = np.zeros_like(current_angles)
        else:
            # 第一周期没有历史数据，速度为零，走中档。
            omegas = np.zeros_like(current_angles)

        # 末端速度 v = J_3d · ω → (vx, vy, vz)
        v_end = J_3d @ omegas
        vz = v_end[2]

        # 8. 三档末端合阻尼预算（mW）
        # 三档是末端总预算，切向关节按雅可比列范数权重分发这个预算。
        end_damping_low = self.config.get("end_damping_low", DEFAULTS["end_damping_low"])
        end_damping_mid = self.config.get("end_damping_mid", DEFAULTS["end_damping_mid"])
        end_damping_high = self.config.get("end_damping_high", DEFAULTS["end_damping_high"])
        max_pw = self.config.get("max_damping_power", DEFAULTS["max_damping_power"])
        base_pw = self.config.get("base_damping_power", DEFAULTS["base_damping_power"])
        vz_threshold = self.config.get("vz_threshold", DEFAULTS["vz_threshold"])

        # 9. 轴向舵机：固定基础阻尼，不参与三档分发。
        for sid in axial_ids:
            manager.set_damping(sid, base_pw)

        # 10. 切向舵机：三档末端预算，按雅可比权重分发到各关节。
        # 先确定档位码（0=非HOLD后的默认, 1=low, 2=mid, 3=high）。
        if vz > vz_threshold:
            mode_code = 1
        elif vz < -vz_threshold:
            mode_code = 3
        else:
            mode_code = 2

        # 收集每个舵机的目标阻尼功率，写入遥测。
        damping_powers = {}
        for sid in axial_ids:
            damping_powers[sid] = base_pw

        if not tangential_ids:
            # 没有切向关节，只记轴向功率，下发后直接返回。
            for sid in axial_ids:
                manager.set_damping(sid, damping_powers[sid])
            self._write_telemetry(v_end, mode_code, damping_powers)
            # 保存本周期角度和时间戳，供下周期差分使用。
            self.prev_angles = current_angles
            self.prev_timestamp = current_time
            return

        # 提取切向列 → 阻尼分发只用切向列，轴向列不参与。
        J_tan = self.extract_tangential_jacobian(J_3d, tangential_indices)

        if mode_code == 1:
            # 低档：逆重力（末端向上），固定低预算。
            weights = self.distribute_damping_3d(J_tan, end_damping_low)
        elif mode_code == 3:
            # 高档：顺重力（末端向下），固定高预算。
            weights = self.distribute_damping_3d(J_tan, end_damping_high)
        else:
            # 中档：水平/静止，固定中预算。
            weights = self.distribute_damping_3d(J_tan, end_damping_mid)

        for i, sid in enumerate(tangential_ids):
            pw = int(min(weights[i], max_pw))
            damping_powers[sid] = pw
            manager.set_damping(sid, pw)

        # 11. 写入遥测数据到共享状态。
        self._write_telemetry(v_end, mode_code, damping_powers)

        # 12. 保存本周期角度和时间戳，供下周期差分使用。
        self.prev_angles = current_angles
        self.prev_timestamp = current_time

    # 控制线程主循环。
    def run(self):
        rate_delay = 1.0 / float(self.config["rate"])

        try:
            # 串口和舵机管理器只在控制线程里打开。
            try:
                self.uart, self.manager = self.open_servo_manager(self.config)
            except Exception as exc:
                self.record_error(exc)
                self.stop_event.set()
                self.rospy.logerr("舵机串口打开失败: %s", exc)
                return
            self.rospy.loginfo("舵机串口已打开: %s @ %d", self.config["port"], self.config["baudrate"])

            # 启动时默认进入 START 状态：同步回零 + 终端交互。
            self.rospy.loginfo("启动状态: STATE_START（同步回零 + 终端交互）")

            while not self.stop_event.is_set() and not self.rospy.is_shutdown():
                try:
                    new_states = self.read_sync_monitor(self.manager, self.config["servo_ids"])
                    self.update_shared_servo_state(self.shared_state, self.state_lock, new_states)
                    self.run_state_machine_once(self.manager)
                except Exception as exc:
                    self.record_error(exc)
                    self.stop_event.set()
                    self.rospy.logerr("控制线程异常: %s", exc)
                    return

                time.sleep(rate_delay)
        finally:
            if self.manager is not None and self.config["release_on_shutdown"]:
                self.release_servos(self.manager)
            if self.uart is not None:
                try:
                    self.uart.close()
                except Exception:
                    pass

    # 记录控制线程错误并切换到 ERROR 状态。
    def record_error(self, exc):
        self.state_lock.acquire()
        try:
            self.shared_state["last_error"] = str(exc)
            self.shared_state["control_state"] = STATE_ERROR
        finally:
            self.state_lock.release()

# 启动一个后台线程。
def start_thread(name, target, args):
    thread = threading.Thread(name=name, target=target, args=args)
    thread.daemon = True
    thread.start()
    return thread

# 程序入口，负责装配线程和管理生命周期。
def main():
    # 初始化 ROS 节点。
    rospy.init_node("hxj_duoji_node")

    # 创建配置读取对象。
    config_reader = ServoConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=0.0,
        rate=50.0,
        angle_topic="/servo_angles",
        current_topic="/servo_currents",
        release_on_shutdown=False,
        arm_config=None,
    )

    # 后面的 RosPublisher 和 Controller 都共用这一份配置。
    config = config_reader.values

    # 创建线程共享状态。
    shared_state = config_reader.create_shared_state()
    # 线程锁。
    state_lock = threading.Lock()
    # 停止信号。
    stop_event = threading.Event()

    # 创建 ROS 线程对象。
    ros_publisher = RosPublisher(config, shared_state, state_lock, stop_event, rospy)
    # 创建控制线程对象。
    controller = Controller(config, shared_state, state_lock, stop_event, rospy)

    # 启动 ROS 发布线程。
    ros_thread = start_thread("ros_thread", ros_publisher.run, ())
    # 启动控制线程。
    control_thread = start_thread("control_thread", controller.run, ())

    # 启动按键锁定/解锁线程（按住 'k' 解锁 HOLD，松开锁定 LOCKED）。
    # 运行前需在终端执行 xset r rate 250 40。
    key_thread = KeyHoldThread(
        controller=controller,
        shared_state=shared_state,
        state_lock=state_lock,
        stop_event=stop_event,
        rospy=rospy,
    )
    start_thread("key_hold_thread", key_thread.run, ())

    try:
        # main 主线程不做具体控制，只负责活着等待。
        # 只要 ROS 没有关闭，就每 0.2 秒睡一下，避免空转占 CPU。
        while not rospy.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()

        # join() 等待线程结束，写 2.0 是为了避免某个线程卡住时，主程序永远退不出去。
        ros_thread.join(2.0)

        control_thread.join(2.0)


if __name__ == "__main__":
    main()
