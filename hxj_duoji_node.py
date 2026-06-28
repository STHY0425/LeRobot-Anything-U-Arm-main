#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机单文件 ROS 节点框架。

本文件仍然只保留一个 py 文件，但内部按职责分成三个类：

1. DuojiConfig：整理手动配置和读取 config/example.json。
2. RosPublisher：ROS 发布线程，只读共享状态，不访问舵机 SDK。
3. Controller：控制线程，唯一访问串口和 UartServoManager。
"""

import json
import math
import os
import threading
import time

import rospy
import serial
from std_msgs.msg import Float64MultiArray
import fashionstar_uart_sdk as uservo


# 控制状态机的状态名。
#
# 状态机可以理解成：程序每一轮先看自己现在是什么状态，
# 然后进入对应的 handle_xxx() 方法执行一小段逻辑。
STATE_IDLE = "IDLE"        # 空闲：不主动下发运动命令
STATE_MANUAL = "MANUAL"    # 手动：预留给手动调试或单关节控制
STATE_PLAN = "PLAN"        # 规划：预留给逆解、DH 表和控制算法
STATE_MOVE = "MOVE"        # 运动：预留给正式运动控制
STATE_HOLD = "HOLD"        # 保持：预留给位置保持或锁定
STATE_ERROR = "ERROR"      # 错误：预留给停机、释放、等待人工处理


# 机械臂整体参数。
class Arm:
    # 初始化机械臂参数。
    # joint_types 是每个关节的类型字符串列表，取值为 "axial"（轴向）或 "tangential"（切向）。
    # 轴向关节的旋转轴沿连杆纵轴，改变下游切向关节所在平面的朝向，不参与平面 DH 运算。
    # 切向关节的旋转轴垂直于连杆，构成平面 N-R 串联机构，参与 DH 表和阻尼分发。
    def __init__(self, arm_name="", dof=0, joints_length=None,joints_mass=None,actuator_mass=None, joint_types=None):
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

class DuojiConfig:
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
        end_damping=0.0,
        # 轴向舵机的固定基础阻尼功率（mW）。
        # 轴向关节不参与末端阻尼分发，始终保持这个功率值。
        # 500mW 是轻微微阻力，适合让操作者感受到关节存在但不卡顿。
        base_damping_power=500,
        # 单个舵机阻尼功率的硬上限（mW）。
        # 这款舵机单机阻尼上限就是 1000mW，到达后不再增加，
        # 末端合阻尼不足就不足，属于硬件限制。
        max_damping_power=1000,
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
        # 轴向关节固定阻尼功率（mW），不参与末端阻尼分发。
        self.base_damping_power = int(base_damping_power)
        # 阻尼功率安全上限（mW），所有关节的阻尼功率都会被限制在此范围内。
        self.max_damping_power = int(max_damping_power)

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
        # staticmethod 表示这个方法不需要使用 self。
        # 这样既能放在类里面，又能写成 DuojiConfig.load_arm_params_file(path)。
        # 这里使用 staticmethod，是因为“读取某个路径的 JSON”不依赖某一个对象内部状态。

        # with open(...) as file_obj 会在读取结束后自动关闭文件。
        # encoding="utf-8" 用来避免 Windows 上读取中文注释或中文字段时乱码。
        with open(path, "r", encoding="utf-8") as file_obj:
            return DuojiConfig.build_arm_from_dict(json.load(file_obj))

    @staticmethod
    def build_arm_from_dict(raw_params):
        arm_name = str(DuojiConfig.require_key(raw_params, "arm_name", "arm"))
        dof = int(DuojiConfig.require_key(raw_params, "dof", "arm"))
        joints_data = DuojiConfig.require_key(raw_params, "joints", "arm")

        joints_length = []
        joints_mass = []
        actuator_mass = []
        # 关节类型列表，从 JSON 的 "type" 字段读取。
        # 如果某个 joint 没有写 type 字段，默认当作 "tangential"（切向）处理。
        joint_types = []
        for joint_index in range(dof):
            joint_name = "joint%d" % (joint_index + 1)
            joint_data = DuojiConfig.require_key(joints_data, joint_name, "joints")
            link_data = DuojiConfig.require_key(joint_data, "link", joint_name)

            joints_length.append(float(DuojiConfig.require_key(link_data, "length", joint_name + ".link")))
            joints_mass.append(float(DuojiConfig.require_key(link_data, "mass", joint_name + ".link")))
            actuator_mass.append(float(DuojiConfig.require_key(joint_data, "actuator_mass", joint_name)))
            # 读取关节类型，缺省时默认 "tangential"。
            joint_types.append(str(joint_data.get("type", "tangential")))

        return Arm(
            arm_name=arm_name,
            dof=dof,
            joints_length=joints_length,
            joints_mass=joints_mass,
            actuator_mass=actuator_mass,
            joint_types=joint_types,
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
    @staticmethod
    def read_config(
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
        end_damping=0.0,
        base_damping_power=500,
        max_damping_power=1000,
    ):
        config_reader = DuojiConfig(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            servo_ids=servo_ids,
            num_servos=num_servos,
            rate=rate,
            angle_topic=angle_topic,
            current_topic=current_topic,
            release_on_shutdown=release_on_shutdown,
            arm_config=arm_config,
            end_damping=end_damping,
            base_damping_power=base_damping_power,
            max_damping_power=max_damping_power,
        )
        return config_reader.to_dict()

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
            # 轴向关节固定阻尼功率（mW）。
            "base_damping_power": self.base_damping_power,
            # 阻尼功率安全上限（mW）。
            "max_damping_power": self.max_damping_power,
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
            "damping_targets": {},
            "control_state": STATE_IDLE,
            "last_error": "",
            "last_update": 0.0,
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
                "damping_targets": dict(shared_state["damping_targets"]),
                "control_state": shared_state["control_state"],
                "last_error": shared_state["last_error"],
                "last_update": shared_state["last_update"],
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

    # ROS 发布线程的主循环。
    def run(self):
        publishers = self.create_publishers(self.rospy, self.config)
        rate = self.rospy.Rate(self.config["rate"])

        while not self.stop_event.is_set() and not self.rospy.is_shutdown():
            state_copy = self.copy_shared_state(self.shared_state, self.state_lock)
            self.publish_state_arrays(publishers, state_copy)
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

    # 安全读取对象属性，同时兼容对象和字典。
    @staticmethod
    def get_attr(obj, name, default_value):
        # 官方 SDK 返回的数据有可能是对象，也有可能是字典。
        # 字典读取用 obj.get("字段名")，对象读取用 getattr(obj, "属性名")。
        if isinstance(obj, dict):
            return obj.get(name, default_value)
        return getattr(obj, name, default_value)

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

    # 空闲状态处理：当前不下发舵机命令。
    def handle_idle(self, manager):
        return

    # 手动控制状态处理：预留给单关节调试或手动目标角。
    def handle_manual(self, manager):
        return

    # 规划状态处理：预留给后续逆解、DH 表和控制算法。
    def handle_plan(self, manager):
        return

    # 运动执行状态处理：预留给正式运动控制。
    def handle_move(self, manager):
        return

    # 保持状态处理：执行末端阻尼分发，优化操作手感。
    # 这是阻尼控制的核心状态：每个周期根据当前位姿重新计算
    # 各切向关节的阻尼功率并下发，轴向关节保持固定基础阻尼。
    def handle_hold(self, manager):
        self.apply_damping(manager)

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

        if current_state == STATE_IDLE:
            self.handle_idle(manager)
        elif current_state == STATE_MANUAL:
            self.handle_manual(manager)
        elif current_state == STATE_PLAN:
            self.handle_plan(manager)
        elif current_state == STATE_MOVE:
            self.handle_move(manager)
        elif current_state == STATE_HOLD:
            self.handle_hold(manager)
        elif current_state == STATE_ERROR:
            self.handle_error(manager)
        else:
            self.set_control_state(self.shared_state, self.state_lock, STATE_ERROR)

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
    # 切向关节的 DH 参数 a（连杆长度）计算要点：
    # 两个切向关节之间可能夹着若干轴向关节，这些轴向关节的连杆长度
    # 是物理上连接两个切向关节的杆长的一部分，必须累加进去。
    #
    # 例如关节序列: tangential(0.20) → axial(0.15) → tangential(0.10)
    # 第一个切向关节的 a = 0.20 + 0.15 = 0.35（包含中间轴向杆长）
    # 第二个切向关节的 a = 0.10（到末端，后面没有其他关节）
    #
    # 返回三元组 (axial_ids, tangential_ids, tangential_lengths)：
    # - axial_ids: 轴向关节对应的舵机 ID 列表
    # - tangential_ids: 切向关节对应的舵机 ID 列表
    # - tangential_lengths: 各切向关节的有效连杆长度（含中间轴向杆长）
    # servo_ids 的第 i 个元素对应 arm_params 的第 i 个关节。
    @staticmethod
    def classify_joints(arm_params, servo_ids):
        axial_ids = []
        tangential_ids = []
        tangential_lengths = []

        # 先收集所有切向关节在原始关节序列中的索引位置。
        tangential_positions = []
        for i in range(arm_params.dof):
            # 关节类型，如果 joint_types 不够长则默认 "tangential"。
            jtype = arm_params.joint_types[i] if i < len(arm_params.joint_types) else "tangential"
            # 舵机 ID，如果 servo_ids 不够长则用索引值兜底。
            servo_id = servo_ids[i] if i < len(servo_ids) else i

            if jtype == "axial":
                # 轴向关节：旋转轴沿连杆纵轴，不参与平面 DH 运算。
                axial_ids.append(servo_id)
            else:
                # 切向关节：旋转轴垂直于连杆，构成平面 N-R 机构。
                tangential_ids.append(servo_id)
                tangential_positions.append(i)

        # 计算每个切向关节的 DH 参数 a：
        # a_k = 从切向关节 k 到下一个切向关节 k+1 之间所有关节的连杆长度之和。
        # 如果 k 是最后一个切向关节，则累加到 dof 末尾（包含其后所有轴向杆长）。
        for k, pos in enumerate(tangential_positions):
            # 下一个切向关节的位置，如果没有则取 dof（到序列末尾）。
            if k + 1 < len(tangential_positions):
                next_pos = tangential_positions[k + 1]
            else:
                next_pos = arm_params.dof

            # 累加从 pos 到 next_pos-1 的所有关节连杆长度。
            # joints_length[j] 表示关节 j 到关节 j+1 的物理距离。
            a = sum(arm_params.joints_length[j] for j in range(pos, next_pos) if j < len(arm_params.joints_length))
            tangential_lengths.append(a)

        return axial_ids, tangential_ids, tangential_lengths

    # 构建切向关节的 DH 参数表。
    # 每行只有一个有效参数 a（连杆长度），α=0、d=0（平面 N-R 机构）。
    # theta 在运行时从共享状态读取角度并转换为弧度后填入。
    # 返回 list of dict: [{"a": l, "theta": 0.0}, ...]
    # 列表长度 = 切向关节数（2-6），由 tangential_lengths 长度决定。
    @staticmethod
    def build_dh_table(tangential_lengths):
        return [{"a": l, "theta": 0.0} for l in tangential_lengths]

    # 平面 N-R 机构的正运动学。
    # N 个切向关节串联，每个关节转动 θ_i，连杆长 a_i。
    # 累积角 cum_angles[k] = θ_1 + θ_2 + ... + θ_k（弧度）。
    # 末端位置:
    #   px = Σ a_k · cos(cum_angles[k])
    #   py = Σ a_k · sin(cum_angles[k])
    # 返回 (px, py, cum_angles)，cum_angles 是各关节累积角度列表。
    @staticmethod
    def forward_kinematics(dh_table):
        cum_angles = []
        acc = 0.0
        for row in dh_table:
            acc += row["theta"]
            cum_angles.append(acc)

        px = sum(row["a"] * math.cos(a) for row, a in zip(dh_table, cum_angles))
        py = sum(row["a"] * math.sin(a) for row, a in zip(dh_table, cum_angles))
        return px, py, cum_angles

    # 计算 2×N 雅可比矩阵。
    # J 描述末端位置 (px, py) 对各切向关节角度 θ_j 的偏导。
    # 第 j 列的物理含义：关节 j 转动单位角度时，末端移动的方向和距离。
    #
    # 公式（对 θ_j 求偏导，注意 θ_j 影响所有 k>=j 的累积角）：
    #   ∂px/∂θ_j = -Σ_{k>=j} a_k · sin(cum_angles[k])
    #   ∂py/∂θ_j =  Σ_{k>=j} a_k · cos(cum_angles[k])
    #
    # 返回 [[Jx_θ1, Jx_θ2, ...], [Jy_θ1, Jy_θ2, ...]]，形状 2×N。
    @staticmethod
    def compute_jacobian(dh_table, cum_angles):
        n = len(dh_table)
        J = [[0.0] * n for _ in range(2)]
        for j in range(n):
            jx = 0.0
            jy = 0.0
            for k in range(j, n):
                jx += -dh_table[k]["a"] * math.sin(cum_angles[k])
                jy +=  dh_table[k]["a"] * math.cos(cum_angles[k])
            J[0][j] = jx
            J[1][j] = jy
        return J

    # 根据雅可比列范数分发末端阻尼预算到各切向关节。
    #
    # 列范数 s_j = ||J[:,j]|| = sqrt((∂px/∂θ_j)² + (∂py/∂θ_j)²)
    # 物理含义：关节 j 转动单位角度时末端移动的距离（杠杆系数）。
    # s_j 大 → 杠杆高 → 转一点就能大幅移动末端 → 需要更多阻尼功率。
    # s_j 小 → 杠杆低 → 对末端影响小 → 分配较少功率。
    #
    # 分发公式：
    #   w_j = s_j / Σ(s_k)          归一化权重
    #   power_j = end_damping × w_j  分配功率
    #
    # 特性：
    # - 预算守恒：Σ power_j = end_damping
    # - 奇异位姿（所有 s_j 接近零）退化到均匀分配，避免除零
    # - 对任意 N（2-6）同一公式，无需特判
    #
    # 返回 [power_0, power_1, ...]，单位 mW。
    @staticmethod
    def distribute_damping(jacobian, end_damping):
        n = len(jacobian[0])
        if n == 0:
            return []

        # 计算每列的范数。
        norms = []
        for j in range(n):
            s = math.sqrt(jacobian[0][j] ** 2 + jacobian[1][j] ** 2)
            norms.append(s)

        total = sum(norms)
        if total < 1e-9:
            # 所有列范数接近零（完全奇异），均匀分配避免除零。
            return [end_damping / n] * n

        # 归一化权重 × 预算。
        return [end_damping * (s / total) for s in norms]

    # 执行一次完整的阻尼解算和下发。
    # 这是控制线程在 HOLD 状态下每周期调用的主方法。
    # 流程：分类关节 → 构建 DH → 读角度 → FK → Jacobian → 分发 → 下发。
    def apply_damping(self, manager):
        arm = self.config.get("arm_params")
        if arm is None:
            return

        # 1. 分类关节：轴向 / 切向。
        axial_ids, tangential_ids, tangential_lengths = \
            self.classify_joints(arm, self.config["servo_ids"])

        # 2. 构建切向 DH 表。
        dh_table = self.build_dh_table(tangential_lengths)

        if not dh_table:
            # 没有切向关节，全部走轴向固定阻尼。
            base_pw = self.config.get("base_damping_power", 500)
            for sid in axial_ids:
                manager.set_damping(sid, base_pw)
            return

        # 3. 从共享状态读取切向舵机角度，转弧度后填入 DH 表。
        #    加锁读取，读完立即释放，不持锁做后续运算。
        self.state_lock.acquire()
        try:
            for i, servo_id in enumerate(tangential_ids):
                servo_state = self.shared_state["servos"].get(servo_id, {})
                angle_deg = servo_state.get("angle")
                if angle_deg is None:
                    # 角度数据尚未就绪，跳过本轮不下发，等下一周期再试。
                    return
                dh_table[i]["theta"] = math.radians(angle_deg)
        finally:
            self.state_lock.release()

        # 4. 正运动学 → 累积角。
        _, _, cum_angles = self.forward_kinematics(dh_table)

        # 5. 雅可比矩阵（2×N，N = 切向关节数）。
        J = self.compute_jacobian(dh_table, cum_angles)

        # 6. 末端阻尼分发：雅可比列范数权重 → 各切向关节功率。
        end_damping = self.config.get("end_damping", 1000)
        tangential_powers = self.distribute_damping(J, end_damping)

        # 7. 安全限幅参数。
        max_pw = self.config.get("max_damping_power", 1000)
        base_pw = self.config.get("base_damping_power", 500)

        # 8. 下发轴向舵机：固定基础阻尼，不参与分发。
        for sid in axial_ids:
            manager.set_damping(sid, base_pw)

        # 9. 下发切向舵机：分发后的阻尼功率，限幅后取整。
        for i, sid in enumerate(tangential_ids):
            pw = int(min(tangential_powers[i], max_pw))
            manager.set_damping(sid, pw)

    # 控制线程主循环。
    def run(self):
        # rate 是每秒循环次数。
        # 例如 rate=50，则每轮间隔 1/50 = 0.02 秒。
        rate_delay = 1.0 / float(self.config["rate"])

        try:
            # 串口和舵机管理器只在控制线程里打开。
            # ROS 线程不碰这些对象，避免两个线程同时读写串口。
            try:
                self.uart, self.manager = self.open_servo_manager(self.config)
            except Exception as exc:
                self.record_error(exc)
                self.stop_event.set()
                self.rospy.logerr("舵机串口打开失败: %s", exc)
                return
            self.rospy.loginfo("舵机串口已打开: %s @ %d", self.config["port"], self.config["baudrate"])

            # 如果配置了末端阻尼（end_damping > 0），启动时直接切到 HOLD 状态。
            # HOLD 状态下每周期调用 apply_damping()，根据当前位姿分发阻尼功率。
            if self.config.get("end_damping", 0.0) > 0:
                self.set_control_state(self.shared_state, self.state_lock, STATE_HOLD)
                self.rospy.loginfo("末端阻尼已启用: end_damping=%.0f mW, base=%.0f mW, max=%.0f mW",
                                   self.config["end_damping"],
                                   self.config.get("base_damping_power", 500),
                                   self.config.get("max_damping_power", 1000))

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
    # 节点名是 hxj_duoji_node，后面 ROS 运行时会用到这个名字。
    rospy.init_node("hxj_duoji_node")

    # 创建配置读取对象。
    # DuojiConfig 会做两件事：
    # 1. 使用下面这组手动设置的串口、频率、话题名等参数。
    # 2. 默认读取 config/example.json，把机械臂参数放进 config["arm_params"]。
    #    servo_ids 和 num_servos 不传，由 JSON 的 dof 自动派生：
    #    dof=3 → servo_ids=[0,1,2], num_servos=3。
    config_reader = DuojiConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=0.0,
        rate=50.0,
        angle_topic="/servo_angles",
        current_topic="/servo_currents",
        release_on_shutdown=False,
        arm_config=None,
        # 末端合阻尼预算（mW）。切向关节按雅可比列范数权重分配这个预算。
        # 1000mW 是中等阻力，实测后可调。轴向关节不使用此值。
        end_damping=1000,
        # 轴向关节固定阻尼功率（mW），不参与末端阻尼分发。
        base_damping_power=500,
        # 单个舵机阻尼功率硬上限（mW），舵机硬件限制。
        max_damping_power=1000,
    )

    # config 是一个普通字典。
    # 后面的 RosPublisher 和 Controller 都共用这一份配置。
    config = config_reader.values

    # 创建线程共享状态。
    shared_state = config_reader.create_shared_state()

    # 是线程锁。
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

    try:
        # main 主线程不做具体控制，只负责活着等待。
        # 只要 ROS 没有关闭，就每 0.2 秒睡一下，避免空转占 CPU。
        while not rospy.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        # 如果用户按 Ctrl+C，会进入这里。
        # 这里不直接做复杂处理，交给 finally 统一通知线程退出。
        pass
    finally:
        # 通知两个子线程准备退出。
        stop_event.set()

        # 等 ROS 线程最多 2 秒。
        # join() 的意思是等待线程结束。
        # 写 2.0 是为了避免某个线程卡住时，主程序永远退不出去。
        ros_thread.join(2.0)

        # 等控制线程最多 2 秒。
        # 控制线程退出时会在自己的 finally 里关闭串口。
        control_thread.join(2.0)


if __name__ == "__main__":
    main()
