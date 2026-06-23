#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机单文件 ROS 节点框架。

本文件故意把结构写得直接一些，方便后续继续补状态机和控制算法。
当前只保留三个线程角色：

1. main 主线程：创建资源、启动线程、负责退出。
2. ROS 线程：发布 ROS 话题，不直接访问舵机 SDK。
3. 控制线程：访问舵机 SDK，同步读取状态，运行控制状态机。
"""

import threading
import time


STATE_IDLE = "IDLE"
STATE_MANUAL = "MANUAL"
STATE_PLAN = "PLAN"
STATE_MOVE = "MOVE"
STATE_HOLD = "HOLD"
STATE_ERROR = "ERROR"


def parse_int_list(raw_value):
    """解析 ROS 参数里的整数列表。

    支持三种常见形式：
    - 直接传单个数字，例如 3
    - Python 列表，例如 [0, 1, 2]
    - 字符串列表，例如 "[0,1,2]"
    """

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


def get_attr(obj, name, default_value):
    """安全读取对象属性。

    官方 SDK 的同步监控数据可能是对象，也可能随版本变化成字典。
    这里做一个很薄的兼容层，避免状态机代码关心这些细节。
    """

    if isinstance(obj, dict):
        return obj.get(name, default_value)
    return getattr(obj, name, default_value)


def build_servo_state(servo_id, monitor_data):
    """把官方同步监控数据转换成普通字典。

    字段保持简单，后续控制状态机直接读这些 key。
    """

    angle = get_attr(monitor_data, "angle_monitor", None)
    if angle is None:
        angle = get_attr(monitor_data, "angle", None)

    return {
        "id": int(servo_id),
        "angle": None if angle is None else float(angle),
        "current": float(get_attr(monitor_data, "current", 0.0)),
        "voltage": float(get_attr(monitor_data, "voltage", 0.0)),
        "power": float(get_attr(monitor_data, "power", 0.0)),
        "temp": float(get_attr(monitor_data, "temp", 0.0)),
        "status": int(get_attr(monitor_data, "status", 0)),
        "online": angle is not None,
        "stamp": time.time(),
    }


def get_servo_from_manager(manager, servo_id):
    """从官方 SDK 管理器缓存里取单个舵机对象。"""

    if not hasattr(manager, "servos"):
        return None

    servos = manager.servos
    if hasattr(servos, "get"):
        return servos.get(servo_id)

    try:
        return servos[servo_id]
    except Exception:
        return None


def create_shared_state(servo_ids, num_servos):
    """创建线程之间共享的状态字典。"""

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
            "online": False,
            "stamp": 0.0,
        }

    return {
        "servo_ids": list(servo_ids),
        "num_servos": int(num_servos),
        "servos": servos,
        "control_state": STATE_IDLE,
        "last_error": "",
        "last_update": 0.0,
    }


def make_angle_array(shared_state):
    """生成 /servo_angles 话题需要的数组。

    数组下标仍然对应舵机 ID。没有数据的位置填 0。
    当前版本不做零点标定，直接发布同步读取到的角度。
    """

    data = []
    for _ in range(shared_state["num_servos"]):
        data.append(0.0)

    for servo_id, servo_state in shared_state["servos"].items():
        if servo_id >= 0 and servo_id < len(data):
            angle = servo_state.get("angle")
            if angle is not None:
                data[servo_id] = float(angle)
    return data


def make_current_array(shared_state):
    """生成 /servo_currents 话题需要的数组。"""

    data = []
    for _ in range(shared_state["num_servos"]):
        data.append(0.0)

    for servo_id, servo_state in shared_state["servos"].items():
        if servo_id >= 0 and servo_id < len(data):
            data[servo_id] = float(servo_state.get("current", 0.0))
    return data


def read_ros_config(rospy):
    """读取 ROS 参数。

    这里返回普通字典，不额外拆类，后续想改参数名也比较直观。
    """

    servo_ids = parse_int_list(rospy.get_param("~servo_ids", [0, 1, 2, 3, 4, 5, 6]))
    num_servos = int(rospy.get_param("~num_servos", 7))

    return {
        "port": rospy.get_param("~port", "/dev/ttyUSB0"),
        "baudrate": int(rospy.get_param("~baudrate", 115200)),
        "timeout": float(rospy.get_param("~timeout", 0.0)),
        "servo_ids": servo_ids,
        "num_servos": num_servos,
        "rate": float(rospy.get_param("~rate", 50.0)),
        "angle_topic": rospy.get_param("~angle_topic", "/servo_angles"),
        "current_topic": rospy.get_param("~current_topic", "/servo_currents"),
        "read_current": bool(rospy.get_param("~read_current", False)),
        "release_on_shutdown": bool(rospy.get_param("~release_on_shutdown", False)),
    }


def open_servo_manager(config):
    """打开串口并创建官方 SDK 管理器。"""

    import serial
    import fashionstar_uart_sdk as uservo

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


def read_sync_monitor(manager, servo_ids):
    """使用官方同步读取 API 读取一组舵机状态。

    官方示例调用后从 manager.servos[id] 取数据。官方文档也描述该 API 会返回
    同步监控数据，所以这里同时兼容返回值和 manager.servos 两种形式。
    """

    result = manager.send_sync_servo_monitor(servo_ids)
    states = {}

    for servo_id in servo_ids:
        monitor_data = None
        if isinstance(result, dict):
            monitor_data = result.get(servo_id)
        elif result is not None and hasattr(result, "__iter__"):
            for item in result:
                item_id = get_attr(item, "servo_id", None)
                if item_id is None:
                    item_id = get_attr(item, "id", None)
                if item_id == servo_id:
                    monitor_data = item
                    break

        if monitor_data is None:
            monitor_data = get_servo_from_manager(manager, servo_id)

        if monitor_data is None:
            states[int(servo_id)] = {
                "id": int(servo_id),
                "angle": None,
                "current": 0.0,
                "voltage": 0.0,
                "power": 0.0,
                "temp": 0.0,
                "status": 0,
                "online": False,
                "stamp": time.time(),
            }
        else:
            states[int(servo_id)] = build_servo_state(servo_id, monitor_data)

    return states


def update_shared_servo_state(shared_state, state_lock, new_states):
    """把控制线程读到的舵机状态写入共享状态。"""

    state_lock.acquire()
    try:
        for servo_id, servo_state in new_states.items():
            shared_state["servos"][int(servo_id)] = servo_state
        shared_state["last_update"] = time.time()
    finally:
        state_lock.release()


def set_control_state(shared_state, state_lock, new_state):
    """切换控制状态机状态。"""

    state_lock.acquire()
    try:
        shared_state["control_state"] = new_state
    finally:
        state_lock.release()


def copy_shared_state(shared_state, state_lock):
    """复制一份共享状态给 ROS 线程读取。"""

    state_lock.acquire()
    try:
        copied = {
            "servo_ids": list(shared_state["servo_ids"]),
            "num_servos": shared_state["num_servos"],
            "servos": {},
            "control_state": shared_state["control_state"],
            "last_error": shared_state["last_error"],
            "last_update": shared_state["last_update"],
        }
        for servo_id, servo_state in shared_state["servos"].items():
            copied["servos"][servo_id] = dict(servo_state)
        return copied
    finally:
        state_lock.release()


def handle_idle(manager, config, shared_state, state_lock):
    """空闲状态。

    这里暂时不下发舵机命令。后续可以在这里处理待机、安全释放等逻辑。
    """

    return


def handle_manual(manager, config, shared_state, state_lock):
    """手动控制状态。

    后续可以从命令队列取单个舵机目标角，并调用 set_servo_angle。
    """

    return


def handle_plan(manager, config, shared_state, state_lock):
    """规划状态。

    这里保留机械臂逆解或轨迹准备入口，当前不做具体计算。
    """

    return


def handle_move(manager, config, shared_state, state_lock):
    """运动执行状态。

    后续根据规划结果选择控制方案，并下发舵机角度命令。
    """

    return


def handle_hold(manager, config, shared_state, state_lock):
    """保持状态。

    后续可以锁定当前位置，或周期性修正目标角。
    """

    return


def handle_error(manager, config, shared_state, state_lock):
    """错误保护状态。

    后续可以在这里释放舵机、停止运动或等待人工复位。
    """

    return


def run_state_machine_once(manager, config, shared_state, state_lock):
    """执行一次控制状态机。"""

    state_lock.acquire()
    try:
        current_state = shared_state["control_state"]
    finally:
        state_lock.release()

    if current_state == STATE_IDLE:
        handle_idle(manager, config, shared_state, state_lock)
    elif current_state == STATE_MANUAL:
        handle_manual(manager, config, shared_state, state_lock)
    elif current_state == STATE_PLAN:
        handle_plan(manager, config, shared_state, state_lock)
    elif current_state == STATE_MOVE:
        handle_move(manager, config, shared_state, state_lock)
    elif current_state == STATE_HOLD:
        handle_hold(manager, config, shared_state, state_lock)
    elif current_state == STATE_ERROR:
        handle_error(manager, config, shared_state, state_lock)
    else:
        set_control_state(shared_state, state_lock, STATE_ERROR)


def release_servos(manager, servo_ids):
    """释放所有舵机。"""

    for servo_id in servo_ids:
        try:
            manager.stop_on_control_mode(int(servo_id), method=0x10, power=0)
        except Exception:
            pass


def control_thread_main(config, shared_state, state_lock, stop_event, rospy):
    """控制线程入口。

    本线程是唯一访问舵机 SDK 的线程。
    """

    uart = None
    manager = None
    rate_delay = 1.0 / float(config["rate"])

    try:
        uart, manager = open_servo_manager(config)
        rospy.loginfo("舵机串口已打开: %s @ %d", config["port"], config["baudrate"])

        while not stop_event.is_set() and not rospy.is_shutdown():
            try:
                new_states = read_sync_monitor(manager, config["servo_ids"])
                update_shared_servo_state(shared_state, state_lock, new_states)
                run_state_machine_once(manager, config, shared_state, state_lock)
            except Exception as exc:
                state_lock.acquire()
                try:
                    shared_state["last_error"] = str(exc)
                    shared_state["control_state"] = STATE_ERROR
                finally:
                    state_lock.release()
                rospy.logerr("控制线程异常: %s", exc)

            time.sleep(rate_delay)
    finally:
        if manager is not None and config["release_on_shutdown"]:
            release_servos(manager, config["servo_ids"])
        if uart is not None:
            try:
                uart.close()
            except Exception:
                pass


def ros_thread_main(config, shared_state, state_lock, stop_event, rospy):
    """ROS 发布线程入口。"""

    from std_msgs.msg import Float64MultiArray

    angle_pub = rospy.Publisher(config["angle_topic"], Float64MultiArray, queue_size=10)
    current_pub = None
    if config["read_current"]:
        current_pub = rospy.Publisher(config["current_topic"], Float64MultiArray, queue_size=10)

    rate = rospy.Rate(config["rate"])
    rospy.loginfo("角度话题发布到: %s", config["angle_topic"])

    while not stop_event.is_set() and not rospy.is_shutdown():
        state_copy = copy_shared_state(shared_state, state_lock)
        angle_pub.publish(Float64MultiArray(data=make_angle_array(state_copy)))
        if current_pub is not None:
            current_pub.publish(Float64MultiArray(data=make_current_array(state_copy)))
        rate.sleep()


def start_thread(name, target, args):
    """启动一个后台线程。"""

    thread = threading.Thread(name=name, target=target, args=args)
    thread.daemon = True
    thread.start()
    return thread


def main():
    """程序入口。"""

    import rospy

    rospy.init_node("hxj_duoji_node")
    config = read_ros_config(rospy)
    shared_state = create_shared_state(config["servo_ids"], config["num_servos"])
    state_lock = threading.Lock()
    stop_event = threading.Event()

    ros_thread = start_thread(
        "ros_thread",
        ros_thread_main,
        (config, shared_state, state_lock, stop_event, rospy),
    )
    control_thread = start_thread(
        "control_thread",
        control_thread_main,
        (config, shared_state, state_lock, stop_event, rospy),
    )

    try:
        while not rospy.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ros_thread.join(2.0)
        control_thread.join(2.0)


if __name__ == "__main__":
    main()
