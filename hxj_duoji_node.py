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


# 控制状态机的状态名。
#
# 这里先用字符串常量，不用 enum，是为了保持代码直接、好改、好打印。
# 后续你在控制线程里补具体策略时，可以只围绕这些状态写 if/elif。
STATE_IDLE = "IDLE"        # 空闲：不主动下发运动命令
STATE_MANUAL = "MANUAL"    # 手动：预留给手动调试或单关节控制
STATE_PLAN = "PLAN"        # 规划：预留给逆解、轨迹准备
STATE_MOVE = "MOVE"        # 运动：预留给正式运动控制
STATE_HOLD = "HOLD"        # 保持：预留给位置保持或锁定
STATE_ERROR = "ERROR"      # 错误：预留给停机、释放、等待人工处理


# 解析 ROS 参数里的整数列表，支持数字、列表和 "[0,1,2]" 字符串。
def parse_int_list(raw_value):
    # ROS 参数有时会从 launch 文件里以字符串形式传进来，比如 "[0,1,2]"。
    # 这里集中解析，后面的代码只处理真正的 int 列表。
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


# 安全读取对象属性，同时兼容对象和字典。
def get_attr(obj, name, default_value):
    # 官方 SDK 的同步监控数据可能是对象，也可能随版本变化成字典。
    # 这里做一个很薄的兼容层，避免状态机代码关心这些细节。
    if isinstance(obj, dict):
        return obj.get(name, default_value)
    return getattr(obj, name, default_value)


# 把官方同步监控数据转换成普通字典，供共享状态和状态机使用。
def build_servo_state(servo_id, monitor_data):
    # 字段保持简单，后续控制状态机直接读这些 key。
    # 官方同步监控示例里角度字段是 angle_monitor。
    # 这里额外兼容 angle，是为了不同 SDK 版本字段名变化时更容易跑起来。
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


# 从官方 SDK 管理器缓存里取单个舵机对象。
def get_servo_from_manager(manager, servo_id):
    if not hasattr(manager, "servos"):
        return None

    # 本地示例里使用 manager.servos[id] 读取；
    # 有些 SDK/封装可能让 servos 表现得像 dict，所以同时兼容 .get()。
    servos = manager.servos
    if hasattr(servos, "get"):
        return servos.get(servo_id)

    try:
        return servos[servo_id]
    except Exception:
        return None


# 创建线程之间共享的状态字典。
def create_shared_state(servo_ids, num_servos):
    # shared_state 是 ROS 线程和控制线程之间唯一共享的数据。
    # 访问它时必须配合 state_lock，避免一个线程读到另一个线程写到一半的数据。
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


# 生成 /servo_angles 话题需要的数组。
def make_angle_array(shared_state):
    # 数组下标仍然对应舵机 ID。没有数据的位置填 0。
    # 当前版本不做零点标定，直接发布同步读取到的角度。
    # ROS 的 Float64MultiArray 是数组，不是 dict。
    # 为了和原工程保持一致，数组下标仍然直接使用舵机 ID。
    data = []
    for _ in range(shared_state["num_servos"]):
        data.append(0.0)

    for servo_id, servo_state in shared_state["servos"].items():
        if servo_id >= 0 and servo_id < len(data):
            angle = servo_state.get("angle")
            if angle is not None:
                data[servo_id] = float(angle)
    return data


# 生成 /servo_currents 话题需要的数组。
def make_current_array(shared_state):
    data = []
    for _ in range(shared_state["num_servos"]):
        data.append(0.0)

    for servo_id, servo_state in shared_state["servos"].items():
        if servo_id >= 0 and servo_id < len(data):
            data[servo_id] = float(servo_state.get("current", 0.0))
    return data


# 读取 ROS 参数并整理成普通字典。
def read_ros_config(rospy):
    # 这里返回普通字典，不额外拆类，后续想改参数名也比较直观。
    # 这里只读取当前框架真正需要的参数。
    # 旧工程里的 URDF、重力补偿、关节方向等参数已经故意移除。
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


# 打开串口并创建官方 SDK 管理器。
def open_servo_manager(config):
    # 串口和 SDK 延迟导入，方便 Windows 上做纯 Python 测试。
    # 真正运行 ROS 节点时，控制线程会调用这里。
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


# 使用官方同步读取 API 读取一组舵机状态。
def read_sync_monitor(manager, servo_ids):
    # 官方示例调用后从 manager.servos[id] 取数据。官方文档也描述该 API 会返回
    # 同步监控数据，所以这里同时兼容返回值和 manager.servos 两种形式。
    # 这是本工程读取舵机状态的主路径：
    # 一次同步读取多个舵机，避免逐个 query_servo_angle/query_current。
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


# 把控制线程读到的舵机状态写入共享状态。
def update_shared_servo_state(shared_state, state_lock, new_states):
    # 不使用 with state_lock 是为了让代码对 Python 初学者更直观：
    # acquire() 和 release() 成对出现，finally 保证异常时也会释放锁。
    state_lock.acquire()
    try:
        for servo_id, servo_state in new_states.items():
            shared_state["servos"][int(servo_id)] = servo_state
        shared_state["last_update"] = time.time()
    finally:
        state_lock.release()


# 切换控制状态机状态。
def set_control_state(shared_state, state_lock, new_state):
    state_lock.acquire()
    try:
        shared_state["control_state"] = new_state
    finally:
        state_lock.release()


# 复制一份共享状态给 ROS 线程读取。
def copy_shared_state(shared_state, state_lock):
    # ROS 线程发布前先复制一份快照，复制完成就释放锁。
    # 这样发布话题不会长期占用锁，也不会阻塞控制线程读舵机。
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


# 空闲状态处理：当前不下发舵机命令。
def handle_idle(manager, config, shared_state, state_lock):
    # 后续可以在这里处理待机、安全释放等逻辑。
    return


# 手动控制状态处理：预留给单关节调试或手动目标角。
def handle_manual(manager, config, shared_state, state_lock):
    # 后续可以从命令队列取单个舵机目标角，并调用 set_servo_angle。
    return


# 规划状态处理：预留给机械臂逆解或轨迹准备。
def handle_plan(manager, config, shared_state, state_lock):
    # 这里保留机械臂逆解或轨迹准备入口，当前不做具体计算。
    return


# 运动执行状态处理：预留给正式运动控制。
def handle_move(manager, config, shared_state, state_lock):
    # 后续根据规划结果选择控制方案，并下发舵机角度命令。
    return


# 保持状态处理：预留给位置保持或锁定当前位置。
def handle_hold(manager, config, shared_state, state_lock):
    # 后续可以锁定当前位置，或周期性修正目标角。
    return


# 错误保护状态处理：预留给停机、释放或等待人工复位。
def handle_error(manager, config, shared_state, state_lock):
    # 后续可以在这里释放舵机、停止运动或等待人工复位。
    return


# 执行一次控制状态机分发。
def run_state_machine_once(manager, config, shared_state, state_lock):
    # 先把当前状态读出来，再释放锁。
    # 各状态处理函数如果需要读写共享状态，会自己加锁。
    state_lock.acquire()
    try:
        current_state = shared_state["control_state"]
    finally:
        state_lock.release()

    # 状态分发保持 if/elif 的形式，方便后续直接在对应函数里补控制策略。
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


# 释放所有舵机。
def release_servos(manager, servo_ids):
    # method=0x10, power=0 来自官方示例，含义是停止并释放控制。
    for servo_id in servo_ids:
        try:
            manager.stop_on_control_mode(int(servo_id), method=0x10, power=0)
        except Exception:
            pass


# 控制线程入口，本线程是唯一访问舵机 SDK 的线程。
def control_thread_main(config, shared_state, state_lock, stop_event, rospy):
    # 控制线程是唯一访问 UartServoManager 的线程。
    # ROS 线程只读 shared_state，不碰串口，避免串口读写交叉。
    uart = None
    manager = None
    rate_delay = 1.0 / float(config["rate"])

    try:
        uart, manager = open_servo_manager(config)
        rospy.loginfo("舵机串口已打开: %s @ %d", config["port"], config["baudrate"])

        while not stop_event.is_set() and not rospy.is_shutdown():
            try:
                # 1. 同步读取所有舵机状态。
                new_states = read_sync_monitor(manager, config["servo_ids"])
                # 2. 写入共享状态，供 ROS 线程发布。
                update_shared_servo_state(shared_state, state_lock, new_states)
                # 3. 根据当前运动状态执行一轮控制逻辑。
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


# ROS 发布线程入口。
def ros_thread_main(config, shared_state, state_lock, stop_event, rospy):
    # ROS 相关 import 放在线程函数里，Windows 上导入本文件跑测试时不需要 ROS。
    from std_msgs.msg import Float64MultiArray

    angle_pub = rospy.Publisher(config["angle_topic"], Float64MultiArray, queue_size=10)
    current_pub = None
    if config["read_current"]:
        current_pub = rospy.Publisher(config["current_topic"], Float64MultiArray, queue_size=10)

    rate = rospy.Rate(config["rate"])
    rospy.loginfo("角度话题发布到: %s", config["angle_topic"])

    while not stop_event.is_set() and not rospy.is_shutdown():
        # 发布线程只发布最近一帧共享状态，不主动读取硬件。
        state_copy = copy_shared_state(shared_state, state_lock)
        angle_pub.publish(Float64MultiArray(data=make_angle_array(state_copy)))
        if current_pub is not None:
            current_pub.publish(Float64MultiArray(data=make_current_array(state_copy)))
        rate.sleep()


# 启动一个后台线程。
def start_thread(name, target, args):
    # daemon=True 表示主程序退出时不会被后台线程卡住。
    # main 里仍然会主动 join 一小段时间，尽量让串口正常关闭。
    thread = threading.Thread(name=name, target=target, args=args)
    thread.daemon = True
    thread.start()
    return thread


# 程序入口，负责装配线程和管理生命周期。
def main():
    # main 线程只负责装配和生命周期，不写控制算法。
    # 控制算法统一放到 control_thread_main 和各 handle_xxx 函数里。
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
