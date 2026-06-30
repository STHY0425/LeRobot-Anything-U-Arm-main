#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥测调试节点：订阅主节点的 ROS 话题，终端单帧刷新显示。

订阅话题：
  /servo_angles         — 各舵机角度 (度)
  /servo_end_velocity   — 末端 3D 速度 [vx, vy, vz] (m/s)
  /servo_damping_mode   — 阻尼档位码 [0/1/2/3]
  /servo_damping_powers — 各舵机目标阻尼功率 (mW)

显示方式：每帧清屏刷新（ANSI 转义码），不流式刷屏。
刷新频率默认 10Hz，可通过 ROS 参数 ~refresh_rate 调整。

DOF 自适应：舵机数量不需要手动配置，从 /servo_angles 消息长度
自动推导。主节点换 JSON 从 3 轴改到 7 轴，debug node 不用改任何参数。
"""

import sys
import threading

import rospy
from std_msgs.msg import Float64MultiArray


# 阻尼档位码 → 可读名称。
MODE_NAMES = {
    0: "NONE  (非HOLD)",
    1: "LOW   (逆重力/上抬)",
    2: "MID   (水平/静止)",
    3: "HIGH  (顺重力/下坠)",
}


# 线程安全的最新数据缓存。
# num_servos 由 /servo_angles 消息长度自动推导，不需要外部传入。
class TopicCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {}
        self._num_servos = 0

    def update(self, key, values):
        with self._lock:
            self._data[key] = values
            # /servo_angles 是舵机数量的权威来源：
            # 主节点的 servo_ids = [0, 1, ..., dof-1]，发布数组长度 = dof。
            if key == "angles" and values is not None:
                self._num_servos = len(values)

    def get(self, key):
        with self._lock:
            return self._data.get(key)

    def snapshot(self):
        with self._lock:
            return dict(self._data), self._num_servos


# 格式化浮点数组为等宽数值串。
def fmt_array(values, width=8, precision=4):
    if values is None:
        return "(无数据)"
    return "[" + ", ".join("%*.4f" % (width, v) for v in values) + "]"


# 把角度数组和阻尼功率数组并排显示，方便对比每个舵机。
def fmt_servio_table(angles, powers, num_servos):
    header = "  ID  |  Angle(deg)  |  Damping(mW)"
    sep    = "------+--------------+--------------"
    lines = [header, sep]
    if num_servos == 0:
        lines.append("  (等待数据...)")
        return "\n".join(lines)

    for i in range(num_servos):
        ang = angles[i] if angles is not None and i < len(angles) else 0.0
        pw = powers[i] if powers is not None and i < len(powers) else 0.0
        lines.append("  %2d  |  %10.4f  |  %10.1f" % (i, ang, pw))
    return "\n".join(lines)


# 单帧刷新显示。
def render(cache):
    data, num_servos = cache.snapshot()

    angles = data.get("angles")
    velocity = data.get("end_velocity")
    mode_raw = data.get("damping_mode")
    powers = data.get("damping_powers")

    # 档位名称翻译。
    if mode_raw is not None and len(mode_raw) > 0:
        mode_code = int(round(mode_raw[0]))
        mode_str = MODE_NAMES.get(mode_code, "UNKNOWN(%d)" % mode_code)
    else:
        mode_code = None
        mode_str = "(无数据)"

    # ANSI 清屏 + 光标到左上角。
    sys.stdout.write("\033[2J\033[H")

    lines = [
        "╔══════════════════════════════════════════════════╗",
        "║       HXJ 遥测调试监视器  —  telemetry_debug     ║",
        "╚══════════════════════════════════════════════════╝",
        "",
        "【舵机状态】  (DOF: %s)" % (num_servos if num_servos > 0 else "?"),
        fmt_servio_table(angles, powers, num_servos),
        "",
        "【末端速度 /servo_end_velocity (m/s)】",
        "  " + fmt_array(velocity),
        "",
        "【阻尼档位 /servo_damping_mode】",
        "  %s" % mode_str,
        "",
        "─" * 50,
        "Ctrl+C 退出",
    ]

    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


# 节点入口。
def main():
    rospy.init_node("telemetry_debug_node", anonymous=True)

    refresh_rate = rospy.get_param("~refresh_rate", 10.0)
    if refresh_rate <= 0:
        refresh_rate = 10.0

    cache = TopicCache()

    # 订阅回调：把 Float64MultiArray.data 转 list 存入缓存。
    def make_callback(key):
        def cb(msg):
            cache.update(key, list(msg.data))
        return cb

    rospy.Subscriber("/servo_angles",        Float64MultiArray, make_callback("angles"))
    rospy.Subscriber("/servo_end_velocity",  Float64MultiArray, make_callback("end_velocity"))
    rospy.Subscriber("/servo_damping_mode",   Float64MultiArray, make_callback("damping_mode"))
    rospy.Subscriber("/servo_damping_powers", Float64MultiArray, make_callback("damping_powers"))

    rate = rospy.Rate(refresh_rate)

    rospy.loginfo("telemetry_debug_node 启动 (refresh=%.1fHz, DOF 自动检测)", refresh_rate)

    while not rospy.is_shutdown():
        render(cache)
        rate.sleep()

    # 退出时清屏。
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass