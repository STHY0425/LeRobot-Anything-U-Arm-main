#!/usr/bin/env python3
"""
华馨京舵机纯按键控制脚本：
- 按住 'k' 键：释放扭矩（允许摇操）
- 松开 'k' 键：锁定当前角度
- xset r rate 250 40  首键延迟从 500ms 缩短到 250ms,即系统发生第一次按下到确定为按住的时间为250ms，之后每40ms重复一次按住事件（如果持续按住），直到松开为止。
    40：重复频率为40，a每秒发生40次，
    （在终端输入该命令）
"""
import rospy
import serial
from std_msgs.msg import Float64MultiArray
from fashionstar_uart_sdk import UartServoManager
import sys
import select
import tty
import termios
import time

# 默认配置
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_SERVO_IDS = [0]
DEFAULT_NUM_SERVOS = 7
DEFAULT_RATE_HZ = 50
deafault_hold_timeout = 0.25 # 按键松开判定的时间阈值（秒）

class SimpleHoldNode:
    def __init__(self):
        rospy.init_node("hxj_servo_simple_hold_node")
        self.pub = rospy.Publisher("/servo_angles", Float64MultiArray, queue_size=10)

        # 参数获取
        self.rate_hz = rospy.get_param("~rate", DEFAULT_RATE_HZ)
        self.rate = rospy.Rate(self.rate_hz)
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baudrate = rospy.get_param("~baudrate", DEFAULT_BAUDRATE)
        self.servo_ids = rospy.get_param("~servo_ids", DEFAULT_SERVO_IDS)
        self.num_servos = int(rospy.get_param("~num_servos", DEFAULT_NUM_SERVOS))
        
        # 按住超时阈值：如果 0.5s 内没收到 'k'，视为松开
        self.hold_timeout = float(rospy.get_param("~hold_timeout", deafault_hold_timeout))

        try:
            uart = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0)
            self.manager = UartServoManager(uart)
            rospy.loginfo("串口已打开: %s @ %d", self.port, self.baudrate)
        except Exception as e:
            rospy.logerr("串口打开失败: %s", e)
            raise

        # 零点校准
        self.zero_angles = [0.0] * self.num_servos
        self._init_zero_angles()

        # 键盘状态
        self._orig_termios = None
        self._setup_keyboard()
        self.last_k_time = 0.0
        self.key_was_held = False # 记录上一帧的状态

        rospy.on_shutdown(self._restore_keyboard)

    def _init_zero_angles(self):
        for servo_id in self.servo_ids:
            angle = self.manager.query_servo_angle(servo_id)
            if angle is not None and 0 <= servo_id < self.num_servos:
                self.zero_angles[servo_id] = angle
        rospy.loginfo("零点校准完成")

    def _setup_keyboard(self):
        try:
            if sys.stdin.isatty():
                self._orig_termios = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
        except Exception: pass

    def _restore_keyboard(self):
        if self._orig_termios:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_termios)

    def _check_keyboard(self):
        """清空缓冲区并更新按键时间戳"""
        try:
            if not sys.stdin.isatty(): return
            has_k = False
            while True:
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    if sys.stdin.read(1) == 'k':
                        has_k = True
                else: break
            if has_k:
                self.last_k_time = time.time()  
        except Exception: pass

    def _do_lock(self):
        """锁定所有舵机到当前位置"""
        rospy.loginfo(">>> 松开按键：锁定位置")
        for servo_id in self.servo_ids:
            angle = self.manager.query_servo_angle(servo_id)
            if angle is not None:
                # 锁定指令：设置目标角度为当前读取的角度
                self.manager.set_servo_angle(servo_id, angle, velocity=80)

    def _do_unlock(self):
        """释放所有舵机扭矩"""
        rospy.loginfo("<<< 按住按键：释放扭矩")
        for servo_id in self.servo_ids:
            # 释放指令
            self.manager.stop_on_control_mode(servo_id, method=0x10, power=0)

    def run(self):
        target_angles = [0.0] * self.num_servos
        
        while not rospy.is_shutdown():
            # 1. 读取并发布角度（用于 ROS 同步）
            for servo_id in self.servo_ids:
                angle = self.manager.query_servo_angle(servo_id)
                if angle is not None:
                    # 计算相对于零点的偏移
                    target_angles[servo_id] = angle - self.zero_angles[servo_id]

            self.pub.publish(Float64MultiArray(data=target_angles))

            # 2. 键盘交互逻辑
            self._check_keyboard()
            now = time.time()
            currently_held = (now - self.last_k_time) < self.hold_timeout   # 当前是否视为按键被按住

            # 状态切换检测（边沿触发）
            if currently_held and not self.key_was_held:
                # 刚按下
                self._do_unlock()
            elif not currently_held and self.key_was_held:
                # 刚松开
                self._do_lock()

            self.key_was_held = currently_held
            self.rate.sleep()

if __name__ == "__main__":
    try:
        SimpleHoldNode().run()
    except rospy.ROSInterruptException: pass