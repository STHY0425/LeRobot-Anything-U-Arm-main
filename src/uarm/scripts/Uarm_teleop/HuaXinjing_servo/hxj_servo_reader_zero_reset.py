#!/usr/bin/env python3
"""
华馨京舵机读取节点（支持运行时重置零点）。
发布 /servo_angles (Float64MultiArray)，默认 7 维。
"""
import time
import rospy
import serial
from std_msgs.msg import Float64MultiArray, Bool
from fashionstar_uart_sdk import UartServoManager

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
DEFAULT_NUM_SERVOS = 7
DEFAULT_RATE_HZ = 50
DEFAULT_TIMEOUT = 0


def open_manager(port, baudrate, timeout):
    uart = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=timeout,
    )
    return UartServoManager(uart)


def read_angle(manager, servo_id):
    return manager.query_servo_angle(servo_id)


def release_torque(manager, servo_id):
    manager.stop_on_control_mode(servo_id, method=0x10, power=0)


class ServoReaderZeroReset:
    def __init__(self):
        rospy.init_node("servo_reader_zero_reset")
        self.pub = rospy.Publisher("/servo_angles", Float64MultiArray, queue_size=10)

        self.zero_pub = rospy.Publisher(
            "/servo_zero_angles", Float64MultiArray, queue_size=1, latch=True
        )
        rospy.Subscriber("/reset_servo_zero", Bool, self.reset_zero_callback)
        rospy.Subscriber("/release_servo_torque", Bool, self.release_torque_callback)

        self.rate_hz = rospy.get_param("~rate", DEFAULT_RATE_HZ)
        self.rate = rospy.Rate(self.rate_hz)
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baudrate = rospy.get_param("~baudrate", DEFAULT_BAUDRATE)
        self.servo_ids = rospy.get_param("~servo_ids", DEFAULT_SERVO_IDS)
        self.num_servos = int(rospy.get_param("~num_servos", DEFAULT_NUM_SERVOS))
        self.timeout = rospy.get_param("~timeout", DEFAULT_TIMEOUT)

        if not self.servo_ids:
            rospy.logerr("servo_ids 为空，请设置 ~servo_ids")
            raise ValueError("servo_ids is empty")

        try:
            self.manager = open_manager(self.port, self.baudrate, self.timeout)
            rospy.loginfo("串口已打开: %s @ %d", self.port, self.baudrate)
        except serial.SerialException as exc:
            rospy.logerr("串口打开失败: %s", exc)
            raise

        self.zero_angles = [0.0] * self.num_servos
        self.current_absolute_angles = [0.0] * self.num_servos

        self._init_servos()

    def _init_servos(self):
        rospy.loginfo("=== 舵机零点校准 ===")
        rospy.loginfo("步骤 1: 释放扭矩")
        for servo_id in self.servo_ids:
            release_torque(self.manager, servo_id)
            time.sleep(0.02)

        rospy.loginfo("步骤 2: 5 秒内将主臂调整到期望零位")
        time.sleep(5.0)

        rospy.loginfo("步骤 3: 记录零点角度")
        self._record_zero_point()
        rospy.loginfo("零点校准完成: %s", [f"{z:.1f}" for z in self.zero_angles])

    def _record_zero_point(self):
        for servo_id in self.servo_ids:
            angle = read_angle(self.manager, servo_id)
            if angle is None:
                rospy.logwarn("舵机 %d 读取失败，沿用上次零点", servo_id)
                continue
            if 0 <= servo_id < self.num_servos:
                self.zero_angles[servo_id] = angle

        zero_msg = Float64MultiArray()
        zero_msg.data = self.zero_angles
        self.zero_pub.publish(zero_msg)

    def reset_zero_callback(self, msg):
        if msg.data:
            rospy.loginfo("收到零点重置请求")
            self._init_servos()

    def release_torque_callback(self, msg):
        if msg.data:
            rospy.loginfo("收到释放扭矩请求")
            for servo_id in self.servo_ids:
                release_torque(self.manager, servo_id)
                time.sleep(0.02)

    def read_all_servos(self):
        angles = [None] * self.num_servos
        for servo_id in self.servo_ids:
            angle = read_angle(self.manager, servo_id)
            if angle is not None:
                if 0 <= servo_id < self.num_servos:
                    angles[servo_id] = angle
                    self.current_absolute_angles[servo_id] = angle
            else:
                if 0 <= servo_id < self.num_servos:
                    angles[servo_id] = self.current_absolute_angles[servo_id]
        return angles

    def run(self):
        angle_offset = [0.0] * self.num_servos
        target_angle_offset = [0.0] * self.num_servos
        num_interp = 5
        step_size = 1.0
        jump_thres = 90.0

        rospy.loginfo("读取节点运行中，可发布到 /reset_servo_zero 重新校准零点")

        while not rospy.is_shutdown():
            absolute_angles = self.read_all_servos()

            for idx in range(self.num_servos):
                if absolute_angles[idx] is None:
                    continue
                new_angle = absolute_angles[idx] - self.zero_angles[idx]
                if abs(new_angle - target_angle_offset[idx]) > jump_thres:
                    rospy.logerr(
                        "Servo %d angle jump: %.1f vs %.1f",
                        idx,
                        new_angle,
                        target_angle_offset[idx],
                    )
                elif abs(new_angle - target_angle_offset[idx]) > step_size:
                    target_angle_offset[idx] = new_angle

            for _ in range(num_interp):
                for idx in range(self.num_servos):
                    delta = target_angle_offset[idx] - angle_offset[idx]
                    angle_offset[idx] += delta * 0.2

                msg = Float64MultiArray()
                msg.data = angle_offset[:]
                self.pub.publish(msg)
                self.rate.sleep()


if __name__ == "__main__":
    try:
        node = ServoReaderZeroReset()
        node.run()
    except rospy.ROSInterruptException:
        pass
