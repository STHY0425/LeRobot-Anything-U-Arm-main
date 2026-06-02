#!/usr/bin/env python3
"""
华馨京舵机基础读取节点。(启动节点时角度设置为零点)
发布 /servo_angles (Float64MultiArray)，默认 7 维。
"""
import rospy
import serial
from std_msgs.msg import Float64MultiArray
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


class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher("/servo_angles", Float64MultiArray, queue_size=10)

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
        self._init_servos()

    def _init_servos(self):
        for servo_id in self.servo_ids:
            angle = read_angle(self.manager, servo_id)
            if angle is None:
                angle = 0.0
            if 0 <= servo_id < self.num_servos:
                self.zero_angles[servo_id] = angle
        rospy.loginfo("零点校准完成")

    def run(self):
        angle_offset = [0.0] * self.num_servos
        target_angle_offset = [0.0] * self.num_servos
        step_size = 1.0
        jump_thres = 90.0

        while not rospy.is_shutdown():
            for servo_id in self.servo_ids:
                angle = read_angle(self.manager, servo_id)
                if angle is not None:
                    new_angle = angle - self.zero_angles[servo_id]
                    if abs(new_angle - target_angle_offset[servo_id]) > jump_thres:
                        rospy.logerr(
                            "舵机 %d 角度跳变: %.1f vs %.1f",
                            servo_id,
                            new_angle,
                            target_angle_offset[servo_id],
                        )
                    elif abs(new_angle - target_angle_offset[servo_id]) > step_size:
                        target_angle_offset[servo_id] = new_angle
                else:
                    rospy.logwarn("舵机 %d 读取失败", servo_id)

            angle_offset = target_angle_offset.copy()
            self.pub.publish(Float64MultiArray(data=angle_offset))
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
