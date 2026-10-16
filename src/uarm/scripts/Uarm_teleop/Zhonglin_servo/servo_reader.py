#!/usr/bin/env python3
import rospy
import serial
import time
import re
import numpy as np
from std_msgs.msg import Float64MultiArray

class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        # 提高发布频率：去掉插值后每次循环只做一次串口轮询+发布，
        # 100Hz 约 10ms/周期，但 7 次串口查询+sleep(0.008) 实际约 56ms，
        # 所以真实有效频率仍受串口通信限制（约 15~20Hz）。
        # 若想进一步提升，请查看舵机控制板是否支持批量读取指令。
        self.rate = rospy.Rate(100)
        self.SERIAL_PORT = '/dev/ttyUSB0'

        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        rospy.loginfo("Serial port opened")

        # 舵机 ID 列表：发布的数组第 i 位对应 servo_ids[i] 号舵机
        # 主臂舵机 ID 若不是 0~6（例如 1~7），用 ~servo_ids 按关节顺序指定实际 ID
        self.servo_ids = rospy.get_param("~servo_ids", [0, 1, 2, 3, 4, 5, 6])
        if not self.servo_ids:
            rospy.logerr("servo_ids 为空，请设置 ~servo_ids")
            raise ValueError("servo_ids is empty")
        self.num_servos = len(self.servo_ids)
        rospy.loginfo(f"Servo IDs: {self.servo_ids}")

        self.gripper_range = 0.48
        self.zero_angles = [0.0] * self.num_servos
        self._init_servos()

    def send_command(self, cmd):
        self.ser.write(cmd.encode('ascii'))
        
        
        time.sleep(0.004)
        # time.sleep(0.008)
      
      
      
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!')
        for slot, servo_id in enumerate(self.servo_ids):
            self.send_command("#000PCSK!")
            self.send_command(f'#{servo_id:03d}PULK!')
            response = self.send_command(f'#{servo_id:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[slot] = angle if angle is not None else 0.0
        rospy.loginfo("Servo initial angle calibration completed")

    def run(self):
        angle_offset = [0.0] * self.num_servos  # 当前发布的角度（相对于零点的偏移）
        target_angle_offset = [0.0] * self.num_servos  # 本次读取到的目标角度
        step_size = 1  # 最小变化阈值，小于此值视为未变化
        danger_thres = 90

        while not rospy.is_shutdown():
            for slot, servo_id in enumerate(self.servo_ids):
                response = self.send_command(f'#{servo_id:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    new_angle = angle - self.zero_angles[slot]
                    # 异常跳变检测：如果角度变化超过 90 度，说明可能读数异常或硬件拨动
                    if abs(new_angle - target_angle_offset[slot]) > danger_thres:
                        raise Warning(f"Servo {servo_id} angle jump too large: {new_angle} vs {target_angle_offset[slot]}, check if hardware adjustment is needed.")
                    elif abs(new_angle - target_angle_offset[slot]) > step_size:
                        target_angle_offset[slot] = new_angle
                else:
                    rospy.logwarn(f"Servo {servo_id} response error: {response.strip()}")
                    
            angle_offset = target_angle_offset.copy()
            self.pub.publish(Float64MultiArray(data=angle_offset))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
