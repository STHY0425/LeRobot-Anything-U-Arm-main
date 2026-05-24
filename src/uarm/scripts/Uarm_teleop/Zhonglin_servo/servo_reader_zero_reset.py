#!/usr/bin/env python3
"""
Servo Reader with Dynamic Zero Reset
支持运行时动态重置零点的舵机读取节点
"""

import rospy
import serial
import time
import re
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool


class ServoReaderZeroReset:
    def __init__(self):
        rospy.init_node("servo_reader_zero_reset")
        self.pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        
        # 零点重置相关
        self.zero_pub = rospy.Publisher('/servo_zero_angles', Float64MultiArray, queue_size=1, latch=True)
        rospy.Subscriber('/reset_servo_zero', Bool, self.reset_zero_callback)
        rospy.Subscriber('/release_servo_torque', Bool, self.release_torque_callback)
        
        self.rate = rospy.Rate(50)
        self.SERIAL_PORT = rospy.get_param("~serial_port", '/dev/ttyUSB0')
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
            rospy.loginfo(f"Serial port opened: {self.SERIAL_PORT} @ {self.BAUDRATE}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            raise

        self.gripper_range = 0.48
        self.zero_angles = [0.0] * 7
        self.current_absolute_angles = [0.0] * 7
        
        # 初始化零点
        self._init_servos()

    def send_command(self, cmd, delay=0.008):
        """发送命令并返回响应"""
        self.ser.write(cmd.encode('ascii'))
        time.sleep(delay)
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        """PWM转换为角度"""
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        """初始化：释放力矩并记录零点"""
        rospy.loginfo("========================================")
        rospy.loginfo("【舵机零点校准】")
        rospy.loginfo("1. 释放舵机力矩...")
        
        self.send_command('#000PVER!')
        for i in range(7):
            self.send_command("#000PCSK!")
            self.send_command(f'#{i:03d}PULK!')  # 释放力矩
            time.sleep(0.01)
        
        rospy.loginfo("2. 请在5秒内调整主臂到期望的零点位置...")
        time.sleep(5.0)
        
        rospy.loginfo("3. 正在记录零点...")
        self._record_zero_point()
        
        rospy.loginfo("4. ✅ 零点校准完成！")
        rospy.loginfo(f"   零点角度: {[f'{z:.1f}' for z in self.zero_angles]}")
        rospy.loginfo("========================================")

    def _record_zero_point(self):
        """记录当前位置为零点"""
        for i in range(7):
            response = self.send_command(f'#{i:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            if angle is not None:
                self.zero_angles[i] = angle
            else:
                rospy.logwarn(f"  舵机{i}读取失败，使用上次值: {self.zero_angles[i]:.1f}")
                
        # 发布零点信息供其他节点使用
        zero_msg = Float64MultiArray()
        zero_msg.data = self.zero_angles
        self.zero_pub.publish(zero_msg)

    def reset_zero_callback(self, msg):
        """ROS回调：重置零点"""
        if msg.data:
            rospy.loginfo("【收到零点重置命令】")
            self._init_servos()
            rospy.loginfo("【零点已重置】")

    def release_torque_callback(self, msg):
        """ROS回调：释放力矩（便于手动调整）"""
        if msg.data:
            rospy.loginfo("【释放舵机力矩】")
            for i in range(7):
                self.send_command(f'#{i:03d}PULK!')
                time.sleep(0.01)
            rospy.loginfo("【力矩已释放，可手动调整主臂】")

    def read_all_servos(self):
        """读取所有舵机当前角度"""
        angles = [None] * 7
        for i in range(7):
            response = self.send_command(f'#{i:03d}PRAD!', delay=0.005)
            angle = self.pwm_to_angle(response.strip())
            if angle is not None:
                angles[i] = angle
                self.current_absolute_angles[i] = angle
            else:
                # 使用上次有效值
                angles[i] = self.current_absolute_angles[i]
        return angles

    def run(self):
        """主循环"""
        angle_offset = [0.0] * 7
        target_angle_offset = [0.0] * 7
        num_interp = 5
        step_size = 1
        
        rospy.loginfo("舵机读取节点运行中...")
        rospy.loginfo("提示：发布 Bool(True) 到 /reset_servo_zero 可重新校准零点")
        
        while not rospy.is_shutdown():
            # 读取所有舵机绝对角度
            absolute_angles = self.read_all_servos()
            
            # 计算相对零点的偏移
            for i in range(7):
                if absolute_angles[i] is not None:
                    new_angle = absolute_angles[i] - self.zero_angles[i]
                    
                    # 跳变检测
                    if abs(new_angle - target_angle_offset[i]) > 90:
                        rospy.logerr(f"舵机{i}角度跳变过大: {new_angle:.1f} vs {target_angle_offset[i]:.1f}")
                    elif abs(new_angle - target_angle_offset[i]) > step_size:
                        target_angle_offset[i] = new_angle

            # 插值平滑
            for step in range(num_interp):
                for i in range(7):
                    delta = target_angle_offset[i] - angle_offset[i]
                    angle_offset[i] += delta * 0.2
                
                msg = Float64MultiArray()
                msg.data = angle_offset[:]
                self.pub.publish(msg)
                self.rate.sleep()


if __name__ == '__main__':
    try:
        node = ServoReaderZeroReset()
        node.run()
    except rospy.ROSInterruptException:
        pass
