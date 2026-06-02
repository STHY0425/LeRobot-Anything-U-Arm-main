#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取舵机电流并转换为扭矩，发布话题。

启动节点：rosrun uarm hxj_test_convert_cur_to_tor.py(暂时默认1维)
获取话题数据：rostopic echo /servo_currents
            rostopic echo /servo_torques
"""
import rospy
import time
import serial
from std_msgs.msg import Float64MultiArray
from fashionstar_uart_sdk import UartServoManager

# ==================== 默认参数配置 ====================
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200
DEFAULT_SERVO_ID = 0 #id 从0开始，默认读取第一个舵机
DEFAULT_NUM_SERVOS = 7
DEFAULT_KT = 2.45 / 3.0
DEFAULT_IDLE_CURRENT = 0.03
DEFAULT_INTERVAL = 0.1
# ===================================================

class CurrentToTorqueNode:
    def __init__(self):
        rospy.init_node("convert_current_to_torque_node")
        
        # 参数获取
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baudrate = rospy.get_param("~baudrate", DEFAULT_BAUDRATE)
        self.servo_id = rospy.get_param("~servo_id", DEFAULT_SERVO_ID)
        self.num_servos = int(rospy.get_param("~num_servos", DEFAULT_NUM_SERVOS))
        self.kt = float(rospy.get_param("~kt", DEFAULT_KT))
        self.idle_current = float(rospy.get_param("~idle_current", DEFAULT_IDLE_CURRENT))
        self.interval = rospy.get_param("~interval", DEFAULT_INTERVAL)
        
        # 发布话题
        self.pub_currents = rospy.Publisher("/servo_currents", Float64MultiArray, queue_size=10)
        self.pub_torques = rospy.Publisher("/servo_torques", Float64MultiArray, queue_size=10)

        try:
            uart = serial.Serial(
                port=self.port, 
                baudrate=self.baudrate, 
                parity=serial.PARITY_NONE, 
                stopbits=1, 
                bytesize=8, 
                timeout=0
            )
            self.manager = UartServoManager(uart)
            rospy.loginfo(f"串口已打开: {self.port} @ {self.baudrate}")
        except Exception as e:
            rospy.logerr(f"串口打开失败: {e}")
            raise

    def run(self):
        rospy.loginfo(f"开始读取舵机 ID {self.servo_id} 的电流并发布话题...")
        rate = rospy.Rate(1.0 / self.interval)
        
        while not rospy.is_shutdown():
            try:
                current = self.manager.query_current(self.servo_id)
                if current is not None:
                    # 计算负载电流和扭矩
                    net_current = max(0.0, current - self.idle_current)
                    torque = net_current * self.kt
                    
                    # 准备发布数据（数组形式，对应索引发布）
                    current_arr = [0.0] * self.num_servos
                    torque_arr = [0.0] * self.num_servos
                    
                    if 0 <= self.servo_id < self.num_servos:
                        current_arr[self.servo_id] = net_current
                        torque_arr[self.servo_id] = torque
                    
                    self.pub_currents.publish(Float64MultiArray(data=current_arr))
                    self.pub_torques.publish(Float64MultiArray(data=torque_arr))
                else:
                    rospy.logwarn(f"舵机 {self.servo_id} 电流读取失败")
            except Exception as e:
                rospy.logerr(f"运行错误: {e}")
            
            rate.sleep()

def main():
    try:
        node = CurrentToTorqueNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"程序异常退出: {e}")

if __name__ == '__main__':
    main()
