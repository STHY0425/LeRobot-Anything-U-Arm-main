#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主臂设备Mock节点
模拟主臂舵机角度变化，用于测试遥操逻辑
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


class MockMasterArm:
    """
    模拟主臂设备
    支持多种测试模式：
    - 静止模式: 输出固定角度
    - 正弦模式: 周期性运动
    - 随机模式: 随机微动
    - 步进模式: 单步测试
    """
    
    TEST_MODES = ['static', 'sine', 'random', 'step']
    
    def __init__(self):
        rospy.init_node("mock_master_arm")
        rospy.loginfo("[MockMaster] 启动模拟主臂设备...")
        
        # 参数
        self.mode = rospy.get_param("~mode", "sine")  # static, sine, random, step
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)
        self.amplitude = rospy.get_param("~amplitude", 30.0)  # 运动幅度（度）
        self.frequency = rospy.get_param("~frequency", 0.1)   # 运动频率（Hz）
        self.joint_idx = rospy.get_param("~joint_idx", 0)     # 步进模式：指定关节
        
        # 初始角度
        self.angles = [0.0] * 7  # 7个舵机
        self.start_time = rospy.Time.now()
        
        # 发布者
        self.servo_pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        
        # 定时器
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_loop)
        
        rospy.loginfo(f"[MockMaster] 测试模式: {self.mode}")
        rospy.loginfo(f"[MockMaster] 发布频率: {self.publish_rate} Hz")
        
        if self.mode == 'static':
            rospy.loginfo("[MockMaster] 当前为静止模式，角度保持不变")
        elif self.mode == 'sine':
            rospy.loginfo(f"[MockMaster] 当前为正弦模式，幅度: {self.amplitude}°")
        elif self.mode == 'random':
            rospy.loginfo("[MockMaster] 当前为随机模式")
        elif self.mode == 'step':
            rospy.loginfo(f"[MockMaster] 当前为步进模式，测试关节: {self.joint_idx}")
            rospy.loginfo("[MockMaster] 使用服务切换步进:")
            rospy.loginfo("  rosservice call /mock_master/step_joint '{joint: 0, delta: 10.0}'")
        
        # 服务
        if self.mode == 'step':
            from std_srvs.srv import Trigger, TriggerResponse
            from custom_msgs.srv import JointStep
            rospy.Service('/mock_master/step_joint', Trigger, self.step_joint_service)

    def publish_loop(self, event=None):
        """发布循环"""
        if self.mode == 'static':
            pass  # 保持当前角度
            
        elif self.mode == 'sine':
            # 正弦运动
            t = (rospy.Time.now() - self.start_time).to_sec()
            for i in range(6):
                phase = i * np.pi / 3  # 各关节相位差
                self.angles[i] = self.amplitude * np.sin(2 * np.pi * self.frequency * t + phase)
            # 夹爪开合
            self.angles[6] = (self.amplitude / 2) * (1 + np.sin(2 * np.pi * self.frequency * t))
            
        elif self.mode == 'random':
            # 随机微动
            for i in range(7):
                self.angles[i] += np.random.uniform(-2.0, 2.0)
                self.angles[i] = np.clip(self.angles[i], -90, 90)
                
        elif self.mode == 'step':
            # 步进模式 - 等待服务调用
            pass
        
        # 发布角度
        msg = Float64MultiArray()
        msg.data = self.angles
        self.servo_pub.publish(msg)
    
    def step_joint_service(self, req):
        """步进服务"""
        self.angles[self.joint_idx] += 10.0
        return TriggerResponse(success=True, message=f"Joint {self.joint_idx} stepped to {self.angles[self.joint_idx]}")

    def run(self):
        """主循环"""
        rospy.loginfo("[MockMaster] 模拟主臂运行中...")
        rospy.spin()


def main():
    try:
        node = MockMasterArm()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
