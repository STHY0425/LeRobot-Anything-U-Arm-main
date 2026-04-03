#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealMan (瑞尔曼)驱动Mock节点
模拟真实的瑞尔曼ROS驱动，用于无实物测试
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from rm_msgs.msg import JointPos, Gripper_Set, Arm_Current_State


class MockRealManDriver:
    """
    模拟瑞尔曼机械臂驱动
    - 模拟关节运动
    - 模拟错误状态
    - 响应控制命令
    """
    
    def __init__(self):
        rospy.init_node("mock_rm_driver")
        rospy.loginfo("[MockDriver] 启动模拟瑞尔曼驱动...")
        
        # 参数
        self.simulation_rate = rospy.get_param("~simulation_rate", 30.0)
        self.noise_level = rospy.get_param("~noise_level", 0.1)  # 模拟噪声
        self.enable_error_sim = rospy.get_param("~enable_error_sim", False)
        
        # 状态
        self.joint_angles = np.array([0.0, -20.0, -90.0, 0.0, 90.0, 0.0], dtype=np.float32)
        self.target_angles = self.joint_angles.copy()
        self.gripper_pos = 0
        self.target_gripper = 0
        self.errors = []
        self.is_enabled = True
        
        # 平滑系数
        self.smooth_factor = 0.1
        
        # 发布者
        self.state_pub = rospy.Publisher('/rm_driver/Arm_Current_State', Arm_Current_State, queue_size=10)
        
        # 订阅者
        rospy.Subscriber('/rm_driver/JointPos', JointPos, self.joint_cmd_callback)
        rospy.Subscriber('/rm_driver/Gripper_Set', Gripper_Set, self.gripper_cmd_callback)
        rospy.Subscriber('/mock_driver/trigger_error', Bool, self.error_trigger_callback)
        
        # 定时器
        rospy.Timer(rospy.Duration(1.0 / self.simulation_rate), self.update_loop)
        
        rospy.loginfo("[MockDriver] 模拟驱动已启动")
        rospy.loginfo("[MockDriver] 控制命令将驱动虚拟机械臂运动")

    def joint_cmd_callback(self, msg):
        """接收关节控制命令"""
        if len(msg.joint) >= 6:
            self.target_angles = np.array(msg.joint[:6], dtype=np.float32)
            rospy.logdebug(f"[MockDriver] 收到关节目标: {self.target_angles}")

    def gripper_cmd_callback(self, msg):
        """接收夹爪控制命令"""
        self.target_gripper = msg.position
        rospy.logdebug(f"[MockDriver] 收到夹爪目标: {self.target_gripper}")

    def error_trigger_callback(self, msg):
        """触发错误模拟"""
        if msg.data:
            # 模拟碰撞错误
            self.errors = [0x100D]
            rospy.logerr("[MockDriver] ⚠️ 模拟触发碰撞错误 0x100D")
        else:
            self.errors = []
            rospy.loginfo("[MockDriver] 错误已清除")

    def update_loop(self, event=None):
        """更新循环 - 模拟机械臂运动"""
        # 平滑运动到目标位置
        self.joint_angles += (self.target_angles - self.joint_angles) * self.smooth_factor
        self.gripper_pos += (self.target_gripper - self.gripper_pos) * self.smooth_factor
        
        # 添加微小噪声模拟真实传感器
        noise = np.random.normal(0, self.noise_level, 6)
        noisy_angles = self.joint_angles + noise
        
        # 构建状态消息
        state_msg = Arm_Current_State()
        state_msg.joint = noisy_angles.tolist()
        state_msg.dof = 6
        state_msg.err = self.errors if self.errors else []
        
        # 填充位姿（简单的正向运动学模拟）
        # 这里使用简化的占位值
        state_msg.Pose = [0.3, 0.0, 0.4, 3.14, 0.0, 0.0]
        
        self.state_pub.publish(state_msg)

    def run(self):
        """主循环"""
        rospy.loginfo("[MockDriver] 模拟驱动运行中...")
        rospy.loginfo("[MockDriver] 提示: 可以发送错误测试命令:")
        rospy.loginfo("  rostopic pub /mock_driver/trigger_error std_msgs/Bool true")
        rospy.spin()


def main():
    try:
        node = MockRealManDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
