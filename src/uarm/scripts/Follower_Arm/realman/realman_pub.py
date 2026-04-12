#!/usr/bin/env python3
"""
RealMan 机械臂状态发布节点
功能：读取 RealMan 关节状态并发布到 /robot_state 话题
架构：参考 xarm_pub.py 的简洁设计
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rm_msgs.msg import Arm_Current_State


class RealManStatePublisher:
    """RealMan 机械臂状态发布节点"""
    
    def __init__(self):
        rospy.init_node('realman_state_publisher')
        rospy.loginfo("[RealManPub] 启动状态发布节点...")
        
        # 参数配置
        self.dof = rospy.get_param("~dof", 6)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)  # Hz
        
        # 状态缓存
        self.current_joints = [0.0] * self.dof
        self.current_gripper = 0
        self.last_state_time = rospy.Time.now()
        
        # 发布者
        self.state_pub = rospy.Publisher('/robot_state', Float64MultiArray, queue_size=10)
        
        # 订阅 RealMan 驱动状态
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.arm_state_callback)
        
        # 定时发布
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_state)
        
        rospy.loginfo(f"[RealManPub] 状态发布节点初始化完成，频率: {self.publish_rate}Hz")
    
    def arm_state_callback(self, msg):
        """接收 RealMan 驱动状态"""
        if len(msg.joint) >= self.dof:
            self.current_joints = list(msg.joint[:self.dof])
            self.last_state_time = rospy.Time.now()
        
        # 尝试从扩展字段获取夹爪状态（如果有）
        # RealMan 的夹爪状态可能需要从其他话题获取，这里暂设为0
        self.current_gripper = 0
    
    def publish_state(self, event):
        """发布机械臂状态"""
        # 检查状态是否过期（2秒无更新视为超时）
        time_since_update = (rospy.Time.now() - self.last_state_time).to_sec()
        if time_since_update > 2.0:
            rospy.logwarn_throttle(5.0, f"[RealManPub] 状态更新超时: {time_since_update:.1f}s")
        
        # 构造状态消息 [j1, j2, j3, j4, j5, j6, gripper]
        full_state = self.current_joints[:6] + [float(self.current_gripper)]
        
        msg = Float64MultiArray()
        msg.data = full_state
        self.state_pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"[RealManPub] 发布状态: {[f'{x:.2f}' for x in full_state]}")


def main():
    try:
        node = RealManStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
