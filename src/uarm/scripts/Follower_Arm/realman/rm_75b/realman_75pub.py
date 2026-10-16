#!/usr/bin/env python3
"""
RealMan RM75(7轴) 机械臂状态发布节点
功能：订阅 /rm_driver/Arm_Current_State，整理为统一的 /robot_state 供数据记录
说明：常用配置集中在文件顶部"快捷配置区"；rosparam 可覆盖同名默认值
      默认无夹爪：/robot_state = [j1..j7]（单位度）
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rm_msgs.msg import Arm_Current_State

# ==================== 快捷配置区（一般只改这里） ====================
DOF = 7                  # 机械臂自由度：RM75=7
PUBLISH_RATE = 10.0      # 状态发布频率 (Hz)
APPEND_GRIPPER = False   # 状态数组末尾是否附加夹爪位（当前夹爪状态固定为 0）
# ===================================================================


class RealManStatePublisher:
    """RealMan 机械臂状态发布节点"""

    def __init__(self):
        rospy.init_node('realman_75state_publisher')
        rospy.loginfo("[RealMan75Pub] 启动状态发布节点（RM75, 7轴）...")

        # 参数配置（rosparam 可覆盖顶部快捷配置区）
        self.dof = rospy.get_param("~dof", DOF)
        self.publish_rate = rospy.get_param("~publish_rate", PUBLISH_RATE)
        self.append_gripper = rospy.get_param("~append_gripper", APPEND_GRIPPER)

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

        rospy.loginfo(f"[RealMan75Pub] 状态发布节点初始化完成，频率: {self.publish_rate}Hz，维度: {self.dof + 1 if self.append_gripper else self.dof}")

    def arm_state_callback(self, msg):
        """接收 RealMan 驱动状态（7轴时 msg.joint 为 7 维，单位度）"""
        if len(msg.joint) >= self.dof:
            self.current_joints = list(msg.joint[:self.dof])
            self.last_state_time = rospy.Time.now()
        else:
            rospy.logwarn_throttle(5.0, f"[RealMan75Pub] 状态维度不足: {len(msg.joint)} < {self.dof}")

        # 夹爪状态暂固定为 0（RealMan 夹爪状态需从其他话题获取）
        self.current_gripper = 0

    def publish_state(self, event):
        """发布机械臂状态"""
        # 检查状态是否过期（2秒无更新视为超时）
        time_since_update = (rospy.Time.now() - self.last_state_time).to_sec()
        if time_since_update > 2.0:
            rospy.logwarn_throttle(5.0, f"[RealMan75Pub] 状态更新超时: {time_since_update:.1f}s")

        # 构造状态消息 [j1, ..., j7]（APPEND_GRIPPER=True 时末尾附加夹爪位）
        full_state = list(self.current_joints)
        if self.append_gripper:
            full_state += [float(self.current_gripper)]

        msg = Float64MultiArray()
        msg.data = full_state
        self.state_pub.publish(msg)
        rospy.loginfo_throttle(1.0, f"[RealMan75Pub] 发布状态: {[f'{x:.2f}' for x in full_state]}")


def main():
    try:
        node = RealManStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
