#!/usr/bin/env python3
"""
Servo 到 RealMan 的遥操作控制节点
功能：订阅 /servo_angles 话题，控制 RealMan 机械臂跟随运动
架构：参考 servo2xarm.py 的简洁设计
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rm_msgs.msg import JointPos, Gripper_Set


class ServoToRealMan:
    """Servo 数据转换为 RealMan 控制命令"""
    
    def __init__(self):
        rospy.init_node('servo_to_realman')
        rospy.loginfo("[Servo2RealMan] 启动遥操作控制节点...")
        
        # ====== 参数配置 ======
        self.dof = rospy.get_param("~dof", 6)
        self.control_rate = rospy.get_param("~control_rate", 1000.0)  # Hz
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 30.0)  # 度/秒
        
        # RealMan 初始姿态（零点姿态）
        self.init_qpos_deg = rospy.get_param("~init_qpos", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 关节映射参数（缩放和方向）
        self.joint_scale = np.array(rospy.get_param("~joint_scale", [0.015] * 6))
        self.joint_invert = np.array(rospy.get_param("~joint_invert", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0]))
        
        # 夹爪参数
        self.gripper_range = 1000
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        
        # 平滑滤波系数
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.3)
        
        # ====== 状态变量 ======
        self.servo_angles = [0.0] * 7  # 从 /servo_angles 接收到的数据
        self.target_joints = np.array(self.init_qpos_deg, dtype=np.float32)
        self.smoothed_joints = np.array(self.init_qpos_deg, dtype=np.float32)
        self.last_joint_cmd = np.array(self.init_qpos_deg, dtype=np.float32)
        self.gripper_position = 0
        
        self.last_cmd_time = rospy.Time.now()
        self._inited_cmd = False
        
        # ====== ROS 发布者 ======
        self.joint_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=10)
        self.gripper_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
        self.action_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)
        
        # ====== ROS 订阅者 ======
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.servo_callback)
        
        rospy.loginfo("[Servo2RealMan] 初始化完成，等待 /servo_angles 数据...")
        rospy.loginfo(f"[Servo2RealMan] 初始姿态: {self.init_qpos_deg}")
        rospy.loginfo(f"[Servo2RealMan] 关节缩放: {self.joint_scale}")
        rospy.loginfo(f"[Servo2RealMan] 关节方向: {self.joint_invert}")
    
    def servo_callback(self, msg):
        """
        接收 Servo 角度数据（来自遥操作主臂）
        数据格式: [j1, j2, j3, j4, j5, j6, gripper] （单位：度，相对于零点偏移）
        """
        if len(msg.data) >= 7:
            self.servo_angles = list(msg.data[:7])
        else:
            rospy.logwarn_throttle(1.0, f"[Servo2RealMan] 收到数据长度不足: {len(msg.data)}")
    
    def process_servo_data(self):
        """
        处理 Servo 数据，转换为 RealMan 目标角度
        映射公式: target = init_qpos + servo_offset * scale * invert
        """
        # 计算目标角度（带缩放和方向）
        new_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        for i in range(min(self.dof, 6)):
            offset = self.servo_angles[i] * self.joint_scale[i] * self.joint_invert[i]
            new_angles[i] = self.init_qpos_deg[i] + offset
        
        # 低通平滑滤波
        self.smoothed_joints = (
            self.filter_alpha * new_angles + 
            (1 - self.filter_alpha) * self.smoothed_joints
        )
        
        # 映射夹爪位置
        gripper_deg = self.servo_angles[6] if len(self.servo_angles) > 6 else 0
        gripper_norm = (gripper_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
        gripper_norm = np.clip(gripper_norm, 0.0, 1.0)
        self.gripper_position = int(gripper_norm * self.gripper_range)
        
        return self.smoothed_joints.copy(), self.gripper_position
    
    def apply_velocity_limit(self, desired_angles):
        """应用速度限制，防止关节运动过快"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_cmd_time).to_sec()
        
        # 时间异常处理
        if dt < 0 or dt > 1.0:
            dt = 1.0 / self.control_rate
        
        self.last_cmd_time = current_time
        
        if not self._inited_cmd:
            self._inited_cmd = True
            self.last_joint_cmd = desired_angles.copy()
            return desired_angles
        
        # 计算最大允许变化量
        max_delta = self.max_joint_speed * dt
        
        # 限制每个关节的变化
        limited_angles = np.zeros_like(desired_angles)
        for i in range(self.dof):
            delta = desired_angles[i] - self.last_joint_cmd[i]
            delta_clipped = np.clip(delta, -max_delta, max_delta)
            limited_angles[i] = self.last_joint_cmd[i] + delta_clipped
        
        self.last_joint_cmd = limited_angles.copy()
        return limited_angles
    
    def publish_commands(self, joint_angles, gripper_pos):
        """发布控制命令到 RealMan 驱动"""
        # 发布关节位置命令
        joint_msg = JointPos()
        joint_msg.joint = joint_angles.tolist()
        if self.dof > 6:
            joint_msg.expand = 0.0
        self.joint_pub.publish(joint_msg)
        
        # 发布夹爪命令
        gripper_msg = Gripper_Set()
        gripper_msg.position = int(gripper_pos)
        self.gripper_pub.publish(gripper_msg)
        
        # 发布动作反馈（用于数据记录）
        action_msg = Float64MultiArray()
        action_data = list(joint_angles)[:6] + [int(gripper_pos)]
        action_msg.data = action_data
        self.action_pub.publish(action_msg)
    
    def run(self):
        """主循环：处理数据并发送控制命令"""
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo(f"[Servo2RealMan] 控制循环启动，频率: {self.control_rate}Hz")
        
        while not rospy.is_shutdown():
            # 处理 servo 数据
            target_joints, gripper_pos = self.process_servo_data()
            
            # 应用速度限制
            safe_joints = self.apply_velocity_limit(target_joints)
            
            # 发布控制命令
            self.publish_commands(safe_joints, gripper_pos)
            
            rospy.loginfo_throttle(1.0, 
                f"[Servo2RealMan] 目标: {[f'{x:.2f}' for x in safe_joints]}, 夹爪: {gripper_pos}")
            
            rate.sleep()


def main():
    try:
        node = ServoToRealMan()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
