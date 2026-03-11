#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealMan (瑞尔曼)机械臂安全遥操控制节点
包含完整的数据错误处理机制与安全锁功能

安全特性：
1. 速度限制（参考ARX）
2. 关节限位检查
3. 错误状态监控与自动急停
4. 通信超时检测
5. 使能控制
"""

import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, Bool, UInt16MultiArray
from rm_msgs.msg import JointPos, Gripper_Set, Arm_Current_State, Stop


class RealManSafeTeleop:
    """
    瑞尔曼机械臂安全遥操节点
    集成多种安全保护机制
    """
    
    # 瑞尔曼RM-65/75/ECO65 系列关节限位（度）
    JOINT_LIMITS = {
        'RM_65': {
            'min': [-178, -130, -135, -178, -130, -178],
            'max': [178, 130, 135, 178, 130, 178]
        },
        'RM_75': {
            'min': [-178, -130, -135, -178, -130, -178],
            'max': [178, 130, 135, 178, 130, 178]
        },
        'ECO_65': {
            'min': [-178, -130, -135, -178, -130, -178],
            'max': [178, 130, 135, 178, 130, 178]
        }
    }
    
    # 瑞尔曼错误码（关键）
    ERR_CODES = {
        0x0000: "正常",
        0x1001: "关节通信异常",
        0x1002: "目标角度超过限位",
        0x1003: "不可达（奇异点）",
        0x1004: "内核通信错误",
        0x1005: "关节总线错误",
        0x1006: "规划层内核错误",
        0x1007: "关节超速",
        0x1008: "末端接口板无法连接",
        0x1009: "超速度限制",
        0x100A: "超加速度限制",
        0x100B: "关节抱闸未打开",
        0x100C: "拖动示教超速",
        0x100D: "机械臂碰撞",
        0x1010: "关节掉使能",
    }
    
    def __init__(self):
        rospy.init_node("realman_safe_teleop")
        rospy.loginfo("[RealManSafe] 启动安全遥操节点...")
        
        # ====== 参数配置 ======
        self.arm_model = rospy.get_param("~arm_model", "RM_65")
        self.arm_ip = rospy.get_param("~arm_ip", "192.168.1.18")
        self.dof = rospy.get_param("~dof", 6)
        self.init_qpos_deg = rospy.get_param("~init_qpos", [0.0, -20.0, -90.0, 0.0, 90.0, 0.0])
        
        # 速度限制参数（参考ARX）
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 30.0)  # 度/秒
        self.max_gripper_speed = rospy.get_param("~max_gripper_speed", 200.0)  # 位置/秒
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)  # Hz
        
        # 安全锁参数
        self.enable_emergency_stop = rospy.get_param("~enable_emergency_stop", True)
        self.error_check_interval = rospy.get_param("~error_check_interval", 0.1)  # 秒
        self.comm_timeout = rospy.get_param("~comm_timeout", 0.5)  # 通信超时（秒）
        
        # ====== 状态变量 ======
        self.target_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.last_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.gripper_position = 0
        self.last_gripper_position = 0
        self.angle_offset = [0.0] * 7
        
        # 速度限制状态（参考ARX）
        self._last_cmd_time = time.time()
        self._inited_cmd = False
        
        # 安全锁状态
        self.is_arm_enabled = False
        self.is_emergency_stopped = False
        self.last_arm_error = []
        self.last_state_time = time.time()
        
        # 关节限位
        if self.arm_model in self.JOINT_LIMITS:
            self.joint_min = np.array(self.JOINT_LIMITS[self.arm_model]['min'][:self.dof])
            self.joint_max = np.array(self.JOINT_LIMITS[self.arm_model]['max'][:self.dof])
        else:
            # 默认限位
            self.joint_min = np.array([-178, -130, -135, -178, -130, -178][:self.dof])
            self.joint_max = np.array([178, 130, 135, 178, 130, 178][:self.dof])
        
        # 关节映射
        self.joint_scale = np.array(rospy.get_param("~joint_scale", [1.0]*6))
        self.joint_invert = np.array(rospy.get_param("~joint_invert", [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]))
        
        # 夹爪映射
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_range = 1000
        
        # ====== ROS发布者 ======
        self.joint_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=10)
        self.gripper_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
        self.action_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/realman_teleop/status', Bool, queue_size=10)
        
        # 急停发布者
        if self.enable_emergency_stop:
            self.stop_pub = rospy.Publisher('/rm_driver/Stop', Stop, queue_size=1)
            self.emergency_pub = rospy.Publisher('/realman_teleop/emergency', Bool, queue_size=1)
        
        # ====== ROS订阅者 ======
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.servo_callback)
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.arm_state_callback)
        
        # 安全控制订阅
        rospy.Subscriber('/realman_teleop/enable', Bool, self.enable_callback)
        rospy.Subscriber('/realman_teleop/emergency_stop', Bool, self.emergency_callback)
        
        # ====== 定时器 ======
        # 错误检查定时器
        rospy.Timer(rospy.Duration(self.error_check_interval), self.safety_check)
        
        rospy.loginfo(f"[RealManSafe] 机械臂型号: {self.arm_model}, DOF: {self.dof}")
        rospy.loginfo(f"[RealManSafe] 速度限制: {self.max_joint_speed}°/s")
        rospy.loginfo(f"[RealManSafe] 急停功能: {'启用' if self.enable_emergency_stop else '禁用'}")
        rospy.loginfo(f"[RealManSafe] 等待使能信号 (/realman_teleop/enable)...")

    def enable_callback(self, msg):
        """使能/失能控制"""
        if msg.data and not self.is_arm_enabled:
            self.is_arm_enabled = True
            self.is_emergency_stopped = False
            rospy.loginfo("[RealManSafe] ✅ 机械臂已使能，遥操开始")
        elif not msg.data and self.is_arm_enabled:
            self.is_arm_enabled = False
            rospy.loginfo("[RealManSafe] ⛔ 机械臂已失能，遥操暂停")

    def emergency_callback(self, msg):
        """急停信号处理"""
        if msg.data and not self.is_emergency_stopped:
            self.trigger_emergency_stop("外部急停信号触发")

    def trigger_emergency_stop(self, reason):
        """触发急停"""
        if self.is_emergency_stopped:
            return
        
        self.is_emergency_stopped = True
        self.is_arm_enabled = False
        rospy.logerr(f"[RealManSafe] 🚨 急停触发！原因: {reason}")
        
        # 发送急停命令
        if self.enable_emergency_stop:
            stop_msg = Stop()
            stop_msg.stop_mode = 0  # 立即停止
            self.stop_pub.publish(stop_msg)
            self.emergency_pub.publish(Bool(True))
        
        rospy.logerr("[RealManSafe] 机械臂已停止，请检查安全后复位")

    def reset_emergency_stop(self):
        """复位急停"""
        if not self.is_emergency_stopped:
            return
        
        rospy.logwarn("[RealManSafe] 正在复位急停...")
        self.is_emergency_stopped = False
        self.emergency_pub.publish(Bool(False))
        rospy.loginfo("[RealManSafe] 急停已复位，等待重新使能")

    def servo_callback(self, msg):
        """处理主臂角度输入"""
        if not self.is_arm_enabled or self.is_emergency_stopped:
            return
        
        if len(msg.data) < 7:
            rospy.logwarn_throttle(1.0, f"[RealManSafe] 接收到异常数据长度: {len(msg.data)}")
            return
        
        self.angle_offset = list(msg.data)
        self._update_target_angles()

    def _update_target_angles(self):
        """更新目标角度（带安全限制）"""
        new_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        
        # 应用偏移和映射
        for i in range(min(self.dof, 6)):
            offset = self.angle_offset[i] * self.joint_scale[i] * self.joint_invert[i]
            new_angles[i] = self.init_qpos_deg[i] + offset
        
        # ====== 关节限位检查 ======
        for i in range(self.dof):
            if new_angles[i] < self.joint_min[i] or new_angles[i] > self.joint_max[i]:
                rospy.logerr_throttle(1.0, 
                    f"[RealManSafe] ⚠️ 关节{i+1}角度越限: {new_angles[i]:.2f}° "
                    f"(限位: [{self.joint_min[i]}, {self.joint_max[i]}])")
                # 裁剪到限位内
                new_angles[i] = np.clip(new_angles[i], self.joint_min[i], self.joint_max[i])
        
        self.target_joint_angles = new_angles
        
        # 夹爪位置
        gripper_deg = self.angle_offset[6]
        self.gripper_position = self._map_gripper(gripper_deg)

    def _map_gripper(self, angle_deg):
        """映射夹爪位置"""
        grip_norm = (angle_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
        grip_norm = np.clip(grip_norm, 0.0, 1.0)
        return int(grip_norm * self.gripper_range)

    def arm_state_callback(self, msg):
        """处理机械臂状态反馈"""
        self.last_state_time = time.time()
        
        # 检查错误码
        if len(msg.err) > 0:
            new_errors = [e for e in msg.err if e != 0]
            if new_errors != self.last_arm_error:
                self.last_arm_error = new_errors
                for err in new_errors:
                    err_msg = self.ERR_CODES.get(err, f"未知错误(0x{err:04X})")
                    rospy.logerr(f"[RealManSafe] 机械臂错误: {err_msg} (0x{err:04X})")

    def safety_check(self, event=None):
        """安全检查定时器回调"""
        # 检查通信超时
        time_since_last_state = time.time() - self.last_state_time
        if time_since_last_state > self.comm_timeout:
            rospy.logerr_throttle(5.0, 
                f"[RealManSafe] ⚠️ 通信超时: {time_since_last_state:.2f}s 未收到状态")
            if self.is_arm_enabled and self.enable_emergency_stop:
                self.trigger_emergency_stop("通信超时")
        
        # 检查是否在使能状态但没有状态反馈
        if self.is_arm_enabled and len(self.last_arm_error) > 0:
            # 检查严重错误
            critical_errors = [0x100D, 0x1007, 0x1009, 0x100A]  # 碰撞、超速等
            if any(err in self.last_arm_error for err in critical_errors):
                self.trigger_emergency_stop(f"检测到严重错误: {self.last_arm_error}")

    def _apply_velocity_limit(self, desired_angles):
        """
        应用速度限制（参考ARX实现）
        限制每周期最大角度变化
        """
        current_time = time.time()
        dt = current_time - self._last_cmd_time
        self._last_cmd_time = current_time
        
        if not self._inited_cmd:
            self._inited_cmd = True
            self.last_joint_angles = desired_angles.copy()
            return desired_angles
        
        # 计算最大允许变化量
        max_delta = self.max_joint_speed * dt  # 度
        
        # 限制每个关节的变化
        limited_angles = np.zeros_like(desired_angles)
        for i in range(self.dof):
            delta = desired_angles[i] - self.last_joint_angles[i]
            delta_clipped = np.clip(delta, -max_delta, max_delta)
            limited_angles[i] = self.last_joint_angles[i] + delta_clipped
        
        self.last_joint_angles = limited_angles.copy()
        return limited_angles

    def _apply_gripper_velocity_limit(self, desired_pos):
        """夹爪速度限制"""
        current_time = time.time()
        dt = current_time - getattr(self, '_last_gripper_time', current_time)
        self._last_gripper_time = current_time
        
        max_delta = self.max_gripper_speed * dt
        delta = desired_pos - self.last_gripper_position
        delta_clipped = int(np.clip(delta, -max_delta, max_delta))
        
        limited_pos = self.last_gripper_position + delta_clipped
        self.last_gripper_position = limited_pos
        return limited_pos

    def _publish_commands(self):
        """发布控制命令"""
        if not self.is_arm_enabled or self.is_emergency_stopped:
            return
        
        # 应用速度限制
        safe_angles = self._apply_velocity_limit(self.target_joint_angles)
        safe_gripper = self._apply_gripper_velocity_limit(self.gripper_position)
        
        # 发布关节命令
        joint_msg = JointPos()
        joint_msg.joint = safe_angles.tolist()
        if self.dof > 6:
            joint_msg.expand = 0.0
        self.joint_pub.publish(joint_msg)
        
        # 发布夹爪命令
        gripper_msg = Gripper_Set()
        gripper_msg.position = safe_gripper
        self.gripper_pub.publish(gripper_msg)
        
        # 发布动作反馈
        action_msg = Float64MultiArray()
        action_data = safe_angles.tolist()[:6] + [safe_gripper]
        action_msg.data = action_data
        self.action_pub.publish(action_msg)

    def run(self):
        """主循环"""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("[RealManSafe] 安全遥操节点运行中...")
        rospy.loginfo("[RealManSafe] 提示: 发布 Bool(True) 到 /realman_teleop/enable 使能机械臂")
        
        while not rospy.is_shutdown():
            try:
                # 发布状态
                self.status_pub.publish(Bool(self.is_arm_enabled and not self.is_emergency_stopped))
                
                # 发布控制命令
                self._publish_commands()
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"[RealManSafe] 运行时错误: {e}")
                continue
        
        # 退出前确保机械臂安全
        if self.is_arm_enabled:
            rospy.logwarn("[RealManSafe] 节点退出，自动失能机械臂")
            self.is_arm_enabled = False


def main():
    """主函数"""
    try:
        node = RealManSafeTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[RealManSafe] 节点初始化失败: {e}")


if __name__ == "__main__":
    main()
