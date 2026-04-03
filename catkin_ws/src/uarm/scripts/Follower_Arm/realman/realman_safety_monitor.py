#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RealMan (瑞尔曼)机械臂安全监控节点
实时监控机械臂状态、错误码，提供安全控制接口
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool, UInt16MultiArray, String
from rm_msgs.msg import Arm_Current_State, Stop


class RealManSafetyMonitor:
    """
    瑞尔曼机械臂安全监控器
    功能：
    - 实时监控机械臂错误状态
    - 显示关节角度和速度
    - 提供安全控制按钮（急停、复位、使能）
    - 错误历史记录
    """
    
    # 瑞尔曼错误码详解
    ERR_CODES_DETAIL = {
        0x0000: ("正常", "info"),
        0x1001: ("关节通信异常", "error"),
        0x1002: ("目标角度超过限位", "warning"),
        0x1003: ("不可达（奇异点）", "warning"),
        0x1004: ("内核通信错误", "error"),
        0x1005: ("关节总线错误", "error"),
        0x1006: ("规划层内核错误", "error"),
        0x1007: ("关节超速", "critical"),
        0x1008: ("末端接口板无法连接", "error"),
        0x1009: ("超速度限制", "warning"),
        0x100A: ("超加速度限制", "warning"),
        0x100B: ("关节抱闸未打开", "warning"),
        0x100C: ("拖动示教超速", "warning"),
        0x100D: ("机械臂碰撞检测", "critical"),
        0x100E: ("无该工作坐标系", "warning"),
        0x100F: ("无该工具坐标系", "warning"),
        0x1010: ("关节掉使能", "error"),
    }
    
    def __init__(self):
        rospy.init_node("realman_safety_monitor")
        rospy.loginfo("[SafetyMonitor] 启动安全监控节点...")
        
        # 参数
        self.monitor_rate = rospy.get_param("~monitor_rate", 10.0)  # Hz
        self.history_size = rospy.get_param("~history_size", 100)  # 错误历史记录数
        
        # 状态变量
        self.current_joint_angles = [0.0] * 6
        self.current_errors = []
        self.error_history = []
        self.is_arm_enabled = False
        self.is_emergency_stopped = False
        self.last_update_time = rospy.Time.now()
        
        # 统计
        self.error_count = 0
        self.start_time = rospy.Time.now()
        
        # ROS订阅
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.arm_state_callback)
        rospy.Subscriber('/realman_teleop/status', Bool, self.teleop_status_callback)
        rospy.Subscriber('/realman_teleop/emergency', Bool, self.emergency_callback)
        
        # ROS发布（控制接口）
        self.enable_pub = rospy.Publisher('/realman_teleop/enable', Bool, queue_size=1)
        self.emergency_pub = rospy.Publisher('/realman_teleop/emergency_stop', Bool, queue_size=1)
        self.stop_pub = rospy.Publisher('/rm_driver/Stop', Stop, queue_size=1)
        
        # 状态发布
        self.status_pub = rospy.Publisher('/realman_monitor/status', String, queue_size=10)
        
        # 定时器
        rospy.Timer(rospy.Duration(1.0 / self.monitor_rate), self.publish_status)
        rospy.Timer(rospy.Duration(5.0), self.print_summary)
        
        rospy.loginfo("[SafetyMonitor] 监控节点就绪")
        rospy.loginfo("[SafetyMonitor] 服务调用示例:")
        rospy.loginfo("  使能: rostopic pub /realman_teleop/enable std_msgs/Bool '{data: true}'")
        rospy.loginfo("  急停: rostopic pub /realman_teleop/emergency_stop std_msgs/Bool '{data: true}'")

    def arm_state_callback(self, msg):
        """处理机械臂状态"""
        self.last_update_time = rospy.Time.now()
        
        # 更新关节角度
        if len(msg.joint) >= 6:
            self.current_joint_angles = list(msg.joint[:6])
        
        # 检查错误
        if len(msg.err) > 0:
            new_errors = [e for e in msg.err if e != 0]
            
            # 检测新错误
            for err in new_errors:
                if err not in self.current_errors:
                    self._handle_new_error(err)
            
            self.current_errors = new_errors
            
            # 检查严重错误并自动急停
            critical_errors = [0x100D, 0x1007]  # 碰撞、超速
            if any(err in critical_errors for err in new_errors):
                if not self.is_emergency_stopped:
                    rospy.logerr("[SafetyMonitor] ⚠️ 检测到严重错误，自动触发急停！")
                    self.trigger_emergency_stop()

    def _handle_new_error(self, err_code):
        """处理新错误"""
        err_info = self.ERR_CODES_DETAIL.get(err_code, (f"未知错误(0x{err_code:04X})", "warning"))
        err_msg, level = err_info
        
        self.error_count += 1
        timestamp = rospy.Time.now()
        
        # 记录到历史
        self.error_history.append({
            'time': timestamp,
            'code': err_code,
            'msg': err_msg,
            'level': level
        })
        
        # 限制历史记录大小
        if len(self.error_history) > self.history_size:
            self.error_history.pop(0)
        
        # 根据级别打印
        log_msg = f"[SafetyMonitor] 错误 0x{err_code:04X}: {err_msg}"
        if level == "critical":
            rospy.logerr(f"🚨 {log_msg}")
        elif level == "error":
            rospy.logerr(log_msg)
        elif level == "warning":
            rospy.logwarn(log_msg)
        else:
            rospy.loginfo(log_msg)

    def teleop_status_callback(self, msg):
        """遥操状态"""
        self.is_arm_enabled = msg.data

    def emergency_callback(self, msg):
        """急停状态"""
        self.is_emergency_stopped = msg.data
        if msg.data:
            rospy.logerr("[SafetyMonitor] 🛑 机械臂已急停")

    def trigger_emergency_stop(self):
        """触发急停"""
        self.is_emergency_stopped = True
        
        # 发布急停命令
        stop_msg = Stop()
        stop_msg.stop_mode = 0
        self.stop_pub.publish(stop_msg)
        self.emergency_pub.publish(Bool(True))
        
        rospy.logerr("[SafetyMonitor] 🚨 急停已触发！")

    def reset_emergency_stop(self):
        """复位急停"""
        self.is_emergency_stopped = False
        self.emergency_pub.publish(Bool(False))
        rospy.loginfo("[SafetyMonitor] 急停已复位")

    def enable_arm(self, enable=True):
        """使能/失能机械臂"""
        self.enable_pub.publish(Bool(enable))
        rospy.loginfo(f"[SafetyMonitor] 机械臂{'使能' if enable else '失能'}指令已发送")

    def publish_status(self, event=None):
        """发布状态信息"""
        status = {
            'enabled': self.is_arm_enabled,
            'emergency': self.is_emergency_stopped,
            'errors': len(self.current_errors),
            'error_codes': [f"0x{e:04X}" for e in self.current_errors],
            'joints': [f"{j:.1f}" for j in self.current_joint_angles],
            'uptime': (rospy.Time.now() - self.start_time).to_sec()
        }
        
        status_str = f"状态: {'运行中' if self.is_arm_enabled else '已停止'} | "
        status_str += f"错误: {len(self.current_errors)} | "
        status_str += f"关节: {status['joints']}"
        
        self.status_pub.publish(String(data=status_str))

    def print_summary(self, event=None):
        """打印状态摘要"""
        uptime = (rospy.Time.now() - self.start_time).to_sec()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("[SafetyMonitor] 运行状态摘要")
        rospy.loginfo(f"  运行时间: {uptime/60:.1f} 分钟")
        rospy.loginfo(f"  遥操状态: {'✅ 运行中' if self.is_arm_enabled else '⏸️ 已停止'}")
        rospy.loginfo(f"  急停状态: {'🚨 已触发' if self.is_emergency_stopped else '✅ 正常'}")
        rospy.loginfo(f"  当前错误: {len(self.current_errors)} 个")
        
        if self.current_errors:
            for err in self.current_errors:
                err_msg = self.ERR_CODES_DETAIL.get(err, ("未知", "unknown"))[0]
                rospy.loginfo(f"    - 0x{err:04X}: {err_msg}")
        
        rospy.loginfo(f"  累计错误: {self.error_count} 次")
        rospy.loginfo(f"  关节角度: {[f'{j:.1f}°' for j in self.current_joint_angles]}")
        rospy.loginfo("=" * 60)

    def run(self):
        """主循环"""
        rospy.spin()


def main():
    """主函数"""
    try:
        node = RealManSafetyMonitor()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
