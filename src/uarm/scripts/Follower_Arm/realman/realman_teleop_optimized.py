#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time
import serial
from collections import deque
from std_msgs.msg import Float64MultiArray, Bool, UInt16MultiArray
from rm_msgs.msg import JointPos, Gripper_Set, Arm_Current_State, Stop


class RealManOptimizedTeleop:
    """
    瑞尔曼机械臂优化遥操节点
    基于 teleop_sim_trigger_test_optimized_v2.py 的优化方案
    """
    
    # 关节限位
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
    
    # 错误码
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
        rospy.init_node("realman_optimized_teleop")
        rospy.loginfo("[RealManOpt] 启动优化遥操节点...")
        
        # ====== 参数配置 ======
        self.arm_model = rospy.get_param("~arm_model", "RM_65")
        self.dof = rospy.get_param("~dof", 6)
        
        # 零点姿态配置（关键参数）
        # servo_reader.py 发布的是舵机相对零点的偏移量（度数）
        # 这里配置瑞尔曼机械臂的对应零点姿态
        # 
        # 常用配置：
        # 1. 直立姿态（全零）: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # 2. 标准初始姿态:   [0.0, -20.0, -90.0, 0.0, 90.0, 0.0]
        # 3. 工作姿态:       [0.0, -45.0, -90.0, 0.0, 45.0, 0.0]
        self.init_qpos_deg = rospy.get_param("~init_qpos", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 优化参数（新增）
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 30.0)  # 度/秒
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)  # Hz
        self.comm_timeout = rospy.get_param("~comm_timeout", 0.5)  # 通信超时
        
        # ====== 延迟容错优化参数（从仿真引入）=======
        self.angle_queue_size = rospy.get_param("~angle_queue_size", 1)  # 只保留最新数据
        self.prediction_enabled = rospy.get_param("~prediction_enabled", True)  # 启用预测
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.3)  # 低通滤波系数
        self.packet_loss_tolerance = rospy.get_param("~packet_loss_tolerance", 3)  # 允许连续丢包数
        
        # ====== 舵机主臂控制参数（新增）=======
        self.enable_servo_control = rospy.get_param("~enable_servo_control", True)  # 是否控制舵机
        self.servo_serial_port = rospy.get_param("~servo_serial_port", "/dev/ttyUSB0")  # 舵机串口
        self.servo_baudrate = rospy.get_param("~servo_baudrate", 115200)  # 波特率
        self.release_torque_on_init = rospy.get_param("~release_torque_on_init", True)  # 启动时释放力矩
        
        # 舵机串口对象（初始化为None）
        self.servo_ser = None
        if self.enable_servo_control:
            self._init_servo_serial()
        
        # ====== 状态变量 ======
        self.target_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.last_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.smoothed_angles = np.array(self.init_qpos_deg, dtype=np.float32)  # 平滑后的角度
        self.gripper_position = 0
        self.last_gripper_position = 0  # 夹爪速度限制用
        
        # 预测相关（从仿真引入）
        self.angle_history = deque(maxlen=5)  # 历史角度用于预测
        self.velocity_estimate = np.zeros(7)  # 速度估计
        self.last_input_time = time.time()
        
        # 丢包容错
        self.consecutive_packet_loss = 0
        self.last_valid_input = np.zeros(7)
        
        # 速度限制
        self._last_cmd_time = time.time()
        self._inited_cmd = False
        
        # 安全锁
        self.is_arm_enabled = False
        self.is_emergency_stopped = False
        self.last_arm_error = []
        self.last_state_time = time.time()
        
        # 关节限位
        if self.arm_model in self.JOINT_LIMITS:
            self.joint_min = np.array(self.JOINT_LIMITS[self.arm_model]['min'][:self.dof])
            self.joint_max = np.array(self.JOINT_LIMITS[self.arm_model]['max'][:self.dof])
        else:
            self.joint_min = np.array([-178, -130, -135, -178, -130, -178][:self.dof])
            self.joint_max = np.array([178, 130, 135, 178, 130, 178][:self.dof])
        
        # 映射
        self.joint_scale = np.array(rospy.get_param("~joint_scale", [1.0]*6))
        self.joint_invert = np.array(rospy.get_param("~joint_invert", [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]))
        
        # 夹爪映射
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_range = 1000
        
        # ====== ROS发布者 ======
        self.joint_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=1)  # 改为1减少延迟
        self.gripper_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=1)
        self.action_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/realman_teleop/status', Bool, queue_size=1)
        self.stop_pub = rospy.Publisher('/rm_driver/Stop', Stop, queue_size=1)
        self.emergency_pub = rospy.Publisher('/realman_teleop/emergency', Bool, queue_size=1)
        
        # ====== ROS订阅者（优化：使用队列）=======
        self.angle_buffer = deque(maxlen=self.angle_queue_size)
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.servo_callback_optimized)
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.arm_state_callback)
        rospy.Subscriber('/realman_teleop/enable', Bool, self.enable_callback)
        rospy.Subscriber('/realman_teleop/emergency_stop', Bool, self.emergency_callback)
        
        # 舵机释放力矩控制
        if self.enable_servo_control:
            rospy.Subscriber('/realman_teleop/release_servos', Bool, self.release_servos_callback)
        
        # ====== 定时器 ======
        rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        rospy.loginfo(f"[RealManOpt] 机械臂型号: {self.arm_model}, DOF: {self.dof}")
        rospy.loginfo(f"[RealManOpt] 零点姿态: {self.init_qpos_deg}")
        rospy.loginfo(f"[RealManOpt] 关节缩放: {self.joint_scale}")
        rospy.loginfo(f"[RealManOpt] 关节方向: {self.joint_invert}")
        rospy.loginfo(f"[RealManOpt] 速度限制: {self.max_joint_speed}°/s")
        rospy.loginfo(f"[RealManOpt] 预测补偿: {'启用' if self.prediction_enabled else '禁用'}")
        rospy.loginfo(f"[RealManOpt] 滤波系数: {self.filter_alpha}")
        rospy.loginfo(f"[RealManOpt] 舵机控制: {'启用' if self.enable_servo_control else '禁用'}")
        if self.enable_servo_control:
            rospy.loginfo(f"[RealManOpt] 舵机串口: {self.servo_serial_port}:{self.servo_baudrate}")
        rospy.loginfo(f"[RealManOpt] 等待使能信号...")

    # ====== 舵机控制方法（新增）=======
    def _init_servo_serial(self):
        """初始化舵机串口连接"""
        try:
            self.servo_ser = serial.Serial(
                self.servo_serial_port,
                self.servo_baudrate,
                timeout=0.1
            )
            time.sleep(0.1)
            self.servo_ser.reset_input_buffer()
            rospy.loginfo(f"[Servo] 串口已打开: {self.servo_serial_port}")
            
            # 启动时释放力矩（如配置）
            if self.release_torque_on_init:
                self._release_servo_torque()
                
        except serial.SerialException as e:
            rospy.logwarn(f"[Servo] 无法打开串口 {self.servo_serial_port}: {e}")
            rospy.logwarn("[Servo] 将继续运行，但无法直接控制舵机")
            self.servo_ser = None
    
    def _release_servo_torque(self):
        """释放所有舵机力矩（PULK命令）"""
        if self.servo_ser is None:
            rospy.logwarn("[Servo] 串口未打开，无法释放力矩")
            return
        
        rospy.loginfo("[Servo] 正在释放舵机力矩...")
        try:
            for i in range(7):
                cmd = f'#{i:03d}PULK!'.encode('ascii')
                self.servo_ser.write(cmd)
                time.sleep(0.01)  # 10ms间隔确保稳定
            rospy.loginfo("[Servo] ✅ 舵机力矩已释放，可手动调整主臂姿态")
        except Exception as e:
            rospy.logerr(f"[Servo] 释放力矩失败: {e}")
    
    def _hold_servo_position(self, servo_id: int, pwm: int, move_time: int = 30):
        """保持舵机位置（用于锁定状态）"""
        if self.servo_ser is None:
            return
        try:
            cmd = f'#{servo_id:03d}P{pwm:04d}T{move_time:04d}!'.encode('ascii')
            self.servo_ser.write(cmd)
        except Exception as e:
            rospy.logerr(f"[Servo] 保持位置失败: {e}")
    
    def release_servos_callback(self, msg):
        """ROS回调：释放舵机力矩"""
        if msg.data:
            self._release_servo_torque()

    def release_servos(self):
        """外部调用：释放舵机力矩"""
        self._release_servo_torque()
    
    def cleanup(self):
        """清理资源（关闭串口）"""
        # 如果机械臂还在使能状态，先失能
        if self.is_arm_enabled:
            self.is_arm_enabled = False
            rospy.loginfo("[RealManOpt] 机械臂已失能")
        
        # 关闭舵机串口
        if self.servo_ser is not None:
            try:
                self.servo_ser.close()
                rospy.loginfo("[Servo] 串口已关闭")
            except:
                pass
        
        rospy.loginfo("[RealManOpt] 节点已安全退出")

    def enable_callback(self, msg):
        """使能/失能控制"""
        if msg.data and not self.is_arm_enabled:
            self.is_arm_enabled = True
            self.is_emergency_stopped = False
            # 重置状态
            self.angle_history.clear()
            self.velocity_estimate = np.zeros(7)
            self.consecutive_packet_loss = 0
            rospy.loginfo("[RealManOpt] ✅ 机械臂已使能，遥操开始")
        elif not msg.data and self.is_arm_enabled:
            self.is_arm_enabled = False
            rospy.loginfo("[RealManOpt] ⛔ 机械臂已失能")

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
        rospy.logerr(f"[RealManOpt] 🚨 急停触发！原因: {reason}")
        
        stop_msg = Stop()
        stop_msg.stop_mode = 0
        self.stop_pub.publish(stop_msg)
        self.emergency_pub.publish(Bool(True))

    # ====== 优化1: 使用队列缓冲 + 只保留最新数据 ========
    def servo_callback_optimized(self, msg):
        """
        优化的伺服角度回调
        使用队列只保留最新数据，减少延迟累积
        """
        if not self.is_arm_enabled or self.is_emergency_stopped:
            return
        
        if len(msg.data) < 7:
            rospy.logwarn_throttle(1.0, f"[RealManOpt] 数据长度异常: {len(msg.data)}")
            return
        
        # 将数据放入队列（自动丢弃旧数据）
        input_data = list(msg.data)
        self.angle_buffer.append(input_data)
        self.last_input_time = time.time()
        
        # 立即处理最新数据（从队列取出）
        if len(self.angle_buffer) > 0:
            latest_data = self.angle_buffer[-1]  # 取最新
            self.angle_buffer.clear()  # 清空队列避免累积
            self._process_input_data(latest_data)

    def _process_input_data(self, angle_offset):
        """
        处理输入数据（带预测补偿和平滑滤波）
        
        数据流说明：
        1. servo_reader.py 读取舵机角度，计算相对零点的偏移量（度数）
        2. 通过 /servo_angles 话题发布偏移量
        3. 本节点接收偏移量，映射到瑞尔曼机械臂
        
        映射公式：
            瑞尔曼目标角度 = init_qpos_deg[i] + angle_offset[i] * scale * invert
        
        其中：
            - init_qpos_deg: 瑞尔曼零点姿态（与舵机零点姿态对应）
            - angle_offset: 舵机相对零点的偏移（度数，来自/servo_angles）
            - scale/invert: 关节映射系数（处理主从方向差异）
        """
        current_time = time.time()
        dt = current_time - self.last_input_time
        
        # 基本角度计算
        # new_angles[i] = 瑞尔曼零点 + 舵机偏移量 * 缩放 * 方向
        new_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        for i in range(min(self.dof, 6)):
            offset = angle_offset[i] * self.joint_scale[i] * self.joint_invert[i]
            new_angles[i] = self.init_qpos_deg[i] + offset
        
        # 夹爪（度数直接映射到0-1000范围）
        gripper_deg = angle_offset[6] if len(angle_offset) > 6 else 0
        
        # ====== 优化2: 预测补偿 ========
        if self.prediction_enabled and len(self.angle_history) >= 2:
            # 计算速度
            last_angles = np.array(self.angle_history[-1])
            velocity = (new_angles - last_angles) / max(dt, 0.001)
            self.velocity_estimate = velocity
            
            # 预测未来位置（补偿通信延迟约20-30ms）
            prediction_time = 0.025  # 25ms
            predicted_angles = new_angles + velocity * prediction_time
            
            # 限制预测幅度（避免过度预测）
            max_prediction = 2.0  # 最大预测2度
            for i in range(self.dof):
                prediction_delta = predicted_angles[i] - new_angles[i]
                prediction_delta = np.clip(prediction_delta, -max_prediction, max_prediction)
                new_angles[i] = new_angles[i] + prediction_delta
        
        # 保存历史
        self.angle_history.append(new_angles.copy())
        
        # ====== 优化3: 低通平滑滤波 ========
        # smoothed = alpha * new + (1-alpha) * old
        self.smoothed_angles = (
            self.filter_alpha * new_angles + 
            (1 - self.filter_alpha) * self.smoothed_angles
        )
        
        # 关节限位检查
        for i in range(self.dof):
            if self.smoothed_angles[i] < self.joint_min[i] or self.smoothed_angles[i] > self.joint_max[i]:
                rospy.logerr_throttle(1.0, 
                    f"[RealManOpt] 关节{i+1}越限: {self.smoothed_angles[i]:.2f}°")
                self.smoothed_angles[i] = np.clip(self.smoothed_angles[i], 
                                                   self.joint_min[i], self.joint_max[i])
        
        self.target_joint_angles = self.smoothed_angles
        self.gripper_position = self._map_gripper(gripper_deg)
        
        # 标记为有效数据
        self.consecutive_packet_loss = 0
        self.last_valid_input = angle_offset

    def _map_gripper(self, angle_deg):
        """映射夹爪位置"""
        grip_norm = (angle_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
        grip_norm = np.clip(grip_norm, 0.0, 1.0)
        return int(grip_norm * self.gripper_range)

    def arm_state_callback(self, msg):
        """处理机械臂状态反馈"""
        self.last_state_time = time.time()
        
        if len(msg.err) > 0:
            new_errors = [e for e in msg.err if e != 0]
            if new_errors != self.last_arm_error:
                self.last_arm_error = new_errors
                for err in new_errors:
                    err_msg = self.ERR_CODES.get(err, f"未知错误(0x{err:04X})")
                    rospy.logerr(f"[RealManOpt] 机械臂错误: {err_msg} (0x{err:04X})")

    def safety_check(self, event=None):
        """安全检查定时器回调"""
        current_time = time.time()
        
        # 检查输入数据超时（丢包检测）
        time_since_input = current_time - self.last_input_time
        if time_since_input > self.comm_timeout:
            self.consecutive_packet_loss += 1
            if self.consecutive_packet_loss <= self.packet_loss_tolerance:
                # 丢包容限内，使用最后有效数据并预测
                rospy.logwarn_throttle(2.0, 
                    f"[RealManOpt] 数据延迟: {time_since_input:.2f}s，使用预测值")
                
                # 基于速度预测
                if self.prediction_enabled:
                    prediction_time = time_since_input
                    max_prediction = 5.0  # 最大5度
                    for i in range(self.dof):
                        predicted_delta = self.velocity_estimate[i] * prediction_time
                        predicted_delta = np.clip(predicted_delta, -max_prediction, max_prediction)
                        self.target_joint_angles[i] = self.last_valid_input[i] + predicted_delta
            else:
                # 超过容限，触发急停
                if self.is_arm_enabled:
                    self.trigger_emergency_stop("数据丢失超过容限")
        
        # 检查状态反馈超时
        time_since_state = current_time - self.last_state_time
        if time_since_state > self.comm_timeout * 2:
            rospy.logerr_throttle(5.0, 
                f"[RealManOpt] 状态反馈超时: {time_since_state:.2f}s")
        
        # 检查严重错误
        if self.is_arm_enabled and len(self.last_arm_error) > 0:
            critical_errors = [0x100D, 0x1007, 0x1009, 0x100A]  # 碰撞、超速等
            if any(err in self.last_arm_error for err in critical_errors):
                self.trigger_emergency_stop(f"严重错误: {self.last_arm_error}")

    def _apply_velocity_limit(self, desired_angles):
        """应用速度限制（带预测的速度限制）"""
        current_time = time.time()
        dt = current_time - self._last_cmd_time
        self._last_cmd_time = current_time
        
        if not self._inited_cmd:
            self._inited_cmd = True
            self.last_joint_angles = desired_angles.copy()
            return desired_angles
        
        # 计算最大允许变化量
        max_delta = self.max_joint_speed * dt
        
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
        
        max_delta = 200.0 * dt  # 200位置/秒
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
        
        rospy.loginfo("[RealManOpt] 优化遥操节点运行中...")
        rospy.loginfo("[RealManOpt] 提示: 发布 Bool(True) 到 /realman_teleop/enable 使能")
        rospy.loginfo("[RealManOpt] 提示: 发布 Bool(True) 到 /realman_teleop/release_servos 释放舵机力矩")
        
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
                rospy.logerr(f"[RealManOpt] 运行时错误: {e}")
                continue
        
        # 退出前确保机械臂安全
        if self.is_arm_enabled:
            rospy.logwarn("[RealManOpt] 节点退出，自动失能机械臂")
            self.is_arm_enabled = False
        
        # 清理资源
        self.cleanup()


def main():
    """主函数"""
    node = None
    try:
        node = RealManOptimizedTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[RealManOpt] 节点初始化失败: {e}")
    finally:
        if node is not None:
            node.cleanup()


if __name__ == "__main__":
    main()
