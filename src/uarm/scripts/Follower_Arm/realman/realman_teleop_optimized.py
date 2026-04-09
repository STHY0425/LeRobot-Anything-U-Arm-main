#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
瑞尔曼机械臂遥操节点 - 动态零点校准版本
基于 teleop_sim_trigger_test_optimized_v2.py 的零点校准方案

特性：
1. 直接读取舵机，不依赖 servo_reader.py
2. 启动时自动记录当前舵机位置作为零点
3. 瑞尔曼初始在0点，舵机偏移实时映射
"""

import rospy
import numpy as np
import time
import serial
import re
import threading
from threading import Thread, Event, Lock
from queue import Queue, Empty

from std_msgs.msg import Float64MultiArray, Bool, UInt16MultiArray
from rm_msgs.msg import JointPos, Gripper_Set, Arm_Current_State, Stop


class RealManOptimizedTeleop:
    """
    瑞尔曼机械臂优化遥操节点 - 独立舵机读取版本
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
    
    # 舵机数据异常阈值
    SERVO_PWM_MIN = 400      # PWM最小值
    SERVO_PWM_MAX = 2600     # PWM最大值
    SERVO_ANGLE_MIN = 0.0    # 角度最小值(度)
    SERVO_ANGLE_MAX = 270.0  # 角度最大值(度)
    SERVO_MAX_DELTA = 30.0   # 单帧最大角度变化(度)
    SERVO_FAIL_THRESHOLD = 5  # 单个关节连续失败阈值
    
    def __init__(self):
        rospy.init_node("realman_optimized_teleop")
        rospy.loginfo("[RealManOpt] 启动优化遥操节点（动态零点校准版）...")
        
        # ====== 参数配置 ======
        self.arm_model = rospy.get_param("~arm_model", "RM_65")
        self.dof = rospy.get_param("~dof", 6)
        
        # 瑞尔曼初始姿态（全零姿态）
        # 当舵机偏移为0时，瑞尔曼保持此姿态
        self.init_qpos_deg = rospy.get_param("~init_qpos", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # 优化参数
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 30.0)  # 度/秒
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)  # Hz
        self.comm_timeout = rospy.get_param("~comm_timeout", 0.5)  # 通信超时
        
        # 延迟容错优化参数
        self.angle_queue_size = rospy.get_param("~angle_queue_size", 1)
        self.filter_alpha = rospy.get_param("~filter_alpha", 0.3)
        self.packet_loss_tolerance = rospy.get_param("~packet_loss_tolerance", 3)
        
        # 舵机串口参数
        self.servo_serial_port = rospy.get_param("~servo_serial_port", "/dev/ttyUSB0")
        self.servo_baudrate = rospy.get_param("~servo_baudrate", 115200)
        
        # ====== 初始化舵机串口 ======
        self.servo_ser = None
        self.zero_angles = [0.0] * 7  # 动态零点（启动时读取）
        self.current_servo_angles = [0.0] * 7  # 当前舵机角度
        self._init_servo_serial()
        
        # 单个舵机失效追踪（必须在 _init_servo_zero_points 之前初始化）
        self.servo_fail_count = [0] * 7  # 各关节连续失败计数
        self.servo_last_valid_angle = [None] * 7  # 各关节上次有效角度
        self.servo_status = [True] * 7  # 各关节当前状态(True=正常)
        
        # 如果串口打开成功，读取零点
        if self.servo_ser is not None:
            self._init_servo_zero_points()
        else:
            rospy.logerr("[RealManOpt] 舵机串口未打开，无法继续")
            raise RuntimeError("舵机串口打开失败")
        
        # ====== 状态变量 ======
        self.target_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.last_joint_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.smoothed_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        self.gripper_position = 0
        self.last_gripper_position = 0
        
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
        
        # 映射参数（Version1测试机械臂参数）
        self.joint_scale = np.array(rospy.get_param("~joint_scale", [0.015]*6))
        self.joint_invert = np.array(rospy.get_param("~joint_invert", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0]))

        # # 映射参数（模块化控制预留）
        # self.joint_scale = np.array(rospy.get_param("~joint_scale", [0.03]*6))
        # self.joint_invert = np.array(rospy.get_param("~joint_invert", [1.0, 1.0, 1.0, -1.0, 1.0, 1.0]))

        # 夹爪映射
        self.gripper_min_deg = -10.0
        self.gripper_max_deg = 30.0
        self.gripper_range = 1000
        
        # ====== 线程间通信队列（双线程架构）======
        self.cmd_queue = Queue(maxsize=1)  # 只保留最新命令，避免延迟
        self.queue_lock = Lock()  # 队列操作锁
        self._cleanup_done = False  # 防止重复清理
        
        # ====== ROS发布者 ======
        self.joint_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=1)
        self.gripper_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=1)
        self.action_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/realman_teleop/status', Bool, queue_size=1)
        self.stop_pub = rospy.Publisher('/rm_driver/Stop', Stop, queue_size=1)
        self.emergency_pub = rospy.Publisher('/realman_teleop/emergency', Bool, queue_size=1)
        
        # 发布零点信息供调试
        self.zero_pub = rospy.Publisher('/realman_teleop/zero_angles', Float64MultiArray, queue_size=1, latch=True)
        zero_msg = Float64MultiArray()
        zero_msg.data = self.zero_angles
        self.zero_pub.publish(zero_msg)
        
        # ====== ROS订阅者 ======
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.arm_state_callback)
        rospy.Subscriber('/realman_teleop/enable', Bool, self.enable_callback)
        rospy.Subscriber('/realman_teleop/emergency_stop', Bool, self.emergency_callback)
        rospy.Subscriber('/realman_teleop/release_servos', Bool, self.release_servos_callback)
        
        # ====== 启动双线程（与仿真文件架构一致）======
        self.stop_event = Event()
        
        # 线程1: 舵机读取线程（生产者）
        self.servo_thread = Thread(target=self._servo_read_loop, daemon=True)
        self.servo_thread.start()
        rospy.loginfo("[RealManOpt] 舵机读取线程已启动")
        
        # 线程2: 命令发布线程（消费者）
        self.publish_thread = Thread(target=self._command_publish_loop, daemon=True)
        self.publish_thread.start()
        rospy.loginfo("[RealManOpt] 命令发布线程已启动")
        
        # 安全检查定时器
        rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        rospy.loginfo(f"[RealManOpt] 机械臂型号: {self.arm_model}, DOF: {self.dof}")
        rospy.loginfo(f"[RealManOpt] 瑞尔曼零点姿态: {self.init_qpos_deg}")
        rospy.loginfo(f"[RealManOpt] 舵机零点位置: {[f'{z:.1f}' for z in self.zero_angles]}")
        rospy.loginfo(f"[RealManOpt] 关节缩放: {self.joint_scale}")
        rospy.loginfo(f"[RealManOpt] 关节方向: {self.joint_invert}")
        rospy.loginfo(f"[RealManOpt] 速度限制: {self.max_joint_speed}°/s")
        rospy.loginfo(f"[RealManOpt] 等待使能信号...")
        rospy.loginfo(f"[RealManOpt] 提示: 发布 Bool(True) 到 /realman_teleop/enable 使能")

    # ====== 舵机控制方法 ======
    def _init_servo_serial(self):
        """初始化舵机串口连接"""
        try:
            self.servo_ser = serial.Serial(
                self.servo_serial_port,
                self.servo_baudrate,
                timeout=0.02
            )
            time.sleep(0.1)
            self.servo_ser.reset_input_buffer()
            rospy.loginfo(f"[Servo] 串口已打开: {self.servo_serial_port} @ {self.servo_baudrate}")
        except serial.SerialException as e:
            rospy.logerr(f"[Servo] 无法打开串口 {self.servo_serial_port}: {e}")
            self.servo_ser = None
    
    def _parse_angle(self, response: str, servo_id: int = None) -> float:
        """解析PWM响应为角度，带范围验证"""
        match = re.search(r'P(\d{4})', response)
        if match:
            pwm = int(match.group(1))
            # PWM范围检查
            if pwm < self.SERVO_PWM_MIN or pwm > self.SERVO_PWM_MAX:
                if servo_id is not None:
                    rospy.logwarn_throttle(1.0, f"[Servo] 舵机{servo_id} PWM值{pwm}超出范围[{self.SERVO_PWM_MIN}, {self.SERVO_PWM_MAX}]")
                return None
            angle = (pwm - 500) / 2000 * 270
            # 角度范围检查
            if angle < self.SERVO_ANGLE_MIN or angle > self.SERVO_ANGLE_MAX:
                if servo_id is not None:
                    rospy.logwarn_throttle(1.0, f"[Servo] 舵机{servo_id} 角度{angle:.1f}°超出范围")
                return None
            return angle
        return None
    
    def _read_servo_angle(self, servo_id: int) -> float:
        """读取单个舵机角度，带跳变检测"""
        if self.servo_ser is None:
            return None
        try:
            self.servo_ser.reset_input_buffer()
            self.servo_ser.write(f'#{servo_id:03d}PRAD!'.encode('ascii'))
            time.sleep(0.02)
            response = self.servo_ser.read_all().decode('ascii', errors='ignore')
            angle = self._parse_angle(response, servo_id)
            
            if angle is not None:
                # 跳变检测：与上次有效角度比较
                last_angle = self.servo_last_valid_angle[servo_id]
                if last_angle is not None:
                    delta = abs(angle - last_angle)
                    if delta > self.SERVO_MAX_DELTA:
                        rospy.logwarn(f"[Servo] 舵机{servo_id} 角度跳变: {last_angle:.1f}°→{angle:.1f}° (Δ{delta:.1f}°)，忽略")
                        return None
                # 更新状态
                self.servo_last_valid_angle[servo_id] = angle
                self.servo_fail_count[servo_id] = 0
                self.servo_status[servo_id] = True
            else:
                # 记录失败
                self.servo_fail_count[servo_id] += 1
                if self.servo_fail_count[servo_id] >= self.SERVO_FAIL_THRESHOLD:
                    if self.servo_status[servo_id]:  # 第一次超过阈值时报警
                        self.servo_status[servo_id] = False
                        rospy.logerr(f"[Servo] 舵机{servo_id} 连续失败{self.servo_fail_count[servo_id]}次，标记为异常")
            
            return angle
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[Servo] 读取舵机{servo_id}失败: {e}")
            self.servo_fail_count[servo_id] += 1
            return None
    
    def _read_all_servos(self) -> list:
        """读取所有7个舵机角度"""
        angles = [None] * 7
        for i in range(7):
            angles[i] = self._read_servo_angle(i)
        return angles
    
    def _init_servo_zero_points(self):
        """
        初始化舵机零点 - 关键功能
        释放力矩并读取当前位置作为零点
        """
        rospy.loginfo("[Servo] 正在初始化舵机零点...")
        rospy.loginfo("[Servo] 释放力矩，请调整主臂到期望的零点位置...")
        
        # 释放所有舵机力矩
        for i in range(7):
            try:
                self.servo_ser.write(f'#{i:03d}PULK!'.encode('ascii'))
                time.sleep(0.02)
            except Exception as e:
                rospy.logwarn(f"[Servo] 释放舵机{i}力矩失败: {e}")
        
        # 等待用户调整姿态
        rospy.loginfo("[Servo] 等待3秒让您调整主臂姿态...")
        time.sleep(3.0)
        
        # 读取当前位置作为零点
        rospy.loginfo("[Servo] 正在记录零点位置...")
        zero_fail_count = 0
        for i in range(7):
            # 多次尝试读取，确保获取有效值
            angle = None
            for attempt in range(3):
                angle = self._read_servo_angle(i)
                if angle is not None:
                    break
                time.sleep(0.05)
            
            if angle is not None:
                self.zero_angles[i] = angle
                self.servo_last_valid_angle[i] = angle  # 初始化上次有效角度
                rospy.loginfo(f"[Servo] 舵机{i}: 零点 = {angle:.1f}°")
            else:
                # 读取失败使用默认值135°
                self.zero_angles[i] = 135.0
                self.servo_last_valid_angle[i] = 135.0
                zero_fail_count += 1
                rospy.logwarn(f"[Servo] 舵机{i}: 读取失败，使用默认值 135.0°")
        
        if zero_fail_count > 2:
            rospy.logerr(f"[Servo] ⚠️ 警告: {zero_fail_count}个舵机零点读取失败，请检查硬件连接")
        
        rospy.loginfo("[Servo] ✅ 零点校准完成！")
        rospy.loginfo(f"[Servo] 零点角度: {[f'{z:.1f}' for z in self.zero_angles]}")
    
    def _release_servo_torque(self):
        """释放所有舵机力矩"""
        if self.servo_ser is None:
            rospy.logwarn("[Servo] 串口未打开，无法释放力矩")
            return
        
        rospy.loginfo("[Servo] 正在释放舵机力矩...")
        try:
            for i in range(7):
                cmd = f'#{i:03d}PULK!'.encode('ascii')
                self.servo_ser.write(cmd)
                time.sleep(0.01)
            rospy.loginfo("[Servo] ✅ 舵机力矩已释放，可手动调整主臂姿态")
        except Exception as e:
            rospy.logerr(f"[Servo] 释放力矩失败: {e}")
    
    def release_servos_callback(self, msg):
        """ROS回调：释放舵机力矩"""
        if msg.data:
            self._release_servo_torque()
    
    def cleanup(self):
        """清理资源 - 防止重复调用"""
        with self.queue_lock:
            if self._cleanup_done:
                return
            self._cleanup_done = True
            
            if self.is_arm_enabled:
                self.is_arm_enabled = False
                rospy.loginfo("[RealManOpt] 机械臂已失能")
        
        rospy.loginfo("[RealManOpt] 正在清理资源...")
        
        # 停止所有线程
        if hasattr(self, 'stop_event'):
            self.stop_event.set()
        
        # 等待舵机读取线程结束
        if hasattr(self, 'servo_thread') and self.servo_thread.is_alive():
            self.servo_thread.join(timeout=2.0)
            if not self.servo_thread.is_alive():
                rospy.loginfo("[RealManOpt] 舵机读取线程已停止")
            else:
                rospy.logwarn("[RealManOpt] 舵机读取线程停止超时")
        
        # 等待发布线程结束
        if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
            if not self.publish_thread.is_alive():
                rospy.loginfo("[RealManOpt] 命令发布线程已停止")
            else:
                rospy.logwarn("[RealManOpt] 命令发布线程停止超时")
        
        # 关闭舵机串口
        if self.servo_ser is not None:
            try:
                self.servo_ser.close()
                rospy.loginfo("[Servo] 串口已关闭")
            except Exception as e:
                rospy.logwarn(f"[Servo] 关闭串口时出错: {e}")
        
        rospy.loginfo("[RealManOpt] 节点已安全退出")

    def enable_callback(self, msg):
        """使能/失能控制"""
        with self.queue_lock:
            if msg.data and not self.is_arm_enabled:
                # 使能前检查舵机状态
                failed_servos = [i for i, status in enumerate(self.servo_status) if not status]
                if failed_servos:
                    rospy.logerr(f"[RealManOpt] ⚠️ 使能失败: 舵机{failed_servos}处于异常状态")
                    return
                
                self.is_arm_enabled = True
                self.is_emergency_stopped = False
                # 重置状态
                self.consecutive_packet_loss = 0
                # 重置舵机失败计数
                self.servo_fail_count = [0] * 7
                # 清空队列
                try:
                    while not self.cmd_queue.empty():
                        self.cmd_queue.get_nowait()
                except:
                    pass
                rospy.loginfo("[RealManOpt] ✅ 机械臂已使能，遥操开始")
                rospy.loginfo("[RealManOpt] 提示: 移动主臂，从臂将跟随移动")
            elif not msg.data and self.is_arm_enabled:
                self.is_arm_enabled = False
                rospy.loginfo("[RealManOpt] ⛔ 机械臂已失能")

    def emergency_callback(self, msg):
        """急停信号处理"""
        with self.queue_lock:
            if msg.data and not self.is_emergency_stopped:
                self.trigger_emergency_stop("外部急停信号触发")

    def trigger_emergency_stop(self, reason):
        """触发急停"""
        with self.queue_lock:
            if self.is_emergency_stopped:
                return
            
            self.is_emergency_stopped = True
            self.is_arm_enabled = False
        
        rospy.logerr(f"[RealManOpt] 🚨 急停触发！原因: {reason}")
        
        stop_msg = Stop()
        stop_msg.stop_mode = 0
        self.stop_pub.publish(stop_msg)
        self.emergency_pub.publish(Bool(True))

    # ====== 线程1: 舵机读取线程（生产者）======
    def _servo_read_loop(self):
        """
        独立线程读取舵机角度，计算目标位置，放入队列
        实际频率: ~7Hz（串行读取7个舵机，每个约20ms，总计约140ms）
        """
        rospy.loginfo("[ServoThread] 舵机读取线程启动")
        
        period = 1.0 / self.publish_rate  # 33ms for 30Hz
        next_time = time.monotonic()
        frame_count = 0
        last_print = time.monotonic()
        
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            loop_start = time.monotonic()
            
            # 只有在使能状态下才读取和处理
            with self.queue_lock:
                is_enabled = self.is_arm_enabled and not self.is_emergency_stopped
            
            if is_enabled:
                # 读取所有舵机
                angles = self._read_all_servos()
                
                # 检查读取结果（至少要有5个成功）
                valid_count = sum(1 for a in angles if a is not None)
                
                if valid_count >= 5:
                    # 检查是否有舵机处于持续异常状态
                    failed_servos = [i for i, status in enumerate(self.servo_status) if not status]
                    if failed_servos:
                        rospy.logerr_throttle(1.0, f"[RealManOpt] 舵机{failed_servos}异常，暂停控制")
                        with self.queue_lock:
                            self.consecutive_packet_loss += 1
                        continue
                    
                    # 保存当前角度
                    self.current_servo_angles = angles
                    
                    # 计算偏移量（相对于零点）
                    angle_offset = [0.0] * 7
                    for i in range(7):
                        if angles[i] is not None:
                            angle_offset[i] = angles[i] - self.zero_angles[i]
                        else:
                            # 使用上次有效角度计算偏移（保持位置）
                            if self.servo_last_valid_angle[i] is not None:
                                angle_offset[i] = self.servo_last_valid_angle[i] - self.zero_angles[i]
                                rospy.logwarn_throttle(2.0, f"[Servo] 舵机{i} 使用上次有效角度计算偏移")
                    
                    # 处理数据（计算目标角度）
                    target_angles, gripper_pos = self._process_input_data(angle_offset)
                    
                    # 更新状态
                    with self.queue_lock:
                        self.last_valid_input = np.array(angle_offset)
                        self.consecutive_packet_loss = 0
                        self.last_input_time = time.time()
                    
                    # 放入队列（只保留最新，避免延迟累积）
                    cmd_data = {
                        'joints': target_angles,
                        'gripper': gripper_pos
                    }
                    try:
                        # 清空旧数据，只保留最新
                        while not self.cmd_queue.empty():
                            self.cmd_queue.get_nowait()
                        self.cmd_queue.put_nowait(cmd_data)
                    except:
                        pass
                    
                    frame_count += 1
                else:
                    with self.queue_lock:
                        self.consecutive_packet_loss += 1
            
            # 性能统计（每3秒打印一次）
            now = time.monotonic()
            if now - last_print > 3.0:
                with self.queue_lock:
                    loss_count = self.consecutive_packet_loss
                actual_fps = frame_count / (now - last_print)
                rospy.loginfo(f"[ServoThread] 读取频率: {actual_fps:.1f}Hz, 丢包: {loss_count}")
                frame_count = 0
                last_print = now
            
            # 自适应sleep以维持目标频率
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()
        
        rospy.loginfo("[ServoThread] 舵机读取线程已停止")
    
    # ====== 线程2: 命令发布线程（消费者）======
    def _command_publish_loop(self):
        """
        独立线程从队列取数据，发布给机械臂
        目标频率: 30Hz
        """
        rospy.loginfo("[PublishThread] 命令发布线程启动")
        
        period = 1.0 / self.publish_rate  # 33ms for 30Hz
        next_time = time.monotonic()
        pub_count = 0
        last_print = time.monotonic()
        
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            # 检查状态
            with self.queue_lock:
                is_enabled = self.is_arm_enabled and not self.is_emergency_stopped
            
            if is_enabled:
                # 从队列取最新命令
                cmd_data = None
                try:
                    # 只取最新，清空旧数据
                    while not self.cmd_queue.empty():
                        cmd_data = self.cmd_queue.get_nowait()
                except Empty:
                    pass
                
                # 如果有新命令，处理并发布
                if cmd_data is not None:
                    target_angles = cmd_data['joints']
                    gripper_pos = cmd_data['gripper']
                    
                    # 应用速度限制
                    safe_angles = self._apply_velocity_limit(target_angles)
                    safe_gripper = self._apply_gripper_velocity_limit(gripper_pos)
                    
                    # 发布到ROS
                    self._publish_to_ros(safe_angles, safe_gripper)
                    pub_count += 1
                else:
                    # 没有新数据，继续发布上一次的速度限制后角度（保持位置）
                    safe_angles = self._apply_velocity_limit(self.last_joint_angles)
                    safe_gripper = self._apply_gripper_velocity_limit(self.last_gripper_position)
                    self._publish_to_ros(safe_angles, safe_gripper)
            
            # 性能统计
            now = time.monotonic()
            if now - last_print > 3.0:
                actual_fps = pub_count / (now - last_print)
                rospy.loginfo(f"[PublishThread] 发布频率: {actual_fps:.1f}Hz")
                pub_count = 0
                last_print = now
            
            # 维持30Hz
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()
        
        rospy.loginfo("[PublishThread] 命令发布线程已停止")
    
    def _publish_to_ros(self, joint_angles, gripper_pos):
        """发布命令到ROS话题"""
        # 发布关节命令
        joint_msg = JointPos()
        joint_msg.joint = joint_angles.tolist() if hasattr(joint_angles, 'tolist') else list(joint_angles)
        if self.dof > 6:
            joint_msg.expand = 0.0
        self.joint_pub.publish(joint_msg)
        
        # 发布夹爪命令
        gripper_msg = Gripper_Set()
        gripper_msg.position = int(gripper_pos)
        self.gripper_pub.publish(gripper_msg)
        
        # 发布动作反馈
        action_msg = Float64MultiArray()
        action_data = list(joint_angles)[:6] + [int(gripper_pos)]
        action_msg.data = action_data
        self.action_pub.publish(action_msg)

    def _process_input_data(self, angle_offset):
        """
        处理输入数据（平滑滤波）
        返回: (target_angles, gripper_position)
        
        映射公式：
            瑞尔曼目标角度 = init_qpos_deg[i] + angle_offset[i] * scale * invert
        """
        # 基本角度计算
        new_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        for i in range(min(self.dof, 6)):
            offset = angle_offset[i] * self.joint_scale[i] * self.joint_invert[i]
            new_angles[i] = self.init_qpos_deg[i] + offset
        
        # 夹爪（度数直接映射到0-1000范围）
        gripper_deg = angle_offset[6] if len(angle_offset) > 6 else 0
        
        # 低通平滑滤波
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
        
        # 更新共享状态（供速度限制使用）
        self.target_joint_angles = self.smoothed_angles.copy()
        self.gripper_position = self._map_gripper(gripper_deg)
        
        return self.smoothed_angles.copy(), self.gripper_position

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
        
        with self.queue_lock:
            is_enabled = self.is_arm_enabled
            time_since_input = current_time - self.last_input_time
            loss_count = self.consecutive_packet_loss
            has_errors = len(self.last_arm_error) > 0
            errors = self.last_arm_error.copy()
        
        # 检查输入数据超时
        if time_since_input > self.comm_timeout:
            with self.queue_lock:
                self.consecutive_packet_loss += 1
                loss_count = self.consecutive_packet_loss
            if loss_count > self.packet_loss_tolerance:
                if is_enabled:
                    self.trigger_emergency_stop("数据丢失超过容限")
        
        # 检查状态反馈超时
        time_since_state = current_time - self.last_state_time
        if time_since_state > self.comm_timeout * 2:
            rospy.logerr_throttle(5.0, 
                f"[RealManOpt] 状态反馈超时: {time_since_state:.2f}s")
        
        # 检查严重错误
        if is_enabled and has_errors:
            critical_errors = [0x100D, 0x1007, 0x1009, 0x100A]
            if any(err in errors for err in critical_errors):
                self.trigger_emergency_stop(f"严重错误: {errors}")

    def _apply_velocity_limit(self, desired_angles):
        """应用速度限制（带时间回拨保护）"""
        current_time = time.time()
        dt = current_time - self._last_cmd_time
        
        # 时间回拨检测与保护
        if dt < 0:
            rospy.logwarn(f"[RealManOpt] 检测到时间回拨({dt:.3f}s)，使用最小时间间隔")
            dt = 0.001  # 使用最小安全间隔
        elif dt > 1.0:
            # 时间跳跃过大（如程序暂停后恢复）
            rospy.logwarn(f"[RealManOpt] 检测到时间跳跃({dt:.3f}s)，重置速度限制")
            self._last_cmd_time = current_time
            self.last_joint_angles = desired_angles.copy()
            return desired_angles
        
        self._last_cmd_time = current_time
        
        if not self._inited_cmd:
            self._inited_cmd = True
            self.last_joint_angles = desired_angles.copy()
            return desired_angles
        
        max_delta = self.max_joint_speed * dt
        
        limited_angles = np.zeros_like(desired_angles)
        for i in range(self.dof):
            delta = desired_angles[i] - self.last_joint_angles[i]
            delta_clipped = np.clip(delta, -max_delta, max_delta)
            limited_angles[i] = self.last_joint_angles[i] + delta_clipped
        
        self.last_joint_angles = limited_angles.copy()
        return limited_angles

    def _apply_gripper_velocity_limit(self, desired_pos):
        """夹爪速度限制（带时间回拨保护）"""
        current_time = time.time()
        last_time = getattr(self, '_last_gripper_time', current_time)
        dt = current_time - last_time
        
        # 时间回拨检测
        if dt < 0:
            rospy.logwarn(f"[RealManOpt] 夹爪控制检测到时间回拨({dt:.3f}s)")
            dt = 0.001
        elif dt > 1.0:
            rospy.logwarn(f"[RealManOpt] 夹爪控制检测到时间跳跃({dt:.3f}s)，重置限制")
            self._last_gripper_time = current_time
            self.last_gripper_position = desired_pos
            return desired_pos
        
        self._last_gripper_time = current_time
        
        max_delta = 200.0 * dt
        delta = desired_pos - self.last_gripper_position
        delta_clipped = int(np.clip(delta, -max_delta, max_delta))
        
        limited_pos = self.last_gripper_position + delta_clipped
        self.last_gripper_position = limited_pos
        return limited_pos

    def run(self):
        """主循环 - 双线程架构下只负责监控和状态发布"""
        rate = rospy.Rate(10)  # 10Hz状态监控
        
        rospy.loginfo("[RealManOpt] 优化遥操节点运行中（双线程架构）...")
        rospy.loginfo("[RealManOpt] 舵机读取线程: ~7Hz (串行读取7×20ms) | 命令发布线程: 30Hz")
        
        while not rospy.is_shutdown():
            try:
                # 发布状态（主线程负责）
                with self.queue_lock:
                    status = self.is_arm_enabled and not self.is_emergency_stopped
                self.status_pub.publish(Bool(status))
                
                # 检查线程健康状态
                if hasattr(self, 'servo_thread') and not self.servo_thread.is_alive():
                    rospy.logerr("[RealManOpt] 舵机读取线程异常退出！")
                if hasattr(self, 'publish_thread') and not self.publish_thread.is_alive():
                    rospy.logerr("[RealManOpt] 命令发布线程异常退出！")
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"[RealManOpt] 运行时错误: {e}")
                continue
        
        # 退出前确保机械臂安全并清理
        self.cleanup()


def main():
    """主函数 - 确保任何情况下都能清理资源"""
    node = None
    try:
        node = RealManOptimizedTeleop()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[RealManOpt] 收到中断信号，正在退出...")
    except Exception as e:
        rospy.logerr(f"[RealManOpt] 节点异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.cleanup()
        rospy.loginfo("[RealManOpt] 主函数结束")


if __name__ == "__main__":
    main()
