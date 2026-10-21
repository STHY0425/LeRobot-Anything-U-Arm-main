#!/usr/bin/env python3
"""
Servo 到 RealMan RM75(7轴) 的遥操作控制节点
功能：订阅 /servo_angles 话题，控制 RealMan 机械臂跟随运动
架构：参考 servo2xarm.py 的简洁设计
说明：角度映射后直接输出，不做平滑滤波
      常用配置全部集中在文件顶部"快捷配置区"；rosparam 可覆盖同名默认值
      默认无夹爪：/servo_angles = [j1..j7]（id0~id6 舵机，单位度，相对零点偏移）
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rm_msgs.msg import JointPos, Gripper_Set

# ==================== 快捷配置区（一般只改这里） ====================
DOF =  6                     # 机械臂自由度：RM75=7
USE_GRIPPER = False            # 是否映射夹爪：False 时 /servo_angles 全部按关节处理
# ---- 关节映射 ----
CONTROL_RATE = 100.0           # 控制频率 (Hz)
MAX_JOINT_SPEED = 1000.0       # 关节限速 (度/秒)，默认值很大≈不限速
INIT_QPOS = [0.0] * DOF        # RealMan 初始/零点姿态 (度)，按关节顺序
JOINT_SCALE = [0.020] * DOF    # 各关节缩放：舵机偏移 1° → 机械臂动 scale°
JOINT_INVERT = [-1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0]    # 各关节方向：1.0 不反转 / -1.0 反转
# ---- 夹爪映射（仅 USE_GRIPPER=True 时生效，夹爪占 /servo_angles 第 DOF+1 位）----
GRIPPER_MIN_DEG = -10.0        # 夹爪舵机最小角度 (度)
GRIPPER_MAX_DEG = 30.0         # 夹爪舵机最大角度 (度)
GRIPPER_RANGE = 1000           # RealMan 夹爪位置指令范围
# ===================================================================

SERVO_DIM = DOF + 1 if USE_GRIPPER else DOF   # /servo_angles 期望的维度


class ServoToRealMan:
    """Servo 数据转换为 RealMan 控制命令"""

    def __init__(self):
        rospy.init_node('servo_to_75realman')
        rospy.loginfo("[Servo2-75RealMan] 启动遥操作控制节点（RM75, 7轴）...")

        # ====== 参数配置（rosparam 可覆盖顶部快捷配置区）======
        self.dof = rospy.get_param("~dof", DOF)
        self.use_gripper = rospy.get_param("~use_gripper", USE_GRIPPER)
        self.control_rate = rospy.get_param("~control_rate", CONTROL_RATE)
        self.max_joint_speed = rospy.get_param("~max_joint_speed", MAX_JOINT_SPEED)

        # RealMan 初始姿态（零点姿态）
        self.init_qpos_deg = rospy.get_param("~init_qpos", INIT_QPOS)

        # 关节映射参数（缩放和方向）
        self.joint_scale = np.array(rospy.get_param("~joint_scale", JOINT_SCALE), dtype=np.float64)
        self.joint_invert = np.array(rospy.get_param("~joint_invert", JOINT_INVERT), dtype=np.float64)
        # 实际参与映射的关节数（防止 rosparam 传入的数组长度与 dof 不一致）
        self.n_map = min(self.dof, len(self.joint_scale), len(self.joint_invert))

        if len(self.init_qpos_deg) < self.dof:
            rospy.logwarn("[Servo2-75RealMan] init_qpos 长度 %d < dof %d，已用 0 补齐",
                          len(self.init_qpos_deg), self.dof)
            self.init_qpos_deg = list(self.init_qpos_deg) + [0.0] * (self.dof - len(self.init_qpos_deg))

        # 夹爪参数（代码内固定常量，非 ROS 参数）
        self.gripper_range = GRIPPER_RANGE
        self.gripper_min_deg = GRIPPER_MIN_DEG
        self.gripper_max_deg = GRIPPER_MAX_DEG

        # ====== 状态变量 ======
        self.servo_dim = self.dof + 1 if self.use_gripper else self.dof
        self.servo_angles = [0.0] * self.servo_dim  # 从 /servo_angles 接收到的数据
        self.last_joint_cmd = np.array(self.init_qpos_deg, dtype=np.float32)
        self.gripper_position = 0

        self.last_cmd_time = rospy.Time.now()
        self._inited_cmd = False

        # ====== ROS 发布者 ======
        self.joint_pub = rospy.Publisher('/rm_driver/JointPos', JointPos, queue_size=10)
        self.gripper_pub = None
        if self.use_gripper:
            self.gripper_pub = rospy.Publisher('/rm_driver/Gripper_Set', Gripper_Set, queue_size=10)
        self.action_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)

        # ====== ROS 订阅者 ======
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.servo_callback)

        rospy.loginfo("[Servo2-75RealMan] 初始化完成，等待 /servo_angles 数据...")
        rospy.loginfo(f"[Servo2-75RealMan] 自由度: {self.dof}, 夹爪: {self.use_gripper}, /servo_angles 维度: {self.servo_dim}")
        rospy.loginfo(f"[Servo2-75RealMan] 初始姿态: {self.init_qpos_deg}")
        rospy.loginfo(f"[Servo2-75RealMan] 关节缩放: {self.joint_scale.tolist()}")
        rospy.loginfo(f"[Servo2-75RealMan] 关节方向: {self.joint_invert.tolist()}")

    def servo_callback(self, msg):
        """
        接收 Servo 角度数据（来自遥操作主臂）
        数据格式: [j1, ..., j7]（单位：度，相对于零点偏移；id0~id6 对应 7 个关节）
        USE_GRIPPER=True 时末尾多一位夹爪: [j1, ..., j7, gripper]
        主臂轴数少于 DOF 时（如五轴主臂驱动七轴臂），缺失的末尾位自动补 0，
        对应关节保持 init_qpos 不动。
        """
        if len(msg.data) < self.servo_dim:
            rospy.logwarn_throttle(5.0, f"[Servo2-75RealMan] /servo_angles 长度 {len(msg.data)} < 期望 {self.servo_dim}，缺失位按 0 处理（对应关节锁定在 init_qpos）")
        self.servo_angles = (list(msg.data) + [0.0] * self.servo_dim)[:self.servo_dim]

    def process_servo_data(self):
        """
        处理 Servo 数据，转换为 RealMan 目标角度
        映射公式: target = init_qpos + servo_offset * scale * invert
        """
        # 计算目标角度（带缩放和方向），不做滤波，直接输出
        new_angles = np.array(self.init_qpos_deg, dtype=np.float32)
        for i in range(self.n_map):
            offset = self.servo_angles[i] * self.joint_scale[i] * self.joint_invert[i]
            new_angles[i] = self.init_qpos_deg[i] + offset

        # 映射夹爪位置
        if self.use_gripper:
            gripper_deg = self.servo_angles[self.dof] if len(self.servo_angles) > self.dof else 0
            gripper_norm = (gripper_deg - self.gripper_min_deg) / max(1e-6, self.gripper_max_deg - self.gripper_min_deg)
            gripper_norm = np.clip(gripper_norm, 0.0, 1.0)
            self.gripper_position = int(gripper_norm * self.gripper_range)
        else:
            self.gripper_position = 0

        return new_angles, self.gripper_position

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
        # rm_driver 规则：joint[0..5] 为前 6 轴，第 7 轴放 joint[6]（float32[] 可变长）
        joint_msg = JointPos()
        joint_msg.joint = joint_angles.tolist()[:7]
        if self.dof > 6:
            joint_msg.expand = 0.0  # expand 是扩展/升降轴，不是第 7 轴
        self.joint_pub.publish(joint_msg)

        # 发布夹爪命令
        if self.use_gripper and self.gripper_pub is not None:
            gripper_msg = Gripper_Set()
            gripper_msg.position = int(gripper_pos)
            self.gripper_pub.publish(gripper_msg)

        # 发布动作反馈（用于数据记录）
        action_msg = Float64MultiArray()
        action_data = list(joint_angles)[:self.dof]
        if self.use_gripper:
            action_data += [int(gripper_pos)]
        action_msg.data = action_data
        self.action_pub.publish(action_msg)

    def run(self):
        """主循环：处理数据并发送控制命令"""
        rate = rospy.Rate(self.control_rate)

        rospy.loginfo(f"[Servo2-75RealMan] 控制循环启动，频率: {self.control_rate}Hz")

        while not rospy.is_shutdown():
            # 处理 servo 数据
            target_joints, gripper_pos = self.process_servo_data()

            # 应用速度限制
            safe_joints = self.apply_velocity_limit(target_joints)

            # 发布控制命令
            self.publish_commands(safe_joints, gripper_pos)

            rospy.loginfo_throttle(1.0,
                f"[Servo2-75RealMan] 目标: {[f'{x:.2f}' for x in safe_joints]}, 夹爪: {gripper_pos}")

            rate.sleep()


def main():
    try:
        node = ServoToRealMan()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
