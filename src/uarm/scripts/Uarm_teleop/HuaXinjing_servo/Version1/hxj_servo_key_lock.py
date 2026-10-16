#!/usr/bin/env python3
"""
华馨京舵机按键锁定节点，基于 hxj_servo_reader_fixed.py 修改。
按下 'k' 键切换锁定/解锁状态：
- 锁定：将当前角度设置为目标角度并发送命令，持续监测电流稳定后自动记录基线，若持续检测到电流变化则自动解锁。
- 解锁：立即释放扭矩。
适用于需要临时锁定舵机位置但又希望通过外力解锁的场景。发布 /servo_angles (Float64MultiArray)，默认 7 维。
发布 /servo_angles (Float64MultiArray)，默认 7 维。
"""
import rospy
import serial
from std_msgs.msg import Float64MultiArray
from fashionstar_uart_sdk import UartServoManager
import sys
import select
import tty
import termios
import time

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_SERVO_IDS = [0]
DEFAULT_NUM_SERVOS = 7
DEFAULT_RATE_HZ = 50
DEFAULT_TIMEOUT = 0
# 状态定义
STATE_IDLE = 0
STATE_STABILIZING = 1
STATE_LOCKED = 2

# 默认解锁参数（可通过 ROS param 覆盖）
DEFAULT_UNLOCK_DELTA = 0.008  # A ,解锁电流变化阈值
DEFAULT_UNLOCK_CONSEC = 2
DEFAULT_STABLE_THRESHOLD = 0.005  # A，锁定后判断电流稳定波动阈值
DEFAULT_STABLE_WINDOW = 6         # 滑动窗口大小，用于判断电流是否稳定

def open_manager(port, baudrate, timeout):
    uart = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=timeout,
    )
    return UartServoManager(uart)


def read_angle(manager, servo_id):
    return manager.query_servo_angle(servo_id)


class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher("/servo_angles", Float64MultiArray, queue_size=10)

        self.rate_hz = rospy.get_param("~rate", DEFAULT_RATE_HZ)
        # self.rate = rospy.Rate(self.rate_hz)
        self.rate = rospy.Rate(50)
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baudrate = rospy.get_param("~baudrate", DEFAULT_BAUDRATE)
        self.servo_ids = rospy.get_param("~servo_ids", DEFAULT_SERVO_IDS)
        self.num_servos = int(rospy.get_param("~num_servos", DEFAULT_NUM_SERVOS))
        self.timeout = rospy.get_param("~timeout", DEFAULT_TIMEOUT)

        if not self.servo_ids:
            rospy.logerr("servo_ids 为空，请设置 ~servo_ids")
            raise ValueError("servo_ids is empty")

        try:
            self.manager = open_manager(self.port, self.baudrate, self.timeout)
            rospy.loginfo("串口已打开: %s @ %d", self.port, self.baudrate)
        except serial.SerialException as exc:
            rospy.logerr("串口打开失败: %s", exc)
            raise

        self.zero_angles = [0.0] * self.num_servos
        self._init_servos()
        # 键盘输入相关
        self._orig_termios = None
        self._setup_keyboard()
        # 锁定状态，针对所有读取的舵机统一切换
        self.locked = False
        # 基线电流与计数器（按 servo_id 索引）
        self.base_loads = [None] * self.num_servos
        self._over_thresh_counts = [0] * self.num_servos

        # 每舵机的状态与电流滑动窗口（用于稳定检测）
        self.mode_state = [STATE_IDLE] * self.num_servos
        self.current_windows = [[] for _ in range(self.num_servos)]

        # 解锁阈值、连续采样数与稳定判定参数（均可通过 ROS 参数调整）
        self.unlock_delta = float(rospy.get_param("~unlock_delta", DEFAULT_UNLOCK_DELTA))
        self.unlock_consec = int(rospy.get_param("~unlock_consec", DEFAULT_UNLOCK_CONSEC))
        self.stable_threshold = float(rospy.get_param("~stable_threshold", DEFAULT_STABLE_THRESHOLD))
        self.stable_window = int(rospy.get_param("~stable_window", DEFAULT_STABLE_WINDOW))

        # 在 shutdown 时恢复终端设置
        rospy.on_shutdown(self._restore_keyboard)

    def _init_servos(self):
        for servo_id in self.servo_ids:
            angle = read_angle(self.manager, servo_id)
            if angle is None:
                angle = 0.0
            if 0 <= servo_id < self.num_servos:
                self.zero_angles[servo_id] = angle
        rospy.loginfo("零点校准完成")

    def run(self):
        angle_offset = [0.0] * self.num_servos
        target_angle_offset = [0.0] * self.num_servos
        step_size = 1.0
        jump_thres = 90.0

        while not rospy.is_shutdown():  #没有被 ROS 中断
            for servo_id in self.servo_ids:
                angle = read_angle(self.manager, servo_id)
                if angle is not None:
                    new_angle = angle - self.zero_angles[servo_id]
                    if abs(new_angle - target_angle_offset[servo_id]) > jump_thres:
                        rospy.logerr(
                            "舵机 %d 角度跳变: %.1f vs %.1f",
                            servo_id,
                            new_angle,
                            target_angle_offset[servo_id],
                        )
                    elif abs(new_angle - target_angle_offset[servo_id]) > step_size:
                        target_angle_offset[servo_id] = new_angle
                else:
                    rospy.logwarn("舵机 %d 读取失败", servo_id)

            angle_offset = target_angle_offset.copy()
            # 检查是否有按键输入（非阻塞轮询）
            self._check_keyboard()
            # 如果处于自锁状态，监测负载电流是否触发解锁
            if self.locked:
                self._monitor_lock_loads()
            self.pub.publish(Float64MultiArray(data=angle_offset))
            self.rate.sleep()

    def _setup_keyboard(self):
        try:
            if sys.stdin.isatty():
                self._orig_termios = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
        except Exception:
            self._orig_termios = None

    def _restore_keyboard(self):
        try:
            if self._orig_termios is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._orig_termios)
        except Exception:
            pass

    def _check_keyboard(self):
        # 非阻塞读取 stdin，按下 'k' 切换锁定/解锁
        try:
            if not sys.stdin.isatty():
                return
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if not dr:
                return
            ch = sys.stdin.read(1)
            if not ch:
                return
            if ch == 'k':
                # 切换锁定状态
                self.locked = not self.locked
                if self.locked:
                    rospy.loginfo("按下 k：锁定舵机，设置目标角度为当前角度并发送命令")
                    rospy.loginfo("正在锁定并等待电流稳定...")
                    time.sleep(0.30) # 给舵机PID环一点时间稳定在目标点(单位s)
                    # 对每个 ID 读取当前角度并设置为目标角度
                    for servo_id in self.servo_ids:
                        angle = read_angle(self.manager, servo_id)
                        if angle is None:
                            rospy.logwarn("舵机 %d 读取失败，无法锁定", servo_id)
                            continue
                        try:
                            # 使用默认速度 50 进行设置
                            self.manager.set_servo_angle(servo_id, angle, velocity=50)
                            rospy.loginfo("舵机 %d 锁定到角度 %.1f°", servo_id, angle)
                        except Exception as e:
                            rospy.logerr("设置舵机 %d 角度失败: %s", servo_id, e)
                    # 切换每个舵机进入稳定等待状态，稍后由监测函数记录基线
                    for sid in self.servo_ids:
                        if 0 <= sid < self.num_servos:
                            self.mode_state[sid] = STATE_STABILIZING
                            self.current_windows[sid] = []
                            self._over_thresh_counts[sid] = 0
                            # self._over_thresh_counts[servo_id] = max(0, self._over_thresh_counts[servo_id] - 1)
                else:
                    rospy.loginfo("按下 k：解开舵机锁定（释放扭矩）")
                    for servo_id in self.servo_ids:
                        try:
                            # method=0x10 表示释放扭矩
                            self.manager.stop_on_control_mode(servo_id, method=0x10, power=0)
                            rospy.loginfo("舵机 %d 已释放扭矩", servo_id)
                        except Exception as e:
                            rospy.logerr("释放舵机 %d 扭矩失败: %s", servo_id, e)
        except Exception:
            # 键盘读取不应影响主循环
            return

    def _monitor_lock_loads(self):
        # 对每个被锁定的舵机，读取当前电流并与基线对比，先等待稳定再正式监控解锁
        for servo_id in self.servo_ids:
            try:
                current = self.manager.query_current(servo_id)
            except Exception:
                current = None

            if current is None:
                # 无法读取电流则跳过但清理计数
                self._over_thresh_counts[servo_id] = 0
                continue

            state = self.mode_state[servo_id]

            if state == STATE_STABILIZING:
                # 收集滑动窗口样本：8 个样本后判断是否稳定
                win = self.current_windows[servo_id]
                win.append(current)
                if len(win) > self.stable_window:
                    win.pop(0)                  
                # 当窗口已填满时判断波动
                if len(win) >= self.stable_window: # 只有在窗口满时才判断电流是否稳定
                    fluctuation = max(win) - min(win)   # 计算当前窗口的电流波动范围
                    if fluctuation < self.stable_threshold: # 如果波动小于设定的稳定阈值，认为电流已稳定
                        # 电流稳定，记录基线并转入 LOCKED 状态
                        self.base_loads[servo_id] = sum(win) / len(win)
                        self.mode_state[servo_id] = STATE_LOCKED
                        self._over_thresh_counts[servo_id] = 0
                        
                        rospy.loginfo("舵机 %d 自动识别稳定！基线: %.3f A", servo_id, self.base_loads[servo_id])
                        
            elif state == STATE_LOCKED:
                base = self.base_loads[servo_id]
                if base is None:
                    # 没有基线，退回到稳定态重新采样
                    self.mode_state[servo_id] = STATE_STABILIZING
                    self.current_windows[servo_id] = []
                    self._over_thresh_counts[servo_id] = 0
                    continue

                diff = abs(current - base)

                # ================= 循环打印实际电流 =================
                # 使用 throttle 限制输出频率，0.1 表示每 0.1 秒打印一次，防止刷屏卡顿
                # rospy.loginfo_throttle(0.1, "舵机 %d | 基准: %.3f A | 实时: %.3f A | 差值(Δ): %.3f A", 
                #                        servo_id, base, current, diff)
                rospy.loginfo("舵机 %d | 基准: %.3f A | 实时: %.3f A | 差值(Δ): %.3f A", servo_id, base, current, diff)
                # =========================================================

                if diff > self.unlock_delta:        # 如果当前电流与基线的差值超过解锁阈值，增加计数
                    self._over_thresh_counts[servo_id] += 1
                else:
                    # self._over_thresh_counts[servo_id] = 0
                    self._over_thresh_counts[servo_id] = max(0, self._over_thresh_counts[servo_id] - 1)

                if self._over_thresh_counts[servo_id] >= self.unlock_consec:    # 如果连续超过阈值的次数达到设定值，自动解锁
                    rospy.loginfo(
                        "舵机 %d 检测到持续电流变化 (基线=%.3f A, 当前=%.3f A, Δ=%.3f A)，解除自锁",
                        servo_id,
                        base,
                        current,
                        diff,
                    )
                    try:
                        self.manager.stop_on_control_mode(servo_id, method=0x10, power=0)
                    except Exception as e:
                        rospy.logerr("释放舵机 %d 扭矩失败: %s", servo_id, e)
                    # 设置该舵机为闲置状态
                    self.mode_state[servo_id] = STATE_IDLE
                    self._over_thresh_counts[servo_id] = 0
                    self.base_loads[servo_id] = None

        # 更新整体 self.locked 标志：如果任一舵机仍在稳定或锁定状态，则视为仍处于锁定流程
        any_locked = any(self.mode_state[sid] in (STATE_STABILIZING, STATE_LOCKED) for sid in self.servo_ids)
        self.locked = any_locked


if __name__ == "__main__":
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
