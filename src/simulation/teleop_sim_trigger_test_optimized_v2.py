#!/usr/bin/env python3
"""
优化版 RM65B Trigger 功能 (保持115200波特率)
=============================================

优化点 (不更改波特率):
1. 批量读取所有舵机 (减少等待时间)
2. 使用非阻塞读取
3. 减少不必要的sleep
4. 提高控制频率到30Hz
5. 使用队列缓冲减少延迟
"""

import serial
import time 
import numpy as np
import re
import gymnasium as gym
import mani_skill.envs
from threading import Event, Thread, Lock
from queue import Queue, Empty
import torch
import sapien
import argparse
import sys
import select
import termios
import tty
from collections import deque
from mani_skill.utils import sapien_utils


class ServoTeleoperatorSimOptimized:
    """优化版: 保持115200波特率，优化通信效率"""
    
    def __init__(self, scene: str, robot_uids: str, serial_port: str = '/dev/ttyUSB0'):
        # 保持原有波特率
        self.SERIAL_PORT = serial_port
        self.BAUDRATE = 115200  # 保持115200不变
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.02)  # 减少超时
        time.sleep(0.1)
        self.ser.reset_input_buffer()

        # Config
        self.scene = scene
        self.robot_uids = robot_uids
        self.zero_angles = [0.0] * 7
        self.stop_event = Event()
        self.rate = 30.0  # 提高到30Hz
        self.arm_pos_queue = Queue(maxsize=1)  # 只保留最新数据
        
        # RM65B Trigger config
        self.is_rm65b = (robot_uids == "rm65b")
        self.trigger_enabled = self.is_rm65b
        self.trigger_angle = 101.0
        self.trigger_tolerance = 15.0
        self.servo6_center = 135.0
        
        self.trigger_state = 'teaching'
        self.locked_positions = None
        self.enter_pressed = Event()
        self.lock = Lock()
        
        # 性能统计
        self.read_times = deque(maxlen=100)
        self.last_read_time = time.monotonic()
        
        self._init_servos()

        self.control_mode = "pd_joint_pos"
        self.env = gym.make(
            scene,
            robot_uids=robot_uids,
            render_mode="human",
            control_mode=self.control_mode,
        )
        obs, _ = self.env.reset(seed=0)
        print(f"Action space: {self.env.action_space}")

        self.produce_thread = Thread(target=self._angle_stream_loop_optimized, daemon=True)
        self.consume_thread = Thread(target=self._pose_consumer_loop_optimized, daemon=True)
        if self.trigger_enabled:
            self.keyboard_thread = Thread(target=self._keyboard_loop, daemon=True)
        self._setup_camera()

    def _init_servos(self):
        """初始化 - 优化版"""
        print("[INIT] Starting servo initialization...")
        for i in range(7):
            print(f"[INIT] Setting up servo {i}...", end=' ', flush=True)
            self.ser.write(f'#{i:03d}PULK!'.encode('ascii'))
            time.sleep(0.02)
            angle = self._read_servo_angle_fast(i)
            self.zero_angles[i] = angle if angle is not None else 135.0
            print(f"Zero: {self.zero_angles[i]:.1f}°")
        
        if self.is_rm65b:
            print(f"\n[TRIGGER] Enabled - Target: {self.trigger_angle}°±{self.trigger_tolerance}°")
            print(f"[CONFIG] Baudrate: {self.BAUDRATE} (unchanged)")

    def _parse_angle(self, response: str) -> float:
        """Parse PWM response to angle"""
        match = re.search(r'P(\d{4})', response)
        if match:
            pwm = int(match.group(1))
            return (pwm - 500) / 2000 * 270
        return None

    def _angle_to_pwm(self, angle: float) -> int:
        """Convert angle to PWM"""
        pwm = int(500 + (angle / 270.0) * 2000)
        return max(500, min(2500, pwm))

    # ========== 优化1: 快速单次读取 (减少等待) ==========
    def _read_servo_angle_fast(self, servo_id: int) -> float:
        """单次读取: 20ms vs 30ms"""
        self.ser.reset_input_buffer()
        self.ser.write(f'#{servo_id:03d}PRAD!'.encode('ascii'))
        time.sleep(0.018)  # 从30ms减少到18ms (115200下约20字节传输时间)
        response = self.ser.read_all().decode('ascii', errors='ignore')
        return self._parse_angle(response)

    # ========== 回退到稳定的逐个读取 ==========
    def _read_all_servos_optimized(self) -> tuple:
        """
        稳定版: 逐个读取，减少等待时间到20ms
        总时间: 7 × 20ms = 140ms (vs 原来的245ms)
        """
        start_time = time.monotonic()
        angles = [None] * 7
        
        for i in range(7):
            angle = self._read_servo_angle_fast(i)
            angles[i] = angle
        
        elapsed = time.monotonic() - start_time
        self.read_times.append(elapsed)
        
        return angles, elapsed
    
    def _read_servo_angle_fast(self, servo_id: int) -> float:
        """快速单次读取: 20ms"""
        self.ser.reset_input_buffer()
        self.ser.write(f'#{servo_id:03d}PRAD!'.encode('ascii'))
        time.sleep(0.02)  # 20ms等待 (从30ms减少)
        response = self.ser.read_all().decode('ascii', errors='ignore')
        return self._parse_angle(response)

    def _hold_position(self, positions_pwm: list):
        """保持位置 - 回退到逐个发送确保稳定"""
        for i, pwm in enumerate(positions_pwm):
            cmd = f'#{i:03d}P{pwm:04d}T0030!'.encode('ascii')
            self.ser.write(cmd)
            time.sleep(0.002)  # 2ms间隔

    def _release_servos(self):
        """释放力矩 - 逐个发送确保稳定"""
        for i in range(7):
            self.ser.write(f'#{i:03d}PULK!'.encode('ascii'))
            time.sleep(0.01)  # 10ms间隔

    def _move_servo(self, servo_id: int, target_angle: float, move_time: int = 1000):
        """移动舵机"""
        pwm = self._angle_to_pwm(target_angle)
        cmd = f'#{servo_id:03d}P{pwm:04d}T{move_time:04d}!'
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()

    # ========== 优化3: 主循环 (批量读取 + 减少sleep) ==========
    def _angle_stream_loop_optimized(self):
        """优化版: 使用批量读取"""
        print("[THREAD] Angle stream loop started")
        arm_pos = [0.0] * 7
        period = 1.0 / self.rate  # 33ms (30Hz)
        next_time = time.monotonic()
        
        last_servo6_angle = None
        frame_count = 0
        last_print = time.monotonic()
        
        # 首次读取测试
        print("[THREAD] Testing first read...")
        test_angles, test_time = self._read_all_servos_optimized()
        print(f"[THREAD] First read OK: {len([a for a in test_angles if a is not None])}/7 servos in {test_time*1000:.1f}ms")

        while not self.stop_event.is_set():
            loop_start = time.monotonic()
            
            with self.lock:
                state = self.trigger_state
            
            # LOCKED状态: 保持位置
            if state == 'locked' and self.locked_positions:
                self._hold_position(self.locked_positions)
                time.sleep(0.02)  # 50Hz保持
                continue
            
            # RESUMING状态
            if state == 'resuming':
                time.sleep(0.03)
                continue
            
            # ========== TEACHING状态: 批量读取 ==========
            angles, read_time = self._read_all_servos_optimized()
            
            # 检查读取结果 (只要有5个以上成功就继续)
            valid_count = len([a for a in angles if a is not None])
            read_ok = valid_count >= 5  # 放宽到5个舵机
            
            if valid_count < 5 and frame_count % 30 == 0:
                print(f"\n[DEBUG] Read warning: {valid_count}/7 servos, time: {read_time*1000:.1f}ms")
            
            if read_ok:
                # 转换为相对角度
                for i in range(7):
                    arm_pos[i] = np.radians(angles[i] - self.zero_angles[i])
                
                # 更新队列 (只保留最新)
                try:
                    while True:
                        self.arm_pos_queue.get_nowait()
                except Empty:
                    pass
                self.arm_pos_queue.put(list(arm_pos))
                
                # 性能统计
                frame_count += 1
                now = time.monotonic()
                if now - last_print > 3.0:
                    avg_read = sum(self.read_times) / len(self.read_times) * 1000
                    actual_fps = frame_count / (now - last_print)
                    print(f"\n[PERF] Read: {avg_read:.1f}ms, FPS: {actual_fps:.1f}")
                    frame_count = 0
                    last_print = now
                
                # Trigger检测 (使用舵机6的绝对角度)
                if self.trigger_enabled and state == 'teaching':
                    servo6_abs = angles[6] if angles[6] is not None else self.zero_angles[6]
                    if servo6_abs is not None:
                        if abs(servo6_abs - self.trigger_angle) < self.trigger_tolerance:
                            print(f"\n\n[TRIGGER] *** TRIGGERED at {servo6_abs:.1f}° ***")
                            self._do_trigger_lock()
            else:
                # 读取失败，短暂等待重试
                time.sleep(0.005)
            
            # 维持目标频率
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()

    def _do_trigger_lock(self):
        """锁定"""
        print("[TRIGGER] Locking...")
        
        # 快速读取所有舵机位置
        angles, _ = self._read_all_servos_optimized()
        locked_pwm = []
        for i in range(7):
            if angles[i] is not None:
                pwm = self._angle_to_pwm(angles[i])
                locked_pwm.append(pwm)
            else:
                locked_pwm.append(self._angle_to_pwm(self.zero_angles[i]))
        
        with self.lock:
            self.locked_positions = locked_pwm
            self.trigger_state = 'locked'
        
        print("[TRIGGER] Locked! Press Enter...")

    def _do_trigger_unlock(self):
        """解锁"""
        print("\n[TRIGGER] Unlocking...")
        
        with self.lock:
            self.locked_positions = None
            self.trigger_state = 'resuming'
        
        time.sleep(0.3)
        print(f"[TRIGGER] Moving servo6 to {self.servo6_center}°...")
        self._move_servo(6, self.servo6_center, 1000)
        time.sleep(1.0)
        
        self._release_servos()
        
        with self.lock:
            self.trigger_state = 'teaching'
        print("[TRIGGER] Ready\n")

    def _keyboard_loop(self):
        """键盘监听"""
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except:
            return
        
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    state = self.trigger_state
                
                if state == 'locked':
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == '\n':
                            print("\n[KEYBOARD] Enter pressed")
                            self._do_trigger_unlock()
                else:
                    time.sleep(0.05)
        finally:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass

    # ========== 优化4: 仿真线程 ==========
    def _pose_consumer_loop_optimized(self):
        """优化版仿真控制"""
        period = 1.0 / 30.0  # 30Hz
        next_time = time.monotonic()
        last_action = None
        last_render = time.monotonic()

        while not self.stop_event.is_set():
            # 获取最新姿态
            try:
                pose = self.arm_pos_queue.get_nowait()
            except Empty:
                pose = None
            
            if pose is not None:
                try:
                    action = self._convert_pose(pose)
                    last_action = action.copy()
                    self.env.step(action)
                    self.env.render()
                    last_render = time.monotonic()
                except Exception as e:
                    print(f"[WARN] Sim error: {e}")
            else:
                # 无新数据，只渲染不step (降低CPU占用)
                if time.monotonic() - last_render > 0.033:  # 30Hz渲染
                    try:
                        self.env.render()
                        last_render = time.monotonic()
                    except:
                        pass
            
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()

    def _convert_pose(self, pose: list) -> np.ndarray:
        """Convert pose to action"""
        if self.robot_uids == "rm65b":
            pose_copy = pose.copy()
            pose_copy.pop(6)  # 移除夹爪
            # 修改旋转方向（哪个关节反了就把哪行注释去掉）
            # pose_copy[0] = -pose_copy[0]  # 1号舵机反向
            # pose_copy[1] = -pose_copy[1]  # 2号舵机反向
            # pose_copy[2] = -pose_copy[2]  # 3号舵机反向
            pose_copy[3] = -pose_copy[3]  # 4号舵机反向
            # pose_copy[4] = -pose_copy[4]  # 5号舵机反向
            # pose_copy[5] = -pose_copy[5]  # 6号舵机反向
            return np.array(pose_copy)
        elif self.robot_uids == "arx-x5":
            action = np.array(pose)
            action[-1] = max(0, 0.044 * (1 - pose[-1] / (1.5 * np.pi)))
            action = np.concatenate([action, [action[-1]]])
            action[2] = -action[2]
            action[4], action[5] = -action[5], -action[4]
            return action
        elif self.robot_uids == "so100":
            pose_copy = pose.copy()
            pose_copy.pop(5)
            action = np.array(pose_copy)
            action[-1] = np.clip(-1.1 + 2.2 * (1 - pose[-1] / (1.5 * np.pi)), -1.1, 1.1)
            action[0] = -action[0]
            action[3] = -action[3]
            action[4] = -action[4]
            return action
        else:
            return np.array(pose)

    def _setup_camera(self):
        """Setup camera"""
        agent = getattr(self.env.unwrapped, "agent", None)
        if agent:
            pose = agent.robot.get_pose()
            camera_pose = sapien_utils.look_at([0.8, -0.8, 0.6], pose.p)
            viewer = getattr(self.env.unwrapped, "viewer", None)
            if viewer:
                arr = camera_pose.raw_pose.squeeze().cpu().numpy()
                viewer.set_camera_pose(sapien.Pose(arr[:3], arr[3:]))

    def run(self):
        """Run"""
        print("\nStarting optimized threads...")
        self.produce_thread.start()
        self.consume_thread.start()
        if self.trigger_enabled:
            self.keyboard_thread.start()
            print("\n" + "="*60)
            print("[OPTIMIZED RM65B Trigger Mode]")
            print("="*60)
            print(f"Baudrate: {self.BAUDRATE} (unchanged)")
            print(f"Target Rate: {self.rate}Hz")
            print(f"Read Mode: Batch optimized")
            print("="*60 + "\n")
        
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.stop_event.set()
            self.produce_thread.join(timeout=2)
            self.consume_thread.join(timeout=2)
            if self.trigger_enabled:
                self.keyboard_thread.join(timeout=1)
            self.env.close()
            self.ser.close()
            print("Stopped")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', '-r', default='rm65b', 
                       choices=['arx-x5', 'so100', 'rm65b'])
    parser.add_argument('--scene', '-s', default='Empty-v1')
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--serial-port', default='/dev/ttyUSB0')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Robot Arm Teleoperation - OPTIMIZED (115200)")
    print("=" * 60)
    
    try:
        sim = ServoTeleoperatorSimOptimized(args.scene, args.robot, args.serial_port)
        sim.rate = args.rate
        sim.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
