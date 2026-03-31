#!/usr/bin/env python3
"""
简化版 RM65B Trigger 功能
========================

功能：
1. 实时监控6号舵机角度
2. 角度在101°±5°时触发锁定
3. 锁定：持续发送当前位置命令保持舵机不动
4. 按Enter后：6号舵机复位到135°，释放力矩，继续示教
5. 仿真全程正常运行

使用方式：
    python teleop_sim_trigger_test_v2.py --robot rm65b
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
from mani_skill.utils import sapien_utils


class ServoTeleoperatorSim:
    """Robot arm teleoperation simulation system with RM65B trigger support"""
    
    def __init__(self, scene: str, robot_uids: str, serial_port: str = '/dev/ttyUSB0'):
        # Serial port
        self.SERIAL_PORT = serial_port
        self.BAUDRATE = 115200
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        # Config
        self.scene = scene
        self.robot_uids = robot_uids
        self.zero_angles = [0.0] * 7
        self.stop_event = Event()
        self.rate = 20.0
        self.arm_pos_queue = Queue(maxsize=1)
        
        # RM65B Trigger config
        self.is_rm65b = (robot_uids == "rm65b")
        self.trigger_enabled = self.is_rm65b
        self.trigger_angle = 101.0  # 触发角度（绝对角度）
        self.trigger_tolerance = 15.0  # 容差 ±5度
        self.servo6_center = 135.0  # 复位角度
        
        # Trigger state: 'teaching', 'locked', 'resuming'
        self.trigger_state = 'teaching'
        self.locked_positions = None  # 锁定时各舵机位置（PWM值）
        self.enter_pressed = Event()
        self.lock = Lock()
        
        # Initialize servos
        self._init_servos()

        # Create env
        self.control_mode = "pd_joint_pos"
        self.env = gym.make(
            scene,
            robot_uids=robot_uids,
            render_mode="human",
            control_mode=self.control_mode,
        )
        obs, _ = self.env.reset(seed=0)
        print(f"Action space: {self.env.action_space}")

        # Threads
        self.produce_thread = Thread(target=self._angle_stream_loop, daemon=True)
        self.consume_thread = Thread(target=self._pose_consumer_loop, daemon=True)
        if self.trigger_enabled:
            self.keyboard_thread = Thread(target=self._keyboard_loop, daemon=True)
        self._setup_camera()

    def _init_servos(self):
        """Initialize and calibrate zero positions"""
        for i in range(7):
            self.ser.write(f'#{i:03d}PULK!'.encode('ascii'))
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.ser.write(f'#{i:03d}PRAD!'.encode('ascii'))
            time.sleep(0.05)
            response = self.ser.read_all().decode('ascii', errors='ignore')
            angle = self._parse_angle(response)
            self.zero_angles[i] = angle if angle is not None else 135.0
            print(f"Servo {i} zero: {self.zero_angles[i]:.1f}°")
        
        if self.is_rm65b:
            print(f"\n[TRIGGER] Enabled - Target: {self.trigger_angle}°±{self.trigger_tolerance}°")

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

    def _read_servo_angle(self, servo_id: int) -> float:
        """Read servo angle (absolute)"""
        self.ser.reset_input_buffer()
        self.ser.write(f'#{servo_id:03d}PRAD!'.encode('ascii'))
        time.sleep(0.03)
        response = self.ser.read_all().decode('ascii', errors='ignore')
        return self._parse_angle(response)

    def _hold_position(self, positions_pwm: list):
        """Hold servos at specific PWM positions"""
        for i, pwm in enumerate(positions_pwm):
            cmd = f'#{i:03d}P{pwm:04d}T0050!'  # 50ms move time
            self.ser.write(cmd.encode('ascii'))
            time.sleep(0.005)

    def _release_servos(self):
        """Release all servo torque"""
        print("[SERVO] Sending PULK commands...")
        for i in range(7):
            cmd = f'#{i:03d}PULK!'
            self.ser.write(cmd.encode('ascii'))
            time.sleep(0.02)
        print("[SERVO] All servos released")

    def _move_servo(self, servo_id: int, target_angle: float, move_time: int = 1000):
        """Move servo to target angle"""
        pwm = self._angle_to_pwm(target_angle)
        cmd = f'#{servo_id:03d}P{pwm:04d}T{move_time:04d}!'
        print(f"[SERVO] Command: {cmd}")
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()

    def _angle_stream_loop(self):
        """Producer: Read servo angles and handle trigger"""
        arm_pos = [0.0] * 7
        period = 1.0 / self.rate
        next_time = time.monotonic()
        
        # For RM65B trigger detection
        last_servo6_angle = None
        debug_counter = 0

        while not self.stop_event.is_set():
            # Check trigger state
            with self.lock:
                state = self.trigger_state
            
            # If locked, hold position and skip reading
            if state == 'locked' and self.locked_positions:
                self._hold_position(self.locked_positions)
                if debug_counter % 50 == 0:
                    print(f"\r[TRIGGER] LOCKED - Press Enter to unlock", end='', flush=True)
                time.sleep(0.02)
                debug_counter += 1
                continue
            
            # If resuming, wait for completion (skip trigger detection)
            if state == 'resuming':
                if debug_counter % 20 == 0:
                    print(f"\r[TRIGGER] RESUMING - Please wait...", end='', flush=True)
                time.sleep(0.05)
                debug_counter += 1
                continue
            
            # Print teaching status occasionally
            if self.trigger_enabled and debug_counter % 50 == 0:
                print(f"\r[TRIGGER] TEACHING - Move servo6 to {self.trigger_angle}°", end='', flush=True)
            
            # Read all servos
            read_ok = True
            for i in range(7):
                angle = self._read_servo_angle(i)
                if angle is not None:
                    arm_pos[i] = np.radians(angle - self.zero_angles[i])
                else:
                    read_ok = False
                    break
            
            if read_ok:
                # Update queue
                try:
                    while True:
                        self.arm_pos_queue.get_nowait()
                except Empty:
                    pass
                self.arm_pos_queue.put(list(arm_pos))
                
                # RM65B Trigger detection (only in teaching state)
                if self.trigger_enabled and state == 'teaching':
                    servo6_abs = self._read_servo_angle(6)
                    if servo6_abs is not None:
                        # Debug print
                        debug_counter += 1
                        if debug_counter % 30 == 0 or (last_servo6_angle and abs(servo6_abs - last_servo6_angle) > 3):
                            print(f"[TRIGGER] Servo6: {servo6_abs:.1f}° (target: {self.trigger_angle}°)")
                        last_servo6_angle = servo6_abs
                        
                        # Check trigger condition
                        if abs(servo6_abs - self.trigger_angle) < self.trigger_tolerance:
                            print(f"\n\n[TRIGGER] *** TRIGGERED at {servo6_abs:.1f}° ***")
                            self._do_trigger_lock()
            
            # Maintain rate
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()

    def _do_trigger_lock(self):
        """Perform trigger lock"""
        print("[TRIGGER] Locking all servos...")
        
        # Read current positions of all servos
        locked_pwm = []
        for i in range(7):
            angle = self._read_servo_angle(i)
            if angle is not None:
                pwm = self._angle_to_pwm(angle)
                locked_pwm.append(pwm)
            else:
                locked_pwm.append(self._angle_to_pwm(self.zero_angles[i]))
        
        with self.lock:
            self.locked_positions = locked_pwm
            self.trigger_state = 'locked'
        
        print("[TRIGGER] Locked! Press Enter to continue...")

    def _do_trigger_unlock(self):
        """Perform trigger unlock"""
        print("\n[TRIGGER] ========== UNLOCK START ==========")
        
        # Step 1: Set resuming state (disable trigger detection)
        print("[TRIGGER] Step 1: Entering resuming state...")
        with self.lock:
            self.locked_positions = None
            self.trigger_state = 'resuming'  # 禁用触发检测
        print("[TRIGGER] Trigger detection disabled")
        
        # Step 2: Move servo6 to center
        time.sleep(1.5)
        print(f"[TRIGGER] Step 2: Moving servo6 to {self.servo6_center}°...")
        self._move_servo(6, self.servo6_center, 1000)
        print("[TRIGGER] Move command sent, waiting 1.5s...")
        time.sleep(1.5)
        print("[TRIGGER] Move complete")
        
        # Step 3: Wait for servo6 to actually reach center (with tolerance)
        print("[TRIGGER] Step 3: Verifying servo6 position...")
        for _ in range(20):  # 最多等待1秒
            angle = self._read_servo_angle(6)
            if angle and abs(angle - self.servo6_center) < 10:  # 10度容差
                print(f"[TRIGGER] Servo6 reached {angle:.1f}° (target: {self.servo6_center}°)")
                break
            time.sleep(0.05)
        else:
            print("[TRIGGER] Warning: Servo6 may not have reached target")
        
        # Step 4: Release torque
        print("[TRIGGER] Step 4: Releasing servos...")
        self._release_servos()
        print("[TRIGGER] Servos released")
        
        # Step 5: Re-enable trigger detection
        print("[TRIGGER] Step 5: Re-enabling trigger detection...")
        with self.lock:
            self.trigger_state = 'teaching'
        print("[TRIGGER] Ready for teaching")
        
        print("[TRIGGER] ========== UNLOCK COMPLETE ==========\n")

    def _keyboard_loop(self):
        """Keyboard listener for Enter key"""
        print("[KEYBOARD] Thread started, waiting for Enter key...")
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print("[KEYBOARD] Terminal settings saved")
        except Exception as e:
            print(f"[KEYBOARD] Error setting up keyboard: {e}")
            return
        
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    state = self.trigger_state
                
                if state == 'locked':
                    # Check for Enter key
                    try:
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            key = sys.stdin.read(1)
                            if key == '\n':
                                print("\n[KEYBOARD] Enter pressed, unlocking...")
                                self._do_trigger_unlock()
                    except Exception as e:
                        print(f"[KEYBOARD] Error reading key: {e}")
                else:
                    time.sleep(0.05)
        finally:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                print("[KEYBOARD] Terminal settings restored")
            except:
                pass

    def _pose_consumer_loop(self):
        """Consumer: Control simulation"""
        period = 1.0 / self.rate
        next_time = time.monotonic()
        last_action = None

        while not self.stop_event.is_set():
            # Get latest pose
            try:
                pose = self.arm_pos_queue.get_nowait()
            except Empty:
                pose = None
            
            if pose is not None:
                try:
                    # Convert to action
                    action = self._convert_pose(pose)
                    last_action = action.copy()
                    
                    # Step simulation
                    self.env.step(action)
                    self.env.render()
                except Exception as e:
                    print(f"[WARN] Sim error: {e}")
            elif last_action is not None:
                # No new data, keep rendering
                try:
                    self.env.render()
                except:
                    pass
            
            next_time += period
            sleep_dt = next_time - time.monotonic()
            if sleep_dt > 0:
                time.sleep(sleep_dt)
            else:
                next_time = time.monotonic()

    def _convert_pose(self, pose: list) -> np.ndarray:
        """Convert pose to action based on robot type"""
        if self.robot_uids == "rm65b":
            # RM65B: 6 DOF, remove servo 6 (index 6)
            pose_copy = pose.copy()
            pose_copy.pop(6)
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
        """Setup camera pose"""
        agent = getattr(self.env.unwrapped, "agent", None)
        if agent:
            pose = agent.robot.get_pose()
            camera_pose = sapien_utils.look_at([0.8, -0.8, 0.6], pose.p)
            viewer = getattr(self.env.unwrapped, "viewer", None)
            if viewer:
                arr = camera_pose.raw_pose.squeeze().cpu().numpy()
                viewer.set_camera_pose(sapien.Pose(arr[:3], arr[3:]))

    def run(self):
        """Run the system"""
        print("\nStarting threads...")
        self.produce_thread.start()
        self.consume_thread.start()
        if self.trigger_enabled:
            self.keyboard_thread.start()
            print("\n" + "="*60)
            print("[RM65B Trigger Mode]")
            print("="*60)
            print(f"1. Move servo6 to {self.trigger_angle}°±{self.trigger_tolerance}° to LOCK")
            print("2. Press Enter to UNLOCK and reset servo6 to 135°")
            print("3. Ctrl+C to exit")
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
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--serial-port', default='/dev/ttyUSB0')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Robot Arm Teleoperation with Trigger")
    print("=" * 60)
    
    try:
        sim = ServoTeleoperatorSim(args.scene, args.robot, args.serial_port)
        sim.rate = args.rate
        sim.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
