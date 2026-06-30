#!/usr/bin/env python3
"""
舵机控制仿真演示程序（无需硬件）
模拟舵机输入，展示实时控制效果
"""

import gymnasium as gym
import mani_skill.envs
import numpy as np
import time
import argparse
from threading import Thread, Event


class SimulatedServoTeleop:
    """模拟舵机遥操作（无需硬件）"""
    
    def __init__(self, robot_uids: str = "panda", mode: str = "sine"):
        """
        Args:
            robot_uids: 机器人类型
            mode: 运动模式 (sine, circle, random, manual)
        """
        self.robot_uids = robot_uids
        self.mode = mode
        self.stop_event = Event()
        
        # 创建仿真环境
        print(f"创建仿真环境: {robot_uids}")
        self.env = gym.make(
            "Empty-v1",
            robot_uids=robot_uids,
            render_mode="human",
            control_mode="pd_joint_pos",
        )
        
        obs, _ = self.env.reset(seed=0)
        print(f"动作空间: {self.env.action_space}")
        
        # 初始姿态
        self.home_pose = self._get_home_pose()
        self.current_time = 0.0
        
    def _get_home_pose(self):
        """获取机器人的 home 姿态"""
        if self.robot_uids == "panda":
            return np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.78, 0.04])
        elif self.robot_uids == "so100":
            return np.array([0.0, -1.57, 1.57, 0.66, 0.0, -1.1])
        elif self.robot_uids == "xarm6_robotiq":
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            # 默认全零
            return np.zeros(self.env.action_space.shape[0])
    
    def generate_sine_motion(self):
        """生成正弦波运动"""
        # 正弦波参数
        amplitude = 0.5  # 幅度
        frequency = 0.5  # 频率 (Hz)
        
        action = self.home_pose.copy()
        
        # 第一个关节做正弦运动
        action[0] = amplitude * np.sin(2 * np.pi * frequency * self.current_time)
        
        # 第二个关节做余弦运动
        if len(action) > 1:
            action[1] = self.home_pose[1] + 0.3 * np.cos(2 * np.pi * frequency * self.current_time)
        
        return action
    
    def generate_circle_motion(self):
        """生成圆周运动（末端画圆）"""
        action = self.home_pose.copy()
        
        # 使用前两个关节画圆
        radius = 0.3
        frequency = 0.3
        
        action[0] = radius * np.cos(2 * np.pi * frequency * self.current_time)
        if len(action) > 1:
            action[1] = self.home_pose[1] + radius * np.sin(2 * np.pi * frequency * self.current_time)
        
        return action
    
    def generate_random_motion(self):
        """生成随机运动"""
        # 在 home 姿态附近随机扰动
        noise = np.random.randn(len(self.home_pose)) * 0.1
        action = self.home_pose + noise
        
        # 限制范围
        action = np.clip(action, -2.0, 2.0)
        
        return action
    
    def generate_wave_motion(self):
        """生成波浪运动（所有关节依次运动）"""
        action = self.home_pose.copy()
        
        # 每个关节有不同的相位
        for i in range(min(6, len(action))):
            phase = i * np.pi / 3  # 60度相位差
            action[i] = self.home_pose[i] + 0.3 * np.sin(
                2 * np.pi * 0.5 * self.current_time + phase
            )
        
        return action
    
    def run(self):
        """运行仿真"""
        print("=" * 60)
        print("  舵机控制仿真演示（无需硬件）")
        print("=" * 60)
        print(f"机器人: {self.robot_uids}")
        print(f"运动模式: {self.mode}")
        print("-" * 60)
        print("按 Ctrl+C 停止")
        print("=" * 60)
        
        dt = 0.02  # 50 Hz
        
        try:
            while not self.stop_event.is_set():
                # 根据模式生成动作
                if self.mode == "sine":
                    action = self.generate_sine_motion()
                elif self.mode == "circle":
                    action = self.generate_circle_motion()
                elif self.mode == "random":
                    action = self.generate_random_motion()
                elif self.mode == "wave":
                    action = self.generate_wave_motion()
                else:
                    action = self.home_pose
                
                # 执行动作
                self.env.step(action)
                self.env.render()
                
                # 更新时间
                self.current_time += dt
                time.sleep(dt)
                
        except KeyboardInterrupt:
            print("\n收到中断信号，停止...")
        finally:
            self.env.close()
            print("仿真已关闭")


class InteractiveServoTeleop:
    """交互式舵机控制（键盘输入）"""
    
    def __init__(self, robot_uids: str = "panda"):
        self.robot_uids = robot_uids
        
        # 创建环境
        self.env = gym.make(
            "Empty-v1",
            robot_uids=robot_uids,
            render_mode="human",
            control_mode="pd_joint_pos",
        )
        
        self.env.reset()
        
        # 当前姿态
        self.current_qpos = self._get_home_pose()
        
        # 控制参数
        self.active_joint = 0
        self.delta = 0.05
        
    def _get_home_pose(self):
        if self.robot_uids == "panda":
            return np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.78, 0.04])
        elif self.robot_uids == "so100":
            return np.array([0.0, -1.57, 1.57, 0.66, 0.0, -1.1])
        else:
            return np.zeros(self.env.action_space.shape[0])
    
    def print_help(self):
        print("=" * 60)
        print("  交互式舵机控制")
        print("=" * 60)
        print("键盘控制:")
        print("  1-7/8: 选择关节")
        print("  w/s: 增加/减少角度")
        print("  a/d: 快速调整")
        print("  r: 重置到 home 姿态")
        print("  h: 显示帮助")
        print("  q: 退出")
        print("=" * 60)
        print(f"当前机器人: {self.robot_uids}")
        print(f"关节数量: {len(self.current_qpos)}")
        print("=" * 60)
    
    def run(self):
        """运行交互式控制"""
        self.print_help()
        
        print("\n提示: 由于终端限制，请使用以下命令:")
        print("  w - 增加当前关节角度")
        print("  s - 减少当前关节角度")
        print("  1-7 - 选择关节")
        print()
        
        try:
            import sys
            import tty
            import termios
            
            # 保存终端设置
            old_settings = termios.tcgetattr(sys.stdin)
            
            try:
                tty.setcbreak(sys.stdin.fileno())
                
                print(f"当前控制关节: {self.active_joint}")
                
                while True:
                    # 执行当前姿态
                    self.env.step(self.current_qpos)
                    self.env.render()
                    
                    # 检查键盘输入（非阻塞）
                    import select
                    if select.select([sys.stdin], [], [], 0.02)[0]:
                        key = sys.stdin.read(1)
                        
                        if key in '12345678':
                            self.active_joint = int(key) - 1
                            if self.active_joint < len(self.current_qpos):
                                print(f"\n选择关节 {self.active_joint}")
                            else:
                                print(f"\n关节 {self.active_joint} 不存在")
                                self.active_joint = 0
                        
                        elif key == 'w':
                            self.current_qpos[self.active_joint] += self.delta
                            print(f"\n关节 {self.active_joint}: {self.current_qpos[self.active_joint]:.3f} rad ({np.degrees(self.current_qpos[self.active_joint]):.1f}°)")
                        
                        elif key == 's':
                            self.current_qpos[self.active_joint] -= self.delta
                            print(f"\n关节 {self.active_joint}: {self.current_qpos[self.active_joint]:.3f} rad ({np.degrees(self.current_qpos[self.active_joint]):.1f}°)")
                        
                        elif key == 'a':
                            self.current_qpos[self.active_joint] -= self.delta * 2
                            print(f"\n关节 {self.active_joint}: {self.current_qpos[self.active_joint]:.3f} rad")
                        
                        elif key == 'd':
                            self.current_qpos[self.active_joint] += self.delta * 2
                            print(f"\n关节 {self.active_joint}: {self.current_qpos[self.active_joint]:.3f} rad")
                        
                        elif key == 'r':
                            self.current_qpos = self._get_home_pose()
                            print("\n重置到 home 姿态")
                        
                        elif key == 'h':
                            self.print_help()
                        
                        elif key == 'q':
                            break
                    
                    time.sleep(0.02)
            
            finally:
                # 恢复终端设置
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        except ImportError:
            print("警告: 无法使用交互式控制（需要 Unix 系统）")
            print("使用自动演示模式...")
            
            # 回退到自动模式
            demo = SimulatedServoTeleop(self.robot_uids, "sine")
            demo.run()
        
        except KeyboardInterrupt:
            print("\n停止...")
        
        finally:
            self.env.close()


def main():
    parser = argparse.ArgumentParser(
        description='舵机控制仿真演示（无需硬件）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--robot', '-r',
        type=str,
        default='panda',
        choices=['panda', 'so100', 'xarm6_robotiq', 'arx-x5', 'piper'],
        help='机器人类型'
    )
    
    parser.add_argument(
        '--mode', '-m',
        type=str,
        default='sine',
        choices=['sine', 'circle', 'random', 'wave', 'interactive'],
        help='运动模式'
    )
    
    args = parser.parse_args()
    
    if args.mode == 'interactive':
        teleop = InteractiveServoTeleop(args.robot)
        teleop.run()
    else:
        teleop = SimulatedServoTeleop(args.robot, args.mode)
        teleop.run()


if __name__ == "__main__":
    main()
