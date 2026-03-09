import gymnasium as gym
import mani_skill.envs
import numpy as np

print('='*60)
print('  快速图形化仿真测试')
print('='*60)
print()

# 使用最简单的 PickCube 任务（不需要下载场景）
env = gym.make(
    'PickCube-v1',
    robot_uids='panda',
    render_mode='human',
    control_mode='pd_joint_pos'
)

print('✅ 环境创建成功！')
print(f'观察空间: {list(env.observation_space.keys()) if hasattr(env.observation_space, "keys") else type(env.observation_space)}')
print(f'动作空间: {env.action_space}')
print()
print('按 Ctrl+C 退出')
print()

obs, info = env.reset()

try:
    step = 0
    while True:
        # 随机动作
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()
        
        step += 1
        if step % 50 == 0:
            print(f'步骤: {step}, 奖励: {reward:.3f}')
        
        if terminated or truncated:
            print('回合结束，重置环境')
            obs, info = env.reset()
            step = 0
            
except KeyboardInterrupt:
    print()
    print('退出测试')
finally:
    env.close()
    print('环境已关闭')
