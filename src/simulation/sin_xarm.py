import time
import numpy as np
import gymnasium as gym
import mani_skill.envs  # Must import to register all env/agent

def main():
    env = gym.make(
        "PushCube-v1", # "Empty-v1",
        robot_uids="xarm6_robotiq", 
        render_mode="human",        # Can be changed to "human" for window rendering
        control_mode="pd_joint_pos",
    )
    obs, _ = env.reset(seed=0)
    print("Action space:", env.action_space)

    # Two sets of joint angles (radian examples), can be adjusted as needed:
    pose_a = np.radians([14.1, -8, -24.7, 196.9, 62.3, -8.8, 0.0])
    pose_b = np.radians([-30, -8, 0, 196.9, 62.3, -8.8, 0.0])

    steps = 200            # Number of steps for one round trip
    dwell = 0.01           # Dwell time per step (seconds)

    while True:
        # Gradually transition from pose_a to pose_b
        for t in np.linspace(0, 1, steps):
            action = (1 - t) * pose_a + t * pose_b
            env.step(action)
            env.render()   # If "sensors", can get returned images for processing here
            time.sleep(dwell)

        # Then gradually return from pose_b to pose_a
        for t in np.linspace(0, 1, steps):
            action = (1 - t) * pose_b + t * pose_a
            env.step(action)
            env.render()
            time.sleep(dwell)

    env.close()

if __name__ == "__main__":
    main()
