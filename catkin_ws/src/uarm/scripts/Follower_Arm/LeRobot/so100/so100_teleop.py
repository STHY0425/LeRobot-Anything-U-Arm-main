import time
import numpy as np

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.teleoperators.uarm import ServoReader

FPS = 30

# Create the robot and teleoperator configurations
robot_so_config = SO100FollowerConfig(port="/dev/ttyACM1", id="my_so_100")
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

# Initialize the robot and teleoperator
robot_so = SO100Follower(robot_so_config)
keyboard = KeyboardTeleop(keyboard_config)

robot_so.connect()
keyboard.connect()
reader = ServoReader(port='/dev/ttyUSB0', baudrate=115200)

# Init rerun viewer
init_rerun(session_name="so100_teleop")

print("Starting teleop loop...")
while True:
    t0 = time.perf_counter()

    # Get robot observation
    observation = robot_so.get_observation()

    offset = reader.get_action_offset()

    so_action = {
        "shoulder_pan.pos": -offset[0] * 1.5,
        "shoulder_lift.pos": offset[1] * 1.5,
        "elbow_flex.pos": offset[2] * 1.5,
        "wrist_flex.pos": -offset[4] * 1.5,
        "wrist_roll.pos": -offset[5] * 1.5-offset[3] * 1.5,
        "gripper.pos": offset[6] * 1.5,
    }
    keyboard_keys = keyboard.get_action()

    # Send action to robot
    if "q" in keyboard_keys:
        print("Quitting teleop...")
        robot_so.disconnect()
        keyboard.disconnect()
        break
    _ = robot_so.send_action(so_action)

    # Visualize
    log_rerun_data(observation=observation, action=so_action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
