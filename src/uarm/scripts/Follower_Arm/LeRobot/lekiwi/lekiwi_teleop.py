import time
import numpy as np

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.teleoperators.uarm import ServoReader

FPS = 30

# Create the robot and teleoperator configurations
robot_config = LeKiwiConfig(port="/dev/ttyACM0", id="my_lekiwi")
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

# Initialize the robot and teleoperator
robot = LeKiwi(robot_config)
keyboard = KeyboardTeleop(keyboard_config)

robot.connect()
keyboard.connect()
reader = ServoReader(port='/dev/ttyUSB0', baudrate=115200)

# Init rerun viewer
init_rerun(session_name="lekiwi_teleop")

print("Starting teleop loop...")
while True:
    t0 = time.perf_counter()

    # Get robot observation
    observation = robot.get_observation()

    offset = reader.get_action_offset()
    
    arm_action = {
        "arm_shoulder_pan.pos": -offset[0] * 1.5,
        "arm_shoulder_lift.pos": offset[1] * 1.5,
        "arm_elbow_flex.pos": offset[2] * 1.5,
        "arm_wrist_flex.pos": -offset[4] * 1.5,
        "arm_wrist_roll.pos": -offset[5] * 1.5-offset[3] * 1.5,
        "arm_gripper.pos": offset[6] * 1.5,
    }

    keyboard_keys = keyboard.get_action()
    base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    # Send action to robot
    if robot.teleop_keys["quit"] in keyboard_keys:
        print("Quitting teleop...")
        robot.disconnect()
        keyboard.disconnect()
        break
    _ = robot.send_action(action)

    # Visualize
    log_rerun_data(observation=observation, action=action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
