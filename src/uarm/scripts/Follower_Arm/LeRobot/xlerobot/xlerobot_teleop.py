import time
import numpy as np

from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.teleoperators.uarm import ServoReader

# XLerobot teleop keys
HEAD_KEYMAP = {
    "head_motor_1+": "<", "head_motor_1-": ">",
    "head_motor_2+": ",", "head_motor_2-": ".",
}

# Head motor mapping
HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.81):
        self.kp = kp
        self.degree_step = 1
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'):
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_1-'):
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_2+'):
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if key_state.get('head_motor_2-'):
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

def main():
    FPS = 30

    # Create the robot and teleoperator configurations
    robot_config = XLerobotConfig() # modify port in the config_xlerobot.py file
    keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    # Initialize the robot and teleoperator
    robot = XLerobot(robot_config)
    keyboard = KeyboardTeleop(keyboard_config)

    robot.connect()
    keyboard.connect()
    reader_left = ServoReader(port='/dev/ttyUSB0', baudrate=115200)
    reader_right = ServoReader(port='/dev/ttyUSB1', baudrate=115200)

    obs = robot.get_observation()
    head_control = SimpleHeadControl(obs)

    # Init rerun viewer
    init_rerun(session_name="xlerobot_teleop")

    print("Starting teleop loop...")
    while True:
        t0 = time.perf_counter()

        # Get robot observation
        observation = robot.get_observation()

        # arm action
        offset_left = reader_left.get_action_offset()
        offset_right = reader_right.get_action_offset()
        
        arm_action_left = {
            "left_arm_shoulder_pan.pos": -offset_left[0] * 1.5,
            "left_arm_shoulder_lift.pos": offset_left[1] * 1.5,
            "left_arm_elbow_flex.pos": offset_left[2] * 1.5,
            "left_arm_wrist_flex.pos": -offset_left[4] * 1.5,
            "left_arm_wrist_roll.pos": -offset_left[5] * 1.5-offset_left[3] * 1.5,
            "left_arm_gripper.pos": offset_left[6] * 1.5,
        }

        arm_action_right = {
            "right_arm_shoulder_pan.pos": -offset_right[0] * 1.5,
            "right_arm_shoulder_lift.pos": offset_right[1] * 1.5,
            "right_arm_elbow_flex.pos": offset_right[2] * 1.5,
            "right_arm_wrist_flex.pos": -offset_right[4] * 1.5,
            "right_arm_wrist_roll.pos": -offset_right[5] * 1.5-offset_right[3] * 1.5,
            "right_arm_gripper.pos": offset_right[6] * 1.5,
        }

        keyboard_keys = keyboard.get_action()

        # head action
        pressed_keys = set(keyboard_keys.keys())
        head_key_state = {action: (key in pressed_keys) for action, key in HEAD_KEYMAP.items()}
        head_control.handle_keys(head_key_state)
        head_action = head_control.p_control_action(robot)

        keyboard_keys = np.array(list(pressed_keys))
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)

        action = {**arm_action_left, **arm_action_right, **head_action, **base_action}

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

if __name__ == '__main__':
    main()
