#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from api import Bestman_Real_CR5
import time
import math
import numpy as np

class RobotControlNode:
    def __init__(self):
        self.robot_ip = "192.168.5.1"  # Please replace with actual robot IP address
        self.robot = Bestman_Real_CR5(ip=self.robot_ip, text_log=True)

        # ROS initialization
        rospy.init_node('dobot_teleop_node')
        # Publish current robot state topic
        self.state_pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)

        # Subscribe to control command topic
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.joint_cmd_callback)

        # Timer to publish robot state
        self.rate = rospy.Rate(10)  # 10hz

        self.init_qpos = np.array([180,-20,-96,62,93,205])
        self.robot.move_arm_to_joint_angles(self.init_qpos,wait=True)

    def joint_cmd_callback(self, msg):
        """Handle received joint control commands"""
        try:
            # Print received data
            print(f"msg.data:{msg.data[:6]}")
            servo_angles = np.array(msg.data)

            servo_angles[1] = -servo_angles[1] #adjustment

            joint_angles = (servo_angles[:6]+self.init_qpos)

            
            # joint_angles = (np.array(msg.data[:6]) + np.array(self.init_qpos)).tolist()
            
            self.robot.move_arm_to_joint_angles_servo(joint_angles,t=0.1,gain=250)
            # rospy.loginfo(f"Moved robot to specified joint angles: {joint_angles}")
        except Exception as e:
            rospy.logerr(f"Failed to move robot: {e}")

    def run(self):
        """Main loop, publish state and continuously listen for control commands"""
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        robot_node = RobotControlNode()
        robot_node.run()
    except rospy.ROSInterruptException:
        pass
