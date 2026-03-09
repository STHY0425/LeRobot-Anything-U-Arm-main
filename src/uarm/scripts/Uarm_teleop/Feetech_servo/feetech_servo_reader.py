#!/usr/bin/env python3
import rospy
import serial
import time
import sys
from std_msgs.msg import Float64MultiArray

sys.path.append("..")
from scservo_sdk import *

class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(50)

        self.portHandler = PortHandler('/dev/ttyUSB0')
        self.packetHandler = sms_sts(self.portHandler)
        self.portHandler.openPort()
        self.portHandler.setBaudRate(1000000)

        rospy.loginfo("Serial port opened")

        self.groupSyncRead = GroupSyncRead(self.packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
        self.gripper_range = 0.48
        self.zero_angles = [0.0] * 7
        self._init_servos()

    def _init_servos(self):
        # Calibrate
        rospy.loginfo("Calibrating servo half positions... DON'T MOVE!!!")
        for i in range(7):
            scs_id = i + 1
            # Unlock EPROM
            self.packetHandler.unLockEprom(scs_id)
            time.sleep(0.1)

            # First write 0 in
            comm, error = self.packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, 0)
            time.sleep(0.1)

            # Read present position
            raw_pos, result, error = self.packetHandler.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)

            Homing_Offset = raw_pos - 2047

            if Homing_Offset < 0:
                encoded_offset = (1 << 11) | abs(Homing_Offset)  # Highest bit is the sign bit
            else:
                encoded_offset = Homing_Offset

            # Write the offset in
            comm, error = self.packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, encoded_offset)
            if error == 0:
                rospy.loginfo(f"Succeeded to set the half position for id:%d" % scs_id)
            time.sleep(0.1)

            # Lock EPROM
            self.packetHandler.LockEprom(scs_id)
            time.sleep(0.1)

        # Read
        for i in range(7):
            scs_id = i + 1
            scs_addparam_result = self.groupSyncRead.addParam(scs_id)
            if scs_addparam_result != True:
                rospy.logwarn("[ID:%03d] groupSyncRead addparam failed" % scs_id)

        scs_comm_result = self.groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            rospy.logwarn("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        for i in range(7):
            scs_id = i + 1
            # Check if groupsyncread data of SCServo#1~7 is available
            scs_data_result, scs_error = self.groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
            if scs_data_result == True:
                # Get SCServo#scs_id present position value
                scs_present_position = self.groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                self.zero_angles[i] = scs_present_position
            else:
                self.zero_angles[i] = 2047
                rospy.logwarn("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                continue
            if scs_error != 0:
                rospy.logwarn("%s" % self.packetHandler.getRxPacketError(scs_error))
        rospy.loginfo(f"Zero positions (raw): {self.zero_angles}")
        self.groupSyncRead.clearParam()


    def run(self):
        angle_offset = [0.0] * 7  # Currently published angles
        target_angle_offset = [0.0] * 7  # Target angle for each servo
        num_interp = 5  # Interpolation steps
        step_size = 1  # Minimum change threshold

        while not rospy.is_shutdown():
            for i in range(7):
                scs_addparam_result = self.groupSyncRead.addParam(i + 1)
                if scs_addparam_result != True:
                    rospy.logwarn("[ID:%03d] groupSyncRead addparam failed" % scs_id)

            scs_comm_result = self.groupSyncRead.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                rospy.logwarn("GroupSyncRead failed")

            for i in range(7):
                scs_id = i + 1
                scs_data_result, scs_error = self.groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)

                if scs_data_result:
                    current_pos = self.groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                    if current_pos is not None:
                        new_angle = (current_pos - self.zero_angles[i]) / 4096.0 * 360.0
                        # please change the sign of the angle according to the direction of the follower arm
                        if abs(new_angle - target_angle_offset[i]) > step_size:
                            target_angle_offset[i] = new_angle
                    else:
                            rospy.logwarn(f"Can't read servo {scs_id}")
                else:
                    rospy.logwarn(f"Failed to get data for ID {scs_id}")

            self.groupSyncRead.clearParam()

            # Interpolate to approach target angle
            for step in range(num_interp):
                for i in range(7):
                    delta = target_angle_offset[i] - angle_offset[i]
                    angle_offset[i] += delta * 0.2  # Lazy interpolation, coefficient < 1 for adjustable smoothness
                self.pub.publish(Float64MultiArray(data=angle_offset))
                self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass