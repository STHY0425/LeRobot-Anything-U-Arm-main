# group read servo position

import sys
import numpy as np

sys.path.append("..")
from scservo_sdk import *


portHandler = PortHandler('/dev/ttyUSB0')
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()


# Set port baudrate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Set all servos' half position
for scs_id in range(1, 9):
    # Unlock EPROM
    packetHandler.unLockEprom(scs_id)
    time.sleep(0.1)

    # First write 0 in
    comm, error = packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, 0)
    time.sleep(0.1)

    # Read present position
    raw_pos, result, error = packetHandler.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
    if result != COMM_SUCCESS:
        print(f"Failed to read the position: {packetHandler.getTxRxResult(result)}")


    Homing_Offset = raw_pos - 2047

    if Homing_Offset < 0:
        encoded_offset = (1 << 11) | abs(Homing_Offset)  # Highest bit is the sign bit
    else:
        encoded_offset = Homing_Offset

    # Write the offset in
    comm, error = packetHandler.write2ByteTxRx(scs_id, SMS_STS_OFS_L, encoded_offset)
    if error == 0:
        print(f"Succeeded to set the half position for id:%d" % scs_id)
    time.sleep(0.1)

    # Lock EPROM
    packetHandler.LockEprom(scs_id)
    time.sleep(0.1)


# Group read
groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
angle_pos = np.zeros(8)


while 1:
    for scs_id in range(1, 9):
        # Add parameter storage for SCServo#1~10 present position value
        scs_addparam_result = groupSyncRead.addParam(scs_id)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % scs_id)

    scs_comm_result = groupSyncRead.txRxPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))

    for scs_id in range(1, 9):
        # Check if groupsyncread data of SCServo#1~10 is available
        scs_data_result, scs_error = groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
        if scs_data_result == True:
            # Get SCServo#scs_id present position value
            scs_present_position = groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
            angle_pos[scs_id - 1] = scs_present_position
        else:
            print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
            continue
        if scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
    print(angle_pos)
    groupSyncRead.clearParam()
    # time.sleep(0.1)

