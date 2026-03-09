# set new id for servo
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

# id for setting. Please set id from #1 to #7 (or #8 for Config 3)
new_id = 8
print(f"Set the servo's id to: {new_id} (Make sure only one servo is connected)...")

# Unlock EPROM
packetHandler.unLockEprom(BROADCAST_ID)
time.sleep(0.1)

# Write new id in
result, error = packetHandler.write1ByteTxRx(BROADCAST_ID, SMS_STS_ID, new_id)
if result != COMM_SUCCESS:
    print(f"Failed to set id: {packetHandler.getTxRxResult(result)}")
    # Lock EPROM
    packetHandler.LockEprom(BROADCAST_ID)
    portHandler.closePort()
    exit()

print(f"Succeed to set id: {new_id}")
time.sleep(0.1)

# Lock EPROM
packetHandler.LockEprom(new_id)
time.sleep(0.1)