#!/bin/bash
# Run servo2Dobot.py script

source /opt/ros/noetic/setup.bash

# Directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "$SCRIPT_DIR/../../../devel/setup.bash"

echo "servo2Dobot.py..."
rosrun UArm servo2Dobot.py & 
PID1=$!


# Run servo_reader.py script
echo "servo_reader.py..."
rosrun UArm servo_reader.py &
PID2=$!


# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2; exit" SIGINT

# === 等待所有子进程结束 ===
wait