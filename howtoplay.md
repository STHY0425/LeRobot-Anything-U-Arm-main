# 🔧 System Setup

## Prerequisites

- **Ubuntu 20.04**
- [**ROS Noetic**](https://wiki.ros.org/noetic/Installation/Ubuntu)
- **Python 3.9+**

---

## Step-by-Step Setup

1. **Install Python Dependencies**

   ```sh
   # install both ros1 and simulation requirements
   pip install -r overall_requirements.txt 
   ```
   
   If your system doesn't support ROS1, you can install the dependencies without ROS1 with the following command which supports simulation teleoperation and check [this note](https://github.com/MINT-SJTU/Lerobot-Anything-U-arm/blob/main/src/simulation/README.md) . 
   ```sh
   pip install -r requirements.txt
   ```

3. **Build Catkin Workspace**

   ```sh
   catkin_make
   source devel/setup.bash
   ```

4. **Verify Installation**

   ```sh
   # Test if ROS can find the package
   rospack find uarm
   ```

---

# 🤖 Plug-and-Play with Real Robot with ROS1
> Zhonglin servo version
## 1. Start ROS Core

Open a terminal and run:

```sh
roscore
```

## 2. Verify Teleop Arm Output

In a new terminal, check servo readings:

```sh
rosrun uarm servo_zero.py
```

This will display real-time angles from all servos. You should check whether `SERIAL_PORT` is available on your device and modify the variable if necessary. 

## 3. Publish Teleop Data

Still in the second terminal, start the teleop publisher:

```sh
rosrun uarm servo_reader.py
```

Your teleop arm now publishes to the `/servo_angles` topic.

## 4. Control the Follower Arm

Choose your robot and run the corresponding script:

- **For Dobot CR5:**
  ```sh
  rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
  ```

- **For xArm:**
  ```sh
  rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
  ```

---

> Feetech servo version (Global Version)
## 1. Start ROS Core

Open a terminal and run:

```sh
roscore
```

## 2. Verify Teleop Arm Output

In a new terminal, check servo readings:

```sh
rosrun uarm feetech_servo_zero.py
```

This will display real-time angles from all servos. You should check whether `SERIAL_PORT` is available on your device and modify the variable if necessary. You may find all servos' angles are `2047` since servo's position is set as `2047` (0~4095 for $360^\circ$) when this code starts running.

## 3. Publish Teleop Data

Still in the second terminal, start the teleop publisher:

```sh
rosrun uarm feetech_servo_reader.py
```

Servo's position is set again as `2047` when this code starts running . **Please return UARM to initial position before starting this script.** Your teleop arm now publishes to the `/servo_angles` topic.

## 4. Control the Follower Arm

Choose your robot and run the corresponding script:

- **For Dobot CR5:**
  ```sh
  rosrun uarm scripts/Follower_Arm/Dobot/servo2Dobot.py
  ```

- **For xArm:**
  ```sh
  rosrun uarm scripts/Follower_Arm/xarm/servo2xarm.py
  ```
---

# 🖥️ Try It Out in Simulation

If you do not have robot hardware, you can try teleoperation in simulation.  
See detailed guidance [here](https://github.com/MINT-SJTU/Lerobot-Anything-U-arm/blob/feat/simulation/src/simulation/README.md).
