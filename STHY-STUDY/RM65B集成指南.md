# 🤖 睿尔曼 RM65-B 机械臂集成指南

## 📋 目录
- [项目概述](#项目概述)
- [已完成的工作](#已完成的工作)
- [环境配置](#环境配置)
- [仿真测试](#仿真测试)
- [API控制](#api控制)
- [故障排除](#故障排除)

---

## 🎯 项目概述

本指南帮助你将睿尔曼 RM65-B 机械臂集成到 LeRobot-Anything 项目中，包括：
- ✅ URDF 模型和网格文件
- ✅ SAPIEN 仿真环境配置
- ✅ ROS API 控制接口
- ✅ 测试脚本和示例

---

## ✅ 已完成的工作

### 1. 下载官方资源

已从睿尔曼官方 GitHub 仓库下载：
```bash
git clone https://github.com/RealManRobot/rm_robot.git
```

**包含内容：**
- ✅ RM65 URDF 文件: `/tmp/rm_robot/rm_description/urdf/RM65/rm_65.urdf`
- ✅ 3D 网格文件: `/tmp/rm_robot/rm_description/meshes/RM65/`
- ✅ ROS 驱动: `/tmp/rm_robot/rm_driver/`
- ✅ 示例代码: `/tmp/rm_robot/rm_demo/`

### 2. 创建项目结构

```
LeRobot-Anything-U-Arm/
├── src/simulation/mani_skill/
│   ├── assets/robots/rm65b/          # URDF 和网格文件
│   │   ├── rm_65.urdf                # 官方 URDF
│   │   ├── rm_65.urdf.xacro          # Xacro 版本
│   │   ├── meshes/RM65/              # 3D 网格文件
│   │   └── rm_65_description.csv     # 参数说明
│   └── agents/robots/rm65b/          # 机器人定义
│       ├── __init__.py
│       └── rm65b.py                  # RM65B 类定义
└── src/uarm/scripts/Follower_Arm/Realman/
    └── rm65b_controller.py           # API 控制器
```

### 3. 机器人定义

创建了 `RM65B` 类，包含：
- ✅ 6个关节的完整定义
- ✅ 多种控制模式（位置、速度、末端位姿）
- ✅ 预定义姿态（rest, home, ready）
- ✅ PD 控制器配置

### 4. API 控制接口

创建了 `RM65BController` 类，支持：
- ✅ 关节空间运动
- ✅ 笛卡尔空间运动（需配置服务）
- ✅ 状态查询
- ✅ 预定义姿态

---

## 🔧 环境配置

### 步骤 1: 安装依赖

```bash
# 1. 安装 ROS Noetic (如果还没有)
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 3. 克隆睿尔曼功能包
git clone https://github.com/RealManRobot/rm_robot.git

# 4. 编译
cd ~/catkin_ws
catkin_make

# 5. 添加到环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 步骤 2: 配置网络（如果有硬件）

如果你有实体机械臂：

```bash
# 1. 设置静态 IP
# 机械臂默认 IP: 192.168.1.18
# 将你的电脑 IP 设置为: 192.168.1.x (x != 18)

# 2. 测试连接
ping 192.168.1.18

# 3. 启动驱动
roslaunch rm_driver rm_65_driver.launch
```

### 步骤 3: 修复 URDF 路径

官方 URDF 使用 ROS package 路径，需要修改为绝对路径：

```bash
# 运行修复脚本
cd ~/2026robotic/LeRobot-Anything-U-Arm-main
python3 scripts/fix_rm65b_urdf.py
```

或手动修改 `src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf`：

将所有：
```xml
<mesh filename="package://rm_description/meshes/RM65/xxx.STL" />
```

替换为：
```xml
<mesh filename="meshes/RM65/xxx.STL" />
```

---

## 🎮 仿真测试

### 测试 0: 验证注册（推荐先运行）

```bash
# 验证 RM65-B 是否正确注册
python3 scripts/test_rm65b_registration.py
```

**预期输出：**
```
✅ rm65b 已注册
✅ 环境创建成功
✅ 机器人类型: RM65B
✅ 所有测试通过！
```

### 测试 1: 静态显示

```bash
cd src/simulation
python3 static_robot_viewer.py --robot rm65b
```

**预期结果：**
- 打开 SAPIEN 窗口
- 显示 RM65-B 机械臂模型
- 机器人处于 rest 姿态

### 测试 2: 不同姿态

```bash
# Home 姿态
python3 static_robot_viewer.py --robot rm65b --pose home

# Ready 姿态
python3 static_robot_viewer.py --robot rm65b --pose ready
```

### 测试 3: 遥操作仿真

```bash
# 如果你有 U-Arm 硬件
python3 teleop_sim.py --robot rm65b
```

---

## 🔌 API 控制

### 方法 1: 使用官方 API（推荐）

```python
#!/usr/bin/env python3
import rospy
from rm_msgs.srv import MoveJ
from rm_msgs.msg import JointPos

# 初始化节点
rospy.init_node('rm65b_test')

# 发布关节命令
pub = rospy.Publisher('/rm_driver/joint_command', JointPos, queue_size=10)
rospy.sleep(1.0)

# 移动到目标位置
msg = JointPos()
msg.joint = [0, -45, 90, 0, 45, 0]  # 角度
msg.speed = 20
pub.publish(msg)

rospy.spin()
```

### 方法 2: 使用封装的控制器

```python
from Realman.rm65b_controller import RM65BController
import numpy as np

# 创建控制器
controller = RM65BController(robot_ip="192.168.1.18")

# 移动到 home 姿态
controller.home()

# 自定义关节运动
target_joints = np.array([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])  # 弧度
controller.move_joint(target_joints, speed=30)

# 获取当前状态
current_pos = controller.get_joint_positions()
print("当前关节位置:", np.rad2deg(current_pos))
```

### 方法 3: ROS 话题订阅（用于遥操作）

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from rm_msgs.msg import JointPos

def servo_angles_callback(msg):
    """接收 U-Arm 的关节角度"""
    # msg.position 包含 U-Arm 的关节角度
    
    # 映射到 RM65-B（根据你的配置调整）
    rm_joints = JointPos()
    rm_joints.joint = [
        msg.position[0],  # joint1
        msg.position[1],  # joint2
        msg.position[2],  # joint3
        msg.position[3],  # joint4
        msg.position[4],  # joint5
        msg.position[5],  # joint6
    ]
    rm_joints.speed = 50
    
    # 发送到机械臂
    pub.publish(rm_joints)

rospy.init_node('uarm_to_rm65b')

# 订阅 U-Arm 话题
sub = rospy.Subscriber('/servo_angles', JointState, servo_angles_callback)

# 发布到 RM65-B
pub = rospy.Publisher('/rm_driver/joint_command', JointPos, queue_size=10)

rospy.spin()
```

---

## 📚 官方 API 参考

### 关节运动

```python
# 服务调用方式
from rm_msgs.srv import MoveJ

rospy.wait_for_service('/rm_driver/movej')
movej = rospy.ServiceProxy('/rm_driver/movej', MoveJ)

# 调用服务
response = movej(
    joint=[0, -45, 90, 0, 45, 0],  # 角度
    speed=20,
    trajectory_connect=0
)
```

### 笛卡尔运动

```python
from rm_msgs.srv import MoveL

rospy.wait_for_service('/rm_driver/movel')
movel = rospy.ServiceProxy('/rm_driver/movel', MoveL)

# 调用服务
response = movel(
    pose=[0.3, 0.0, 0.4, 3.14, 0, 0],  # [x, y, z, rx, ry, rz]
    speed=20,
    trajectory_connect=0
)
```

### 获取状态

```python
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    print("关节位置:", msg.position)
    print("关节速度:", msg.velocity)

rospy.Subscriber('/rm_driver/joint_states', JointState, joint_state_callback)
```

---

## 🐛 故障排除

### 问题 1: URDF 加载失败

**错误信息：**
```
Failed to load URDF: package://rm_description/meshes/RM65/xxx.STL
```

**解决方法：**
```bash
# 修改 URDF 中的网格路径
cd src/simulation/mani_skill/assets/robots/rm65b
sed -i 's|package://rm_description/meshes/RM65/|meshes/RM65/|g' rm_65.urdf
```

### 问题 2: 机器人不显示

**检查步骤：**
```bash
# 1. 确认文件存在
ls src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf
ls src/simulation/mani_skill/assets/robots/rm65b/meshes/RM65/

# 2. 检查注册
python3 -c "
from mani_skill.agents.registration import REGISTERED_AGENTS
print('rm65b' in REGISTERED_AGENTS)
"

# 3. 查看错误日志
python3 static_robot_viewer.py --robot rm65b 2>&1 | grep -i error
```

### 问题 3: 无法连接实体机械臂

**检查步骤：**
```bash
# 1. 测试网络
ping 192.168.1.18

# 2. 检查 ROS 驱动
roslaunch rm_driver rm_65_driver.launch

# 3. 查看话题
rostopic list | grep rm_driver

# 4. 测试发送命令
rostopic pub /rm_driver/joint_command rm_msgs/JointPos "{joint: [0,0,0,0,0,0], speed: 10}"
```

### 问题 4: 关节限位错误

**RM65-B 关节限位（弧度）：**
```python
joint_limits = {
    'joint1': (-3.14, 3.14),    # ±180°
    'joint2': (-2.09, 2.09),    # ±120°
    'joint3': (-2.09, 2.09),    # ±120°
    'joint4': (-3.14, 3.14),    # ±180°
    'joint5': (-2.09, 2.09),    # ±120°
    'joint6': (-3.14, 3.14),    # ±180°
}
```

---

## 📖 参考资源

### 官方文档
- [睿尔曼官网](https://www.realman-robotics.com/)
- [GitHub 仓库](https://github.com/RealManRobot/rm_robot)
- [ROS Wiki](http://wiki.ros.org/rm_robot)

### 技术规格 (RM65-B)
- **自由度**: 6轴
- **负载**: 5kg
- **工作半径**: 650mm
- **重复定位精度**: ±0.02mm
- **关节速度**: 180°/s
- **通信**: TCP/IP (默认 192.168.1.18:8080)

### 相关文件
- URDF: `src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf`
- 机器人定义: `src/simulation/mani_skill/agents/robots/rm65b/rm65b.py`
- 控制器: `src/uarm/scripts/Follower_Arm/Realman/rm65b_controller.py`

---

## 🚀 下一步

1. **测试仿真**
   ```bash
   cd src/simulation
   python3 static_robot_viewer.py --robot rm65b
   ```

2. **配置硬件**（如果有实体机械臂）
   ```bash
   # 安装 rm_robot 功能包
   cd ~/catkin_ws/src
   git clone https://github.com/RealManRobot/rm_robot.git
   cd ~/catkin_ws && catkin_make
   
   # 启动驱动
   roslaunch rm_driver rm_65_driver.launch
   ```

3. **集成遥操作**
   - 参考 `src/uarm/scripts/Follower_Arm/Dobot/servo2Dobot.py`
   - 创建 `servo2Realman.py` 订阅 `/servo_angles` 话题
   - 映射关节角度并发送到 RM65-B

4. **数据采集**
   - 使用 LeRobot 框架采集演示数据
   - 训练策略模型
   - 部署到实体机械臂

---

## ✅ 完成检查清单

- [ ] 已下载官方 rm_robot 功能包
- [ ] URDF 和网格文件已复制到项目
- [ ] 机器人定义已创建并注册
- [ ] 仿真测试通过（static_robot_viewer）
- [ ] ROS 环境配置完成
- [ ] 实体机械臂网络连接成功（如果有硬件）
- [ ] API 控制测试通过
- [ ] 遥操作集成完成

---

**祝你成功集成 RM65-B 机械臂！🎉**

如有问题，请参考：
- 项目文档: `新机械臂快速添加指南.md`
- 官方示例: `/tmp/rm_robot/rm_demo/`
- 社区支持: [睿尔曼技术支持](https://www.realman-robotics.com/support)
