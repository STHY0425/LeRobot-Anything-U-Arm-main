# RealMan 机械臂遥操作控制节点

本项目提供 RealMan（瑞尔曼）机械臂的遥操作控制方案，采用**分离式架构**设计，将舵机数据读取与机械臂控制解耦，提高代码可维护性和灵活性。

## 架构设计

参考 `xarm` 的简洁架构，将原本单体的复杂节点拆分为两个独立节点：

```
┌─────────────────┐      /servo_angles       ┌──────────────────┐
│  servo_reader   │  ─────────────────────→  │  servo2realman   │
│   (舵机读取)     │    Float64MultiArray     │   (控制节点)      │
│   ~50Hz         │                          │   20-30Hz        │
└─────────────────┘                          └──────────────────┘
                                                      ↓
                                               /rm_driver/JointPos
                                               /rm_driver/Gripper_Set
                                                      ↓
                                               ┌──────────────┐
                                               │   RealMan    │
                                               │    机械臂     │
                                               └──────────────┘
```

### 节点说明

| 节点 | 功能 | 数据来源 | 发布话题 |
|------|------|----------|----------|
| `servo_reader.py` | 读取舵机串口数据 | `/dev/ttyUSB0` | `/servo_angles` |
| `servo2realman.py` | 控制 RealMan 机械臂 | `/servo_angles` | `/rm_driver/JointPos`, `/rm_driver/Gripper_Set` |
| `realman_pub.py` | 读取 RealMan 状态 | `/rm_driver/Arm_Current_State` | `/robot_state` |

## 文件说明

```
realman/
├── servo2realman.py          # 主控制节点（推荐）
├── realman_pub.py            # 状态发布节点
├── realman_teleop.py         # 原始完整版（功能最全）
├── realman_teleop_optimized.py  # 优化过渡版
└── README.md                 # 本文件
```

### 版本选择建议

| 版本 | 适用场景 | 特点 |
|------|----------|------|
| `servo2realman.py` | **推荐日常使用** | 简洁、易维护、延迟低 |
| `realman_teleop.py` | 生产环境/高安全要求 | 功能最全，含急停、零点校准、故障检测 |
| `realman_pub.py` | 需要状态反馈时 | 轻量级状态读取 |

## 使用方法

### 方式一：单独启动节点

**终端1 - 启动舵机读取节点：**
```bash
rosrun uarm servo_reader.py
```

**终端2 - 启动 RealMan 控制节点：**
```bash
rosrun uarm servo2realman.py
```

**终端3（可选）- 启动状态发布节点：**
```bash
rosrun uarm realman_pub.py
```

### 方式二：使用 Launch 文件

创建 `realman_optimized.launch`：

```xml
<launch>
    <!-- 舵机读取节点 -->
    <node pkg="uarm" type="servo_reader.py" name="servo_reader" output="screen">
        <param name="baudrate" value="115200"/>
    </node>
    
    <!-- RealMan 控制节点 -->
    <node pkg="uarm" type="servo2realman.py" name="servo2realman" output="screen">
        <param name="control_rate" value="30.0"/>
        <param name="dof" value="6"/>
        <param name="init_qpos" value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"/>
        <param name="joint_scale" value="[0.015, 0.015, 0.015, 0.015, 0.015, 0.015]"/>
        <param name="joint_invert" value="[1.0, 1.0, 1.0, -1.0, 1.0, 1.0]"/>
    </node>
</launch>
```

启动：
```bash
roslaunch uarm realman_optimized.launch
```

## 参数配置

### servo2realman.py 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dof` | int | 6 | 机械臂自由度 |
| `control_rate` | float | 20.0 | 控制频率 (Hz) |
| `max_joint_speed` | float | 30.0 | 最大关节速度 (度/秒) |
| `init_qpos` | list | [0,0,0,0,0,0] | RealMan 初始姿态 (度) |
| `joint_scale` | list | [0.015]*6 | 关节映射缩放系数 |
| `joint_invert` | list | [1,1,1,-1,1,1] | 关节方向反转 |
| `filter_alpha` | float | 0.3 | 平滑滤波系数 (0-1) |

### 坐标映射公式

```
RealMan目标角度 = init_qpos + servo_offset × scale × invert
```

**示例：**
- 如果舵机偏移 `10°`，缩放系数 `0.015`，方向 `1`
- RealMan 关节移动：`10 × 0.015 = 0.15°`

### 夹爪映射

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `gripper_min_deg` | -10.0 | 夹爪最小角度 |
| `gripper_max_deg` | 30.0 | 夹爪最大角度 |
| `gripper_range` | 1000 | RealMan 夹爪位置范围 |

## 话题接口

### 订阅话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/servo_angles` | `Float64MultiArray` | 7维舵机角度 `[j1,j2,j3,j4,j5,j6,gripper]` |

### 发布话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/rm_driver/JointPos` | `JointPos` | 关节位置命令 |
| `/rm_driver/Gripper_Set` | `Gripper_Set` | 夹爪位置命令 |
| `/robot_action` | `Float64MultiArray` | 动作反馈（用于数据记录） |

## 调试与监控

### 查看话题数据

```bash
# 查看舵机数据
rostopic echo /servo_angles

# 查看控制命令
rostopic echo /rm_driver/JointPos

# 查看动作反馈
rostopic echo /robot_action

# 查看话题频率
rostopic hz /servo_angles
```

### 节点连接图

```bash
rosrun rqt_graph rqt_graph
```

### 检查 RealMan 驱动状态

```bash
# 查看机械臂当前状态
rostopic echo /rm_driver/Arm_Current_State
```

## 与 xarm 方案的对比

| 特性 | RealMan | xArm |
|------|---------|------|
| **通信方式** | ROS 话题 | xArm SDK (TCP/IP) |
| **驱动节点** | rm_driver | 内嵌在控制节点 |
| **状态读取** | 订阅 `/rm_driver/Arm_Current_State` | `get_servo_angle()` |
| **控制命令** | 发布 `JointPos` 话题 | `set_servo_angle()` |
| **架构** | 分离式（需配合 rm_driver） | 一体式 |

## 常见问题

### Q1: 舵机数据没有接收到？

检查：
```bash
# 确认 servo_reader 节点在运行
rosnode list | grep servo

# 确认话题有数据
rostopic echo /servo_angles

# 检查串口权限
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

### Q2: 机械臂不跟随运动？

检查：
1. RealMan 驱动 (`rm_driver`) 是否已启动
2. 控制频率是否匹配 (`rostopic hz /rm_driver/JointPos`)
3. 关节限位是否触发（查看日志）

### Q3: 运动方向相反？

修改 `joint_invert` 参数，将对应关节的值 `1.0` 改为 `-1.0`：
```python
rosparam set /servo2realman/joint_invert "[1.0, 1.0, -1.0, -1.0, 1.0, 1.0]"
```

### Q4: 运动幅度太小/太大？

调整 `joint_scale` 参数：
```python
# 增大运动幅度
rosparam set /servo2realman/joint_scale "[0.03, 0.03, 0.03, 0.03, 0.03, 0.03]"

# 减小运动幅度
rosparam set /servo2realman/joint_scale "[0.01, 0.01, 0.01, 0.01, 0.01, 0.01]"
```

## 系统要求

- ROS Noetic / Melodic
- Python 3
- `rm_msgs` 包（RealMan 驱动消息）
- `pyserial`（舵机串口通信）

## 相关文件

- `../../Uarm_teleop/Zhonglin_servo/servo_reader.py` - 舵机读取节点
- `../xarm/servo2xarm.py` - xArm 控制节点（参考架构）

## 更新日志

| 日期 | 版本 | 说明 |
|------|------|------|
| 2024-XX | v2.0 | 分离式架构，参考 xarm 优化 |
| 2024-XX | v1.0 | 原始完整版 `realman_teleop.py` |
