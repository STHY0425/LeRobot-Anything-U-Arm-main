# 瑞尔曼机械臂 - 完整操作指南

> **快速开始**：按顺序执行 [软件启动流程](#软件启动流程) 的6个步骤即可运行。

## 目录
1. [硬件连接](#硬件连接)
2. [软件启动流程](#软件启动流程)
3. [数据映射说明](#数据映射说明)
4. [舵机控制](#舵机控制)
5. [故障排查](#故障排查)

---
roslaunch rm_bringup rm_65_robot.launch

## 硬件连接

### 连接示意图

```
┌─────────────────┐      USB      ┌─────────────────┐     网线/WiFi    ┌─────────────────┐
│   电脑/Ubuntu   │  ───────────→  │  舵机主臂控制板  │                │                 │
│   (ROS环境)     │   /dev/ttyUSB0 │  (串口通信)     │                │  瑞尔曼机械臂    │
│                 │                │                 │                │  RM-65          │
│                 │  ───────────────────────────────→│                │  (IP:192.168.   │
│                 │        网线或WiFi连接            │                │      1.18)      │
└─────────────────┘                               └─────────────────┘
```

### 1. 瑞尔曼机械臂连接

#### 1.1 物理连接
1. **电源连接**：将机械臂电源适配器连接到机械臂底座
2. **网络连接**：
   - **有线**：网线连接机械臂和电脑
   - **无线**：机械臂连接WiFi，确保和电脑在同一网络

#### 1.2 网络配置
机械臂默认IP地址通常是 `192.168.1.18`，确保电脑能ping通：

```bash
# 检查网络连接
ping 192.168.1.18

# 如果不通，检查网络配置
# 有线连接时，可能需要设置电脑IP为同一网段
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0
```

#### 1.3 验证机械臂状态
机械臂上电后，听到"滴"声表示启动完成。观察指示灯：
- **绿色常亮**：正常待机
- **红色**：急停状态或错误
- **闪烁**：通信中

### 2. 舵机主臂连接

#### 2.1 串口连接
1. 将舵机控制板通过USB线连接到电脑
2. 检查串口设备：

```bash
# 查看串口设备
ls -la /dev/ttyUSB*

# 输出示例：/dev/ttyUSB0

# 如果没有权限，添加用户到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录或重启
```

#### 2.2 验证串口
```bash
# 安装串口工具
sudo apt-get install picocom

# 测试连接（按Ctrl+A然后X退出）
picocom -b 115200 /dev/ttyUSB0
```

---

## 软件启动流程

### 完整启动顺序

```
步骤1: 启动roscore
    ↓
步骤2: 启动瑞尔曼官方驱动 (rm_driver)
    ↓
步骤3: 启动舵机读取节点 (servo_reader.py)
    ↓
步骤4: 启动遥操节点 (realman_teleop_optimized.py)
    ↓
步骤5: 调整主臂姿态 + 释放力矩
    ↓
步骤6: 使能遥操作
```

### 步骤1: 启动ROS核心

```bash
# 打开终端1
roscore
```

### 步骤2: 启动瑞尔曼官方驱动

```bash
# 打开终端2
# 方法1：使用launch文件（推荐）
roslaunch rm_driver rm_65.launch

# 方法2：如果需要指定IP
roslaunch rm_driver rm_65.launch robot_ip:=192.168.1.18
```

**验证驱动启动成功**：
```bash
# 打开终端，检查话题
rostopic list | grep rm_driver

# 应该看到以下话题：
# /rm_driver/Arm_Current_State
# /rm_driver/JointPos
# /rm_driver/Gripper_Set
# /rm_driver/Stop

# 检查机械臂状态
rostopic echo /rm_driver/Arm_Current_State
# 按Ctrl+C退出
```

### 步骤3: 启动舵机读取节点

```bash
# 打开终端3
cd /home/sthy/2026robotic/LeRobot-Anything-U-Arm-main

# 启动舵机读取节点
rosrun uarm servo_reader.py

# 或者带参数启动
rosrun uarm servo_reader.py _baudrate:=115200
```

**验证舵机读取正常**：
```bash
# 打开终端，检查话题
rrostopic echo /servo_angles

# 应该看到类似输出（7个数值）：
# data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 移动主臂，数值会变化
```

### 步骤4: 启动遥操节点

```bash
# 打开终端4
cd /home/sthy/2026robotic/LeRobot-Anything-U-Arm-main

# 基本启动（自动释放舵机力矩）
rosrun uarm realman_teleop_optimized.py

# 或带参数启动
rosrun uarm realman_teleop_optimized.py \
    _init_qpos:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    _joint_invert:="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
```

**启动日志示例**：
```
[RealManOpt] 启动优化遥操节点...
[Servo] 串口已打开: /dev/ttyUSB0
[Servo] 正在释放舵机力矩...
[Servo] ✅ 舵机力矩已释放，可手动调整主臂姿态
[RealManOpt] 机械臂型号: RM_65, DOF: 6
[RealManOpt] 零点姿态: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[RealManOpt] 等待使能信号...
```

### 步骤5: 调整主臂姿态

1. **手动调整主臂**：由于力矩已释放，可以直接用手移动主臂到期望的零点姿态
2. **观察仿真（如果有）**：如果连接了仿真，可以看到主臂姿态
3. **调整完成后**：准备使能

### 步骤6: 使能遥操作

```bash
# 打开终端5

# 使能遥操作（主从臂开始同步）
rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"

# 验证使能成功
rrostopic echo /realman_teleop/status
# 应该显示：data: True
```

**此时**：
- 移动主臂，从臂应该跟随移动
- 注意从臂速度限制（默认30°/s）

---

## 运行时控制

### 常用命令

```bash
# 1. 查看当前状态
rrostopic echo /realman_teleop/status

# 2. 查看发送给机械臂的角度
rostopic echo /rm_driver/JointPos

# 3. 释放舵机力矩（重新调整主臂）
rostopic pub /realman_teleop/release_servos std_msgs/Bool "data: true"

# 4. 失能遥操作
rostopic pub /realman_teleop/enable std_msgs/Bool "data: false"

# 5. 触发急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "data: true"
```

### 调整映射参数

如果主从臂方向相反或比例不对：

```bash
# 修改关节方向（例如第3个关节反向）
rosrun uarm realman_teleop_optimized.py _joint_invert:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]"

# 修改关节缩放
rosrun uarm realman_teleop_optimized.py _joint_scale:="[1.0, 1.0, 0.8, 1.0, 1.0, 1.0]"
```

---

## 数据映射说明

### 架构图

```
┌─────────────────┐      串口       ┌─────────────────┐     ROS话题      ┌─────────────────┐
│   舵机主臂      │  ───────────→  │  servo_reader   │  ───────────→  │ realman_teleop  │
│  (中位135°)     │   读取角度      │   (Python)      │  /servo_angles │  (本节点)       │
└─────────────────┘                └─────────────────┘                └─────────────────┘
                                                                            ↓
                                                                     ┌──────────────┐
                                                                     │  rm_driver   │
                                                                     │ (官方驱动)   │
                                                                     └──────────────┘
                                                                            ↓
                                                                     ┌──────────────┐
                                                                     │  瑞尔曼机械臂 │
                                                                     └──────────────┘
```

### 数据格式说明

### `/servo_angles` 话题数据

**来源**：`src/uarm/scripts/Uarm_teleop/Zhonglin_servo/servo_reader.py`

**计算方式**：
```python
# servo_reader.py 第60行
new_angle = angle - self.zero_angles[i]
```

| 变量 | 说明 | 示例值 |
|------|------|--------|
| `angle` | 舵机当前绝对角度 | 135° (中位) ~ 270°/0° (极限) |
| `zero_angles[i]` | 零点标定时的角度 | 约135° (舵机中位) |
| `new_angle` | 发布的偏移量 | **0°** (直立时) |

**数据单位**：度数（不是弧度）

## 映射公式

```
瑞尔曼目标角度 = init_qpos_deg[i] + servo_angles[i] × joint_scale[i] × joint_invert[i]
```

### 参数说明

| 参数 | 作用 | 默认值 | 说明 |
|------|------|--------|------|
| `init_qpos_deg` | 瑞尔曼零点姿态 | `[0,0,0,0,0,0]` | 与舵机零点对应的姿态 |
| `joint_scale` | 关节缩放 | `[1,1,1,1,1,1]` | 主从关节比例差异 |
| `joint_invert` | 关节方向 | `[1,1,1,1,1,1]` | 关节旋转方向反转 |

### 常用配置方案

#### 方案1：直立姿态对应（推荐）

**场景**：主臂直立 = 从臂直立

```bash
rosrun uarm realman_teleop_optimized.py \
    _init_qpos:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    _joint_scale:="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]" \
    _joint_invert:="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
```

**效果**：
- 舵机135°（中位/直立）→ 瑞尔曼 [0,0,0,0,0,0]（直立）
- 舵机偏移 +10° → 瑞尔曼对应关节 +10°

#### 方案2：标准初始姿态

**场景**：使用瑞尔曼标准初始姿态

```bash
rosrun uarm realman_teleop_optimized.py \
    _init_qpos:="[0.0, -20.0, -90.0, 0.0, 90.0, 0.0]"
```

**效果**：
- 舵机135°（中位）→ 瑞尔曼 [0,-20,-90,0,90,0]（工作姿态）
- 适合特定工作场景

#### 方案3：带方向反转

**场景**：某个关节需要反向

```bash
rosrun uarm realman_teleop_optimized.py \
    _joint_invert:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]"
```

**效果**：
- 关节3（索引2）方向反转
- 舵机 +10° → 瑞尔曼关节3 -10°

## 调试方法

### 1. 查看当前数据

```bash
# 终端1：查看舵机偏移量
rostopic echo /servo_angles

# 终端2：查看瑞尔曼目标角度
rostopic echo /rm_driver/JointPos
```

### 2. 零点对齐测试

```bash
# 1. 确保主臂处于零点姿态（舵机135°，直立）
# 2. 启动节点（使用直立零点）
rosrun uarm realman_teleop_optimized.py _init_qpos:="[0,0,0,0,0,0]"

# 3. 使能并观察从臂姿态
rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"

# 4. 如果从臂不是直立，调整 init_qpos 或 joint_invert
```

### 3. 单关节测试

```bash
# 单独转动主臂的第1个关节，观察从臂
# 如果方向相反，修改 joint_invert[0] 为 -1
# 如果比例不对，调整 joint_scale[0]
```

## 舵机力矩控制

### 自动释放力矩（启动时）

默认情况下，节点启动时会**自动释放**舵机力矩，让用户可以手动调整主臂姿态：

```bash
# 启动节点（默认自动释放力矩）
rosrun uarm realman_teleop_optimized.py

# 或显式指定
rosrun uarm realman_teleop_optimized.py _release_torque_on_init:=true
```

### 运行时释放力矩

在节点运行过程中，可以随时释放舵机力矩：

```bash
# 释放力矩
rostopic pub /realman_teleop/release_servos std_msgs/Bool "data: true"
```

### 禁用舵机控制

如果你不想让这个节点控制舵机（例如由 servo_reader.py 管理）：

```bash
rosrun uarm realman_teleop_optimized.py _enable_servo_control:=false
```

### 修改串口参数

```bash
rosrun uarm realman_teleop_optimized.py \
    _servo_serial_port:="/dev/ttyUSB1" \
    _servo_baudrate:=115200
```

## 启动流程示例

### 启动阶段

```bash
# 1. 启动瑞尔曼官方驱动
roslaunch rm_driver rm_65.launch

# 2. 启动 servo_reader.py（读取舵机角度）
rosrun uarm servo_reader.py

# 3. 启动优化遥操节点（自动释放舵机力矩）
rosrun uarm realman_teleop_optimized.py

# 4. 调整主臂姿态后，使能从臂跟随
rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"
```

### 退出阶段（重要！）

```bash
# 5. 先将主臂调整到安全/直立姿态
#    （从臂会跟随到对应姿态）

# 6. 失能遥操作
rostopic pub /realman_teleop/enable std_msgs/Bool "data: false"

# 7. 等待机械臂稳定后，按 Ctrl+C 退出节点
#    （或关闭终端）

# 8. （可选）使用官方工具将机械臂回到零点
#    或使用示教器控制
```

## 注意事项

1. **单位统一**：`/servo_angles` 发布的是度数，瑞尔曼也使用度数
2. **零点定义**：舵机135°是中位（直立），不是0°
3. **安全限位**：代码中有 [-178, 178] 等关节限位保护
4. **速度限制**：默认30°/s，可通过 `max_joint_speed` 调整
5. **串口冲突**：如果 servo_reader.py 和本节点同时控制舵机，可能产生冲突
6. **⚠️ 退出警告**：节点退出时机械臂会**停在当前位置**，不会自动回到零点。务必先调整到安全姿态再退出！

---

## 故障排查

### 瑞尔曼驱动启动问题

#### 问题1：`roslaunch rm_driver rm_65.launch` 失败

**现象**：
```
[ERROR] 无法连接到机械臂
```

**排查步骤**：
```bash
# 1. 检查网络连接
ping 192.168.1.18

# 2. 如果ping不通，检查IP配置
# 查看机械臂实际IP（通常在示教器或配置软件中查看）
# 修改launch文件中的IP参数
roslaunch rm_driver rm_65.launch robot_ip:=192.168.1.XX

# 3. 检查防火墙
sudo ufw disable  # 临时关闭防火墙测试

# 4. 检查网线连接
# 有线连接时，设置电脑为静态IP
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0
```

#### 问题2：驱动启动但无话题输出

**排查**：
```bash
# 检查roscore是否运行
rosnode list

# 检查话题
rostopic list | grep rm_driver

# 查看驱动节点日志
roslaunch rm_driver rm_65.launch --screen
```

### 舵机读取问题

#### 问题3：`servo_reader.py` 无法打开串口

**现象**：
```
[ERROR] 无法打开串口 /dev/ttyUSB0
```

**排查步骤**：
```bash
# 1. 检查设备是否存在
ls -la /dev/ttyUSB*

# 2. 如果没有设备，检查USB连接
# 更换USB线或USB口

# 3. 如果有设备但无权限
sudo chmod 666 /dev/ttyUSB0
# 或永久解决
sudo usermod -a -G dialout $USER
# 然后重新登录

# 4. 检查串口被占用
lsof /dev/ttyUSB0
# 结束占用进程
```

#### 问题4：舵机数据不更新

**排查**：
```bash
# 检查话题输出
rostopic hz /servo_angles

# 如果频率为0，检查
# 1. 舵机电源是否连接
# 2. 串口线是否松动
# 3. 波特率是否匹配（默认115200）
```

### 遥操作问题

#### 问题5：使能后从臂不动

**排查步骤**：
```bash
# 1. 检查是否收到/servo_angles
rrostopic echo /servo_angles

# 2. 检查realman_teleop是否发布控制命令
rostopic echo /rm_driver/JointPos

# 3. 检查状态
rostopic echo /realman_teleop/status

# 4. 检查瑞尔曼错误码
rostopic echo /rm_driver/Arm_Current_State
# 查看err字段是否有错误
```

#### 问题6：从臂运动方向相反

**解决**：
```bash
# 停止节点，修改方向参数
rosrun uarm realman_teleop_optimized.py \
    _joint_invert:="[1.0, 1.0, -1.0, 1.0, 1.0, 1.0]"
# 逐一测试，找到反向的关节
```

#### 问题7：运动抖动或延迟大

**解决**：
```bash
# 增大滤波系数（更平滑）
_filter_alpha:=0.1

# 或降低速度限制
_max_joint_speed:=20.0

# 检查网络延迟
ping 192.168.1.18
# 如果延迟高，使用有线连接
```

### 常见问题速查表

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 从臂姿态与主臂相反 | 方向定义不同 | 修改 `joint_invert` 对应关节为 -1 |
| 从臂移动幅度过大 | 比例不匹配 | 减小 `joint_scale` 对应值 |
| 从臂初始姿态不对 | `init_qpos` 不匹配 | 调整 `init_qpos` 参数 |
| 某个关节不响应 | 超出限位 | 检查舵机角度是否在限位范围内 |
| 舵机无法释放力矩 | 串口被占用 | 检查是否有其他程序占用/dev/ttyUSB0 |
| 从臂抖动 | 滤波不够 | 减小 `filter_alpha` |
| 从臂响应慢 | 速度限制 | 增大 `max_joint_speed` |
| 数据丢失 | 网络不稳定 | 使用有线连接，增大`packet_loss_tolerance` |

### 紧急处理

**机械臂失控时**：
```bash
# 方法1：发布急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "data: true"

# 方法2：直接停止驱动
rosnode kill /rm_driver

# 方法3：物理急停按钮
# 按下机械臂上的急停按钮
```

### 获取帮助

查看详细日志：
```bash
# 所有ROS日志
rosconsole echo

# 特定节点日志
rosrun rqt_console rqt_console

# 系统日志
tail -f ~/.ros/log/latest/*.log
```
---

## 附录：快速参考卡

### 启动命令速查

| 步骤 | 终端 | 命令 |
|------|------|------|
| 1 | 终端1 | `roscore` |
| 2 | 终端2 | `roslaunch rm_driver rm_65.launch` |
| 3 | 终端3 | `rosrun uarm servo_reader.py` |
| 4 | 终端4 | `rosrun uarm realman_teleop_optimized.py` |
| 5 | 终端5 | 手动调整主臂姿态 |
| 6 | 终端5 | `rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"` |

### 常用话题速查

| 话题名 | 用途 | 查看命令 |
|--------|------|----------|
| `/servo_angles` | 舵机角度偏移 | `rrostopic echo /servo_angles` |
| `/rm_driver/JointPos` | 发送给机械臂的角度 | `rostopic echo /rm_driver/JointPos` |
| `/rm_driver/Arm_Current_State` | 机械臂状态反馈 | `rostopic echo /rm_driver/Arm_Current_State` |
| `/realman_teleop/status` | 遥操作状态 | `rrostopic echo /realman_teleop/status` |

### 参数配置速查

| 参数 | 默认值 | 说明 | 示例 |
|------|--------|------|------|
| `init_qpos` | `[0,0,0,0,0,0]` | 零点姿态 | `_init_qpos:="[0,-20,-90,0,90,0]"` |
| `joint_scale` | `[1,1,1,1,1,1]` | 关节缩放 | `_joint_scale:="[1,1,0.8,1,1,1]"` |
| `joint_invert` | `[1,1,1,1,1,1]` | 关节方向 | `_joint_invert:="[1,1,-1,1,1,1]"` |
| `filter_alpha` | `0.3` | 滤波系数 | `_filter_alpha:=0.1` |
| `max_joint_speed` | `30.0` | 速度限制 | `_max_joint_speed:=20.0` |
| `servo_serial_port` | `/dev/ttyUSB0` | 舵机串口 | `_servo_serial_port:="/dev/ttyUSB1"` |

### 节点退出后的状态

**当按下 Ctrl+C 退出遥操节点时**：

| 组件 | 状态 | 说明 |
|------|------|------|
| 遥操节点 | 停止运行 | 不再发布 `/rm_driver/JointPos` 话题 |
| 瑞尔曼机械臂 | **保持最后位置** | 由于不再收到新命令，保持在退出时的姿态 |
| 官方驱动 | 继续运行 | `rm_driver` 节点不受影响 |
| 舵机主臂 | 串口关闭 | 无法再通过本节点控制舵机 |

**⚠️ 注意**：
- 退出时机械臂会**突然停在当前位置**，不会自动回到零点
- 如果需要让机械臂回到安全姿态，**先手动控制回到零点再退出**
- 或先失能遥操作（`enable:=false`），再用其他方式控制机械臂

**安全退出流程**：
```bash
# 1. 先将主臂调整到安全姿态（手动调整）

# 2. 失能遥操作（让机械臂跟随到安全位置）
rostopic pub /realman_teleop/enable std_msgs/Bool "data: false"

# 3. 等待机械臂稳定后，再按 Ctrl+C 退出节点
```

### 控制命令速查

```bash
# 使能遥操作
rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"

# 失能遥操作
rostopic pub /realman_teleop/enable std_msgs/Bool "data: false"

# 释放舵机力矩
rostopic pub /realman_teleop/release_servos std_msgs/Bool "data: true"

# 急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "data: true"

# 复位急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "data: false"
rostopic pub /realman_teleop/enable std_msgs/Bool "data: false"
rostopic pub /realman_teleop/enable std_msgs/Bool "data: true"
```

### 网络配置速查

```bash
# 检查机械臂连接
ping 192.168.1.18

# 有线连接时设置电脑IP
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0

# 检查ROS网络配置
echo $ROS_MASTER_URI  # 应该是 http://localhost:11311
echo $ROS_IP          # 应该是本机IP
```

### 串口配置速查

```bash
# 查看串口设备
ls -la /dev/ttyUSB*

# 添加权限
sudo chmod 666 /dev/ttyUSB0

# 永久添加用户到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

---

*文档版本：1.0 | 最后更新：2024年*