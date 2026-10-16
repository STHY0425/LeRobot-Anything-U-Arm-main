# RealMan 机械臂遥操作控制节点

本项目提供 RealMan（瑞尔曼）机械臂的遥操作控制方案，采用**分离式架构**设计，将舵机数据读取、机械臂控制与状态反馈解耦为三个独立节点，提高可维护性和灵活性。

## 架构设计

```
┌──────────────────────┐     /servo_angles      ┌──────────────────┐
│  hxj_servo_reader    │  ───────────────────→  │  servo2realman   │
│  (华馨京主臂舵机读取)  │   Float64MultiArray    │   (控制节点)      │
│  默认 50Hz            │   7维, 相对零点偏移(度)  │   默认 100Hz     │
└──────────────────────┘                        └────────┬─────────┘
                                                         │ JointPos / Gripper_Set
                                                         ↓
                                                  ┌──────────────┐     Arm_Current_State     ┌───────────────┐
                                                  │   rm_driver  │  ─────────────────────→  │  realman_pub  │
                                                  │ RealMan 机械臂│                          │  状态发布 10Hz │
                                                  └──────────────┘                          └──────┬────────┘
                                                                                                 │ /robot_state
                                                                                                 ↓
                                                                                          (数据记录/上位机)
```

数据流：

1. `hxj_servo_reader.py` 通过串口读取华馨京主臂舵机角度，启动时以当前位置为零点，发布相对零点偏移；src/uarm/scripts/Uarm_teleop/HuaXinjing_servo/hxj_servo_reader.py
2. `servo2realman.py` 订阅偏移量，按缩放/方向映射为 RealMan 目标关节角，经限速后发布给 `rm_driver`；src/uarm/scripts/Follower_Arm/realman/servo2realman.py
################单纯运行的话以上文件已经足够！！！！！！！！！！！
3. `realman_pub.py`（可选）订阅驱动状态，整理为统一的 `/robot_state` 供数据记录使用。

### 节点说明

| 节点 | 功能 | 订阅 | 发布 |
|------|------|------|------|
| `hxj_servo_reader.py` | 读取主臂舵机串口数据（fashionstar_uart_sdk） | — | `/servo_angles` |可以订阅该话题数据查看！
| `servo2realman.py` | 主臂偏移 → RealMan 控制命令 | `/servo_angles` | `/rm_driver/JointPos`、`/rm_driver/Gripper_Set`、`/robot_action` |
| `realman_pub.py` | 读取并整理 RealMan 状态 | `/rm_driver/Arm_Current_State` | `/robot_state` |

## 使用方法

### 方式一：单独启动节点

> 使用前需先启动 RealMan 官方驱动 `rm_driver`（如 `roslaunch rm_driver rm_65_driver.launch`），控制节点本身不连接机械臂。
**终端0 - 启动 RealMan 驱动（必须先启动）：**
```bash
下载并编译瑞尔曼相关sdk，然后来到sdk的catkin_ws目录下，
1.source ./devel setup.bash
2.roslaunch rm_driver rm_65_driver.launch
```

**终端1 - 启动主臂舵机读取节点：**（此时主臂状态必须是泄力！！！！笔直即零位状态！！！！！）一旦运行程序，程序会将当前角度作为相对零点传输给瑞尔曼，注意注意注意
```bash
rosrun uarm hxj_servo_reader.py
```
若 rosrun 找不到脚本，先赋予执行权限：
```bash
chmod +x src/uarm/scripts/Uarm_teleop/HuaXinjing_servo/hxj_servo_reader.py
```

**终端2 - 启动 RealMan 控制节点：**
```bash
rosrun uarm servo2realman.py
```

**终端3（可选）- 启动状态发布节点：**
```bash
rosrun uarm realman_pub.py
```

## 参数配置

### hxj_servo_reader.py 参数（节点名 `servo_reader_node`）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | 舵机串口设备 |
| `baudrate` | int | 115200 | 串口波特率 |
| `servo_ids` | list | `[0,1,2,3,4,5,6]` | 按关节顺序排列的舵机 ID 列表 |
| `rate` | int | 50 | 发布频率 (Hz) |
| `timeout` | float | 0 | 串口读取超时 (s) |

注意事项：
- **数组顺序与 `servo_ids` 列表顺序一致**：第 i 位输出 `servo_ids[i]` 号舵机的数据（对应下游关节顺序）。若主臂舵机 ID 不是 0~6（例如 1~7），启动时用 `~servo_ids` 按关节顺序指定实际 ID。
- **零点校准**：节点启动时以各舵机当前位置为零点，之后发布的是相对零点的角度偏移（度）。
- **跳变保护**：偏移跳变超过 90° 视为异常，记录 error 并保留旧值；变化小于 1° 的死区则不更新，避免微小抖动。

### servo2realman.py 参数（节点名 `servo_to_realman`）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dof` | int | 6 | 机械臂自由度 |
| `control_rate` | float | 100.0 | 控制频率 (Hz) |
| `max_joint_speed` | float | 1000.0 | 最大关节速度 (度/秒)。默认值很大，实际不限速；调小可起到保护作用 |
| `init_qpos` | list | `[0,0,0,0,0,0]` | RealMan 初始姿态（零点姿态，度） |
| `joint_scale` | list | `[0.020]*6` | 关节映射缩放系数 |
| `joint_invert` | list | `[-1]*6` | 关节方向反转（1.0 或 -1.0） |

说明：角度映射后直接输出，**不做平滑滤波**；速度限制仅在相邻两次指令间生效（`delta = clamp(delta, ±max_speed×dt)`）。

### realman_pub.py 参数（节点名 `realman_state_publisher`）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dof` | int | 6 | 机械臂自由度 |
| `publish_rate` | float | 10.0 | 状态发布频率 (Hz) |

说明：发布 `[j1..j6, gripper]` 共 7 维；夹爪状态暂固定为 `0`（代码注释：RealMan 夹爪状态需从其他话题获取）。驱动状态超过 2 秒无更新会输出超时告警。

### 坐标映射公式

```
RealMan目标角度 = init_qpos + servo_offset × scale × invert
```

**示例：**
- 舵机偏移 `10°`，缩放系数 `0.020`，方向 `-1.0`
- RealMan 关节移动：`10 × 0.020 × (-1.0) = -0.2°`

### 夹爪映射（代码内固定常量，非 ROS 参数）还没有用到！

| 常量 | 值 | 说明 |
|------|-----|------|
| `gripper_min_deg` | -10.0 | 夹爪舵机最小角度（度） |
| `gripper_max_deg` | 30.0 | 夹爪舵机最大角度（度） |
| `gripper_range` | 1000 | RealMan 夹爪位置指令范围 |

映射：`position = clip((夹爪角度 - (-10)) / 40, 0, 1) × 1000`，取整后发布。

## 话题接口

### 订阅话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/servo_angles` | `std_msgs/Float64MultiArray` | 7 维舵机角度 `[j1,j2,j3,j4,j5,j6,gripper]`，单位度，相对零点偏移 |

### 发布话题

| 话题名 | 类型 | 发布节点 | 说明 |
|--------|------|----------|------|
| `/rm_driver/JointPos` | `rm_msgs/JointPos` | servo2realman | 关节位置命令（`dof>6` 时附带 `expand` 字段） |
| `/rm_driver/Gripper_Set` | `rm_msgs/Gripper_Set` | servo2realman | 夹爪位置命令 (0~1000) |
| `/robot_action` | `std_msgs/Float64MultiArray` | servo2realman | 动作反馈 `[j1..j6, gripper]`，用于数据记录 |
| `/robot_state` | `std_msgs/Float64MultiArray` | realman_pub | 机械臂当前状态 `[j1..j6, gripper]` |

## 调试与监控

### 查看话题数据

```bash
# 查看舵机数据
rostopic echo /servo_angles

# 查看控制命令
rostopic echo /rm_driver/JointPos

# 查看动作反馈
rostopic echo /robot_action

# 查看机械臂状态（需启动 realman_pub）
rostopic echo /robot_state

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

### 修改舵机id与舵机复位相关，使用过程中一定一定要避免舵机堵转！
src/uarm/scripts/Uarm_teleop/HuaXinjing_servo/hxj_config_servo.py

进入终端运行后有清晰使用ui，包括舵机id修改和舵机复位


使用前可以先复位再进行遥操，复位后记得进行‘3’整臂泄力操作
