# 华馨京舵机版 FeelForce 执行逻辑说明

本文档说明 `feel_force_duoji` 工程从 ROS 入口脚本启动，到读取舵机、发布 ROS 话题、执行重力补偿和下发阻尼命令的完整执行链路。

## 1. 工程定位

本工程是原 FeelForce 模块化机械臂思想的华馨京 / FashionStar 舵机适配版。

原 FeelForce 面向 CAN + MIT 电机协议，通常需要拆分：

```text
Communication -> ProtocolDecoder -> Motor -> Arm -> ControlManager / Publisher -> ArmNode
```

华馨京舵机官方 SDK 已经封装了串口通信和协议帧，因此本工程简化为：

```text
FashionStarServoBus -> HuaXinjingArm -> GravityCompensator / Publisher -> Node
```

核心原则是：

- `FashionStarServoBus` 负责直接调用官方 SDK。
- `HuaXinjingArm` 是机械臂唯一语义入口，负责状态快照、零点和角度映射。
- `GravityCompensator` 负责控制策略，不关心串口细节。
- `Publisher` 只消费状态快照，不直接读硬件。
- `node.py` 是唯一装配点，负责创建对象和控制生命周期。

## 2. 文件职责总览

```text
feel_force_duoji/
  hxj_servo_reader.py              # 只读舵机状态的 ROS 入口
  hxj_gravity_compensation.py      # 重力补偿控制的 ROS 入口
  README.md                        # 使用说明
  ARCHITECTURE_HXJ.md              # 模块化架构说明
  EXECUTION_LOGIC.md               # 本文档
  hxj_modular/
    __init__.py                    # 包导出
    state_types.py                 # ServoState / ArmState 数据结构
    config.py                      # HuaXinjingConfig 配置对象
    ros_params.py                  # ROS 参数读取
    servo_bus.py                   # 官方 SDK 调用封装
    arm.py                         # 机械臂状态聚合与关节映射
    dynamics.py                    # Pinocchio 重力力矩与阻尼映射
    gravity_compensation.py        # 重力补偿控制策略
    publisher.py                   # ROS 话题发布
    node.py                        # 节点装配与主循环
```

## 3. 两个 ROS 入口

工程有两个可运行入口。

### 3.1 `hxj_servo_reader.py`

用途：只读取舵机角度和可选电流，然后发布 ROS 话题。

启动命令示例：

```bash
rosrun uarm hxj_servo_reader.py
```

执行入口：

```python
rospy.init_node("hxj_servo_reader")
node = ServoReaderNode.from_ros()
node.run()
```

退出时：

```python
node.close()
```

### 3.2 `hxj_gravity_compensation.py`

用途：读取舵机角度，计算重力补偿或自适应阻尼，然后通过 `set_damping()` 下发阻尼功率。

启动命令示例：

```bash
rosrun uarm hxj_gravity_compensation.py
```

执行入口：

```python
rospy.init_node("hxj_gravity_compensation")
node = GravityCompensationNode.from_ros()
node.run()
```

退出时：

```python
node.close()
```

`close()` 会根据 `~release_on_shutdown` 决定释放舵机，还是恢复基础阻尼。

## 4. 只读发布节点执行逻辑

只读节点对应 `ServoReaderNode`。

### 4.1 创建阶段

调用链：

```text
hxj_servo_reader.py
  -> ServoReaderNode.from_ros()
    -> load_arm_config_from_ros()
    -> ServoReaderNode(config)
      -> HuaXinjingArm(config)
      -> ServoStatePublisher(arm)
      -> rospy.Rate(config.rate_hz)
```

对象关系：

```text
ServoReaderNode
  ├── HuaXinjingConfig
  ├── HuaXinjingArm
  │     └── FashionStarServoBus
  └── ServoStatePublisher
```

### 4.2 连接阶段

`ServoReaderNode.run()` 首先调用：

```python
self.connect()
```

内部执行：

```text
ServoReaderNode.connect()
  -> arm.connect()
    -> bus.connect()
      -> serial.Serial(...)
      -> UartServoManager(...)
    -> calibrate_zero()  # 如果 zero_on_start=True 且 publish_absolute=False
```

`bus.connect()` 会延迟导入：

```python
import serial
from fashionstar_uart_sdk import UartServoManager
```

这样无硬件 SDK 环境仍然可以导入算法模块和做 mock 测试，真正连接硬件时才需要 SDK。

### 4.3 循环阶段

主循环：

```python
while not rospy.is_shutdown():
    state = self.arm.read_state()
    self.publisher.publish_all(state)
    self.rate.sleep()
```

每个周期只读取一次硬件状态。

具体流程：

```text
arm.read_state()
  -> bus.read_all_states(config.servo_ids)
    -> bus.read_state(id)
      -> query_servo_angle(id)
      -> query_current(id)  # 如果 read_current=True
  -> 生成 ArmState
  -> 更新 arm.latest_state
  -> 更新上一帧有效缓存

publisher.publish_all(state)
  -> publish_angles(state)
    -> arm.angle_message_data(state)
    -> 发布 Float64MultiArray 到 /servo_angles
  -> publish_currents(state)  # 如果 read_current=True
    -> arm.current_message_data(state)
    -> 发布 Float64MultiArray 到 /servo_currents
```

重要点：

- 发布器不直接读取硬件。
- `state` 是本周期唯一硬件快照。
- 读取失败时，如果 `use_last_valid=True`，离线舵机会使用上一帧有效角度/电流补值，但 `online=False`。

## 5. 重力补偿节点执行逻辑

重力补偿节点对应 `GravityCompensationNode`。

### 5.1 创建阶段

调用链：

```text
hxj_gravity_compensation.py
  -> GravityCompensationNode.from_ros()
    -> load_arm_config_from_ros()
    -> DampingMap(...)
    -> RobotDynamics(...)
    -> GravityCompensationNode(config, dynamics, damping, mode)
      -> HuaXinjingArm(config)
      -> ServoStatePublisher(arm)
      -> GravityDebugPublisher(...)
      -> GravityCompensator(arm, dynamics, damping, mode)
      -> rospy.Rate(config.rate_hz)
```

对象关系：

```text
GravityCompensationNode
  ├── HuaXinjingConfig
  ├── HuaXinjingArm
  │     └── FashionStarServoBus
  ├── RobotDynamics
  ├── DampingMap
  ├── GravityCompensator
  ├── ServoStatePublisher
  └── GravityDebugPublisher
```

### 5.2 动力学模型加载

`RobotDynamics` 创建时会根据 `~urdf_path` 决定是否加载 URDF。

```text
RobotDynamics(dof, urdf_path, compensation_gain)
  -> 如果 urdf_path 非空:
       import pinocchio
       pin.buildModelFromUrdf(urdf_path)
       model.createData()
       dof = model.nq
  -> 如果失败:
       available=False
       load_error=失败原因
```

模式行为：

- `mode=auto`：动力学可用则用动力学补偿，不可用则退化到自适应阻尼。
- `mode=dynamics`：强制使用动力学补偿，模型不可用会抛出错误。
- `mode=adaptive`：不使用动力学模型，只使用角度变化自适应阻尼。

### 5.3 连接和初始化阶段

`GravityCompensationNode.run()` 首先调用：

```python
self.connect()
```

内部执行：

```text
GravityCompensationNode.connect()
  -> arm.connect()
    -> bus.connect()
    -> calibrate_zero()
  -> compensator.initialize()
    -> 检查 dynamics 模式是否可用
    -> set_all_damping(base_power)
    -> read_state()
    -> 记录 last_angles_deg
```

初始化完成后，舵机会先进入基础阻尼状态。

### 5.4 主循环阶段

主循环：

```python
while not rospy.is_shutdown():
    state = self.arm.read_state()
    self.compensator.step(state)
    self.state_publisher.publish_angles(state)
    self.debug_publisher.publish(
        self.compensator.power_array(),
        self.compensator.torque_array(),
    )
    self.rate.sleep()
```

每个周期严格按以下顺序执行：

```text
读取一帧舵机状态
  -> 使用同一帧状态计算补偿
  -> 下发阻尼功率
  -> 使用同一帧状态发布角度
  -> 发布阻尼功率和补偿力矩调试数据
```

这保证控制和发布看到的是同一帧硬件状态。

## 6. 舵机状态读取链路

读取链路从 `HuaXinjingArm` 开始，底层落到官方 SDK。

```text
HuaXinjingArm.read_state()
  -> FashionStarServoBus.read_all_states()
    -> FashionStarServoBus.read_state(id)
      -> read_angle_deg(id)
        -> UartServoManager.query_servo_angle(id)
      -> read_current_a(id)
        -> UartServoManager.query_current(id)
```

读取结果包装成：

```python
ServoState(
    servo_id=id,
    angle_deg=angle,
    current_a=current,
    online=True,
)
```

如果读取异常或角度为空：

```python
ServoState.offline(id)
```

然后 `HuaXinjingArm` 聚合所有舵机状态：

```python
ArmState(states={id: ServoState, ...}, num_servos=config.num_servos)
```

## 7. `HuaXinjingArm` 的内部状态

`HuaXinjingArm` 是工程里的核心数据中枢，维护以下状态：

```text
zero_angles           # 启动零点，单位 degree
last_raw_angles_deg   # 上一帧有效原始角度
last_angle_message    # 上一帧发布角度，用于跳变过滤
last_currents_a       # 上一帧有效电流
latest_state          # 最近一次 ArmState 快照
```

### 7.1 启动零点

如果：

```text
zero_on_start=True
publish_absolute=False
```

启动时会执行：

```text
calibrate_zero()
  -> read_state(use_last_valid=False)
  -> 把当前 angle_deg 保存到 zero_angles
```

之后发布角度时：

```text
published_angle = current_angle - zero_angle
```

### 7.2 角度消息过滤

`angle_message_data()` 会进行两类过滤：

```text
jump_threshold_deg:
  如果本帧和上一帧差值过大，认为是异常跳变，沿用上一帧发布值。

step_size_deg:
  如果本帧变化过小，认为低于有效分辨率，沿用上一帧发布值。
```

第一帧会直接接受，避免绝对角度被误判成跳变。

### 7.3 动力学关节角映射

动力学模型使用的关节角由 `joint_positions_rad()` 生成。

映射公式：

```text
relative_deg = servo_angle_deg - zero_angle_deg
joint_deg = relative_deg * joint_sign + joint_offset_deg
q_rad = radians(joint_deg)
```

相关参数：

- `~joint_servo_ids`：动力学关节对应的舵机 ID 顺序。
- `~joint_signs`：方向系数，常用 `1` 或 `-1`。
- `~joint_offsets_deg`：进入 URDF 前的角度偏置。

如果 URDF 自由度比舵机数量少，会按动力学 DOF 截断；如果不足，会补 0。

## 8. 动力学重力补偿链路

动力学模式下，执行链路是：

```text
GravityCompensator.step(state)
  -> _step_dynamics(state)
    -> arm.joint_positions_rad(state, ordered_ids, dof)
    -> dynamics.compute_gravity_torque(q_rad)
    -> compensation_torque = -gravity_tau * joint_sign
    -> damping.torque_to_power(compensation_torque)
    -> bus.set_damping(servo_id, power)
```

核心公式：

```text
tau_g(q) = Pinocchio.computeGeneralizedGravity(model, data, q)
tau_comp = -tau_g(q)
power = base_power + abs(tau_comp) * torque_gain
```

注意：

- `tau_comp` 是调试话题 `/gravity_torques` 发布的值。
- `power` 是实际下发给舵机 `set_damping()` 的阻尼功率。
- 舵机阻尼功率不是闭环力矩控制，只是对 FeelForce 力矩输出的近似表达。

## 9. 自适应阻尼链路

当 `mode=adaptive`，或 `mode=auto` 且动力学模型不可用时，走自适应阻尼。

执行链路：

```text
GravityCompensator.step(state)
  -> _step_adaptive_angle(state)
    -> delta_deg = current_angle - previous_angle
    -> damping.angle_delta_to_power(delta_deg, previous_power)
    -> bus.set_damping(servo_id, power)
```

映射规则：

```text
delta_deg > 0:
  target = base_power + delta_deg * angle_gain
  power = max(previous_power, target)

delta_deg <= release_delta:
  power = base_power

其他情况:
  power = previous_power
```

自适应模式的目的只是提供没有 URDF 时的简化手感，不等价于真实重力补偿。

## 10. ROS 话题输出

### 10.1 只读节点

默认发布：

```text
/servo_angles
  类型: std_msgs/Float64MultiArray
  单位: degree
  含义: 舵机角度数组，索引对应舵机 ID
```

可选发布：

```text
/servo_currents
  类型: std_msgs/Float64MultiArray
  单位: A
  含义: 舵机电流数组，索引对应舵机 ID
```

### 10.2 重力补偿节点

发布：

```text
/servo_angles
  类型: std_msgs/Float64MultiArray
  单位: degree

/servo_damping_powers
  类型: std_msgs/Float64MultiArray
  单位: mW

/gravity_torques
  类型: std_msgs/Float64MultiArray
  单位: Nm
```

## 11. 关键 ROS 参数

### 11.1 通信与舵机

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~port` | `/dev/ttyUSB0` | 串口设备 |
| `~baudrate` | `115200` | 串口波特率 |
| `~timeout` | `0.0` | 串口读取超时 |
| `~servo_ids` | `[0,1,2,3,4,5,6]` | 参与读取/控制的舵机 |
| `~num_servos` | `7` | ROS 输出数组长度 |
| `~rate` | `50` | 节点循环频率 |

### 11.2 角度发布

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~zero_on_start` | `true` | 启动时记录当前角度为零点 |
| `~publish_absolute` | `false` | 是否发布绝对角度 |
| `~use_last_valid` | `true` | 读取失败时是否沿用上一帧有效值 |
| `~step_size` | `0.0` | 小变化抑制阈值 |
| `~jump_threshold` | `90.0` | 跳变过滤阈值 |
| `~read_current` | `false` | 是否读取电流 |

### 11.3 动力学和阻尼

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~mode` | `auto` | `auto` / `dynamics` / `adaptive` |
| `~urdf_path` | 空 | URDF 文件路径 |
| `~compensation_gain` | `1.0` | 重力补偿增益 |
| `~base_power` | `100` | 基础阻尼功率 mW |
| `~max_power` | `3000` | 最大阻尼功率 mW |
| `~torque_gain` | `100.0` | Nm 到 mW 的映射增益 |
| `~angle_gain` | `20.0` | degree 到 mW 的自适应增益 |
| `~release_delta` | `-2.0` | 自适应模式释放阈值 |

### 11.4 关节映射

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~joint_servo_ids` | 等于 `~servo_ids` | 动力学关节对应舵机顺序 |
| `~joint_signs` | 全部 `1.0` | 关节方向系数 |
| `~joint_offsets_deg` | 全部 `0.0` | 关节角度偏置 |

## 12. 关闭逻辑

### 12.1 只读节点

```text
finally:
  node.close()
    -> arm.close()
      -> bus.close()
        -> uart.close()
```

### 12.2 重力补偿节点

```text
finally:
  node.close()
    -> compensator.stop(release_on_shutdown)
      -> release_all() 或 set_all_damping(base_power)
    -> arm.close()
      -> bus.close()
```

如果舵机总线尚未连接，`compensator.stop()` 会直接返回，避免异常退出时二次报错。

## 13. 最小运行示例

### 13.1 只读角度

```bash
rosrun uarm hxj_servo_reader.py \
  _port:=/dev/ttyUSB0 \
  _baudrate:=115200 \
  _servo_ids:="[0,1,2,3,4,5,6]" \
  _num_servos:=7 \
  _rate:=50
```

### 13.2 自适应阻尼

```bash
rosrun uarm hxj_gravity_compensation.py \
  _mode:=adaptive \
  _base_power:=300 \
  _angle_gain:=20 \
  _max_power:=2000
```

### 13.3 动力学重力补偿

```bash
rosrun uarm hxj_gravity_compensation.py \
  _mode:=dynamics \
  _urdf_path:=/absolute/path/to/arm.urdf \
  _joint_servo_ids:="[0,1,2,3,4,5,6]" \
  _joint_signs:="[1,1,1,1,1,1,1]" \
  _joint_offsets_deg:="[0,0,0,0,0,0,0]" \
  _compensation_gain:=0.5 \
  _base_power:=100 \
  _torque_gain:=50 \
  _max_power:=1500
```

第一次上硬件建议从较小的 `compensation_gain`、`base_power` 和 `torque_gain` 开始。

## 14. 调试建议

建议按以下顺序调试：

1. 先运行 `hxj_servo_reader.py`，确认 `/servo_angles` 数组索引和舵机 ID 对应正确。
2. 打开 `_read_current:=true`，确认电流读取不会明显拖慢循环。
3. 运行 `mode:=adaptive`，用较小 `base_power` 验证 `set_damping()` 可用。
4. 准备 URDF 后运行 `mode:=auto`，观察是否成功加载动力学模型。
5. 再切到 `mode:=dynamics`，逐个调整 `joint_signs` 和 `joint_offsets_deg`。
6. 观察 `/gravity_torques` 和 `/servo_damping_powers`，确认补偿方向和阻尼幅度合理。

## 15. 常见问题

### 15.1 `mode=dynamics` 启动失败

原因通常是：

- `~urdf_path` 为空或路径错误。
- Python 环境没有安装 Pinocchio。
- URDF 文件结构不被 Pinocchio 接受。

处理：

```text
先使用 mode=auto 或 mode=adaptive 验证舵机链路，
再修复 URDF/Pinocchio 环境。
```

### 15.2 `/servo_angles` 第一帧不是 0

可能原因：

- `publish_absolute=True`，发布的是绝对角度。
- `zero_on_start=False`，没有启动归零。
- 某个舵机启动时读取失败，使用了上一帧默认值。

### 15.3 动力学补偿方向不对

优先检查：

- `~joint_servo_ids` 顺序是否与 URDF 关节顺序一致。
- `~joint_signs` 是否需要把某些关节改为 `-1`。
- `~joint_offsets_deg` 是否需要补偿机械安装零点。

### 15.4 阻尼过大或发热

优先降低：

- `~base_power`
- `~torque_gain`
- `~max_power`
- `~compensation_gain`

华馨京舵机的 `set_damping()` 不是闭环力矩控制，不应直接按理想力矩控制器的参数上硬件。
