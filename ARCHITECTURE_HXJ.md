# 华馨京舵机版模块化架构

本工程保留 FeelForce 的核心思维，但针对华馨京 / FashionStar 舵机做了简化。

## 核心思想

原 FeelForce 面向 CAN + MIT 电机协议，因此拆成：

```text
Communication -> ProtocolDecoder -> Motor -> Arm -> ControlManager / Publisher -> ArmNode
```

华馨京舵机官方 SDK 已经封装串口通信和协议帧，因此本工程不再拆分通信层和协议层，而是融合成：

```text
FashionStarServoBus -> HuaXinjingArm -> GravityCompensator / Publisher -> Node
```

保留下来的模块化思想是：

- 单一装配点：`node.py` 负责创建对象、注入依赖和管理生命周期。
- 单一数据通道：上层通过 `HuaXinjingArm` 读写机械臂语义，不直接访问 SDK。
- 控制策略独立：`GravityCompensator` 只关心状态和目标阻尼功率。
- 发布逻辑独立：`publisher.py` 只把内部状态转换为 ROS 消息。
- 硬件细节收敛：`FashionStarServoBus` 统一封装官方 SDK 调用。

## 分层职责

### 1. `servo_bus.py`

通信和协议融合后的舵机总线。

直接调用官方 SDK：

```python
query_servo_angle(id)
query_current(id)
set_damping(id, power_mw)
stop_on_control_mode(id, ...)
```

### 2. `arm.py`

机械臂聚合层：

- 维护 `servo_ids`
- 读取所有舵机状态
- 保存 `latest_state` 快照，作为发布器和控制器的共同数据源
- 记录启动零点
- 生成 `/servo_angles` 所需数组
- 将舵机角度按 `joint_servo_ids / joint_signs / joint_offsets_deg` 映射为动力学关节角
- 读取失败时沿用上一帧有效值

上层控制器不直接访问 SDK。

### 3. `dynamics.py`


```text
q(rad) -> gravity torque(Nm)
```

如果存在 URDF 和 Pinocchio，就计算真实重力项；否则标记为不可用，让控制器退化到自适应阻尼模式。

### 4. `gravity_compensation.py`

控制策略层：

```text
读取同一帧状态 -> 舵机角度映射 q(rad) -> 算补偿 -> 写阻尼
```

由于华馨京舵机没有直接力矩控制接口，FeelForce 的力矩输出通过以下方式近似：

```text
damping_power = base_power + abs(torque_nm) * torque_gain
```

这里的 `torque_nm` 是 `-tau_g(q)`，也就是原 FeelForce 重力补偿里用于抵消重力的反向力矩。

没有动力学模型时，使用角度变化自适应阻尼：

```text
角度正向变化 -> 增加阻尼
角度反向超过 release_delta -> 回到基础阻尼
```

### 5. `publisher.py`

发布层只消费 `Arm` 的状态快照，不直接访问硬件。

- `ServoStatePublisher` 发布 `/servo_angles` 和可选 `/servo_currents`
- `GravityDebugPublisher` 发布 `/servo_damping_powers` 和 `/gravity_torques`

### 6. `node.py`

装配层是唯一创建模块和管理生命周期的位置：

```text
读取 ROS 参数 -> 创建 Config -> 创建 Arm -> 创建 Control/Publisher -> run()
```

节点循环负责控制硬件读取节奏：

```text
read_state() -> control/publish 使用同一帧 ArmState
```

入口脚本保持很薄，只负责 `rospy.init_node()` 和调用对应 Node。

## ROS 入口

### `hxj_servo_reader.py`

只读入口，实际装配在 `ServoReaderNode`。

发布：

- `/servo_angles`
- 可选 `/servo_currents`

### `hxj_gravity_compensation.py`

控制入口，实际装配在 `GravityCompensationNode`。

发布：

- `/servo_angles`
- `/servo_damping_powers`
- `/gravity_torques`

并持续调用：

```python
set_damping(servo_id, power_mw)
```

## 设计原则

- 舵机版不照搬 CAN 版的复杂协议层。
- 上层永远操作 `Arm`，不直接操作 SDK。
- 控制策略只关心状态和目标功率，不关心串口细节。
- 发布器不触发硬件读取，只发布上一帧或当前帧快照。
- ROS 节点只负责参数、发布和生命周期装配。
