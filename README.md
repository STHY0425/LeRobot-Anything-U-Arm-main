# 华馨京舵机版 FeelForce

这是面向华馨京 / FashionStar 舵机的轻量版 FeelForce 工程。它保留原 FeelForce 的模块化思想，但不会照搬 CAN 电机工程里的通信层和协议层。

## 核心结构

原 FeelForce 面向 CAN + MIT 电机协议，结构是：

```text
Communication -> ProtocolDecoder -> Motor -> Arm -> Control / Publisher -> Node
```

华馨京舵机官方 SDK 已经封装串口通信和协议帧，因此本工程把通信和协议融合为一个舵机总线：

```text
FashionStarServoBus -> HuaXinjingArm -> GravityCompensator / Publisher -> Node
```

保留的核心思维是：

- `node.py` 是唯一装配点，负责创建对象、注入依赖和管理生命周期。
- `HuaXinjingArm` 是唯一机械臂语义入口，负责读舵机状态、保存最新快照、生成 ROS 数组。
- `GravityCompensator` 只负责控制策略：读取同一帧状态、计算补偿、下发阻尼。
- `publisher.py` 只消费 `Arm` 的状态快照，不直接访问硬件。
- `FashionStarServoBus` 直接调用官方 API，不再维护独立 `comm` / `protocol` 层。

## 目录结构

```text
feel_force_duoji/
  hxj_servo_reader.py              # 只读角度/电流的 ROS 薄入口
  hxj_gravity_compensation.py      # 重力补偿 ROS 薄入口
  architecture.md                  # 原 FeelForce 架构说明
  ARCHITECTURE_HXJ.md              # 华馨京舵机版架构说明
  EXECUTION_LOGIC.md               # 工程执行逻辑详细说明
  hxj_modular/
    servo_bus.py                   # 通信 + 协议融合后的舵机总线
    config.py                      # 运行配置
    state_types.py                 # ServoState / ArmState
    arm.py                         # 机械臂聚合层和状态快照
    dynamics.py                    # FeelForce 重力力矩计算与阻尼映射
    gravity_compensation.py        # 重力补偿控制器
    publisher.py                   # ROS 发布层
    ros_params.py                  # ROS 参数读取
    node.py                        # 装配层
```

## 角度读取节点

运行：

```bash
rosrun uarm hxj_servo_reader.py
```

默认发布：

- `/servo_angles`：`std_msgs/Float64MultiArray`
- 数组索引对应舵机 ID
- 默认值为启动零点后的角度偏移，单位 degree

常用参数：

```bash
rosrun uarm hxj_servo_reader.py \
  _port:=/dev/ttyUSB0 \
  _baudrate:=115200 \
  _servo_ids:="[0,1,2,3,4,5,6]" \
  _num_servos:=7 \
  _rate:=50
```

可选电流发布：

```bash
rosrun uarm hxj_servo_reader.py _read_current:=true _current_topic:=/servo_currents
```

## 重力补偿节点

运行：

```bash
rosrun uarm hxj_gravity_compensation.py
```

执行流程：

```text
读取一帧舵机状态
  -> 舵机角度映射为动力学关节角 q(rad)
  -> 计算 tau_g(q)
  -> 取反得到补偿力矩
  -> 映射为阻尼功率
  -> set_damping()
```

发布：

- `/servo_angles`：当前角度偏移
- `/servo_damping_powers`：每个舵机当前阻尼功率，单位 mW
- `/gravity_torques`：动力学模式下的补偿力矩，单位 Nm

## 重力补偿模式

- `auto`：默认。能加载 URDF + Pinocchio 时使用动力学重力补偿，否则退化为角度变化自适应阻尼。
- `dynamics`：强制使用动力学重力补偿。
- `adaptive`：强制使用角度变化自适应阻尼。

动力学模式：

```bash
rosrun uarm hxj_gravity_compensation.py \
  _mode:=dynamics \
  _urdf_path:=/absolute/path/to/arm7dof.urdf \
  _joint_servo_ids:="[0,1,2,3,4,5,6]" \
  _joint_signs:="[1,1,1,1,1,1,1]" \
  _joint_offsets_deg:="[0,0,0,0,0,0,0]" \
  _compensation_gain:=0.8 \
  _torque_gain:=100.0 \
  _base_power:=100 \
  _max_power:=3000
```

自适应阻尼模式：

```bash
rosrun uarm hxj_gravity_compensation.py \
  _mode:=adaptive \
  _base_power:=600 \
  _angle_gain:=20 \
  _max_power:=3000 \
  _release_delta:=-2.0
```

## 关节映射参数

动力学模型里的关节顺序和舵机 ID 不一定一致，因此提供三个参数对齐：

- `~joint_servo_ids`：参与动力学重力补偿的舵机 ID 顺序，默认等于 `~servo_ids`。
- `~joint_signs`：每个关节的方向系数，常用 `1` 或 `-1`。
- `~joint_offsets_deg`：每个关节进入 URDF 模型前额外叠加的角度偏置，单位 degree。

如果 URDF 只有部分主动关节，可以只配置对应舵机：

```bash
rosrun uarm hxj_gravity_compensation.py \
  _mode:=dynamics \
  _urdf_path:=/absolute/path/to/arm3dof.urdf \
  _joint_servo_ids:="[1,2,3]" \
  _joint_signs:="[1,-1,1]"
```

## 重要说明

- 华馨京舵机没有直接力矩控制接口，FeelForce 的力矩输出会被近似映射为阻尼功率。
- `set_damping()` 不是闭环力矩控制，第一次上硬件请使用较小的 `base_power` 和 `torque_gain`。
- `auto` 模式在 URDF 或 Pinocchio 不可用时，会自动使用角度变化自适应阻尼。
- 动力学模式效果依赖 URDF、舵机 ID 顺序、关节方向、机械零点、质量和质心参数是否正确。
- 节点每个循环只读取一次硬件状态；发布器使用同一帧快照，避免控制和发布重复读舵机。
