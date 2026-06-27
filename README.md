# 华馨京舵机单文件节点

这是一个从 STM32/C++ 思路迁移到 Python/ROS 的华馨京舵机控制框架。

当前版本已经放弃旧的多文件模块结构，核心代码集中在：

```text
hxj_duoji_node.py
```

## 当前目标

- 使用一个 Python 文件完成节点主体。
- 只保留三个线程角色：`main` 主线程、ROS 发布线程、控制线程。
- 舵机 SDK 只在控制线程中访问，避免多个线程同时操作串口。
- 使用官方同步读取 API `send_sync_servo_monitor(servo_ids)` 读取舵机状态。
- 暂时不做零点标定。
- 已加入末端合阻尼到切向舵机的分配计算。
- 暂时不做重力补偿。
- 在控制线程中保留状态机框架，方便后续补不同运动状态下的控制方案。

## 文件说明

```text
feel_force_duoji/
  config/example.json           # 机械臂参数 JSON 示例，默认会读取这个文件
  hxj_duoji_node.py              # 单文件 ROS 节点、配置类、ROS 线程类、控制线程类
  tests/test_hxj_duoji_node.py   # 不依赖 ROS/硬件的本地测试
```

旧的 `hxj_modular/`、`hxj_servo_reader.py`、`hxj_gravity_compensation.py`
已经移除。如需查看旧实现，可以通过 git 历史回退或查看 baseline commit。

## 线程结构

```text
main 主线程
  -> 初始化 ROS
  -> 创建 DuojiConfig
  -> 创建共享状态和线程锁
  -> 创建 RosWorker
  -> 创建 ControlWorker
  -> 启动 ROS 发布线程
  -> 启动控制线程
  -> 等待退出并关闭线程

`DuojiConfig` 配置类
  -> 读取 ROS 参数
  -> 默认读取 config/example.json
  -> 把 JSON 内容放到 config["arm_params"]
  -> 创建 shared_state

`RosWorker` ROS 线程类
  -> 读取共享状态快照
  -> 发布 /servo_angles
  -> 可选发布 /servo_currents

`ControlWorker` 控制线程类
  -> 打开 serial.Serial
  -> 创建 UartServoManager
  -> 调用 send_sync_servo_monitor(servo_ids)
  -> 更新共享状态
  -> 执行控制状态机
```

## 类职责

```text
DuojiConfig
  负责读 ROS 参数和 config/example.json。
  后续如果新增配置项，优先改这个类。

RosWorker
  负责 ROS 发布。
  这个类不碰串口、不碰 UartServoManager。

ControlWorker
  负责串口、官方舵机 API、同步读取、控制状态机。
  后续不同运动状态下的控制方案，优先写到这个类的 handle_xxx() 方法里。
```

## 控制状态机

当前状态机已在 `PLAN` 状态加入末端合阻尼分配：

```text
IDLE    空闲
MANUAL  手动调试
PLAN    计算末端合阻尼到切向舵机的分配
MOVE    运动执行预留
HOLD    保持预留
ERROR   错误保护
```

后续主要补 `ControlWorker` 里的这些方法：

```python
ControlWorker.handle_idle()
ControlWorker.handle_manual()
ControlWorker.handle_plan()
ControlWorker.handle_move()
ControlWorker.handle_hold()
ControlWorker.handle_error()
```

## 末端合阻尼分配

机械臂参数从 JSON 读取，格式沿用示例：

```json
{
  "dof": 3,
  "joints": {
    "joint1": { "type": "axial", "link": { "length": 0.18 } },
    "joint2": { "type": "tangential", "link": { "length": 0.15 } },
    "joint3": { "type": "tangential", "link": { "length": 0.10 } }
  }
}
```

舵机 ID 按 joint 自然顺序安排：

```text
joint1 -> servo_id 0
joint2 -> servo_id 1
joint3 -> servo_id 2
```

`axial` 关节不参与阻尼分配。`tangential` 关节按从当前关节到末端的剩余连杆长度和分配。

例如末端合阻尼为 `1000` 时：

```text
joint2 weight = 0.15 + 0.10 = 0.25 -> servo 1 = 714.2857
joint3 weight = 0.10              -> servo 2 = 285.7143
```

## ROS 参数

常用参数：

```text
~port                 默认 /dev/ttyUSB0
~baudrate             默认 115200
~timeout              默认 0.0
~servo_ids            默认 [0,1,2,3,4,5,6]
~num_servos           默认 7
~rate                 默认 50.0
~angle_topic          默认 /servo_angles
~current_topic        默认 /servo_currents
~read_current         默认 false
~release_on_shutdown  默认 false
~arm_config           默认 config/example.json，机械臂 JSON 文件路径
~end_damping          默认 0.0，末端合阻尼
```

## Linux / ROS 运行示例

Windows 当前只能做语法和纯 Python 测试，不能验证 ROS 与舵机硬件。

在 Ubuntu + ROS 环境中运行：

```bash
rosrun uarm hxj_duoji_node.py \
  _port:=/dev/ttyUSB0 \
  _baudrate:=115200 \
  _servo_ids:="[0,1,2,3,4,5,6]" \
  _num_servos:=7 \
  _rate:=50 \
  _end_damping:=1000
```

如果要临时换另一套机械臂参数，可以再额外传：

```bash
_arm_config:=/absolute/path/to/other_arm.json
```

查看角度：

```bash
rostopic echo /servo_angles
```

如需发布电流：

```bash
rosrun uarm hxj_duoji_node.py _read_current:=true
```

## 本地验证

Windows 上可以运行：

```bash
python -m unittest tests.test_hxj_duoji_node
python -m py_compile hxj_duoji_node.py tests/test_hxj_duoji_node.py
```

这些验证只覆盖不依赖 ROS 和舵机硬件的部分。
