# HuaXinjing_servo：华馨京舵机动态阻尼控制

本目录提供 ROS1 舵机控制节点、终端遥测界面，以及独立的舵机配置和零点校准工具。主节点读取 JSON 中的机械臂几何参数，通过三维正运动学计算末端位置，再根据末端升降速度选择阻尼档位，按切向雅可比列范数分配阻尼参数。

当前示例为 3 轴机械臂：ID 0 为轴向关节，ID 1、2 为切向关节。这里的“重力方向手感优化”通过动态阻尼实现，没有根据质量和质心计算重力补偿力矩。

## 1. 当前文件与职责

```text
HuaXinjing_servo/
├── hxj_duoji_node.py       # ROS 主节点：配置、串口反馈、状态机、运动学与阻尼控制
├── key_hold_thread.py     # 由主节点启动的键盘线程，提交锁定/解锁请求
│
├── telemetry_debug_node.py # 独立 ROS 调试节点，订阅话题并刷新终端界面
├── config_servo.py        # 独立硬件工具：ID 配置、响应检查、释放/锁定
├── calibrate_zero.py      # 独立硬件工具：查看角度、逐个或批量设置零点
├── config/
│   └── example.json       # 当前三轴机械臂的类型、DH 几何参数和质量信息
├── README.md
├── .gitignore
└── .idea/                # IDE 工程配置，不参与控制运行
```

日常运行使用 `hxj_duoji_node.py`，另开终端运行 `telemetry_debug_node.py` 查看结果。`key_hold_thread.py` 由主节点导入，无需单独启动。两个硬件工具按需独立运行。

## 2. 运行环境

主节点面向 Linux、Python 3 和 ROS1，例如 ROS Noetic。键盘模块依赖 Linux 的 `fcntl`、`termios` 和输入设备接口，不能直接在原生 Windows Python 下运行。

| 依赖 | 用途 |
| --- | --- |
| ROS1 的 `rospy`、`std_msgs` | 节点通信，发布和订阅 `Float64MultiArray` |
| `numpy` | DH 矩阵、雅可比、速度及阻尼分配计算 |
| `pyserial`，导入名 `serial` | 串口通信 |
| 官方 SDK，导入名 `fashionstar_uart_sdk` | 舵机读写和模式控制 |

官方 SDK 需在运行节点的同一 Python 环境中可导入。主节点使用 `UartServoManager`、`send_sync_servo_monitor()`、`set_servo_angle()`、`set_damping()` 和 `stop_on_control_mode()`；校零工具还需要 `disable_torque()` 和 `set_origin_point()`。

准备好 ROS 环境后，可先检查 Python 依赖能否导入，这条命令不连接舵机：

```bash
python3 -c "import rospy, numpy, serial, fashionstar_uart_sdk; from std_msgs.msg import Float64MultiArray; print('Python dependencies OK')"
```

默认串口是 `/dev/ttyUSB0`，波特率为 `115200`。运行用户需要有对应设备的访问权限。主节点与配置、校零工具都会访问串口，使用硬件工具前先退出主节点；遥测调试节点只订阅 ROS 话题，可以和主节点同时运行。

## 3. 启动主节点与调试界面

以下以 ROS Noetic 为例。将命令中的 `/path/to/LeRobot-Anything-U-Arm-main` 替换为 Linux 上的实际仓库路径；各终端使用同一个 ROS master。

### 3.1 终端一：启动 ROS master

若已经有可用的 ROS master，可跳过：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 3.2 终端二：启动控制节点

**主节点启动后会先向配置中的所有舵机下发 0° 运动命令，再显示菜单。** 确认当前机械结构允许这一动作后运行：

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop/HuaXinjing_servo
python3 hxj_duoji_node.py
```

启动流程是：打开串口并读取反馈，进入 `START`，逐个下发 `set_servo_angle(id, 0.0, interval=2000)`，等待 2.5 秒，然后显示：

```text
请选择目标状态：
  1. HOLD — 阻尼保持（3D 动态阻尼）
  2. LOCKED — 锁死（所有舵机固定）
  3. IDLE — 空闲（不下发命令）
请输入选项编号:
```

这里的回零是移动到已有的舵机 0°，不会重新标定编码器零点；2.5 秒为固定等待时间，没有在此步骤逐轴确认到位。非交互输入遇到 EOF 时，菜单默认选择 `IDLE`，但此前的回零流程仍会执行。

若要使用“按住进入阻尼、松开锁定”的交互，建议先选择 `2` 进入 `LOCKED`，再操作控制键。直接选择 `1` 会立即进入持续阻尼控制，键盘线程按按下/松开的边沿提交切换请求。

### 3.3 键盘控制

主节点自动启动键盘线程，优先尝试 evdev；无法读取输入设备时，回退到主节点所在终端的 stdin 模式。实际启用的模式会写入启动日志。

| 模式 | 当前代码实际监听 | 松开判定 |
| --- | --- | --- |
| evdev | Linux 键码 `45`，对应 **X 键** | 读取设备的按下、松开事件 |
| stdin 回退 | 小写字符 `k` | 根据连续字符及超时推断，需主节点终端获得输入焦点 |

当前 `KEY_K_CODE = 45` 与变量名、注释所称的 K 键不一致；Linux K 键的键码是 37。上表按当前代码行为说明。按下控制键请求 `LOCKED → HOLD`，松开请求 `HOLD → LOCKED`；这些请求不负责把 `IDLE` 切换成 `HOLD`。

stdin 模式默认首键超时 0.5 秒、连续输入阶段超时 0.08 秒，松开后有 0.15 秒冷却期。在 X11 桌面终端使用此模式时，可按脚本的预期设置键盘重复速率：

```bash
xset r rate 250 40
```

这条命令会调整当前 X11 会话的键盘重复设置；evdev 模式不依赖它。

### 3.4 终端三：启动遥测界面

```bash
source /opt/ros/noetic/setup.bash
cd /path/to/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop/HuaXinjing_servo
python3 telemetry_debug_node.py
```

界面默认每秒刷新 10 次；如需更改，可使用该调试节点支持的 ROS 私有参数：

```bash
python3 telemetry_debug_node.py _refresh_rate:=20
```

舵机数量由 `/servo_angles` 的数组长度自动确定，不需要给调试节点指定 DOF。主节点和调试节点在各自终端用 `Ctrl+C` 退出；只退出调试节点不会停止主节点。

主节点默认 `release_on_shutdown=False`，退出时关闭串口，但不主动发送释放指令。

## 4. 配置位置与默认参数

### 4.1 主节点配置

串口、循环频率和 JSON 路径来自 `hxj_duoji_node.py` 的 `main()` 中创建的 `ServoConfig(...)`：

| 配置 | 当前默认值 | 含义 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 串口设备 |
| `baudrate` | `115200` | 串口波特率 |
| `timeout` | `0.0` | 串口读取超时 |
| `rate` | `50.0` | ROS 发布目标频率及控制循环的等待间隔基准 |
| `arm_config` | `None` | 使用脚本所在目录的 `config/example.json` |
| `release_on_shutdown` | `False` | 是否在退出时主动释放舵机 |
| `servo_ids` | 未显式传入 | 默认由 `dof` 生成 `[0, ..., dof-1]` |

主节点当前没有用 `rospy.get_param()` 读取这些配置，也没有命令行参数解析；需要调整时，修改 `main()` 的构造参数。给主节点传入 `_port:=...` 不会改变其串口配置。

### 4.2 机械臂 JSON

`config/example.json` 中 `arm_name="example"`、`dof=3`，当前几何参数如下：

| 关节 / 默认 ID | 类型 | `link.l` → a（m） | `link.alpha`（rad） | `link.d`（m） | `link.theta` 固定偏置（rad） |
| --- | --- | --- | --- | --- | --- |
| joint1 / 0 | axial | 0 | 1.5708 | 0.08 | 0 |
| joint2 / 1 | tangential | 0.084 | 0 | 0 | 1.5708 |
| joint3 / 2 | tangential | 0.25 | 0 | 0 | 0 |

解析器按 `joint1 → joint2 → … → jointN` 读取；`dof` 应与完整的关节配置对应。`servo_ids[i]` 对应第 i 个关节，默认连续 ID 同时便于 ROS 数组按 ID 索引。

```text
实际 DH 角度 theta_i = radians(舵机读数_i) + link.theta_i
```

因此三个舵机都读到 0° 时，进入 DH 的角度为 `[0, 1.5708, 0]` rad。`link.theta` 是模型偏置，与校零工具修改硬件零点的操作不同。

旧字段 `length/twist/offset` 仍作为 `l/alpha/d` 的兼容回退。JSON 中的 `mass`、`actuator_mass` 被读取但未参与当前阻尼计算，`center_of_mass` 未被当前解析器读取。调整构型时需要核对实际 DH 坐标系，不能仅按关节类型推断全部几何参数。

### 4.3 阻尼与锁定参数

以下值集中在主脚本顶部 `DEFAULTS` 中：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `end_damping_low` | 600 mW | LOW 档切向总预算 |
| `end_damping_mid` | 1000 mW | MID 档切向总预算 |
| `end_damping_high` | 1500 mW | HIGH 档切向总预算 |
| `base_damping_power` | 500 mW | 每个轴向关节的独立固定阻尼 |
| `max_damping_power` | 1000 mW | 切向单关节的分配上限 |
| `lock_power` | 6000 mW | LOCKED 状态的锁力参数 |
| `vz_threshold` | 0.01 m/s | 末端竖直速度切档阈值 |

`end_damping` 是仍保留的旧字段；当前控制使用三个分档预算，不能通过把该旧字段设为 0 来关闭动态阻尼。`max_damping_power` 只在切向阻尼分支限幅，轴向固定参数与锁力参数分别处理。

## 5. 控制结构与算法

主脚本中，`ServoConfig` 负责读配置和初始化共享状态，`Controller` 负责硬件通信和控制，`RosPublisher` 负责发布话题。

```text
main 主线程：装配对象，等待 ROS 退出，回收线程
├── control_thread：读取舵机反馈 → 状态机 → 控制计算与下发
├── ros_thread：复制共享状态 → 发布硬件反馈和控制遥测
└── key_hold_thread：检测按键 → 设置锁定/解锁请求

telemetry_debug_node：独立进程，订阅 ROS 话题并显示
```

主节点内部由控制线程独占串口和 SDK，共享状态读写使用锁；键盘请求通过 Controller 的标志字段传递。控制循环每轮处理完毕后等待 `1/rate` 秒，因此包含通信和计算的实际周期不保证严格为 50 Hz。启动菜单会阻塞控制线程，等待期间不持续读取新反馈。

### 状态机

| 状态 | 当前行为 |
| --- | --- |
| START | 下发 0° 运动命令、固定等待、显示状态选择菜单 |
| HOLD | 每周期计算并下发三档动态阻尼 |
| LOCKED | 对在线舵机调用 `stop_on_control_mode(id, 0x11, lock_power)` |
| IDLE | 不主动下发新的控制命令，不撤销舵机之前的硬件模式 |
| ERROR | 预留状态，处理函数当前为空，不包含自动释放动作 |

在正常的非 HOLD 状态处理分支中，控制遥测会被清零。通信或控制循环异常会记录错误并停止工作线程；这不等于已经执行硬件释放，主线程仍可能等待 ROS 退出。

### HOLD 中的计算流程

```text
JSON + 全部舵机角度
    → theta = radians(angle) + theta_offset
    → 所有关节的 DH 变换连乘
    → 末端位置 p_end
        ├── 相邻位置差分 / 时间差 → vz → 选择 LOW、MID、HIGH 预算
        └── 累积变换 → 雅可比 J → 提取切向列 J_tan
                                  → 列范数归一化为权重 w
    → 切向预算 × w → 逐项限幅、int() 取整 → set_damping()
    → 轴向关节另设固定阻尼 → 写入遥测并保存本轮历史
```

```text
v = (p_current - p_previous) / dt
s_j = ||J_tan[:, j]||₂
w_j = s_j / Σs_k
P_j = int(min(B × w_j, max_damping_power))
```

上式表示有有效历史和非零范数总和时的正常路径。首次采样或时间差过小时，速度设为零；切向范数总和小于 `1e-9` 时，函数改为均分预算。权重在阻尼分发时使用，源码没有另外构造加权雅可比矩阵。

| 竖直速度 | 档位 | 切向预算 |
| --- | --- | --- |
| `vz > 0.01`，末端上升 | LOW / 1 | 600 mW |
| `-0.01 ≤ vz ≤ 0.01` | MID / 2 | 1000 mW |
| `vz < -0.01`，末端下降 | HIGH / 3 | 1500 mW |

这里按基坐标系正 z 方向向上解释升降。姿态决定分配比例，速度决定预算；轴向固定参数不从切向预算中扣除。限幅、取整后，切向参数之和可能小于预算。

当前实现的边界：

- 控制与发布使用末端位置差分速度。虽然计算了 `J @ omegas`，但该结果未参与切档或遥测发布。
- `compute_jacobian_3d()` 使用变换后的 z 轴；标准 DH 雅可比应使用前一坐标系的 z 轴。当前示例的第一列有差异，两个 `alpha=0` 的切向列不受此索引问题影响。其他构型需重新核对。
- 速度没有低通滤波，切档没有滞回区；阈值附近可能频繁切换。
- mW 为 SDK 控制参数。本算法没有执行质量、质心和重力力矩计算，也不保证精确的末端阻尼系数。

## 6. ROS 话题与遥测示例

所有下列话题的消息类型均为 `std_msgs/Float64MultiArray`。舵机数组按 ID 索引，当前长度为 3。

| 话题 | 内容 |
| --- | --- |
| `/servo_angles` | 原始角度，单位 deg |
| `/servo_currents` | SDK 电流反馈 |
| `/servo_voltages` | SDK 电压反馈 |
| `/servo_powers` | SDK 功率反馈 |
| `/servo_temps` | SDK 温度反馈 |
| `/servo_statuses` | SDK 状态码，以浮点数组承载 |
| `/servo_turns` | SDK 圈数反馈 |
| `/servo_online` | 在线标记，1 或 0 |
| `/servo_end_position` | 末端位置 `[x,y,z]`，单位 m |
| `/servo_end_velocity` | 位置差分速度 `[vx,vy,vz]`，单位 m/s |
| `/servo_damping_mode` | 单项数组 `[mode]`，0=NONE、1=LOW、2=MID、3=HIGH |
| `/servo_damping_powers` | 目标阻尼参数，单位 mW |

`/servo_damping_powers` 是代码下发目标的记录，与硬件反馈 `/servo_powers` 不同。正常非 HOLD 状态的控制遥测会清零，因此此时零位置不代表机械臂实际在基坐标系原点。异常停止后也不要把界面缓存的最后一帧当作新的反馈。

调试节点显示五个话题：角度、末端速度、末端位置、档位和目标阻尼。各话题独立更新，界面不是严格同步的同周期采样。

以当前 JSON、全部 0°、首次采样或保持静止、默认 HOLD 参数为例，推算的界面数据为：

```text
【舵机状态】  (DOF: 3)
  ID  |  Angle(deg)  |  Damping(mW)
------+--------------+--------------
   0  |      0.0000  |       500.0
   1  |      0.0000  |       571.0
   2  |      0.0000  |       428.0

【末端速度 /servo_end_velocity (m/s)】
  [  0.0000,   0.0000,   0.0000]

【末端位置 /servo_end_position (m)】
  [ -0.0000,  -0.0000,   0.4140]

【阻尼档位 /servo_damping_mode】
  MID   (水平/静止)
```

对应切向权重约为 `[0.571918, 0.428082]`。x、y 的 `-0.0000` 来自 JSON 中 `1.5708` 与精确 `π/2` 的微小差异，经四位小数格式化后显示；上述数值是计算示例，并非实机采集记录。

## 7. 独立硬件工具

两个工具都通过串口直接操作舵机，不需要启动 ROS master，也不会读取 `config/example.json`。在本目录中选择所需工具运行：

| 工具与启动命令 | 菜单功能 |
| --- | --- |
| `python3 config_servo.py` | 1：配置 ID；2：查询所有舵机；3：释放；4：锁定；5：移动到 135°；6：退出 |
| `python3 calibrate_zero.py` | 1：查看角度；2：逐个校零；3：将当前位置批量设零；4：退出 |

使用前核对脚本顶部的 `SERIAL_PORT`、`BAUD_RATE` 和 `SERVO_COUNT`。当前两者均默认 `SERVO_COUNT=7`，与主节点 JSON 的 3 轴配置独立；`config_servo.py` 的 ID 配置流程还直接写有 `range(7)`，只修改 `SERVO_COUNT` 不会改变该流程。

`config_servo.py` 的 ID 配置要求每次仅连接一个舵机，按提示逐个设置和验证。菜单中的“测试”是硬件响应查询。其锁定命令使用 500 mW，与主节点的 `lock_power=6000` 分别配置；“区域复位”会实际转到 135°，不是主节点的 0° 启动动作。

`calibrate_zero.py` 面向支持零点设置的 -M 绝对值编码器型号。逐个校零会释放舵机、等待手动摆位，再调用 `set_origin_point()`；批量校零会在输入 `yes` 后将当前姿态设为硬件零点。重新校零后，应核对该硬件零位与 JSON 的固定角度偏置是否仍匹配。

## 8. 常见排查

| 现象 | 核对内容 |
| --- | --- |
| 无法导入 `rospy` | 当前终端是否加载 ROS1 环境，是否使用对应 Python 3 |
| 无法导入 SDK 或缺少同步监控接口 | 同一 Python 环境中的官方 SDK 是否提供主节点所用 API |
| 串口打开失败或舵机无响应 | 设备路径、访问权限、供电、波特率、实际 ID，以及是否有其他串口程序同时运行 |
| 主节点停在选择菜单 | 在主节点终端输入 1、2 或 3；此时控制线程正在等待输入 |
| 按 k 没有切换 | 查看是否启用了实际监听 X 键的 evdev 模式；stdin 模式需终端焦点，状态切换仅在 HOLD/LOCKED 间处理 |
| 调试界面等待数据或 DOF 显示问号 | 主节点是否运行、是否连接同一 ROS master，以及 `/servo_angles` 是否已到达 |
| 调试界面有位置和速度零值 | 区分 HOLD 中的计算结果与非 HOLD 分支清零；同时查看档位 |
| 工具尝试查询 ID 3–6 | 工具仍使用独立的 7 轴默认值，并未随三轴 JSON 自动调整 |
