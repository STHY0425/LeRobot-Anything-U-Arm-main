# hxj_duoji_node.py 代码详解

这份文档把你当成第一次认真看 Python、ROS、多线程代码的人来写。

目标不是炫技，而是让你知道：

- 程序从哪里开始运行。
- 三个类分别管什么。
- 每个方法输入什么、输出什么。
- 线程之间怎么传数据。
- JSON 参数怎么变成阻尼分配结果。
- 舵机状态怎么从官方 SDK 读出来，再发布成 ROS 话题。

对应代码文件：

```text
hxj_duoji_node.py
```

对应配置文件：

```text
config/example.json
```

## 1. 先看整体

这个工程现在是“一个主 Python 文件 + 一个 config JSON + 本地测试”。

主文件里面只有三个核心类：

```text
DuojiConfig
RosWorker
ControlWorker
```

再加两个顶层函数：

```text
start_thread()
main()
```

你可以先这样理解：

```text
main()
  负责启动整个程序

DuojiConfig
  负责读配置

RosWorker
  负责把数据发布到 ROS 话题

ControlWorker
  负责串口、舵机 SDK、同步读取、控制状态机
```

程序运行起来以后，有三个线程角色：

```text
main 主线程
  只负责初始化、启动两个子线程、等待退出。

ROS 发布线程
  执行 RosWorker.run()
  只读 shared_state，然后发布 ROS 话题。

控制线程
  执行 ControlWorker.run()
  打开串口，读舵机，写 shared_state，执行状态机。
```

最重要的原则：

```text
只有 ControlWorker 访问串口和 UartServoManager。
RosWorker 不碰硬件。
```

这样做是为了避免两个线程同时操作串口。串口这种东西通常不喜欢多人同时抢。

## 2. 文件执行入口

文件最后有这段：

```python
if __name__ == "__main__":
    main()
```

意思是：

- 如果你直接运行 `hxj_duoji_node.py`，就执行 `main()`。
- 如果测试文件只是 `import hxj_duoji_node`，就不会执行 `main()`。

这很重要。

因为单元测试只想检查函数逻辑，不想真的启动 ROS，也不想真的打开串口。

## 3. main() 的完整流程

`main()` 是整个程序的启动总入口。

它的执行顺序是：

```text
1. import rospy
2. rospy.init_node("hxj_duoji_node")
3. 创建 DuojiConfig
4. 拿到 config 字典
5. 创建 shared_state
6. 创建 state_lock
7. 创建 stop_event
8. 创建 RosWorker
9. 创建 ControlWorker
10. 启动 ROS 发布线程
11. 启动控制线程
12. main 主线程自己循环等待
13. 退出时通知两个线程停止
14. 等两个线程结束
```

### 3.1 为什么 rospy 在 main() 里面 import

代码里是：

```python
def main():
    import rospy
```

不是写在文件顶部。

原因是：Windows 上没有 ROS 环境时，我们仍然希望能运行本地测试。

如果文件一开头就 `import rospy`，那么测试时一导入文件就会失败。

把它放在 `main()` 里面后，只有真正运行 ROS 节点时才需要 ROS。

### 3.2 rospy.init_node()

```python
rospy.init_node("hxj_duoji_node")
```

这是告诉 ROS：

```text
我要创建一个节点，节点名字叫 hxj_duoji_node。
```

ROS 里节点可以理解成一个正在运行的小程序。

### 3.3 创建 DuojiConfig

```python
config_reader = DuojiConfig(rospy)
config = config_reader.values
```

这两行做了很多事。

`DuojiConfig(rospy)` 会：

```text
1. 从 ROS 参数服务器读取参数。
2. 找到默认 JSON 路径 config/example.json。
3. 读取机械臂 JSON。
4. 把结果放进 config_reader.values。
```

`config` 是一个普通 Python 字典，大概长这样：

```python
{
    "port": "/dev/ttyUSB0",
    "baudrate": 115200,
    "timeout": 0.0,
    "servo_ids": [0, 1, 2, 3, 4, 5, 6],
    "num_servos": 7,
    "rate": 50.0,
    "angle_topic": "/servo_angles",
    "current_topic": "/servo_currents",
    "read_current": False,
    "release_on_shutdown": False,
    "arm_config": ".../config/example.json",
    "arm_params": {...},
    "end_damping": 0.0,
}
```

后面的 `RosWorker` 和 `ControlWorker` 都读这个 `config`。

### 3.4 创建 shared_state

```python
shared_state = config_reader.create_shared_state()
```

`shared_state` 是线程之间交换数据的地方。

它也是一个普通字典，大概长这样：

```python
{
    "servo_ids": [0, 1, 2, 3, 4, 5, 6],
    "num_servos": 7,
    "servos": {
        0: {"id": 0, "angle": None, "current": 0.0, ...},
        1: {"id": 1, "angle": None, "current": 0.0, ...},
    },
    "damping_targets": {},
    "control_state": "IDLE",
    "last_error": "",
    "last_update": 0.0,
}
```

控制线程会写它：

```text
读到舵机状态 -> 写入 shared_state["servos"]
状态机计算阻尼 -> 写入 shared_state["damping_targets"]
出现错误 -> 写入 shared_state["last_error"]
```

ROS 线程会读它：

```text
复制 shared_state 快照 -> 生成数组 -> 发布到 ROS 话题
```

### 3.5 state_lock 是什么

```python
state_lock = threading.Lock()
```

`state_lock` 是线程锁。

你可以把它想象成厕所门锁：

```text
一个线程进去改 shared_state 时，先锁门。
另一个线程只能在门外等。
第一个线程改完以后开门。
第二个线程再进去读。
```

为什么需要锁？

因为两个线程会同时碰 `shared_state`。

如果不加锁，可能出现这种情况：

```text
控制线程刚写完 angle，还没写 current。
ROS 线程突然读走了半截数据。
```

这种错误不一定每次出现，但一旦出现就很烦。

所以代码里写共享状态时，都用：

```python
state_lock.acquire()
try:
    ...
finally:
    state_lock.release()
```

`finally` 的意思是：

```text
不管 try 里面有没有报错，最后都执行 release()。
```

这能防止锁永远不释放。

### 3.6 stop_event 是什么

```python
stop_event = threading.Event()
```

`stop_event` 是一个“该停了”的信号。

main 退出时会执行：

```python
stop_event.set()
```

两个子线程的循环条件里都有：

```python
while not self.stop_event.is_set() and not self.rospy.is_shutdown():
```

意思是：

```text
只要 stop_event 还没有被 set，并且 ROS 没有关闭，就继续循环。
```

当 main 调用 `stop_event.set()` 后，子线程就会跳出循环。

### 3.7 创建两个 Worker

```python
ros_worker = RosWorker(config, shared_state, state_lock, stop_event, rospy)
control_worker = ControlWorker(config, shared_state, state_lock, stop_event, rospy)
```

两个对象拿到同一份：

```text
config
shared_state
state_lock
stop_event
rospy
```

但职责不同：

```text
RosWorker
  只发布 ROS 话题。

ControlWorker
  只访问串口和舵机 SDK。
```

### 3.8 启动线程

```python
ros_thread = start_thread("ros_thread", ros_worker.run, ())
control_thread = start_thread("control_thread", control_worker.run, ())
```

这两行启动了两个后台线程。

`ros_worker.run` 是线程要执行的方法。

`()` 表示不给这个方法额外传参数。

### 3.9 main 主线程等待

```python
while not rospy.is_shutdown():
    time.sleep(0.2)
```

main 主线程不负责具体控制。

它只是活着等。

每 0.2 秒睡一下，避免一直空转占 CPU。

### 3.10 程序退出

退出时进入：

```python
finally:
    stop_event.set()
    ros_thread.join(2.0)
    control_thread.join(2.0)
```

意思是：

```text
1. 通知两个子线程该停了。
2. 等 ROS 线程最多 2 秒。
3. 等控制线程最多 2 秒。
```

`join(2.0)` 不是无限等。

这样可以避免某个线程卡死时，整个程序永远退不出去。

## 4. DuojiConfig 详解

`DuojiConfig` 负责三类事情：

```text
1. 读取 ROS 参数。
2. 读取 config/example.json。
3. 计算末端合阻尼分配。
```

### 4.1 __init__(self, rospy)

执行逻辑：

```text
1. 保存 rospy。
2. 调用 read_ros_config(rospy)，得到配置字典。
3. 调用 load_arm_params()，读取机械臂 JSON。
```

代码效果：

```python
self.values = {
    "port": "...",
    "servo_ids": [...],
    "arm_params": {...},
}
```

以后外部通过：

```python
config_reader.values
```

拿配置。

### 4.2 load_arm_params_file(path)

作用：

```text
读取一个 JSON 文件，返回 Python 字典。
```

输入：

```text
path: JSON 文件路径
```

输出：

```text
JSON 转成的 dict
```

执行逻辑：

```text
1. import json。
2. 用 UTF-8 打开文件。
3. json.load(file_obj) 把 JSON 变成 Python dict/list。
4. 返回结果。
```

`@staticmethod` 的意思：

```text
这个方法虽然放在类里，但它不需要使用 self。
```

所以可以直接这样调用：

```python
DuojiConfig.load_arm_params_file("config/example.json")
```

### 4.3 joint_number(joint_name)

作用：

```text
从 joint 名字里提取数字。
```

例子：

```text
"joint1" -> 1
"joint3" -> 3
"abc12"  -> 12
"abc"    -> 0
```

为什么需要它？

因为 JSON 里的 joints 是字典：

```json
"joints": {
  "joint1": {...},
  "joint2": {...},
  "joint3": {...}
}
```

程序需要按 joint 编号排序，才能确定：

```text
joint1 -> servo_id 0
joint2 -> servo_id 1
joint3 -> servo_id 2
```

执行逻辑：

```text
1. 把 joint_name 转成字符串。
2. 一个字符一个字符看。
3. 如果字符是 0 到 9，就拼到 digits 里。
4. 如果最后没有数字，返回 0。
5. 否则把 digits 转成 int 返回。
```

### 4.4 ordered_joint_items(arm_params)

作用：

```text
把 JSON 里的 joints 按 joint 数字顺序排好。
```

输入：

```python
arm_params = {
    "joints": {
        "joint1": {...},
        "joint2": {...},
    }
}
```

输出：

```python
[
    ("joint1", {...}),
    ("joint2", {...}),
]
```

执行逻辑：

```text
1. 从 arm_params 里取 joints。
2. 把字典 items 转成列表。
3. 按 joint_number 排序。
4. 返回排序后的列表。
```

这里有一行：

```python
items.sort(key=lambda item: DuojiConfig.joint_number(item[0]))
```

你可以理解成：

```text
排序时，不直接比较整个 item。
排序时拿 item[0]，也就是 joint 名字。
再从名字里取数字。
最后按数字排。
```

### 4.5 joint_link_length(joint_data)

作用：

```text
读取单个关节的 link.length。
```

输入例子：

```python
joint_data = {
    "type": "tangential",
    "link": {
        "length": 0.15
    }
}
```

输出：

```text
0.15
```

执行逻辑：

```text
1. joint_data.get("link", {}) 取 link 字典。
2. link.get("length", 0.0) 取 length。
3. float(...) 转成小数。
```

如果 JSON 缺字段，就按 0.0 处理。

### 4.6 calculate_tangential_weights(arm_params)

作用：

```text
计算每个切向舵机的阻尼分配权重。
```

先看示例 JSON：

```text
joint1: axial,      length = 0.18
joint2: tangential, length = 0.15
joint3: tangential, length = 0.10
```

规则：

```text
axial 不参与分配。
tangential 参与分配。
某个切向关节的权重 = 从当前关节到末端的长度总和。
```

所以：

```text
joint2 权重 = joint2.length + joint3.length = 0.15 + 0.10 = 0.25
joint3 权重 = joint3.length                 = 0.10
```

又因为 servo_id 按 joint 顺序自然安排：

```text
joint1 -> servo 0
joint2 -> servo 1
joint3 -> servo 2
```

所以结果是：

```python
{
    1: 0.25,
    2: 0.10,
}
```

执行逻辑：

```text
1. ordered_joint_items() 得到排序后的关节列表。
2. 用 index 从 0 开始遍历每个关节。
3. 如果 type 不是 tangential，就 continue 跳过。
4. servo_id = index。
5. 从当前 index 加到最后一个关节，累计 length。
6. 如果权重大于 0，保存到 weights。
7. 返回 weights。
```

### 4.7 distribute_end_damping(total_damping, arm_params)

作用：

```text
把末端合阻尼按权重分配给各个切向舵机。
```

输入：

```text
total_damping: 末端合阻尼，例如 1000
arm_params: 机械臂 JSON 参数
```

输出例子：

```python
{
    1: 714.285714,
    2: 285.714285,
}
```

计算过程：

```text
weights = {1: 0.25, 2: 0.10}
sum_weight = 0.25 + 0.10 = 0.35

servo 1 = 1000 * 0.25 / 0.35 = 714.2857
servo 2 = 1000 * 0.10 / 0.35 = 285.7143
```

执行逻辑：

```text
1. calculate_tangential_weights() 得到 weights。
2. 把所有 weight 加起来。
3. 如果总权重 <= 0，返回空字典。
4. 对每个 servo_id 计算分配值。
5. 返回 result。
```

### 4.8 parse_int_list(raw_value)

作用：

```text
把 ROS 参数里的舵机 ID 列表解析成 int 列表。
```

它支持几种输入：

```python
3              -> [3]
[0, 1, 2]      -> [0, 1, 2]
"[0,1,2]"      -> [0, 1, 2]
None           -> []
```

为什么需要它？

ROS 参数可能来自命令行或 launch 文件。

有时你传的是字符串：

```bash
_servo_ids:="[0,1,2]"
```

Python 收到时可能是：

```python
"[0,1,2]"
```

所以要统一解析。

### 4.9 read_ros_config(rospy)

作用：

```text
从 ROS 参数服务器读取配置，整理成普通字典。
```

它读取这些参数：

```text
~port
~baudrate
~timeout
~servo_ids
~num_servos
~rate
~angle_topic
~current_topic
~read_current
~release_on_shutdown
~arm_config
~end_damping
```

如果 ROS 里没传参数，就用默认值。

默认机械臂 JSON 路径是：

```text
当前文件所在目录/config/example.json
```

### 4.10 load_arm_params(self)

作用：

```text
根据 self.values["arm_config"] 读取 JSON，并放入 self.values["arm_params"]。
```

执行逻辑：

```text
1. 从配置字典里取 arm_config。
2. 如果路径不是空字符串，就读取 JSON。
3. 把 JSON 内容写回 self.values["arm_params"]。
```

### 4.11 create_shared_state_from_values(servo_ids, num_servos)

作用：

```text
根据舵机 ID 列表和舵机数量创建 shared_state。
```

输入：

```python
servo_ids = [0, 1, 2]
num_servos = 3
```

输出：

```python
{
    "servo_ids": [0, 1, 2],
    "num_servos": 3,
    "servos": {
        0: {"id": 0, "angle": None, ...},
        1: {"id": 1, "angle": None, ...},
        2: {"id": 2, "angle": None, ...},
    },
    "damping_targets": {},
    "control_state": "IDLE",
    "last_error": "",
    "last_update": 0.0,
}
```

### 4.12 create_shared_state(self)

作用：

```text
从 self.values 里取 servo_ids 和 num_servos，再创建 shared_state。
```

它只是对 `create_shared_state_from_values()` 的一层封装。

## 5. RosWorker 详解

`RosWorker` 只做 ROS 发布。

它不打开串口，不调用舵机 SDK。

### 5.1 __init__()

保存这些对象：

```text
config
shared_state
state_lock
stop_event
rospy
```

`RosWorker` 后面发布话题时要用它们。

### 5.2 make_angle_array(shared_state)

作用：

```text
把 shared_state 里的舵机角度变成 ROS Float64MultiArray 需要的数组。
```

假设：

```python
num_servos = 4
servos = {
    0: {"angle": 10.0},
    2: {"angle": -5.0},
}
```

输出：

```python
[10.0, 0.0, -5.0, 0.0]
```

规则：

```text
数组下标 = servo_id
没有数据的位置填 0.0
angle 是 None 时也填 0.0
```

### 5.3 make_current_array(shared_state)

作用：

```text
把 shared_state 里的电流变成数组。
```

和角度类似。

输出数组的下标也是 servo_id。

### 5.4 copy_shared_state(shared_state, state_lock)

作用：

```text
复制一份 shared_state 快照给 ROS 线程使用。
```

为什么要复制？

因为如果 ROS 线程直接拿原始 shared_state 去发布，控制线程可能同时在修改它。

流程：

```text
1. 加锁。
2. 新建 copied 字典。
3. 复制 servo_ids、num_servos、damping_targets 等字段。
4. 对每个 servo_state 也复制一份小字典。
5. 返回 copied。
6. finally 里释放锁。
```

### 5.5 run()

这是 ROS 发布线程主循环。

执行顺序：

```text
1. 导入 Float64MultiArray。
2. 创建角度发布器 angle_pub。
3. 如果 read_current=True，创建电流发布器 current_pub。
4. 创建 rate，用来控制发布频率。
5. 进入 while 循环。
6. 每轮复制 shared_state。
7. 发布 /servo_angles。
8. 如果需要，发布 /servo_currents。
9. rate.sleep() 等待下一轮。
```

循环条件：

```python
while not self.stop_event.is_set() and not self.rospy.is_shutdown():
```

意思是：

```text
只要主线程没通知停止，并且 ROS 没关闭，就一直发布。
```

## 6. ControlWorker 详解

`ControlWorker` 是控制线程。

它负责：

```text
1. 打开串口。
2. 创建 UartServoManager。
3. 同步读取舵机状态。
4. 写 shared_state。
5. 执行控制状态机。
6. 退出时关闭串口。
```

### 6.1 __init__()

保存：

```text
config
shared_state
state_lock
stop_event
rospy
```

另外初始化：

```python
self.uart = None
self.manager = None
```

它们后面在 `run()` 里才会真正创建。

### 6.2 get_attr(obj, name, default_value)

作用：

```text
兼容对象和字典两种读取方式。
```

如果 `obj` 是字典：

```python
obj.get(name, default_value)
```

如果 `obj` 是对象：

```python
getattr(obj, name, default_value)
```

为什么需要？

官方 SDK 返回的数据版本可能不同。

有的版本返回对象：

```python
servo.angle_monitor
```

有的封装可能返回字典：

```python
servo["angle_monitor"]
```

这个方法就是做兼容。

### 6.3 build_servo_state(servo_id, monitor_data)

作用：

```text
把官方 SDK 的舵机状态数据，转成项目内部统一使用的普通字典。
```

输出格式：

```python
{
    "id": 1,
    "angle": 21.0,
    "current": 0.5,
    "voltage": 0.0,
    "power": 0.0,
    "temp": 0.0,
    "status": 0,
    "online": True,
    "stamp": 1710000000.0,
}
```

执行逻辑：

```text
1. 优先读 angle_monitor。
2. 如果没有，再读 angle。
3. current、voltage、power、temp、status 都用 get_attr 读取。
4. 如果 angle 不为空，online=True。
5. stamp 记录当前时间。
6. 返回统一字典。
```

### 6.4 get_servo_from_manager(manager, servo_id)

作用：

```text
从 manager.servos 里取某个舵机缓存对象。
```

为什么需要？

官方同步读取 API 示例里常见流程是：

```text
manager.send_sync_servo_monitor(servo_ids)
然后从 manager.servos[id] 取数据
```

所以如果 `send_sync_servo_monitor()` 没有直接返回数据，就从 `manager.servos` 里兜底取。

执行逻辑：

```text
1. 如果 manager 没有 servos 属性，返回 None。
2. 如果 servos 支持 .get()，就用 .get(servo_id)。
3. 否则尝试 servos[servo_id]。
4. 失败就返回 None。
```

### 6.5 open_servo_manager(config)

作用：

```text
打开串口，并创建官方 UartServoManager。
```

执行逻辑：

```text
1. import serial。
2. import fashionstar_uart_sdk as uservo。
3. 用 config["port"]、config["baudrate"] 等打开串口。
4. 创建 uservo.UartServoManager(uart)。
5. 返回 uart 和 manager。
```

注意：

这个方法只会在控制线程 `run()` 里调用。

ROS 线程不会调用它。

### 6.6 read_sync_monitor(manager, servo_ids)

作用：

```text
用官方同步读取 API 一次读取多个舵机状态。
```

输入：

```python
manager
servo_ids = [0, 1, 2]
```

输出：

```python
{
    0: {"id": 0, "angle": ..., "online": True, ...},
    1: {"id": 1, "angle": ..., "online": True, ...},
    2: {"id": 2, "angle": None, "online": False, ...},
}
```

执行逻辑：

```text
1. 调用 manager.send_sync_servo_monitor(servo_ids)。
2. 对每个 servo_id 单独处理。
3. 如果 API 返回 dict，就从 dict 里按 ID 取。
4. 如果 API 返回列表，就遍历列表找对应 ID。
5. 如果没找到，就从 manager.servos 兜底取。
6. 如果仍然没有数据，生成离线状态 online=False。
7. 如果有数据，调用 build_servo_state() 转成统一字典。
8. 返回 states。
```

为什么不逐个读？

同步读取一次读多个舵机，通常比逐个查询更整齐，也更接近官方同步 API 的用法。

### 6.7 update_shared_servo_state(shared_state, state_lock, new_states)

作用：

```text
把控制线程刚读到的舵机状态写入 shared_state。
```

执行逻辑：

```text
1. 加锁。
2. 遍历 new_states。
3. 把每个 servo_state 写入 shared_state["servos"]。
4. 更新 shared_state["last_update"]。
5. finally 释放锁。
```

### 6.8 set_control_state(shared_state, state_lock, new_state)

作用：

```text
切换状态机状态。
```

比如：

```python
set_control_state(shared_state, lock, STATE_ERROR)
```

会把：

```python
shared_state["control_state"]
```

改成：

```python
"ERROR"
```

### 6.9 handle_idle(manager)

当前逻辑：

```text
什么都不做。
```

预留用途：

```text
待机、安全释放、等待命令。
```

### 6.10 handle_manual(manager)

当前逻辑：

```text
什么都不做。
```

预留用途：

```text
单关节调试、手动目标角控制。
```

### 6.11 handle_plan(manager)

当前真正有逻辑的方法。

作用：

```text
在 PLAN 状态下，把末端合阻尼分配到切向舵机。
```

执行逻辑：

```text
1. 从 config 里取 arm_params。
2. 如果 arm_params 是 None，就直接 return。
3. 调用 DuojiConfig.distribute_end_damping()。
4. 得到 damping_targets。
5. 加锁。
6. 写入 shared_state["damping_targets"]。
7. 释放锁。
```

示例：

```python
shared_state["damping_targets"] = {
    1: 714.285714,
    2: 285.714285,
}
```

注意：

这里还没有把阻尼值下发到硬件。

它只是把结果算出来并保存。

后续你可以在 `MOVE` 或其他状态里决定怎么用它。

### 6.12 handle_move(manager)

当前逻辑：

```text
什么都不做。
```

预留用途：

```text
正式运动控制、角度下发、阻尼控制策略。
```

### 6.13 handle_hold(manager)

当前逻辑：

```text
什么都不做。
```

预留用途：

```text
位置保持、锁定当前状态。
```

### 6.14 handle_error(manager)

当前逻辑：

```text
什么都不做。
```

预留用途：

```text
错误保护、释放舵机、等待人工复位。
```

### 6.15 run_state_machine_once(manager)

作用：

```text
执行一轮状态机。
```

执行逻辑：

```text
1. 加锁读取 shared_state["control_state"]。
2. 释放锁。
3. 根据状态名进入对应 handle_xxx()。
4. 如果状态名未知，切到 ERROR。
```

为什么读完状态就释放锁？

因为 `handle_xxx()` 里可能会做控制计算。

如果一直拿着锁，ROS 线程就读不到 shared_state。

所以这里只短暂加锁读一个字符串。

状态分发关系：

```text
IDLE   -> handle_idle()
MANUAL -> handle_manual()
PLAN   -> handle_plan()
MOVE   -> handle_move()
HOLD   -> handle_hold()
ERROR  -> handle_error()
其他   -> set_control_state(ERROR)
```

### 6.16 release_servos(manager)

作用：

```text
退出时释放所有舵机。
```

它遍历：

```python
self.config["servo_ids"]
```

然后调用：

```python
manager.stop_on_control_mode(int(servo_id), method=0x10, power=0)
```

这个写法来自官方示例。

如果某个舵机释放失败，代码会 `pass`，继续释放下一个。

### 6.17 run()

这是控制线程主循环。

执行顺序：

```text
1. 根据 rate 计算每轮 sleep 时间。
2. 打开串口和 UartServoManager。
3. 进入 while 循环。
4. 同步读取所有舵机状态。
5. 写入 shared_state。
6. 执行一轮状态机。
7. 如果本轮报错，记录 last_error，并切到 ERROR。
8. sleep 一小段时间。
9. 退出时根据配置释放舵机。
10. 关闭串口。
```

循环条件：

```python
while not self.stop_event.is_set() and not self.rospy.is_shutdown():
```

意思是：

```text
主线程没喊停，并且 ROS 没关闭，就一直循环。
```

## 7. start_thread(name, target, args)

作用：

```text
启动一个后台线程。
```

输入：

```text
name: 线程名字
target: 线程启动后要执行的函数或方法
args: 传给 target 的参数元组
```

代码：

```python
thread = threading.Thread(name=name, target=target, args=args)
thread.daemon = True
thread.start()
return thread
```

解释：

```text
threading.Thread(...)
  创建线程对象。

thread.daemon = True
  主程序退出时，这个后台线程不会强行卡住整个程序。

thread.start()
  真正启动线程。

return thread
  返回线程对象，main 后面可以 join 等它结束。
```

## 8. 数据流总图

最核心的数据流是：

```text
config/example.json
      |
      v
DuojiConfig 读取 JSON 和 ROS 参数
      |
      v
config 字典 + shared_state 字典
      |
      +--------------------+
      |                    |
      v                    v
RosWorker              ControlWorker
发布 ROS 话题          打开串口和舵机 SDK
      |                    |
      |                    v
      |              send_sync_servo_monitor()
      |                    |
      |                    v
      |              统一成 servo_state 字典
      |                    |
      |                    v
      |              写 shared_state["servos"]
      |                    |
      |                    v
      |              执行状态机
      |                    |
      |                    v
      |              PLAN 状态计算 damping_targets
      |                    |
      +---------读 shared_state 快照---------+
```

## 9. shared_state 数据流

### 9.1 初始状态

刚启动时：

```python
shared_state["servos"][0]["angle"] = None
shared_state["control_state"] = "IDLE"
shared_state["damping_targets"] = {}
```

### 9.2 控制线程更新舵机状态

控制线程每轮：

```text
read_sync_monitor()
  -> 得到 new_states
update_shared_servo_state()
  -> 写入 shared_state["servos"]
```

### 9.3 ROS 线程发布

ROS 线程每轮：

```text
copy_shared_state()
  -> 得到快照 state_copy
make_angle_array(state_copy)
  -> 得到角度数组
publish()
  -> 发布 /servo_angles
```

### 9.4 PLAN 状态更新阻尼分配

如果：

```python
shared_state["control_state"] = "PLAN"
```

那么控制线程的状态机会进入：

```python
handle_plan()
```

然后写：

```python
shared_state["damping_targets"] = {
    1: 714.285714,
    2: 285.714285,
}
```

## 10. 末端阻尼分配完整例子

配置文件：

```json
{
  "dof": 3,
  "joints": {
    "joint1": {"type": "axial", "link": {"length": 0.18}},
    "joint2": {"type": "tangential", "link": {"length": 0.15}},
    "joint3": {"type": "tangential", "link": {"length": 0.10}}
  }
}
```

假设：

```python
end_damping = 1000
```

步骤：

```text
1. ordered_joint_items() 得到 joint1、joint2、joint3。
2. joint1 是 axial，跳过。
3. joint2 是 tangential，servo_id = 1。
4. joint2 权重 = 0.15 + 0.10 = 0.25。
5. joint3 是 tangential，servo_id = 2。
6. joint3 权重 = 0.10。
7. 总权重 = 0.25 + 0.10 = 0.35。
8. servo 1 阻尼 = 1000 * 0.25 / 0.35 = 714.2857。
9. servo 2 阻尼 = 1000 * 0.10 / 0.35 = 285.7143。
```

结果：

```python
{
    1: 714.285714,
    2: 285.714285,
}
```

## 11. 如果你要继续写控制逻辑，先看哪里

最常改的位置：

```text
ControlWorker.handle_idle()
ControlWorker.handle_manual()
ControlWorker.handle_plan()
ControlWorker.handle_move()
ControlWorker.handle_hold()
ControlWorker.handle_error()
```

如果你要新增 ROS 参数：

```text
DuojiConfig.read_ros_config()
```

如果你要新增 JSON 字段读取：

```text
DuojiConfig.load_arm_params_file()
DuojiConfig 里的机械臂相关方法
```

如果你要改变 ROS 发布内容：

```text
RosWorker.run()
RosWorker.make_angle_array()
RosWorker.make_current_array()
```

如果你要改变舵机读取：

```text
ControlWorker.read_sync_monitor()
ControlWorker.build_servo_state()
```

## 12. 现在代码还没做什么

当前还没有做：

```text
1. 零点标定。
2. 重力补偿。
3. 真正机械臂位置逆解。
4. 把 damping_targets 下发到舵机硬件。
5. ROS 订阅控制命令。
```

当前已经做了：

```text
1. 单文件结构。
2. 三类职责划分。
3. ROS 发布线程框架。
4. 控制线程框架。
5. 官方同步读取 API 路径。
6. shared_state 数据交换。
7. PLAN 状态下末端合阻尼到切向舵机的分配。
```

## 13. 一句话记住整个程序

```text
main 负责启动；
DuojiConfig 负责读配置；
ControlWorker 负责读硬件和算状态机；
RosWorker 负责把最新状态发到 ROS。
```

