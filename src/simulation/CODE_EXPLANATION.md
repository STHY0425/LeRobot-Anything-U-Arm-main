# teleop_sim_trigger_test.py 代码详解

## 一、整体架构概览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           主程序 (Main)                                  │
│                     创建 ServoTeleoperatorSim 类                          │
│                         启动 run() 方法                                  │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      ServoTeleoperatorSim 类                             │
├─────────────────────────────────────────────────────────────────────────┤
│  成员变量:                                                               │
│  - ser: 串口对象 (连接中灵舵机)                                          │
│  - env: ManiSkill 仿真环境                                               │
│  - trigger_state: 'teaching'/'locked'/'resuming' (三种状态)              │
│  - arm_pos_queue: 队列 (生产者-消费者通信)                               │
│  - lock: 线程锁 (保护共享状态)                                           │
│  - stop_event: 停止事件 (控制线程结束)                                    │
├─────────────────────────────────────────────────────────────────────────┤
│  三个线程:                                                               │
│  1. produce_thread → _angle_stream_loop()  [舵机读取 + Trigger逻辑]       │
│  2. consume_thread → _pose_consumer_loop() [仿真控制]                    │
│  3. keyboard_thread → _keyboard_loop()     [键盘监听]                    │
└─────────────────────────────────────────────────────────────────────────┘
```

## 二、详细流程

### 1. 初始化流程 (`__init__`)

```python
def __init__(self, scene, robot_uids, serial_port):
    # 1. 初始化串口连接
    self.ser = serial.Serial(port, 115200, timeout=0.1)
    
    # 2. 初始化配置
    self.trigger_angle = 101.0      # 触发角度
    self.trigger_tolerance = 15.0   # 容差 ±15度
    self.servo6_center = 135.0      # 复位角度
    self.trigger_state = 'teaching' # 初始状态
    
    # 3. 舵机零点校准
    self._init_servos()  # 读取7个舵机当前角度作为零点
    
    # 4. 创建仿真环境
    self.env = gym.make(scene, robot_uids=robot_uids, ...)
    
    # 5. 创建三个线程 (daemon=True 表示主线程结束时自动结束)
    self.produce_thread = Thread(target=self._angle_stream_loop, daemon=True)
    self.consume_thread = Thread(target=self._pose_consumer_loop, daemon=True)
    self.keyboard_thread = Thread(target=self._keyboard_loop, daemon=True)
```

### 2. 三个核心线程

#### 线程1: 舵机读取线程 (`_angle_stream_loop`)

**作用**: 持续读取舵机角度 + 处理Trigger逻辑

```python
def _angle_stream_loop(self):
    while not self.stop_event.is_set():
        # 获取当前状态 (加锁保护)
        with self.lock:
            state = self.trigger_state
        
        # 状态1: LOCKED - 持续发送位置保持舵机
        if state == 'locked':
            self._hold_position(self.locked_positions)
            time.sleep(0.02)  # 50Hz
            continue
        
        # 状态2: RESUMING - 复位中，不读取角度
        if state == 'resuming':
            time.sleep(0.05)
            continue
        
        # 状态3: TEACHING - 正常读取 + 检测触发
        # 1. 读取7个舵机角度
        for i in range(7):
            angle = self._read_servo_angle(i)
            arm_pos[i] = np.radians(angle - zero_angle)
        
        # 2. 放入队列 (供仿真线程使用)
        self.arm_pos_queue.put(arm_pos)
        
        # 3. 检测6号舵机是否触发 (101°±15°)
        servo6_angle = self._read_servo_angle(6)
        if abs(servo6_angle - 101.0) < 15.0:
            self._do_trigger_lock()  # 触发锁定
```

#### 线程2: 仿真控制线程 (`_pose_consumer_loop`)

**作用**: 从队列获取角度，控制仿真机械臂

```python
def _pose_consumer_loop(self):
    while not self.stop_event.is_set():
        # 从队列获取角度数据
        pose = self.arm_pos_queue.get_nowait()
        
        if pose is not None:
            # 转换为仿真动作
            action = self._convert_pose(pose)
            
            # 执行仿真步进
            self.env.step(action)
            self.env.render()
        
        time.sleep(0.02)  # 50Hz
```

**注意**: 这个线程独立运行，不受Trigger状态影响，仿真全程不停！

#### 线程3: 键盘监听线程 (`_keyboard_loop`)

**作用**: 监听Enter键，用于解锁

```python
def _keyboard_loop(self):
    # 设置终端为非阻塞模式
    tty.setcbreak(sys.stdin.fileno())
    
    while not self.stop_event.is_set():
        # 只有在 locked 状态才监听键盘
        if self.trigger_state == 'locked':
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '\n':  # Enter键
                    self._do_trigger_unlock()  # 执行解锁
        
        time.sleep(0.05)
```

### 3. 状态机 (三种状态)

```
┌─────────────┐       触发(101°)        ┌─────────────┐
│  TEACHING   │ ──────────────────────► │   LOCKED    │
│  (正常示教)  │                         │  (锁定保持)  │
└─────────────┘                         └──────┬──────┘
     ▲                                         │
     │                                         │ Enter键
     │                                         ▼
     │                                  ┌─────────────┐
     │         复位完成(到135°)          │  RESUMING   │
     └─────────────────────────────────│  (复位中)   │
                                       └─────────────┘
```

| 状态 | 含义 | 行为 |
|------|------|------|
| **TEACHING** | 正常示教 | 读取舵机角度，检测触发条件 |
| **LOCKED** | 锁定 | 持续发送位置命令保持舵机不动，监听Enter键 |
| **RESUMING** | 复位中 | 舵机6回到135°，期间不检测触发 |

### 4. 关键函数详解

#### `_do_trigger_lock()` - 锁定函数

```python
def _do_trigger_lock(self):
    # 1. 读取当前所有舵机的PWM位置
    locked_pwm = []
    for i in range(7):
        angle = self._read_servo_angle(i)
        pwm = self._angle_to_pwm(angle)
        locked_pwm.append(pwm)
    
    # 2. 保存位置并切换到 locked 状态
    self.locked_positions = locked_pwm
    self.trigger_state = 'locked'
```

**软件锁定原理**: 不是真的"锁定"舵机，而是持续发送相同位置命令，让舵机保持在该位置。

#### `_do_trigger_unlock()` - 解锁函数

```python
def _do_trigger_unlock(self):
    # Step 1: 进入 resuming 状态 (禁用触发检测)
    self.trigger_state = 'resuming'
    
    # Step 2: 移动舵机6到135°
    self._move_servo(6, 135.0, move_time=1000)
    time.sleep(1.5)  # 等待移动完成
    
    # Step 3: 释放力矩
    self._release_servos()
    
    # Step 4: 回到 teaching 状态
    self.trigger_state = 'teaching'
```

#### `_hold_position()` - 保持位置

```python
def _hold_position(self, positions_pwm):
    """每20ms发送一次位置命令，让舵机保持不动"""
    for i, pwm in enumerate(positions_pwm):
        # P{pwm}T{time} - 目标PWM和移动时间
        cmd = f'#{i:03d}P{pwm:04d}T0050!'  # 50ms移动时间
        self.ser.write(cmd.encode('ascii'))
```

### 5. 中灵舵机通信协议

```
命令格式: #{ID}{CMD}{DATA}!

示例:
- #006PRAD!    → 读取6号舵机角度
- #006P1500!   → 设置6号舵机PWM=1500
- #006PULK!    → 释放6号舵机力矩
- #006P1500T1000! → 1秒内移动到PWM=1500

响应格式:
- P{xxxx}      → 当前PWM值 (如 P1500)
```

PWM与角度转换:
```
PWM 500-2500 对应 角度 0°-270°
角度 = (PWM - 500) / 2000 * 270
PWM = 500 + (角度 / 270) * 2000
```

## 三、线程间通信

```
┌──────────────────┐      ┌──────────────┐      ┌──────────────────┐
│  舵机读取线程     │ ───► │  arm_pos_queue│ ───► │   仿真控制线程    │
│                  │      │   (队列)      │      │                  │
│ - 读取舵机角度   │      │              │      │ - 获取角度       │
│ - 检测触发       │      │  线程安全    │      │ - 更新仿真       │
└──────────────────┘      │  缓冲数据    │      └──────────────────┘
                          └──────────────┘
                                  ▲
                                  │
                          ┌──────────────┐
                          │   键盘线程    │
                          │              │
                          │ - 监听Enter  │
                          │ - 触发解锁   │
                          └──────────────┘
```

## 四、关键问题解答

### Q1: 为什么要用三个线程？

- **舵机读取线程**: 必须20Hz持续读取，否则舵机数据会丢失
- **仿真线程**: 独立运行，保证仿真画面流畅
- **键盘线程**: 非阻塞监听键盘，不干扰其他线程

### Q2: 什么是"软件锁定"？

因为中灵舵机的 `PLOK` 命令不起作用，所以用软件方式：
- 记录当前所有舵机的PWM位置
- 每20ms重复发送这些位置命令
- 舵机收到命令后会努力保持在该位置

### Q3: 为什么要有 RESUMING 状态？

防止解锁时重复触发：
- 解锁时舵机6从101°回到135°
- 如果不禁用检测，经过101°时会再次触发
- RESUMING状态期间跳过触发检测

## 五、调试技巧

```bash
# 1. 查看实时输出
python teleop_sim_trigger_test.py --robot rm65b

# 2. 关键输出信息
[TRIGGER] Servo6: 98.5° (target: 101.0°)   # 当前角度
[TRIGGER] *** TRIGGERED at 101.2° ***      # 触发
[TRIGGER] LOCKED - Press Enter to unlock   # 锁定状态
[TRIGGER] ========== UNLOCK START ========== # 开始解锁
[TRIGGER] ========== UNLOCK COMPLETE ======= # 解锁完成
```

## 六、修改参数

在 `__init__` 中修改:
```python
self.trigger_angle = 101.0      # 修改触发角度
self.trigger_tolerance = 15.0   # 修改容差范围
self.servo6_center = 135.0      # 修改复位角度
```
