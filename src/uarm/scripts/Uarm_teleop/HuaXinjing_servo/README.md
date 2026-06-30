# LeRobot-Anything-U-Arm — 华馨京舵机 3D 动态阻尼控制节点

## 概述

基于 ROS1 的机械臂舵机阻尼控制节点，核心功能是**非对称三档动态阻尼**：

- 通过 3D DH 运动学实时计算末端位置和速度
- 根据末端竖直速度 vz 判断顺逆重力方向
- 切向关节按雅可比列范数权重分发三档固定阻尼预算
- 轴向关节固定基础阻尼，不参与分发
- 支持 3-7 轴任意 axial/tangential 构型，JSON 驱动，不硬编码

## 文件结构

```
LeRobot-Anything-U-Arm-main/
├── hxj_duoji_node.py          # 唯一核心源文件
├── config/
│   └── example.json           # 机械臂参数配置
├── tests/
│   └── test_hxj_duoji_node.py # 单元测试（28 个）
├── fashionstar_servo_python_sdk_api.md  # 舵机官方 SDK 文档
├── ONBOARDING.md              # 新 agent 上手指南
└── README.md                  # 本文件
```

## 架构

单文件三职责：

| 类 | 职责 |
|---|------|
| `ServoConfig` | 读 JSON 机械臂参数 + 串口配置，创建共享状态 |
| `RosPublisher` | ROS 发布线程，只读共享状态发话题，不碰串口 |
| `Controller` | 控制线程，唯一访问串口，3D 运动学 + 三档动态阻尼 |

线程模型：
```
main 主线程（空转等退出）
├── ros_thread — 50Hz 读共享状态 → 发 ROS 话题
└── control_thread — 50Hz 读舵机 → 3D FK → Jacobian → 三档阻尼 → 下发
```

## 核心算法

### 3D DH 运动学

```
所有关节角度 → 3D DH 表（含 α 扭转角）
            → 正运动学 → 末端 (px, py, pz)
            → 3D 雅可比 J (3×N_total)
            → 角度差分 → 关节角速度 ω
            → 末端速度 v = J·ω → (vx, vy, vz)
```

- 轴向关节 α=π/2（平面偏转），切向 α=0（同平面）
- 所有关节参与运动学，确保底座旋转后末端位置准确

### 三档动态阻尼

| 档位 | 触发条件 | 预算 | 手感 |
|------|---------|------|------|
| 低档 | vz > threshold（逆重力/上抬） | 600 mW | 轻盈 |
| 中档 | \|vz\| ≤ threshold（水平/静止） | 1000 mW | 正常 |
| 高档 | vz < -threshold（顺重力/下坠） | 1500 mW | 托住 |

切向关节按 3D 雅可比列范数权重分发所选预算，受单舵机 1000mW 硬上限截断。

轴向关节固定 `base_damping_power=500mW`，不参与三档。

### 阻尼分发保证

```
J_3d (3×N_total) → extract_tangential_jacobian → J_tan (3×N_tan)
                                               → distribute_damping_3d → 只分给切向
```

轴向列在运动学里"通过"了，在分发时被"过滤"掉了。

## 配置

### JSON (`config/example.json`)

```json
{
    "dof": 3,
    "joints": {
        "joint1": {
            "type": "axial",
            "link": { "length": 0.18, "twist": 1.5708, "offset": 0.0 }
        },
        "joint2": {
            "type": "tangential",
            "link": { "length": 0.15, "twist": 0.0, "offset": 0.0 }
        }
    }
}
```

- `type`: axial / tangential，决定阻尼处理方式
- `twist`: DH α 角（弧度），缺省时 axial→π/2，tangential→0
- `offset`: DH d 偏移，缺省 0
- `dof` 决定 servo_ids，不硬编码

### DEFAULTS 字典

所有阻尼参数默认值集中在 `hxj_duoji_node.py` 顶部的 `DEFAULTS` 字典：

```python
DEFAULTS = {
    "end_damping": 1000,
    "end_damping_low": 600,      # 低档
    "end_damping_mid": 1000,     # 中档
    "end_damping_high": 1500,    # 高档
    "base_damping_power": 500,   # 轴向固定
    "max_damping_power": 1000,   # 单舵机上限
    "vz_threshold": 0.01,        # 顺逆重力阈值 (m/s)
}
```

调试时只改这里，全局生效。

## 状态机

| 状态 | 用途 |
|------|------|
| IDLE | 空闲，不下发命令 |
| HOLD | 阻尼控制（核心），3D 三档动态阻尼 |
| LOCKED | 锁死，所有舵机保持锁力（`stop_on_control_mode(method=0x11)`） |
| ERROR | 停机释放 |

LOCKED 触发通过 `lock_requested` flag，后续接外部传感器。

## SDK API

| API | 用途 |
|-----|------|
| `UartServoManager(uart)` | 创建管理器 |
| `send_sync_servo_monitor(ids)` | 同步读取所有舵机状态 |
| `set_damping(id, power)` | 阻尼模式，power 单位 mW |
| `stop_on_control_mode(id, method, power)` | 释放(0x10)/锁力(0x11) |
| `manager.servos[id].angle_monitor` | 读缓存角度 |

## 遥测话题

除舵机硬件状态外，节点还发布 3 个控制遥测话题（均在 HOLD 状态下更新，其他状态清零）：

| 话题名 | 类型 | 数据 | 语义 |
|--------|------|------|------|
| `/servo_end_velocity` | `Float64MultiArray` | `[vx, vy, vz]` | 末端 3D 速度 (m/s)，由 J·ω 计算 |
| `/servo_damping_mode` | `Float64MultiArray` | `[mode]` | 阻尼档位码：`0=非HOLD, 1=low, 2=mid, 3=high` |
| `/servo_damping_powers` | `Float64MultiArray` | `[pw₀, pw₁, ...]` | 代码下发的目标阻尼功率 (mW)，按 servo_id 索引 |

> `/servo_damping_powers` 是控制指令的记录，与硬件反馈的 `/servo_powers` 语义不同，可用于"目标 vs 实际"对比。

## 运行

```bash
# 测试
docker exec ros1-noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /root/ros1_ws/LRrobot/src/LeRobot-Anything-U-Arm-main && PYTHONPATH=.:$PYTHONPATH python3 -m unittest tests.test_hxj_duoji_node -v'

# 运行节点（需要 ROS master + 串口）
rosrun hxj_duoji_node hxj_duoji_node.py
```
