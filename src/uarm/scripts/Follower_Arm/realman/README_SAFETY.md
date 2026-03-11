# RealMan (瑞尔曼)机械臂安全遥操控制

本项目为瑞尔曼(RealMan)机械臂提供**企业级安全遥操作**功能，集成多种数据错误处理机制与安全锁功能。

## 🧪 无实物测试

没有实际机械臂？可以先在仿真环境中验证代码：

```bash
cd test/
./test_teleop.sh
```

支持的测试场景：
- ✅ 基础遥操功能
- ✅ 速度限制验证
- ✅ 错误处理与急停
- ✅ 通信超时保护

详见 [test/README_TEST.md](test/README_TEST.md)

## 🔒 安全特性一览

| 安全功能 | 说明 | 参考来源 |
|---------|------|---------|
| **速度限制** | 限制关节最大运动速度，防止超速 | ARX `max_joint_vel` |
| **关节限位** | 软件层面的关节角度限制 | RealMan SDK `JOINT_LIMIT_ERR` |
| **错误监控** | 实时监测机械臂错误码并自动响应 | RealMan SDK |
| **通信超时检测** | 监控通信状态，超时自动急停 | RealMan SDK |
| **使能控制** | 必须使能后才能运动 | xArm `motion_enable` |
| **急停功能** | 多级急停（软件+硬件） | RealMan SDK `Emergency_Stop` |
| **碰撞检测** | 检测碰撞并自动停止 | RealMan SDK `ARM_ERR_CRASH` |

## 📁 文件说明

### 核心文件
| 文件 | 说明 | 安全等级 |
|------|------|---------|
| `realman_teleop_safe.py` | **安全遥操主节点**（推荐） | ⭐⭐⭐⭐⭐ |
| `realman_safety_monitor.py` | 安全监控节点 | ⭐⭐⭐⭐⭐ |
| `realman_teleop.py` | 基础遥操节点（无安全功能） | ⭐⭐ |
| `realman_pub.py` | 状态发布节点 | ⭐⭐⭐ |

### 配置文件
| 文件 | 说明 |
|------|------|
| `config/realman_safe_config.yaml` | 安全参数配置文件 |

## 🚀 快速启动（安全模式）

### 方式1：一键启动（推荐）
```bash
cd /workspace/src/uarm/scripts/Follower_Arm/realman
./run_realman_safe.sh
```

### 方式2：分步启动

**终端1 - ROS Master**
```bash
roscore
```

**终端2 - 瑞尔曼驱动**
```bash
cd /workspace/src/uarm/scripts/Follower_Arm/realman
source catkin_ws/devel/setup.bash
roslaunch rm_driver rm_65_driver.launch
```

**终端3 - 安全遥操节点**
```bash
cd /workspace/src/uarm/scripts/Follower_Arm/realman
source catkin_ws/devel/setup.bash

# 使用配置文件启动
rosrun uarm realman_teleop_safe.py _config:=config/realman_safe_config.yaml

# 或手动指定参数
rosrun uarm realman_teleop_safe.py \
    _arm_model:="RM_65" \
    _arm_ip:="192.168.1.18" \
    _max_joint_speed:=30.0 \
    _enable_emergency_stop:=true
```

**终端4 - 安全监控（可选）**
```bash
rosrun uarm realman_safety_monitor.py
```

## 🎮 安全控制接口

### 使能控制（必须）
```bash
# 使能机械臂（开始遥操）
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"

# 失能机械臂（停止遥操）
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: false}"
```

### 急停控制
```bash
# 触发急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "{data: true}"

# 复位急停（在排除故障后）
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "{data: false}"
```

### 直接控制瑞尔曼驱动
```bash
# 急停（立即停止）
rostopic pub /rm_driver/Stop rm_msgs/Stop "{stop_mode: 0}"

# 缓停
rostopic pub /rm_driver/Stop rm_msgs/Stop "{stop_mode: 1}"
```

## ⚙️ 配置参数

### 核心参数
| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `arm_model` | "RM_65" | 机械臂型号（RM_65/RM_75/ECO_65） |
| `arm_ip` | "192.168.1.18" | 机械臂IP地址 |
| `dof` | 6 | 自由度 |
| `init_qpos` | [0, -20, -90, 0, 90, 0] | 初始关节角度（度） |

### 安全参数
| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `max_joint_speed` | 30.0 | 最大关节速度（度/秒） |
| `max_gripper_speed` | 200.0 | 夹爪最大速度（位置/秒） |
| `enable_emergency_stop` | true | 启用急停功能 |
| `comm_timeout` | 0.5 | 通信超时时间（秒） |
| `error_check_interval` | 0.1 | 错误检查间隔（秒） |

### 关节映射参数
| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `joint_scale` | [1,1,1,1,1,1] | 关节角度缩放 |
| `joint_invert` | [1,1,-1,1,1,1] | 关节方向反转（1或-1） |

## 🔍 监控话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/realman_teleop/status` | Bool | 遥操状态（是否运行中） |
| `/realman_teleop/emergency` | Bool | 急停状态 |
| `/realman_monitor/status` | String | 监控节点状态信息 |
| `/rm_driver/Arm_Current_State` | Arm_Current_State | 机械臂当前状态（含错误码） |

## ⚠️ 错误码对照表

| 错误码 | 级别 | 说明 | 自动处理 |
|--------|------|------|---------|
| 0x100D | 🚨严重 | 机械臂碰撞 | 自动急停 |
| 0x1007 | 🚨严重 | 关节超速 | 自动急停 |
| 0x1002 | ⚠️警告 | 目标角度越限 | 自动裁剪到限位内 |
| 0x1001 | ❌错误 | 关节通信异常 | 记录并提示 |
| 0x1010 | ❌错误 | 关节掉使能 | 记录并提示 |

## 🔄 安全机制工作流程

```
┌─────────────────────────────────────────────────────────────┐
│                    安全遥操工作流程                         │
└─────────────────────────────────────────────────────────────┘

1. 启动阶段
   ├─ 加载配置文件
   ├─ 初始化ROS节点
   ├─ 订阅/发布话题
   └─ 等待使能信号 ←── 必须执行 enable 才能继续

2. 运行阶段（循环）
   ├─ 接收主臂角度 ←── /servo_angles
   ├─ 应用映射和缩放
   ├─ 关节限位检查 ←── 超出限位则裁剪
   ├─ 速度限制计算 ←── ARX式速度限制
   ├─ 安全检查 ←── 通信/错误/碰撞检测
   └─ 发布控制命令 ←── 如果全部通过

3. 异常处理
   ├─ 通信超时 → 自动急停
   ├─ 碰撞检测 → 自动急停
   ├─ 关节超速 → 自动急停
   └─ 一般错误 → 记录并提示

4. 停止阶段
   ├─ 失能机械臂
   ├─ 发送停止命令
   └─ 清理资源
```

## 🛠️ 故障排除

### 机械臂不响应主臂动作
```bash
# 检查1: 确认已使能
rostopic echo /realman_teleop/status
# 应该显示: data: True

# 检查2: 确认无急停
rostopic echo /realman_teleop/emergency
# 应该显示: data: False

# 检查3: 检查错误码
rostopic echo /rm_driver/Arm_Current_State/err
```

### 运动方向相反
编辑 `config/realman_safe_config.yaml`:
```yaml
mapping:
  joint_invert: [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]  # 将对应值改为-1.0
```

### 速度过快/过慢
```bash
# 调整最大速度
rosrun uarm realman_teleop_safe.py _max_joint_speed:=20.0
```

## 📊 与其他机械臂对比

| 安全功能 | ARX | xArm | Dobot | **RealMan(本项目)** |
|---------|-----|------|-------|---------------------|
| 速度限制 | ✅ | ❌ | ❌ | ✅ 参考ARX实现 |
| 关节限位 | ❌ | ❌ | ❌ | ✅ 软硬件双保险 |
| 错误监控 | ❌ | ❌ | ❌ | ✅ 完整错误码支持 |
| 碰撞检测 | ❌ | ❌ | ❌ | ✅ 自动急停 |
| 通信超时 | ❌ | ❌ | ❌ | ✅ 自动保护 |
| 使能控制 | ❌ | ✅ | ❌ | ✅ 必须使能 |
| 急停功能 | ❌ | ❌ | ❌ | ✅ 多级急停 |

## 📝 参考实现

- **ARX速度限制**: `Follower_Arm/ARX/arx_teleop.py` (第98-112行)
- **xArm使能控制**: `Follower_Arm/xarm/servo2xarm.py` (第24-29行)
- **瑞尔曼错误码**: `realman/catkin_ws/src/rm_robot/rm_driver/src/rm_robot.h` (第214-254行)

## ⚠️ 安全须知

> **警告**: 即使有此安全系统，操作机械臂时仍需：
> 1. 确保机械臂周围无人员和障碍物
> 2. 首次使用先降低速度进行测试
> 3. 随时准备物理急停按钮
> 4. 定期检查关节限位配置

---

**最后更新**: 2025-03-10
