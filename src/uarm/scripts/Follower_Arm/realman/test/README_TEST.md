# RealMan 遥操无实物测试指南

本指南介绍如何在没有实际瑞尔曼机械臂的情况下，验证遥操代码的正确性。

## 🎯 测试方案概述

```
┌─────────────────────────────────────────────────────────────┐
│                    无实物测试架构                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌──────────────┐         ┌──────────────┐               │
│   │ MockMasterArm│         │ MockRMDriver │               │
│   │   (模拟主臂)  │ ──────→ │  (模拟从臂)  │               │
│   │              │         │              │               │
│   │ /servo_angles│         │ /Arm_Current │               │
│   │              │         │    _State    │               │
│   └──────────────┘         └──────┬───────┘               │
│          │                        │                        │
│          │                        │                        │
│          └──────────┬─────────────┘                        │
│                     │                                      │
│          ┌──────────▼──────────┐                          │
│          │ realman_teleop_safe │                          │
│          │     (被测系统)      │                          │
│          │                     │                          │
│          │  速度限制/错误处理  │                          │
│          │  使能控制/安全监控  │                          │
│          └──────────┬──────────┘                          │
│                     │                                      │
│          ┌──────────▼──────────┐                          │
│          │realman_safety_monitor│                         │
│          │   (监控与验证)      │                          │
│          └─────────────────────┘                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 📁 测试文件说明

| 文件 | 说明 |
|------|------|
| `mock_rm_driver.py` | 模拟瑞尔曼驱动，接收控制命令并模拟机械臂运动 |
| `mock_master_arm.py` | 模拟主臂设备，输出舵机角度 |
| `test_teleop.sh` | 一键测试脚本，包含多种测试场景 |
| `README_TEST.md` | 本测试指南 |

## 🚀 快速开始

### 方式1：一键测试（推荐）

```bash
cd /workspace/src/uarm/scripts/Follower_Arm/realman/test

# 赋予执行权限
chmod +x test_teleop.sh

# 运行测试
./test_teleop.sh
```

然后选择测试场景：
- `1)` 基础遥操测试
- `2)` 安全功能测试
- `3)` 错误处理测试
- `4)` 通信超时测试

### 方式2：手动启动

**终端1 - ROS Master**
```bash
roscore
```

**终端2 - 模拟驱动**
```bash
cd /workspace/src/uarm/scripts/Follower_Arm/realman/test
source ../catkin_ws/devel/setup.bash
rosrun uarm mock_rm_driver.py
```

**终端3 - 模拟主臂**
```bash
source ../catkin_ws/devel/setup.bash

# 正弦运动模式
rosrun uarm mock_master_arm.py _mode:="sine" _amplitude:=30.0

# 或随机运动模式
rosrun uarm mock_master_arm.py _mode:="random"

# 或静止模式
rosrun uarm mock_master_arm.py _mode:="static"
```

**终端4 - 遥操节点**
```bash
source ../catkin_ws/devel/setup.bash
rosrun uarm realman_teleop_safe.py
```

**终端5 - 使能机械臂**
```bash
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
```

## 🧪 测试场景详解

### 测试1：基础遥操功能

**目的**: 验证遥操基本功能是否正常

**步骤**:
```bash
./test_teleop.sh
# 选择 1
```

**预期结果**:
- `/servo_angles` 话题有正弦变化的数据
- `/rm_driver/Arm_Current_State/joint` 跟随变化
- 从臂角度平滑跟踪主臂

**可视化**:
```bash
# 查看话题数据
rostopic echo /servo_angles
rostopic echo /rm_driver/Arm_Current_State/joint

# 图形化显示
rqt_plot /servo_angles/data[0] /rm_driver/Arm_Current_State/joint[0]
```

### 测试2：速度限制功能

**目的**: 验证ARX式速度限制是否生效

**步骤**:
```bash
# 启动模拟环境
rosrun uarm mock_rm_driver.py
rosrun uarm mock_master_arm.py _mode:="sine" _amplitude:=60.0  # 大幅度运动

# 使用较低速度限制启动遥操
rosrun uarm realman_teleop_safe.py _max_joint_speed:=10.0
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
```

**验证方法**:
```bash
# 查看实际输出速度
rostopic hz /rm_driver/JointPos
rostopic echo /rm_driver/JointPos/joint[0]

# 主臂要求快速变化，但输出应该被限制
```

**预期结果**:
- 即使主臂变化很快（60°幅度），从臂输出变化平稳
- 关节速度不超过10°/s

### 测试3：错误处理与急停

**目的**: 验证碰撞检测和自动急停功能

**步骤**:
```bash
# 启动环境
./test_teleop.sh
# 选择 3

# 或手动触发错误
rostopic pub /mock_driver/trigger_error std_msgs/Bool "{data: true}"
```

**验证**:
```bash
# 观察遥操状态
rostopic echo /realman_teleop/emergency
# 应该显示: data: True

rostopic echo /realman_teleop/status
# 应该显示: data: False
```

**预期结果**:
- 触发错误后，遥操自动失能
- `/realman_teleop/emergency` 变为 True
- 控制命令停止输出

**复位测试**:
```bash
# 清除错误
rostopic pub /mock_driver/trigger_error std_msgs/Bool "{data: false}"

# 复位急停
rostopic pub /realman_teleop/emergency_stop std_msgs/Bool "{data: false}"

# 重新使能
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
```

### 测试4：通信超时保护

**目的**: 验证通信超时自动急停

**步骤**:
```bash
# 启动低速模拟驱动
rosrun uarm mock_rm_driver.py _simulation_rate:=1.0

# 启动遥操（设置短超时）
rosrun uarm realman_teleop_safe.py _comm_timeout:=0.5
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
```

**预期结果**:
- 由于驱动只以1Hz发布，遥操期待至少2Hz
- 触发通信超时，自动急停
- 日志显示: "通信超时，自动急停"

### 测试5：使能控制

**目的**: 验证必须使能才能运动

**步骤**:
```bash
# 正常启动但不使能
rosrun uarm mock_rm_driver.py
rosrun uarm mock_master_arm.py _mode:="sine"
rosrun uarm realman_teleop_safe.py
# 注意：不要发送enable
```

**验证**:
```bash
# 观察控制输出
rostopic echo /rm_driver/JointPos
# 应该保持初始角度不变

# 使能后
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
# 现在应该开始跟踪主臂
```

### 测试6：关节限位保护

**目的**: 验证软件关节限位

**步骤**:
```bash
rosrun uarm mock_rm_driver.py

# 模拟超出限位的主臂
rosrun uarm mock_master_arm.py _mode:="sine" _amplitude:=200.0

rosrun uarm realman_teleop_safe.py
rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}"
```

**预期结果**:
- 日志显示: "关节X角度越限"
- 输出被裁剪到限位范围内

## 📊 性能测试

### 延迟测试

```bash
# 使用rostopic测量延迟
rostopic delay /rm_driver/Arm_Current_State
```

### 吞吐量测试

```bash
# 检查发布频率
rostopic hz /servo_angles
rostopic hz /rm_driver/JointPos
```

### CPU占用

```bash
# 监控节点资源占用
rosnode info /realman_teleop_safe
```

## 🔍 故障注入测试

### 测试异常输入

```bash
# 发送异常长度的数据
rostopic pub /servo_angles std_msgs/Float64MultiArray "{data: [1.0, 2.0]}"

# 发送空数据
rostopic pub /servo_angles std_msgs/Float64MultiArray "{data: []}"
```

**预期结果**: 遥操节点应该记录警告但保持运行

### 测试高频输入

```bash
# 高频发布（测试处理能力）
rostopic pub -r 100 /servo_angles std_msgs/Float64MultiArray "{data: [0,0,0,0,0,0,0]}"
```

## ✅ 验证清单

在提交代码前，请确认以下测试通过：

- [ ] 基础遥操：主臂运动，从臂跟随
- [ ] 速度限制：超速输入被限制
- [ ] 使能控制：未使能时不运动
- [ ] 急停功能：错误触发急停
- [ ] 通信超时：超时自动保护
- [ ] 关节限位：越限被裁剪
- [ ] 错误恢复：复位后可重新使能
- [ ] 异常处理：异常输入不崩溃
- [ ] 性能：30Hz稳定运行
- [ ] 资源：CPU占用<10%

## 🛠️ 调试技巧

### 查看日志

```bash
# 查看遥操节点日志
rosrun uarm realman_teleop_safe.py 2>&1 | tee teleop.log

# 或查看ROS日志
roscd log
cat realman_teleop_safe-*.log
```

### 使用rqt工具

```bash
# 图形化话题监控
rqt_topic

# 图形化绘图
rqt_plot

# 消息发布测试
rqt_publisher
```

### 断点调试

```python
# 在代码中添加调试断点
import pdb; pdb.set_trace()

# 或使用ROS调试
rospy.logdebug("调试信息")
# 启动时设置日志级别
rosrun uarm realman_teleop_safe.py __log_level:=debug
```

## 📝 测试报告模板

```markdown
## 测试报告

### 环境信息
- 日期: 202X-XX-XX
- ROS版本: Noetic/Melodic
- Python版本: 3.X

### 测试结果
| 测试项 | 结果 | 备注 |
|--------|------|------|
| 基础遥操 | ✅/❌ | |
| 速度限制 | ✅/❌ | |
| 使能控制 | ✅/❌ | |
| 错误处理 | ✅/❌ | |
| 通信超时 | ✅/❌ | |

### 发现的问题
1. ...
2. ...

### 改进建议
...
```

---

**注意**: 通过全部测试不代表代码在实际机械臂上一定正确，实际测试时仍需：
1. 降低速度开始
2. 准备物理急停
3. 逐步验证每个关节
