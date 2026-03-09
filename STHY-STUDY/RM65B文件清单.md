# 📋 RM65-B 集成文件清单

## 🎯 项目概述

已成功为睿尔曼 RM65-B 机械臂完成以下工作：
1. ✅ 下载官方 URDF 和网格文件
2. ✅ 创建仿真环境配置
3. ✅ 实现 API 控制接口
4. ✅ 编写测试和文档

---

## 📁 创建的文件

### 1. 文档文件

| 文件 | 说明 | 位置 |
|------|------|------|
| `RM65B集成指南.md` | 完整的集成指南，包含环境配置、仿真测试、API控制 | 项目根目录 |
| `RM65B快速启动.md` | 快速启动指南，三种使用场景 | 项目根目录 |
| `RM65B文件清单.md` | 本文件，所有创建文件的清单 | 项目根目录 |
| `src/uarm/scripts/Follower_Arm/Realman/README.md` | API 使用说明 | Realman 目录 |

### 2. 仿真相关文件

| 文件 | 说明 | 位置 |
|------|------|------|
| `src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf` | 官方 URDF（已修复路径） | assets/robots/rm65b/ |
| `src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf.xacro` | Xacro 版本 URDF | assets/robots/rm65b/ |
| `src/simulation/mani_skill/assets/robots/rm65b/meshes/RM65/` | 3D 网格文件目录 | assets/robots/rm65b/ |
| `src/simulation/mani_skill/agents/robots/rm65b/__init__.py` | 模块初始化 | agents/robots/rm65b/ |
| `src/simulation/mani_skill/agents/robots/rm65b/rm65b.py` | RM65B 机器人类定义 | agents/robots/rm65b/ |

### 3. API 控制文件

| 文件 | 说明 | 位置 |
|------|------|------|
| `src/uarm/scripts/Follower_Arm/Realman/rm65b_controller.py` | RM65-B 控制器封装类 | Realman/ |
| `src/uarm/scripts/Follower_Arm/Realman/servo2realman.py` | U-Arm 到 RM65-B 遥操作桥接 | Realman/ |
| `src/uarm/scripts/Follower_Arm/Realman/test_rm65b.py` | 测试脚本 | Realman/ |

### 4. 工具脚本

| 文件 | 说明 | 位置 |
|------|------|------|
| `scripts/fix_rm65b_urdf.py` | URDF 路径修复工具 | scripts/ |

### 5. 修改的文件

| 文件 | 修改内容 | 位置 |
|------|----------|------|
| `src/simulation/mani_skill/agents/robots/__init__.py` | 添加 `from .rm65b import RM65B` | agents/robots/ |

---

## 📦 下载的资源

### 官方功能包（位于 `/tmp/rm_robot`）

```
/tmp/rm_robot/
├── rm_description/          # 机器人描述
│   ├── urdf/RM65/          # URDF 文件
│   │   ├── rm_65.urdf      # 主 URDF
│   │   ├── rm_65.urdf.xacro
│   │   └── ...
│   └── meshes/RM65/        # 3D 网格
│       ├── base_link.STL
│       ├── Link1.STL
│       └── ...
├── rm_driver/              # ROS 驱动
│   ├── src/
│   │   ├── rm_driver.cpp
│   │   └── rm_robot.h
│   └── launch/
│       └── rm_65_driver.launch
├── rm_demo/                # 示例代码
│   └── src/
│       ├── api_rm65_pick_place_demo.cpp
│       └── ...
├── rm_msgs/                # ROS 消息定义
├── rm_control/             # 控制配置
├── rm_moveit_config/       # MoveIt 配置
└── ...
```

---

## 🔍 文件详细说明

### 核心文件

#### 1. `rm65b.py` - 机器人定义

**功能:**
- 定义 RM65B 类，继承自 BaseAgent
- 配置 6 个关节的控制器
- 实现多种控制模式（位置、速度、末端位姿）
- 定义预设姿态（rest, home, ready）

**关键配置:**
```python
uid = "rm65b"  # 机器人 ID
arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
arm_stiffness = 1e3
arm_damping = 1e2
```

#### 2. `rm65b_controller.py` - API 控制器

**功能:**
- 封装 RM65-B 的 ROS API
- 提供简单的 Python 接口
- 支持关节空间和笛卡尔空间运动
- 实时状态查询

**主要方法:**
```python
move_joint(positions, speed, block)  # 关节运动
get_joint_positions()                # 获取位置
home()                               # 回到 home 姿态
```

#### 3. `servo2realman.py` - 遥操作桥接

**功能:**
- 订阅 U-Arm 的 `/servo_angles` 话题
- 转换并发送到 RM65-B
- 支持关节映射和角度偏移
- 实时状态监控

**配置参数:**
```python
speed = 50                           # 运动速度
scale = 1.0                          # 角度缩放
joint_mapping = [0,1,2,3,4,5]       # 关节映射
joint_offset = [0.0]*6               # 角度偏移
```

#### 4. `fix_rm65b_urdf.py` - URDF 修复工具

**功能:**
- 自动修复 URDF 中的网格路径
- 将 ROS package 路径转换为相对路径
- 自动备份原文件

**使用:**
```bash
python3 scripts/fix_rm65b_urdf.py
```

---

## 🎮 使用流程

### 仅仿真
```bash
cd src/simulation
python3 static_robot_viewer.py --robot rm65b
```

### API 控制
```bash
# 终端 1
roslaunch rm_driver rm_65_driver.launch

# 终端 2
cd src/uarm/scripts/Follower_Arm/Realman
python3 test_rm65b.py
```

### 遥操作
```bash
# 终端 1: U-Arm
python3 servo_reader.py

# 终端 2: RM65-B 驱动
roslaunch rm_driver rm_65_driver.launch

# 终端 3: 桥接
python3 servo2realman.py
```

---

## 📊 技术规格

### RM65-B 参数
- **自由度**: 6 轴
- **负载**: 5 kg
- **工作半径**: 650 mm
- **重复定位精度**: ±0.02 mm
- **关节速度**: 180°/s
- **通信**: TCP/IP (192.168.1.18:8080)

### 关节限位（弧度）
```python
joint1: (-3.14, 3.14)   # ±180°
joint2: (-2.09, 2.09)   # ±120°
joint3: (-2.09, 2.09)   # ±120°
joint4: (-3.14, 3.14)   # ±180°
joint5: (-2.09, 2.09)   # ±120°
joint6: (-3.14, 3.14)   # ±180°
```

---

## 🔗 依赖关系

### Python 包
```
rospy
numpy
sensor_msgs
rm_msgs
```

### ROS 功能包
```
rm_robot (官方)
rm_driver
rm_msgs
rm_description
```

### 仿真环境
```
mani_skill
sapien
gymnasium
```

---

## 📝 待完成工作

### 可选增强
- [ ] 添加力控制支持
- [ ] 实现轨迹规划
- [ ] 添加碰撞检测
- [ ] 集成 MoveIt
- [ ] 添加视觉伺服
- [ ] 实现自动标定

### 数据采集
- [ ] 创建 LeRobot 数据采集脚本
- [ ] 定义任务环境
- [ ] 采集演示数据
- [ ] 训练策略模型

---

## 🐛 已知问题

### 1. URDF 路径问题
**问题**: 官方 URDF 使用 ROS package 路径
**解决**: 运行 `fix_rm65b_urdf.py` 自动修复

### 2. 网格文件大小
**问题**: STL 文件较大，加载可能较慢
**解决**: 正常现象，首次加载需要时间

### 3. 关节映射
**问题**: U-Arm 和 RM65-B 的关节配置可能不同
**解决**: 在 `servo2realman.py` 中调整 `joint_mapping`

---

## 📚 参考资源

### 官方资源
- [睿尔曼官网](https://www.realman-robotics.com/)
- [GitHub 仓库](https://github.com/RealManRobot/rm_robot)
- [产品手册](https://www.realman-robotics.com/products/rm65)

### 项目文档
- [LeRobot-Anything 主页](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm)
- [ManiSkill 文档](https://maniskill.readthedocs.io/)
- [SAPIEN 文档](https://sapien.ucsd.edu/docs/)

---

## ✅ 验证清单

### 文件完整性
- [x] 所有文档文件已创建
- [x] 仿真文件已配置
- [x] API 控制文件已创建
- [x] 工具脚本已创建
- [x] URDF 路径已修复

### 功能测试
- [ ] 仿真显示正常
- [ ] API 控制正常（需要硬件）
- [ ] 遥操作正常（需要硬件）

---

## 🎉 总结

已成功为 RM65-B 机械臂创建完整的集成方案，包括：

1. **仿真环境**: 可在 SAPIEN 中显示和测试
2. **API 接口**: 封装的 Python 控制器
3. **遥操作**: U-Arm 到 RM65-B 的桥接
4. **文档**: 完整的使用指南和参考文档
5. **工具**: 自动化脚本和测试程序

**下一步**: 根据 `RM65B快速启动.md` 开始测试！
