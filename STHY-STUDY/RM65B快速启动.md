# 🚀 RM65-B 快速启动指南

## ✅ 已完成的工作

### 1. 文件结构
```
LeRobot-Anything-U-Arm/
├── RM65B集成指南.md                    # 完整集成文档
├── RM65B快速启动.md                    # 本文件
├── scripts/
│   └── fix_rm65b_urdf.py              # URDF 路径修复工具 ✅
├── src/simulation/mani_skill/
│   ├── assets/robots/rm65b/           # 官方 URDF 和网格 ✅
│   │   ├── rm_65.urdf                 # 已修复路径 ✅
│   │   ├── rm_65.urdf.xacro
│   │   └── meshes/RM65/               # 3D 模型文件
│   └── agents/robots/rm65b/           # 机器人定义 ✅
│       ├── __init__.py
│       └── rm65b.py                   # RM65B 类
└── src/uarm/scripts/Follower_Arm/Realman/
    ├── README.md                      # API 使用说明 ✅
    ├── rm65b_controller.py            # 控制器封装 ✅
    ├── servo2realman.py               # 遥操作桥接 ✅
    └── test_rm65b.py                  # 测试脚本 ✅
```

### 2. 下载的资源
- ✅ 官方 rm_robot 功能包 (位于 `/tmp/rm_robot`)
- ✅ RM65 URDF 文件
- ✅ 3D 网格文件 (STL 格式)
- ✅ ROS 驱动和示例代码

---

## 🎯 三种使用场景

### 场景 1: 仅仿真（无硬件）⭐ 推荐先试这个

**适用于:** 没有实体机械臂，只想在仿真中测试

```bash
# 1. 测试机器人显示
cd src/simulation
python3 static_robot_viewer.py --robot rm65b

# 2. 测试不同姿态
python3 static_robot_viewer.py --robot rm65b --pose home
python3 static_robot_viewer.py --robot rm65b --pose ready

# 3. 如果有 U-Arm，测试遥操作仿真
python3 teleop_sim.py --robot rm65b
```

**预期结果:**
- 打开 SAPIEN 窗口
- 显示 RM65-B 3D 模型
- 可以看到机械臂的各个连杆和关节

---

### 场景 2: 仅 API 控制（有硬件）

**适用于:** 有实体 RM65-B，想通过 API 控制

#### 步骤 1: 安装 ROS 功能包

```bash
# 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆官方功能包
git clone https://github.com/RealManRobot/rm_robot.git

# 编译
cd ~/catkin_ws
catkin_make

# 添加到环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 步骤 2: 配置网络

```bash
# 1. 设置你的电脑 IP 为 192.168.1.x (x != 18)
# 例如: 192.168.1.100

# 2. 测试连接
ping 192.168.1.18

# 3. 如果无法 ping 通，检查:
#    - 机械臂是否上电
#    - 网线是否连接
#    - IP 配置是否正确
```

#### 步骤 3: 启动驱动

```bash
# 终端 1: 启动 ROS 驱动
roslaunch rm_driver rm_65_driver.launch

# 终端 2: 测试控制
cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Follower_Arm/Realman
python3 test_rm65b.py
```

---

### 场景 3: 完整遥操作（U-Arm + RM65-B）

**适用于:** 有 U-Arm 和 RM65-B，想实现遥操作

#### 步骤 1: 启动 U-Arm

```bash
# 终端 1: 启动 U-Arm 读取节点
cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop
python3 servo_reader.py
```

#### 步骤 2: 启动 RM65-B 驱动

```bash
# 终端 2: 启动 RM65-B 驱动
roslaunch rm_driver rm_65_driver.launch
```

#### 步骤 3: 启动桥接

```bash
# 终端 3: 启动遥操作桥接
cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Follower_Arm/Realman
python3 servo2realman.py
```

**现在移动 U-Arm，RM65-B 应该会跟随运动！**

---

## 🔧 常见问题

### Q1: 仿真中看不到机器人

**检查步骤:**
```bash
# 1. 确认文件存在
ls src/simulation/mani_skill/assets/robots/rm65b/rm_65.urdf
ls src/simulation/mani_skill/assets/robots/rm65b/meshes/RM65/

# 2. 检查是否注册
python3 -c "
from mani_skill.agents.registration import REGISTERED_AGENTS
print('rm65b' in REGISTERED_AGENTS)
"

# 3. 如果返回 False，检查
cat src/simulation/mani_skill/agents/robots/__init__.py | grep rm65b
```

### Q2: 无法连接实体机械臂

**解决方法:**
```bash
# 1. 检查网络
ping 192.168.1.18

# 2. 检查 ROS 话题
rostopic list | grep rm_driver

# 3. 如果没有话题，重启驱动
roslaunch rm_driver rm_65_driver.launch

# 4. 手动测试发送命令
rostopic pub /rm_driver/joint_command rm_msgs/JointPos "{joint: [0,0,0,0,0,0], speed: 10}"
```

### Q3: URDF 路径错误

**错误信息:**
```
Failed to load mesh: package://rm_description/meshes/RM65/xxx.STL
```

**解决方法:**
```bash
# 运行修复脚本
python3 scripts/fix_rm65b_urdf.py
```

### Q4: 关节运动异常

**检查关节限位:**
```python
# RM65-B 关节限位 (弧度)
joint_limits = {
    'joint1': (-3.14, 3.14),    # ±180°
    'joint2': (-2.09, 2.09),    # ±120°
    'joint3': (-2.09, 2.09),    # ±120°
    'joint4': (-3.14, 3.14),    # ±180°
    'joint5': (-2.09, 2.09),    # ±120°
    'joint6': (-3.14, 3.14),    # ±180°
}
```

---

## 📝 测试清单

### 仿真测试
- [ ] `python3 static_robot_viewer.py --robot rm65b` 成功显示
- [ ] 可以看到完整的机械臂模型
- [ ] 不同姿态 (home, ready, rest) 正常切换

### API 测试（如果有硬件）
- [ ] 网络连接成功 (`ping 192.168.1.18`)
- [ ] ROS 驱动启动成功
- [ ] 可以看到 `/rm_driver/joint_states` 话题
- [ ] `test_rm65b.py` 所有测试通过
- [ ] 机械臂可以移动到 home 和 rest 姿态

### 遥操作测试（如果有 U-Arm）
- [ ] U-Arm 读取节点正常运行
- [ ] RM65-B 驱动正常运行
- [ ] 桥接节点正常运行
- [ ] 移动 U-Arm 时 RM65-B 跟随运动
- [ ] 运动平滑无抖动

---

## 📚 下一步

### 1. 调整控制参数

编辑 `src/simulation/mani_skill/agents/robots/rm65b/rm65b.py`:

```python
# 如果运动太快/太慢
arm_stiffness = 1e3  # 增大 = 更快响应
arm_damping = 1e2    # 增大 = 更稳定

# 如果有震荡
arm_stiffness = 5e2  # 降低刚度
arm_damping = 2e2    # 增加阻尼
```

### 2. 自定义关节映射

编辑 `src/uarm/scripts/Follower_Arm/Realman/servo2realman.py`:

```python
# 如果 U-Arm 和 RM65-B 的关节顺序不同
joint_mapping = [0, 1, 2, 3, 4, 5]  # 修改这里

# 如果需要角度偏移
joint_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 修改这里
```

### 3. 数据采集

使用 LeRobot 框架采集演示数据:

```bash
# 参考其他机械臂的示例
cd src/uarm/scripts/Follower_Arm/LeRobot
# 创建 RM65B 的数据采集脚本
```

---

## 🎓 学习资源

### 官方文档
- [睿尔曼官网](https://www.realman-robotics.com/)
- [GitHub 仓库](https://github.com/RealManRobot/rm_robot)
- [API 文档](https://github.com/RealManRobot/rm_robot/tree/main/rm_doc)

### 项目文档
- [完整集成指南](RM65B集成指南.md)
- [新机械臂添加指南](新机械臂快速添加指南.md)
- [仿真系统说明](如何使用仿真.md)

### 示例代码
- 官方示例: `/tmp/rm_robot/rm_demo/`
- 项目示例: `src/uarm/scripts/Follower_Arm/`

---

## 💡 提示

1. **先测试仿真**: 即使没有硬件，也可以在仿真中验证 URDF 是否正确
2. **逐步测试**: 先测试连接，再测试简单运动，最后测试遥操作
3. **保存日志**: 遇到问题时，保存终端输出便于调试
4. **备份配置**: 修改参数前先备份原文件

---

## ✅ 成功标志

当你看到以下情况时，说明集成成功：

### 仿真成功
- ✅ SAPIEN 窗口显示完整的 RM65-B 模型
- ✅ 所有关节和连杆清晰可见
- ✅ 可以切换不同姿态

### API 成功
- ✅ `test_rm65b.py` 所有测试通过
- ✅ 机械臂可以平滑运动
- ✅ 可以准确到达目标位置

### 遥操作成功
- ✅ U-Arm 和 RM65-B 同步运动
- ✅ 运动平滑无延迟
- ✅ 关节映射正确

---

**祝你成功！🎉**

如有问题，请查看 [完整集成指南](RM65B集成指南.md) 或参考官方文档。
