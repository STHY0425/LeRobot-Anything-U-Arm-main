# 🤖 睿尔曼 RM65-B 集成完成

## ✅ 已完成

1. **下载官方资源** - 从 GitHub 克隆 rm_robot 功能包
2. **配置仿真环境** - URDF 和网格文件已就位
3. **创建 API 接口** - Python 控制器封装
4. **编写文档** - 完整的使用指南

## 📚 文档导航

- **[RM65B快速启动.md](RM65B快速启动.md)** ⭐ 从这里开始
- **[RM65B集成指南.md](RM65B集成指南.md)** - 完整技术文档
- **[RM65B文件清单.md](RM65B文件清单.md)** - 所有文件列表

## 🚀 快速测试

### 仅仿真（无需硬件）
```bash
cd src/simulation
python3 static_robot_viewer.py --robot rm65b
```

### 有硬件
```bash
# 1. 安装功能包
cd ~/catkin_ws/src
git clone https://github.com/RealManRobot/rm_robot.git
cd ~/catkin_ws && catkin_make

# 2. 启动驱动
roslaunch rm_driver rm_65_driver.launch

# 3. 测试
cd src/uarm/scripts/Follower_Arm/Realman
python3 test_rm65b.py
```

## 📁 关键文件

- 仿真: `src/simulation/mani_skill/agents/robots/rm65b/rm65b.py`
- API: `src/uarm/scripts/Follower_Arm/Realman/rm65b_controller.py`
- 遥操作: `src/uarm/scripts/Follower_Arm/Realman/servo2realman.py`

**开始使用: 阅读 [RM65B快速启动.md](RM65B快速启动.md)**
