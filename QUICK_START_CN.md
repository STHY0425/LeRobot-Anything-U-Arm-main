# 🚀 快速开始 - 仿真环境配置

## 当前状态
✓ Python 3.10.12 已安装  
✓ NumPy、OpenCV、Pillow 已安装  
✗ 需要安装仿真核心包

## 方案 A: 自动安装（推荐）

```bash
# 运行自动安装脚本
bash install_simulation.sh
```

## 方案 B: 手动分步安装

### 步骤 1: 安装 Vulkan 图形支持（必需）
```bash
sudo apt update
sudo apt install -y vulkan-tools libvulkan-dev mesa-vulkan-drivers
```

### 步骤 2: 升级 pip
```bash
python3 -m pip install --upgrade pip --user
```

### 步骤 3: 安装核心仿真包
```bash
# 安装 ManiSkill (机器人仿真核心)
python3 -m pip install --user mani_skill

# 安装 Gymnasium (环境接口)
python3 -m pip install --user gymnasium

# 安装 PyTorch (CPU版本，更快)
python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### 步骤 4: 验证安装
```bash
python3 check_environment.py
```

### 步骤 5: 测试仿真
```bash
cd src/simulation
python3 static_robot_viewer.py --robot panda
```

## 测试不同的机器人

```bash
cd src/simulation

# Franka Panda (7轴机械臂)
python3 static_robot_viewer.py --robot panda

# XArm6 + Robotiq 夹爪
python3 static_robot_viewer.py --robot xarm6_robotiq

# ARX-X5 (6轴)
python3 static_robot_viewer.py --robot arx-x5

# SO-100 (5轴)
python3 static_robot_viewer.py --robot so100

# Piper (6轴)
python3 static_robot_viewer.py --robot piper
```

## 常见问题

### 1. 权限错误 "not writeable"
这是正常的，使用 `--user` 参数安装到用户目录即可

### 2. Vulkan 错误
```bash
# 检查 Vulkan 是否安装
vulkaninfo | grep deviceName

# 如果没有输出，安装驱动
sudo apt install -y mesa-vulkan-drivers

# NVIDIA 显卡用户
sudo apt install -y nvidia-driver-535
```

### 3. 显示窗口无法打开
如果是远程服务器或没有图形界面：
```python
# 修改 static_robot_viewer.py 中的 render_mode
render_mode="rgb_array"  # 改为离屏渲染
```

### 4. 安装太慢
使用国内镜像源：
```bash
python3 -m pip install --user -i https://pypi.tuna.tsinghua.edu.cn/simple mani_skill gymnasium torch
```

## 下一步

1. ✅ 完成环境安装
2. ✅ 测试静态机器人查看器
3. 📚 查看 `src/simulation/README.md` 了解更多功能
4. 🎮 尝试遥操作仿真（如果有硬件）

## 预期效果

运行成功后，你会看到：
- 一个 3D 渲染窗口
- 显示选定的机器人模型
- 可以用鼠标旋转、缩放视角
- 机器人保持在设定的姿态

## 需要帮助？

- 查看项目 README: `README_CN.md`
- 仿真文档: `src/simulation/README.md`
- ManiSkill 官方文档: https://maniskill.readthedocs.io/
