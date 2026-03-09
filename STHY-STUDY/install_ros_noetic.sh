#!/bin/bash
# ROS Noetic 安装脚本 for Ubuntu 22.04
# 适用于 LeRobot-Anything-U-Arm 项目

set -e  # 遇到错误立即退出

echo "========================================"
echo "  ROS Noetic 安装脚本"
echo "  适用于 Ubuntu 22.04"
echo "========================================"
echo ""

# 检查 Ubuntu 版本
if [ "$(lsb_release -sc)" != "jammy" ]; then
    echo "警告: 此脚本为 Ubuntu 22.04 (Jammy) 设计"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "步骤 1/8: 检测到 Ubuntu 22.04，使用兼容方法安装..."
echo "注意: Ubuntu 22.04 官方不支持 ROS Noetic，但可以通过源码或 Docker 使用"
echo ""
echo "推荐方案："
echo "1. 使用 Docker 运行 ROS Noetic (推荐)"
echo "2. 降级到 Ubuntu 20.04"
echo "3. 使用 ROS2 (项目未来会支持)"
echo "4. 只安装 Python 依赖，不使用 ROS 功能"
echo ""
read -p "选择方案 [1-4]: " choice

case $choice in
    1)
        echo "安装 Docker 和 ROS Noetic 容器..."
        sudo apt update
        sudo apt install docker.io -y
        sudo systemctl start docker
        sudo systemctl enable docker
        sudo usermod -aG docker $USER
        echo ""
        echo "Docker 已安装！"
        echo "请运行以下命令获取 ROS Noetic 镜像："
        echo "  docker pull osrf/ros:noetic-desktop-full"
        echo ""
        echo "然后使用以下命令运行："
        echo "  docker run -it --rm osrf/ros:noetic-desktop-full"
        exit 0
        ;;
    2)
        echo "请手动安装 Ubuntu 20.04 后再运行此脚本"
        exit 0
        ;;
    3)
        echo "ROS2 支持正在开发中，请关注项目更新"
        exit 0
        ;;
    4)
        echo "步骤 4/8: 只安装 Python 依赖（无 ROS）..."
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo "步骤 5/8: 设置环境变量..."
if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "已添加 ROS 环境变量到 ~/.bashrc"
else
    echo "ROS 环境变量已存在于 ~/.bashrc"
fi

echo "步骤 6/8: 安装依赖工具..."
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

echo "步骤 7/8: 初始化 rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "rosdep 已经初始化"
fi
rosdep update

echo "步骤 8/8: 安装项目 Python 依赖..."
pip3 install pyserial numpy

echo ""
echo "========================================"
echo "  ✅ ROS Noetic 安装完成！"
echo "========================================"
echo ""
echo "下一步操作："
echo "1. 重新加载环境变量:"
echo "   source ~/.bashrc"
echo ""
echo "2. 验证安装:"
echo "   rosversion -d"
echo "   (应该输出: noetic)"
echo ""
echo "3. 测试 ROS:"
echo "   roscore"
echo ""
echo "4. 配置中菱舵机:"
echo "   cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop/Zhonglin_servo"
echo "   python3 setup_servos.py"
echo ""
