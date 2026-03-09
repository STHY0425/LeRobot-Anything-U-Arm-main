#!/bin/bash
# 在 Docker 容器内设置 ROS 工作空间

echo "========================================"
echo "  设置 ROS 工作空间"
echo "========================================"
echo ""

# 检查是否在容器内
if [ ! -f "/.dockerenv" ]; then
    echo "⚠️  此脚本应该在 Docker 容器内运行"
    echo "   请先运行: ./run_ros_ready.sh"
    exit 1
fi

cd /workspace

echo "步骤 1/3: 安装 catkin 工具..."
apt-get update -qq > /dev/null 2>&1
apt-get install -y -qq python3-catkin-tools > /dev/null 2>&1

echo "步骤 2/3: 构建工作空间..."
source /opt/ros/noetic/setup.bash
catkin_make

echo "步骤 3/3: 设置环境变量..."
source devel/setup.bash
echo "source /workspace/devel/setup.bash" >> ~/.bashrc

echo ""
echo "========================================"
echo "  ✅ ROS 工作空间设置完成！"
echo "========================================"
echo ""
echo "现在可以使用 rosrun 命令了："
echo "  rosrun uarm servo_reader.py"
echo ""
