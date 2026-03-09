#!/bin/bash
# 中菱舵机最小化配置脚本
# 不依赖 ROS，直接使用 Python

set -e

echo "========================================"
echo "  中菱舵机配置工具（无需 ROS）"
echo "========================================"
echo ""

echo "步骤 1/4: 安装 Python 依赖..."
pip3 install pyserial numpy

echo "步骤 2/4: 检查串口设备..."
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    echo "✅ 找到串口设备:"
    ls -l /dev/ttyUSB*
else
    echo "⚠️  未找到 /dev/ttyUSB* 设备"
    echo "   请确保 USB 转 TTL 已连接"
fi

echo ""
echo "步骤 3/4: 设置串口权限..."
sudo usermod -aG dialout $USER
echo "✅ 已添加用户到 dialout 组"
echo "⚠️  需要重新登录才能生效，或者临时使用:"
echo "   sudo chmod 666 /dev/ttyUSB0"

echo ""
echo "步骤 4/4: 测试舵机配置工具..."
cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop/Zhonglin_servo

if [ -f "setup_servos.py" ]; then
    echo "✅ 找到配置工具"
else
    echo "❌ 未找到 setup_servos.py"
    exit 1
fi

echo ""
echo "========================================"
echo "  ✅ 配置完成！"
echo "========================================"
echo ""
echo "下一步操作："
echo ""
echo "1. 给串口临时权限（或重新登录）:"
echo "   sudo chmod 666 /dev/ttyUSB0"
echo ""
echo "2. 配置舵机 ID:"
echo "   cd ~/2026robotic/LeRobot-Anything-U-Arm-main/src/uarm/scripts/Uarm_teleop/Zhonglin_servo"
echo "   python3 setup_servos.py"
echo ""
echo "3. 测试舵机读取:"
echo "   python3 servo_zero.py"
echo ""
echo "注意: 如果需要仿真功能，需要安装 ROS"
echo "      但配置和测试舵机不需要 ROS"
echo ""
