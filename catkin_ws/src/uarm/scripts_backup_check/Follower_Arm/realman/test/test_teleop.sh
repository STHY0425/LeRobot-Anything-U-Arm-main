#!/bin/bash
# RealMan 遥操测试脚本
# 无实物测试指南

echo "=========================================="
echo "   RealMan 遥操无实物测试套件          "
echo "=========================================="

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

check_ros() {
    if ! rostopic list > /dev/null 2>&1; then
        echo -e "${RED}错误: ROS Master 未启动${NC}"
        echo "请先启动: roscore"
        exit 1
    fi
}

show_menu() {
    echo ""
    echo "请选择测试场景:"
    echo "  1) 基础遥操测试（正弦运动）"
    echo "  2) 安全功能测试（速度限制）"
    echo "  3) 错误处理测试（碰撞模拟）"
    echo "  4) 通信超时测试"
    echo "  5) 查看所有话题"
    echo "  q) 退出"
    echo ""
}

# 测试1: 基础遥操
test_basic_teleop() {
    echo -e "${GREEN}启动基础遥操测试...${NC}"
    echo "此测试将:"
    echo "  - 启动模拟驱动"
    echo "  - 启动模拟主臂（正弦运动）"
    echo "  - 启动遥操节点"
    echo "  - 自动使能"
    echo ""
    
    # 检查并启动模拟驱动
    if ! rostopic list | grep -q "/rm_driver"; then
        echo -e "${YELLOW}启动模拟驱动...${NC}"
        rosrun uarm mock_rm_driver.py &
        DRIVER_PID=$!
        sleep 2
    fi
    
    # 启动模拟主臂
    echo -e "${YELLOW}启动模拟主臂（正弦模式）...${NC}"
    rosrun uarm mock_master_arm.py _mode:="sine" _amplitude:=30.0 &
    MASTER_PID=$!
    sleep 2
    
    # 启动遥操节点
    echo -e "${YELLOW}启动安全遥操节点...${NC}"
    rosrun uarm realman_teleop_safe.py &
    TELEOP_PID=$!
    sleep 2
    
    # 自动使能
    echo -e "${YELLOW}自动使能机械臂...${NC}"
    rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}" --once
    
    echo ""
    echo -e "${GREEN}测试已启动！${NC}"
    echo "观察以下话题:"
    echo "  rostopic echo /rm_driver/Arm_Current_State/joint"
    echo "  rostopic echo /servo_angles"
    echo ""
    echo "查看rqt_plot:"
    echo "  rqt_plot /servo_angles/data[0] /rm_driver/Arm_Current_State/joint[0]"
    echo ""
    echo -e "${YELLOW}按Ctrl+C停止测试${NC}"
    
    trap "kill $DRIVER_PID $MASTER_PID $TELEOP_PID 2>/dev/null; echo -e '${GREEN}测试结束${NC}'; exit" INT
    wait
}

# 测试2: 安全功能
test_safety_features() {
    echo -e "${GREEN}启动安全功能测试...${NC}"
    
    # 启动基础环境
    rosrun uarm mock_rm_driver.py &
    DRIVER_PID=$!
    sleep 2
    
    rosrun uarm mock_master_arm.py _mode:="sine" _amplitude:=60.0 &
    MASTER_PID=$!
    sleep 2
    
    # 使用较低的速度限制启动遥操
    echo -e "${YELLOW}启动遥操（速度限制: 10°/s）...${NC}"
    rosrun uarm realman_teleop_safe.py _max_joint_speed:=10.0 &
    TELEOP_PID=$!
    sleep 2
    
    rosrun uarm realman_safety_monitor.py &
    MONITOR_PID=$!
    sleep 2
    
    # 使能
    rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}" --once
    
    echo ""
    echo -e "${GREEN}安全测试已启动！${NC}"
    echo "测试内容:"
    echo "  1. 观察速度限制是否生效（主臂幅度60°，但输出被限制）"
    echo "  2. 测试急停:"
    echo "     rostopic pub /realman_teleop/emergency_stop std_msgs/Bool true"
    echo "  3. 测试复位:"
    echo "     rostopic pub /realman_teleop/emergency_stop std_msgs/Bool false"
    echo "     rostopic pub /realman_teleop/enable std_msgs/Bool true"
    echo ""
    
    trap "kill $DRIVER_PID $MASTER_PID $TELEOP_PID $MONITOR_PID 2>/dev/null; echo -e '${GREEN}测试结束${NC}'; exit" INT
    wait
}

# 测试3: 错误处理
test_error_handling() {
    echo -e "${GREEN}启动错误处理测试...${NC}"
    
    rosrun uarm mock_rm_driver.py &
    DRIVER_PID=$!
    sleep 2
    
    rosrun uarm mock_master_arm.py _mode:="static" &
    MASTER_PID=$!
    sleep 2
    
    rosrun uarm realman_teleop_safe.py &
    TELEOP_PID=$!
    sleep 2
    
    rosrun uarm realman_safety_monitor.py &
    MONITOR_PID=$!
    sleep 2
    
    rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}" --once
    
    echo ""
    echo -e "${GREEN}错误处理测试已启动！${NC}"
    echo "模拟碰撞错误:"
    echo "  rostopic pub /mock_driver/trigger_error std_msgs/Bool true"
    echo ""
    echo "观察遥操节点是否自动触发急停！"
    echo ""
    
    trap "kill $DRIVER_PID $MASTER_PID $TELEOP_PID $MONITOR_PID 2>/dev/null; echo -e '${GREEN}测试结束${NC}'; exit" INT
    wait
}

# 测试4: 通信超时
test_comm_timeout() {
    echo -e "${GREEN}启动通信超时测试...${NC}"
    
    rosrun uarm mock_rm_driver.py _simulation_rate:=1.0 &
    DRIVER_PID=$!
    sleep 2
    
    rosrun uarm mock_master_arm.py &
    MASTER_PID=$!
    sleep 2
    
    # 设置较短的超时时间
    rosrun uarm realman_teleop_safe.py _comm_timeout:=0.5 &
    TELEOP_PID=$!
    sleep 2
    
    rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}" --once
    
    echo ""
    echo -e "${GREEN}通信超时测试已启动！${NC}"
    echo "模拟驱动频率已降低为1Hz，遥操超时时间为0.5s"
    echo "观察是否触发通信超时急停..."
    echo ""
    
    trap "kill $DRIVER_PID $MASTER_PID $TELEOP_PID 2>/dev/null; echo -e '${GREEN}测试结束${NC}'; exit" INT
    wait
}

# 查看话题
show_topics() {
    echo -e "${BLUE}当前活跃的话题:${NC}"
    rostopic list | grep -E "(realman|rm_driver|servo)"
    echo ""
    echo "查看消息类型:"
    echo "  rostopic info /rm_driver/Arm_Current_State"
    echo "  rostopic info /servo_angles"
}

# 主程序
check_ros

while true; do
    show_menu
    read -p "请输入选项: " choice
    
    case $choice in
        1) test_basic_teleop ;;
        2) test_safety_features ;;
        3) test_error_handling ;;
        4) test_comm_timeout ;;
        5) show_topics ;;
        q) echo "退出"; exit 0 ;;
        *) echo -e "${RED}无效选项${NC}" ;;
    esac
done
