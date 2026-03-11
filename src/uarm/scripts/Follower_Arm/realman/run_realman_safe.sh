#!/bin/bash
# RealMan (瑞尔曼)机械臂安全遥操启动脚本
# 包含完整的安全监控功能

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   RealMan 机械臂安全遥操控制系统    ${NC}"
echo -e "${GREEN}   Safety Enhanced Version           ${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: 未检测到ROS环境，请先source ROS setup.bash${NC}"
    echo "例如: source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo -e "${YELLOW}ROS版本: $ROS_DISTRO${NC}"

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 检测并设置工作空间路径
detect_workspace() {
    if [ -d "${SCRIPT_DIR}/catkin_ws/devel" ]; then
        echo "${SCRIPT_DIR}/catkin_ws"
        return
    fi
    if [ -d "/workspace/src/uarm/scripts/Follower_Arm/realman/catkin_ws/devel" ]; then
        echo "/workspace/src/uarm/scripts/Follower_Arm/realman/catkin_ws"
        return
    fi
    if [ -d "./catkin_ws/devel" ]; then
        echo "./catkin_ws"
        return
    fi
    echo ""
}

CATKIN_WS=$(detect_workspace)

if [ -z "$CATKIN_WS" ]; then
    echo -e "${RED}错误: 未找到catkin_ws工作空间${NC}"
    exit 1
fi

echo -e "${YELLOW}工作空间: $CATKIN_WS${NC}"

# Source瑞尔曼驱动工作空间
source "${CATKIN_WS}/devel/setup.bash"

# 检查ROS Master
if ! rostopic list > /dev/null 2>&1; then
    echo -e "${RED}错误: 未检测到ROS Master${NC}"
    echo "请先在其他终端启动: roscore"
    exit 1
fi

echo -e "${GREEN}ROS Master已连接${NC}"

# 检查瑞尔曼驱动
if ! rostopic list | grep -q "/rm_driver"; then
    echo -e "${YELLOW}警告: 未检测到瑞尔曼驱动节点${NC}"
    echo "请先在其他终端启动瑞尔曼驱动:"
    echo "  source ${CATKIN_WS}/devel/setup.bash"
    echo "  roslaunch rm_driver rm_65_driver.launch"
    
    read -p "是否继续启动? [y/N]: " continue_anyway
    if [[ ! $continue_anyway =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 显示安全信息
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}           安全功能说明               ${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "✅ 速度限制 - 防止关节超速"
echo -e "✅ 关节限位 - 软件层面角度限制"
echo -e "✅ 错误监控 - 实时检测并自动响应"
echo -e "✅ 通信超时 - 自动急停保护"
echo -e "✅ 使能控制 - 必须使能后才能运动"
echo -e "✅ 碰撞检测 - 自动触发急停"
echo -e "${BLUE}========================================${NC}"
echo ""

# 选择运行模式
echo -e "${GREEN}请选择运行模式:${NC}"
echo "  1) 仅启动安全遥操节点"
echo "  2) 启动遥操节点 + 安全监控节点"
echo "  3) 启动遥操节点 + 监控节点 + 自动使能"
echo "  q) 退出"
echo ""

read -p "请输入选项 [1/2/3/q]: " choice

case $choice in
    1)
        echo -e "${GREEN}启动安全遥操节点...${NC}"
        echo -e "${YELLOW}提示: 启动后需要手动使能:${NC}"
        echo -e "  rostopic pub /realman_teleop/enable std_msgs/Bool '{data: true}'"
        
        rosrun uarm realman_teleop_safe.py \
            __name:=realman_teleop_safe \
            _arm_model:="RM_65" \
            _arm_ip:="192.168.1.18" \
            _max_joint_speed:=30.0 \
            _enable_emergency_stop:=true
        ;;
        
    2)
        echo -e "${GREEN}启动遥操节点 + 安全监控...${NC}"
        
        # 启动遥操节点（后台）
        rosrun uarm realman_teleop_safe.py \
            __name:=realman_teleop_safe \
            _arm_model:="RM_65" \
            _arm_ip:="192.168.1.18" \
            _max_joint_speed:=30.0 \
            _enable_emergency_stop:=true &
        TELEOP_PID=$!
        
        sleep 2
        
        # 启动监控节点（后台）
        rosrun uarm realman_safety_monitor.py &
        MONITOR_PID=$!
        
        echo ""
        echo -e "${GREEN}节点已启动:${NC}"
        echo -e "  遥操节点 PID: $TELEOP_PID"
        echo -e "  监控节点 PID: $MONITOR_PID"
        echo ""
        echo -e "${YELLOW}控制命令:${NC}"
        echo -e "  使能: rostopic pub /realman_teleop/enable std_msgs/Bool '{data: true}'"
        echo -e "  急停: rostopic pub /realman_teleop/emergency_stop std_msgs/Bool '{data: true}'"
        echo ""
        echo -e "${YELLOW}按Ctrl+C停止所有节点${NC}"
        
        # 等待中断
        trap "echo -e '${YELLOW}停止节点...${NC}'; kill $TELEOP_PID $MONITOR_PID 2>/dev/null; exit" INT
        wait
        ;;
        
    3)
        echo -e "${GREEN}启动完整系统（含自动使能）...${NC}"
        
        # 启动遥操节点（后台）
        rosrun uarm realman_teleop_safe.py \
            __name:=realman_teleop_safe \
            _arm_model:="RM_65" \
            _arm_ip:="192.168.1.18" \
            _max_joint_speed:=30.0 \
            _enable_emergency_stop:=true &
        TELEOP_PID=$!
        
        sleep 2
        
        # 启动监控节点（后台）
        rosrun uarm realman_safety_monitor.py &
        MONITOR_PID=$!
        
        sleep 2
        
        # 自动使能
        echo -e "${YELLOW}3秒后自动使能机械臂...${NC}"
        sleep 3
        rostopic pub /realman_teleop/enable std_msgs/Bool "{data: true}" --once
        
        echo ""
        echo -e "${GREEN}✅ 系统已启动并使能${NC}"
        echo -e "${YELLOW}急停命令（紧急时使用）:${NC}"
        echo -e "  rostopic pub /realman_teleop/emergency_stop std_msgs/Bool '{data: true}'"
        echo ""
        echo -e "${YELLOW}按Ctrl+C停止所有节点${NC}"
        
        # 等待中断
        trap "echo -e '${YELLOW}停止节点...${NC}'; kill $TELEOP_PID $MONITOR_PID 2>/dev/null; exit" INT
        wait
        ;;
        
    q|Q)
        echo "退出"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac
