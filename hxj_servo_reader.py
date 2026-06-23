#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机只读 ROS 入口。"""

from __future__ import annotations

import rospy

from hxj_modular.node import ServoReaderNode


def main() -> None:
    """启动华馨京舵机只读节点。

    入口脚本保持很薄：这里只初始化 ROS 节点、从参数服务器装配业务节点，
    并在退出时确保串口资源被关闭。
    """

    node = None
    try:
        rospy.init_node("hxj_servo_reader")
        node = ServoReaderNode.from_ros()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.close()


if __name__ == "__main__":
    main()
