#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""华馨京舵机重力补偿 ROS 入口。"""

from __future__ import annotations

import rospy

from hxj_modular.node import GravityCompensationNode


def main() -> None:
    """启动华馨京舵机重力补偿节点。

    入口脚本保持很薄：这里只初始化 ROS 节点、从参数服务器装配业务节点，
    并在退出时恢复阻尼或释放舵机。
    """

    node = None
    try:
        rospy.init_node("hxj_gravity_compensation")
        node = GravityCompensationNode.from_ros()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.close()


if __name__ == "__main__":
    main()
