"""
睿尔曼 RM65-B 机械臂定义
基于官方 URDF: https://github.com/RealManRobot/rm_robot
"""
from copy import deepcopy

import numpy as np
import sapien
import torch

from mani_skill import PACKAGE_ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.utils import common, sapien_utils
from mani_skill.utils.structs.actor import Actor


@register_agent()
class RM65B(BaseAgent):
    """睿尔曼 RM65-B 6轴协作机械臂"""
    
    uid = "rm65b"
    urdf_path = f"{PACKAGE_ASSET_DIR}/robots/rm65b/rm_65.urdf"
    
    # URDF 配置
    urdf_config = dict(
        _materials=dict(
            gripper=dict(
                static_friction=2.0,
                dynamic_friction=2.0,
                restitution=0.0
            )
        ),
    )
    
    # 关节名称（从官方 URDF 获取）
    arm_joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    
    # 末端执行器连杆
    ee_link_name = "Link6"
    
    # 控制参数（根据 RM65-B 规格调整）
    # RM65-B 负载: 5kg, 重复定位精度: ±0.02mm
    arm_stiffness = 1e3
    arm_damping = 1e2
    arm_force_limit = 100
    
    # 预定义姿态
    keyframes = dict(
        rest=Keyframe(
            qpos=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            pose=sapien.Pose(),
        ),
        home=Keyframe(
            qpos=np.array([0.0, -0.785, 1.57, 0.0, 0.785, 0.0]),  # 典型 home 姿态
            pose=sapien.Pose(),
        ),
        ready=Keyframe(
            qpos=np.array([0.0, -1.047, 1.57, 0.0, 1.047, 0.0]),  # 准备姿态
            pose=sapien.Pose(),
        ),
    )
    
    @property
    def _controller_configs(self):
        # -------------------------------------------------------------------------- #
        # 手臂控制器
        # -------------------------------------------------------------------------- #
        
        # PD 关节位置控制
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=None,  # 使用 URDF 中的限位
            upper=None,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            normalize_action=False,
        )
        
        # PD 关节增量位置控制
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            use_delta=True,
        )
        
        arm_pd_joint_target_delta_pos = deepcopy(arm_pd_joint_delta_pos)
        arm_pd_joint_target_delta_pos.use_target = True
        
        # PD 末端位置控制
        arm_pd_ee_delta_pos = PDEEPosControllerConfig(
            joint_names=self.arm_joint_names,
            pos_lower=-0.1,
            pos_upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            ee_link=self.ee_link_name,
            urdf_path=self.urdf_path,
        )
        
        # PD 末端位姿控制
        arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
            joint_names=self.arm_joint_names,
            pos_lower=-0.1,
            pos_upper=0.1,
            rot_lower=-0.1,
            rot_upper=0.1,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            ee_link=self.ee_link_name,
            urdf_path=self.urdf_path,
        )
        
        arm_pd_ee_pose = PDEEPoseControllerConfig(
            joint_names=self.arm_joint_names,
            pos_lower=None,
            pos_upper=None,
            stiffness=self.arm_stiffness,
            damping=self.arm_damping,
            force_limit=self.arm_force_limit,
            ee_link=self.ee_link_name,
            urdf_path=self.urdf_path,
            use_delta=False,
            normalize_action=False,
        )
        
        arm_pd_ee_target_delta_pos = deepcopy(arm_pd_ee_delta_pos)
        arm_pd_ee_target_delta_pos.use_target = True
        arm_pd_ee_target_delta_pose = deepcopy(arm_pd_ee_delta_pose)
        arm_pd_ee_target_delta_pose.use_target = True
        
        # PD 关节速度控制
        arm_pd_joint_vel = PDJointVelControllerConfig(
            self.arm_joint_names,
            -1.0,
            1.0,
            self.arm_damping,
            self.arm_force_limit,
        )
        
        # PD 关节位置和速度控制
        arm_pd_joint_pos_vel = PDJointPosVelControllerConfig(
            self.arm_joint_names,
            None,
            None,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            normalize_action=False,
        )
        
        arm_pd_joint_delta_pos_vel = PDJointPosVelControllerConfig(
            self.arm_joint_names,
            -0.1,
            0.1,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
            use_delta=True,
        )
        
        # 组合所有控制器
        controller_configs = dict(
            pd_joint_pos=dict(arm=arm_pd_joint_pos),
            pd_joint_delta_pos=dict(arm=arm_pd_joint_delta_pos),
            pd_ee_delta_pos=dict(arm=arm_pd_ee_delta_pos),
            pd_ee_delta_pose=dict(arm=arm_pd_ee_delta_pose),
            pd_ee_pose=dict(arm=arm_pd_ee_pose),
            pd_joint_target_delta_pos=dict(arm=arm_pd_joint_target_delta_pos),
            pd_ee_target_delta_pos=dict(arm=arm_pd_ee_target_delta_pos),
            pd_ee_target_delta_pose=dict(arm=arm_pd_ee_target_delta_pose),
            pd_joint_vel=dict(arm=arm_pd_joint_vel),
            pd_joint_pos_vel=dict(arm=arm_pd_joint_pos_vel),
            pd_joint_delta_pos_vel=dict(arm=arm_pd_joint_delta_pos_vel),
        )
        
        return deepcopy_dict(controller_configs)
    
    def _after_loading_articulation(self):
        """加载机器人后的初始化"""
        super()._after_loading_articulation()
        
        # 获取末端执行器连杆
        self.tcp_link = self.robot.links_map[self.ee_link_name]
    
    def _after_init(self):
        """初始化后处理"""
        self.tcp = sapien_utils.get_obj_by_name(
            self.robot.get_links(), self.ee_link_name
        )
    
    @property
    def tcp_pos(self):
        """获取工具中心点位置"""
        return self.tcp.pose.p
    
    @property
    def tcp_pose(self):
        """获取工具中心点位姿"""
        return self.tcp.pose
    
    def is_static(self, threshold: float = 0.2):
        """检查机器人是否静止"""
        qvel = self.robot.get_qvel()
        return torch.max(torch.abs(qvel), 1)[0] <= threshold
    
    @staticmethod
    def build_grasp_pose(approaching, closing, center):
        """构建抓取位姿"""
        assert np.abs(1 - np.linalg.norm(approaching)) < 1e-3
        assert np.abs(1 - np.linalg.norm(closing)) < 1e-3
        assert np.abs(approaching @ closing) <= 1e-3
        ortho = np.cross(closing, approaching)
        T = np.eye(4)
        T[:3, :3] = np.stack([ortho, closing, approaching], axis=1)
        T[:3, 3] = center
        return sapien.Pose(T)
