from copy import deepcopy

import numpy as np
import sapien
import sapien.physx as physx
import torch

from mani_skill import PACKAGE_ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.utils import common, sapien_utils
from mani_skill.utils.structs.actor import Actor
from mani_skill.sensors.camera import CameraConfig


@register_agent()
class Piper(BaseAgent):
    uid = "piper"
    urdf_path = f"{PACKAGE_ASSET_DIR}/robots/piper/piper.urdf"
    
    # Based on piper.urdf analysis, set material configuration to improve grasping performance
    urdf_config = dict(
        _materials=dict(
            gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
        ),
        link=dict(
            link7=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            link8=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
        ),
    )

    # Keyframe configuration - based on homestate in config file
    keyframes = dict(
        rest=Keyframe(
            qpos=np.array(
                [
                    0.0,  # joint1
                    0.0,  # joint2
                    0.0,  # joint3
                    0.0,  # joint4
                    0.0,  # joint5
                    0.0,  # joint6
                    0.02, # joint7 (gripper open position)
                    0.02, # joint8 (gripper open position)
                ]
            ),
            pose=sapien.Pose(),
        )
    )

    # Joint name definitions
    arm_joint_names = [
        "joint1",
        "joint2", 
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    gripper_joint_names = [
        "joint7",
        "joint8",
    ]
    ee_link_name = "link6"  # End effector link

    # Control parameters - based on settings in config.yml
    arm_stiffness = 1000  # joint_stiffness from config
    arm_damping = 200     # joint_damping from config
    arm_force_limit = 100

    gripper_stiffness = 1000  # gripper_stiffnes from config
    gripper_damping = 200     # gripper_damping from config
    gripper_force_limit = 10   # Based on effort limit of gripper joints in URDF

    @property
    def _controller_configs(self):
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=None,
            upper=None,
            stiffness=100,
            damping=10,
            normalize_action=False,
        )
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=100,
            damping=10,
            use_delta=True,
        )

        gripper_pd_joint_pos = PDJointPosControllerConfig(
            self.gripper_joint_names,
            lower=None,
            upper=None,
            stiffness=100,
            damping=10,
            normalize_action=False,
        )
        gripper_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.gripper_joint_names,
            lower=-0.1,
            upper=0.1,
            stiffness=100,
            damping=10,
            use_delta=True,
        )

        controller_configs = dict(
            pd_joint_pos=dict(
                arm=arm_pd_joint_pos,
                gripper_active=gripper_pd_joint_pos,
            ),
            pd_joint_delta_pos=dict(
                arm=arm_pd_joint_delta_pos,
                gripper_active=gripper_pd_joint_delta_pos,
            ),
        )

        return deepcopy_dict(controller_configs)