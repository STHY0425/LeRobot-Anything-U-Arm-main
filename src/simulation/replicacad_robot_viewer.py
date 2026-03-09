import gymnasium as gym
import mani_skill.envs  # Must import to register all env/agent
import numpy as np
import argparse
import time
import sapien
from mani_skill.utils import sapien_utils

class StaticRobotViewer:
    """Static robot arm viewer
    
    Display specified type of robot arm in simulation environment, supports multiple robot arm types:
    arx-x5, so100, xarm6_robotiq, panda, x_fetch, unitree_h1
    """
    
    def __init__(self, scene: str, robot_uids: str, pose_name: str = "default"):
        """Initialize static robot arm viewer
        
        Args:
            scene: Simulation scene name
            robot_uids: Robot arm type identifier
            pose_name: Pose name (default, standing, t_pose, etc.)
        """
        self.scene = scene
        self.robot_uids = robot_uids
        self.pose_name = pose_name
        
        # Select control mode based on robot type
        if robot_uids == "x_fetch": 
            self.control_mode = "pd_joint_pos_dual_arm"
        elif robot_uids == "unitree_h1":
            self.control_mode = "pd_joint_delta_pos"
        else: 
            self.control_mode = "pd_joint_pos"

        # Create simulation environment
        self.env = gym.make(
            scene,
            robot_uids=robot_uids,
            render_mode="human",
            control_mode=self.control_mode,
            sensor_configs=dict(shader_pack="rt-fast"),
            human_render_camera_configs=dict(shader_pack="rt-fast"),
            viewer_camera_configs=dict(shader_pack="rt-fast"),
        )
        
        # Get action space information
        obs, _ = self.env.reset(seed=0)
        print(f"Action space: {self.env.action_space}")
        print(f"Observation space: {self.env.observation_space}")
        
        # Set robot arm pose
        self._setup_robot_pose()
        self._setup_camera_pose()

    def _setup_camera_pose(self):
        agent = getattr(self.env.unwrapped, "agent", None)
        pose = sapien.Pose()
        if agent is not None:
            pose = agent.robot.get_pose()  # Returns sapien.Pose
            print(f"Robot initial position: {pose}")
        camera_pose = sapien_utils.look_at(
            [-1.4, -1.1, 1.7], pose.p
        )
        camera_viewer = getattr(self.env.unwrapped, "viewer", None)
        if camera_viewer is not None:
            print(camera_pose)
            camera_pose_arr = camera_pose.raw_pose.squeeze().cpu().numpy()
            camera_position = camera_pose_arr[:3]
            camera_quaternion = camera_pose_arr[3:]
            camera_viewer.set_camera_pose(sapien.Pose(camera_position, camera_quaternion))

    def _setup_robot_pose(self):
        """Set robot arm pose based on robot type and pose name"""
        try:
            agent = getattr(self.env.unwrapped, "agent", None)
            if agent is None:
                print("Warning: Unable to get robot agent")
                return
                
            if self.robot_uids == "unitree_h1":
                # Special handling for H1 humanoid robot
                if self.pose_name == "standing":
                    if hasattr(agent, "keyframes") and "standing" in agent.keyframes:
                        standing_keyframe = agent.keyframes["standing"]
                        agent.reset(standing_keyframe.qpos)
                        agent.robot.set_root_pose(standing_keyframe.pose)
                        print("H1 set to standing pose")
                    else:
                        print("Warning: H1 standing pose not available, using default pose")
                elif self.pose_name == "t_pose":
                    if hasattr(agent, "keyframes") and "t_pose" in agent.keyframes:
                        t_pose_keyframe = agent.keyframes["t_pose"]
                        agent.reset(t_pose_keyframe.qpos)
                        agent.robot.set_root_pose(t_pose_keyframe.pose)
                        print("H1 set to T-pose")
                    else:
                        print("Warning: H1 T-pose not available, using default pose")
                else:
                    print("H1 using default pose")
                    
            elif self.robot_uids == "x_fetch":
                # Fetch dual-arm robot
                if self.pose_name == "home":
                    # Set dual arms to home position
                    home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    agent.reset(home_pose)
                    print("Fetch set to home pose")
                else:
                    print("Fetch using default pose")
                    
            elif self.robot_uids == "panda":
                # Panda robot arm
                if self.pose_name == "home":
                    # Set to home position
                    home_pose = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854, 0.0])
                    agent.reset(home_pose)
                    print("Panda set to home pose")
                else:
                    print("Panda using default pose")
                    
            elif self.robot_uids == "xarm6_robotiq":
                # XArm6 robot arm
                if self.pose_name == "home":
                    # Set to home position
                    home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    agent.reset(home_pose)
                    print("XArm6 set to home pose")
                else:
                    print("XArm6 using default pose")
                    
            elif self.robot_uids == "arx-x5":
                # ARX-X5 robot arm
                if self.pose_name == "home":
                    # Set to home position
                    home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    agent.reset(home_pose)
                    print("ARX-X5 set to home pose")
                else:
                    print("ARX-X5 using default pose")
                    
            elif self.robot_uids == "so100":
                # SO100 robot arm
                if self.pose_name == "home":
                    # Set to home position
                    home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    agent.reset(home_pose)
                    print("SO100 set to home pose")
                else:
                    print("SO100 using default pose")
                    
        except Exception as e:
            print(f"Error setting robot pose: {e}")
    
    def get_robot_info(self):
        """Get robot information"""
        try:
            agent = getattr(self.env.unwrapped, "agent", None)
            if agent is None:
                return "Unable to get robot information"
                
            info = f"Robot type: {self.robot_uids}\n"
            info += f"Control mode: {self.control_mode}\n"
            
            if hasattr(agent, "robot"):
                robot = agent.robot
                info += f"Robot name: {robot.name}\n"
                
                # Get joint information
                if hasattr(robot, "get_active_joints"):
                    joints = robot.get_active_joints()
                    info += f"Number of joints: {len(joints)}\n"
                    for i, joint in enumerate(joints):
                        info += f"  Joint {i}: {joint.name}\n"
                        
                # Get current joint positions
                if hasattr(agent, "get_qpos"):
                    qpos = agent.get_qpos()
                    info += f"Current joint positions: {qpos}\n"
                    
            return info
            
        except Exception as e:
            return f"Error getting robot information: {e}"
    
    def run(self, duration: float = None):
        """Run static robot arm viewer
        
        Args:
            duration: Runtime duration (seconds), None means infinite run
        """
        print("=" * 60)
        print("    Static Robot Arm Viewer")
        print("=" * 60)
        print(f"Robot arm type: {self.robot_uids}")
        print(f"Simulation scene:   {self.scene}")
        print(f"Pose name:   {self.pose_name}")
        print(f"Control mode:   {self.control_mode}")
        print("-" * 60)
        
        # Display robot information
        print(self.get_robot_info())
        print("-" * 60)
        
        start_time = time.time()
        
        try:
            print("Starting rendering, press Ctrl+C to stop...")
            
            while True:
                # Render current frame
                self.env.render()
                
                # Check if timeout
                if duration is not None and (time.time() - start_time) > duration:
                    print(f"Runtime reached {duration} seconds, auto-stopping")
                    break
                    
                # Brief delay to control rendering frequency
                time.sleep(0.033)  # About 30 FPS
                
        except KeyboardInterrupt:
            print("Received interrupt signal, preparing to stop...")
        finally:
            self.env.close()
            print("Resource cleanup completed")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Static robot arm viewer',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--robot', '-r', 
        type=str, 
        default='so100',
        choices=['arx-x5', 'so100', 'xarm6_robotiq', 'panda', 'x_fetch', 'unitree_h1'],
        help='Select robot arm type to display'
    )
    parser.add_argument(
        '--scene', '-s', 
        type=str, 
        default='ReplicaCAD_SceneManipulation-v1',
        help='Simulation scene name'
    )
    parser.add_argument(
        '--pose', '-p',
        type=str,
        default='default',
        choices=['default', 'home', 'standing', 't_pose'],
        help='Robot arm pose name'
    )
    parser.add_argument(
        '--duration', '-d',
        type=float,
        default=None,
        help='Runtime duration (seconds), default infinite run'
    )
    
    args = parser.parse_args()
    
    # Create and run viewer
    try:
        viewer = StaticRobotViewer(
            scene=args.scene, 
            robot_uids=args.robot, 
            pose_name=args.pose
        )
        viewer.run(duration=args.duration)
    except Exception as e:
        print(f"Program runtime error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 