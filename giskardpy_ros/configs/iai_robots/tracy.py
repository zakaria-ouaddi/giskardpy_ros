from typing import Optional

import numpy as np

from giskardpy.model.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.robot_interface_config import RobotInterfaceConfig, StandAloneRobotInterfaceConfig
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy.data_types.data_types import Derivatives


class TracyWorldConfig(WorldWithFixedRobot):
    """
    Configures the world model for Tracy, a fixed-base dual-arm robot.
    """

    def __init__(self, urdf: Optional[str] = None, map_name: str = 'map'):
        super().__init__(urdf=urdf, map_name=map_name)

    def setup(self, robot_name: Optional[str] = None) -> None:
        super().setup(robot_name)
        self.set_default_limits({Derivatives.velocity: 0.2,
                                 Derivatives.acceleration: np.inf,
                                 Derivatives.jerk: None})


class TracyCollisionAvoidanceConfig(CollisionAvoidanceConfig):
    """
    Configures detailed collision avoidance for Tracy to enable robust dual-arm manipulation.
    """

    def __init__(self, collision_checker: CollisionCheckerLib = CollisionCheckerLib.bpb):
        super().__init__(collision_checker=collision_checker)

    def setup(self):
        # 1. Load the standard SRDF matrix
        self.load_self_collision_matrix(
            'package://giskardpy_ros/self_collision_matrices/iai/tracy.srdf')

        # 2. General External Safety (Table/Walls)
        # Reduced to 0.02 (2cm) for better manipulation
        self.set_default_external_collision_avoidance(soft_threshold=0.02, hard_threshold=0.0)

        # 3. General Self Collision Safety
        self.set_default_self_collision_avoidance(soft_threshold=0.02, hard_threshold=0.01)

        # 4. Relax Elbow Safety for manipulation
        for joint_name in ['left_elbow_joint', 'right_elbow_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.01,  # Very low
                                                        number_of_repeller=1)

        # 5. Wrists need to get very close to things
        for joint_name in ['left_wrist_3_joint', 'right_wrist_3_joint']:
            self.overwrite_external_collision_avoidance(joint_name,
                                                        soft_threshold=0.005,  # 5mm
                                                        number_of_repeller=2)  # Reduced repellers

        # 6. CRITICAL: Allow gripper fingers to contact objects during manipulation
        gripper_links = [
            'left_robotiq_85_left_finger_link', 'left_robotiq_85_right_finger_link',
            'left_robotiq_85_left_finger_tip_link', 'left_robotiq_85_right_finger_tip_link',
            'right_robotiq_85_left_finger_link', 'right_robotiq_85_right_finger_link',
            'right_robotiq_85_left_finger_tip_link', 'right_robotiq_85_right_finger_tip_link'
        ]

        for link_name in gripper_links:
            self.overwrite_external_collision_avoidance(link_name,
                                                        soft_threshold=0.0,  # ZERO threshold - allow contact
                                                        hard_threshold=0.0,
                                                        number_of_repeller=0)  # No repellers

        # 7. Allow arm links to be close to each other (Dual arm manipulation)
        critical_arm_links = [
            'left_forearm_link', 'left_wrist_1_link', 'left_wrist_2_link', 'left_wrist_3_link',
            'right_forearm_link', 'right_wrist_1_link', 'right_wrist_2_link', 'right_wrist_3_link'
        ]
        for link_name in critical_arm_links:
            self.overwrite_self_collision_avoidance(link_name,
                                                    soft_threshold=0.005,  # 5mm
                                                    hard_threshold=0.0)

        # 8. CRITICAL FIX: Explicitly disable collision for Wrist-to-Gripper connections.
        # Left Arm
        self.add_disable_collision_link_pair('left_wrist_3_link', 'left_robotiq_85_base_link')
        self.add_disable_collision_link_pair('left_wrist_3_link', 'left_robotiq_85_left_knuckle_link')
        self.add_disable_collision_link_pair('left_wrist_3_link', 'left_robotiq_85_right_knuckle_link')
        self.add_disable_collision_link_pair('left_wrist_3_link', 'left_robotiq_85_left_inner_knuckle_link')
        self.add_disable_collision_link_pair('left_wrist_3_link', 'left_robotiq_85_right_inner_knuckle_link')

        # Right Arm
        self.add_disable_collision_link_pair('right_wrist_3_link', 'right_robotiq_85_base_link')
        self.add_disable_collision_link_pair('right_wrist_3_link', 'right_robotiq_85_left_knuckle_link')
        self.add_disable_collision_link_pair('right_wrist_3_link', 'right_robotiq_85_right_knuckle_link')
        self.add_disable_collision_link_pair('right_wrist_3_link', 'right_robotiq_85_left_inner_knuckle_link')
        self.add_disable_collision_link_pair('right_wrist_3_link', 'right_robotiq_85_right_inner_knuckle_link')

        # 9. Fix joints for the gripper fingers so they don't try to avoid themselves
        self.fix_joints_for_collision_avoidance([
            # Left Gripper Joints
            'left_robotiq_85_left_knuckle_joint', 'left_robotiq_85_right_knuckle_joint',
            'left_robotiq_85_left_inner_knuckle_joint', 'left_robotiq_85_right_inner_knuckle_joint',
            'left_robotiq_85_left_finger_tip_joint', 'left_robotiq_85_right_finger_tip_joint',
            # Right Gripper Joints
            'right_robotiq_85_left_knuckle_joint', 'right_robotiq_85_right_knuckle_joint',
            'right_robotiq_85_left_inner_knuckle_joint', 'right_robotiq_85_right_inner_knuckle_joint',
            'right_robotiq_85_left_finger_tip_joint', 'right_robotiq_85_right_finger_tip_joint',
        ])


class TracyJointTrajServerMujocoInterface(RobotInterfaceConfig):
    """Configures ROS interfaces for controlling Tracy via joint trajectory servers."""

    def setup(self):
        self.sync_joint_state_topic('joint_states')
        self.add_follow_joint_trajectory_server(
            namespace='/left_arm/scaled_pos_joint_traj_controller_left')
        self.add_follow_joint_trajectory_server(
            namespace='/right_arm/scaled_pos_joint_traj_controller_right')
        


class TracyStandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    """Defines the set of arm joints to be actively controlled by Giskard."""

    def __init__(self):
        super().__init__([
            # Left arm
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
            # Right arm
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint',
            # Left gripper
            'left_robotiq_85_base_joint',
            'left_robotiq_85_left_knuckle_joint',
            'left_robotiq_85_left_finger_joint',
            'left_robotiq_85_left_finger_tip_joint',
            'left_robotiq_85_right_knuckle_joint',
            'left_robotiq_85_right_finger_joint',
            'left_robotiq_85_right_finger_tip_joint',
            'left_robotiq_85_left_inner_knuckle_joint',
            'left_robotiq_85_right_inner_knuckle_joint',
            # Right gripper
            'right_robotiq_85_base_joint',
            'right_robotiq_85_left_knuckle_joint',
            'right_robotiq_85_left_finger_joint',
            'right_robotiq_85_left_finger_tip_joint',
            'right_robotiq_85_right_knuckle_joint',
            'right_robotiq_85_right_finger_joint',
            'right_robotiq_85_right_finger_tip_joint',
            'right_robotiq_85_left_inner_knuckle_joint',
            'right_robotiq_85_right_inner_knuckle_joint',
        ])
        
        


            # Left Arm
            'left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint',
            'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint',
            # Left Gripper (CRITICAL ADDITION)
            'left_robotiq_85_left_knuckle_joint',

            # Right Arm
            'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint',
            'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint',
            # Right Gripper (CRITICAL ADDITION)
            'right_robotiq_85_left_knuckle_joint',
        ])
