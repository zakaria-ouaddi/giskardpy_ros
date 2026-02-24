from typing import Optional, TYPE_CHECKING

import numpy as np
from pkg_resources import resource_filename

from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.robot_interface_config import (
    RobotInterfaceConfig,
    StandAloneRobotInterfaceConfig,
)
from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.derivatives import Derivatives
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.world_description.world_entity import CollisionCheckingConfig
from semantic_digital_twin.world_description.connections import ActiveConnection

if TYPE_CHECKING:
    from semantic_digital_twin.world import World

class TracyVelocityInterface(RobotInterfaceConfig):

    def setup(self):
        self.sync_joint_state_topic('/left_arm/joint_states')
        self.sync_joint_state_topic('/right_arm/joint_states')
        self.sync_joint_state_topic('/right_gripper/joint_states')
        self.sync_joint_state_topic('/left_gripper/joint_states')
        joints_left = ['left_shoulder_pan_joint',
                       'left_shoulder_lift_joint',
                       'left_elbow_joint',
                       'left_wrist_1_joint',
                       'left_wrist_2_joint',
                       'left_wrist_3_joint']
        self.add_joint_velocity_group_controller(cmd_topic='/left_arm/forward_velocity_controller/commands',
                                                 connections=joints_left)
        joints_right = ['right_shoulder_pan_joint',
                        'right_shoulder_lift_joint',
                        'right_elbow_joint',
                        'right_wrist_1_joint',
                        'right_wrist_2_joint',
                        'right_wrist_3_joint']
        self.add_joint_velocity_group_controller(cmd_topic='/right_arm/forward_velocity_controller/commands',
                                                 connections=joints_right)


class WorldWithTracyConfig(WorldWithFixedRobot):
    """
    Optimized Tracy world config to reduce jitter.
    Based on WorldWithTracyConfig but with fine-tuned collision settings similar to PR2.
    """
    def __init__(self, urdf: Optional[str] = None):
        super().__init__(
            urdf=urdf,
            root_name=PrefixedName("map2"),
            urdf_view=Tracy
        )

    def setup_collision_config(self):
        path_to_srdf = resource_filename(
            "giskardpy", "../self_collision_matrices/iai/tracy.srdf"
        )
        self.world.load_collision_srdf(path_to_srdf)

        # 1. Relax default collision config for the main body
        for body in self.robot.bodies_with_collisions:
            collision_config = CollisionCheckingConfig(
                buffer_zone_distance=0.003, violated_distance=0.0
            )
            body.set_static_collision_config(collision_config)

        # 2. Freeze Gripper Joints (Crucial for stability)
        # Assuming standard Robotiq joint names based on link names
        gripper_joints = [
            'left_robotiq_85_left_knuckle_joint',
            'left_robotiq_85_right_knuckle_joint',
            'left_robotiq_85_left_inner_knuckle_joint',
            'left_robotiq_85_right_inner_knuckle_joint',
            'left_robotiq_85_left_finger_tip_joint',
            'left_robotiq_85_right_finger_tip_joint',
            'right_robotiq_85_left_knuckle_joint',
            'right_robotiq_85_right_knuckle_joint',
            'right_robotiq_85_left_inner_knuckle_joint',
            'right_robotiq_85_right_inner_knuckle_joint',
            'right_robotiq_85_left_finger_tip_joint',
            'right_robotiq_85_right_finger_tip_joint',
        ]
        for joint_name in gripper_joints:
            try:
                c: ActiveConnection = self.world.get_connection_by_name(joint_name)
                c.frozen_for_collision_avoidance = True
            except Exception:
                pass

        # 3. Configure Shoulders
        # Standard PR2-like config: moderate buffer, no violation allowed
        shoulder_joints = [
            'left_shoulder_pan_joint', 'left_shoulder_lift_joint',
            'right_shoulder_pan_joint', 'right_shoulder_lift_joint'
        ]
        for joint_name in shoulder_joints:
            try:
                connection: ActiveConnection = self.world.get_connection_by_name(joint_name)
                collision_config = CollisionCheckingConfig(
                    buffer_zone_distance=0.003, violated_distance=0.0, max_avoided_bodies=4
                )
                connection.set_static_collision_config_for_direct_child_bodies(
                    collision_config
                )
            except Exception:
                pass

        # 4. Configure Elbows
        elbow_joints = [
            'left_elbow_joint',
            'right_elbow_joint'
        ]
        for joint_name in elbow_joints:
            try:
                connection: ActiveConnection = self.world.get_connection_by_name(joint_name)
                collision_config = CollisionCheckingConfig(
                    buffer_zone_distance=0.05, violated_distance=0.0, max_avoided_bodies=2
                )
                connection.set_static_collision_config_for_direct_child_bodies(
                    collision_config
                )
            except Exception:
                pass

        # 5. Configure Wrists
        # Increased buffer to 0.05 (smoother gradient) but kept max_avoided_bodies=1 (stability)
        wrist_joints = [
            'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint',
            'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint'
        ]
        for joint_name in wrist_joints:
            try:
                connection: ActiveConnection = self.world.get_connection_by_name(joint_name)
                collision_config = CollisionCheckingConfig(
                    buffer_zone_distance=0.05, violated_distance=0.0, max_avoided_bodies=1
                )
                connection.set_static_collision_config_for_direct_child_bodies(
                    collision_config
                )
            except Exception:
                pass

    def setup_world(self, robot_name: Optional[str] = None) -> None:
        super().setup_world()
        self.robot = self.world.get_semantic_annotations_by_type(Tracy)[0]


class TracyJointTrajServerMujocoInterface(RobotInterfaceConfig):
    def setup(self):
        self.sync_joint_state_topic('joint_states')
        self.add_follow_joint_trajectory_server(
            namespace='/left_arm/scaled_pos_joint_traj_controller_left')
        self.add_follow_joint_trajectory_server(
            namespace='/right_arm/scaled_pos_joint_traj_controller_right')


class TracyStandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    def __init__(self):
        super().__init__([
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint',
        ])
