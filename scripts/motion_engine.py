#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from typing import List, Optional

# Giskard Imports
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.model.collision_matrix_manager import CollisionRequest, CollisionAvoidanceTypes
from giskardpy.motion_statechart.goals.templates import Sequence
from semantic_digital_twin.spatial_types import TransformationMatrix
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.world_description.world_entity import Body

from semantic_digital_twin.world_description.connections import FixedConnection

class GiskardMotionEngine:
    """
    A dedicated engine for handling robot motion using Giskard.
    Responsibilities:
    - Receiving Poses
    - Executing Smooth Motion (Point-to-Point or Sequences)
    - Handling Collision Avoidance settings for motion
    
    Explicitly excludes:
    - Gripper actuation (handled by a separate system)
    - Perception
    - High-level planning logic
    """
    def __init__(self, root_link: str = "map2", tip_link: str = "l_gripper_tool_frame"):
        # Initialize ROS Node if not already initialized
        try:
            rospy.init_node('giskard_motion_engine')
        except Exception:
            pass # Node might already be init

        self.giskard = GiskardWrapperNode()
        self.giskard.spin_in_background()
        
        # Try to detect root link from world
        if self.giskard.world and hasattr(self.giskard.world, 'root') and self.giskard.world.root:
             self.root_link = str(self.giskard.world.root.name)
        else:
             self.root_link = root_link
        
        self.tip_link = tip_link

    def destroy(self):
        if hasattr(self.giskard, 'node_handle'):
            self.giskard.node_handle.destroy_node()

    def _to_giskard_pose(self, pose: PoseStamped) -> TransformationMatrix:
        p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        q = cas.Quaternion(
            pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w
        )
        # Resolve frame
        frame_name = pose.header.frame_id if pose.header.frame_id else self.root_link
        ref_frame = self.giskard.world.get_kinematic_structure_entity_by_name(frame_name)
        
        return cas.TransformationMatrix.from_point_rotation_matrix(
            p, q.to_rotation_matrix(), reference_frame=ref_frame
        )

    def attach_object(self, object_name: str, link_name: str):
        """
        Attaches an object to a robot link.
        """
        print(f"Attaching {object_name} to {link_name}...")
        with self.giskard.world.modify_world():
            obj_body = self.giskard.world.get_body_by_name(object_name)
            parent_link = self.giskard.world.get_kinematic_structure_entity_by_name(link_name)
            
            # Compute relative pose: parent_T_object
            parent_T_obj = self.giskard.world.compute_forward_kinematics(
                root=parent_link,
                tip=obj_body
            )
            
            # Create new connection
            new_connection = FixedConnection(
                parent=parent_link,
                child=obj_body,
                parent_T_connection_expression=parent_T_obj
            )
            
            # Remove old connection and add new one
            self.giskard.world.remove_connection(obj_body.parent_connection)
            self.giskard.world.add_connection(new_connection)
        print(f"Attached {object_name} to {link_name}.")

    def detach_object(self, object_name: str):
        """
        Detaches an object (attaches it back to the world root).
        """
        print(f"Detaching {object_name}...")
        with self.giskard.world.modify_world():
            obj_body = self.giskard.world.get_body_by_name(object_name)
            root_link = self.giskard.world.root
            
            # Compute pose in world frame
            root_T_obj = self.giskard.world.compute_forward_kinematics(
                root=root_link,
                tip=obj_body
            )
            
            new_connection = FixedConnection(
                parent=root_link,
                child=obj_body,
                parent_T_connection_expression=root_T_obj
            )
            
            self.giskard.world.remove_connection(obj_body.parent_connection)
            self.giskard.world.add_connection(new_connection)
        print(f"Detached {object_name}.")

    def move_to_pose(self, pose: PoseStamped, object_to_allow_collision: Optional[str] = None, environment_objects_to_allow_collision: List[str] = []):
        """
        Moves to a single pose smoothly.
        """
        print(f"Moving to pose: {pose.pose.position}")
        
        msc = MotionStatechart()
        
        # 1. Define the Move Task
        target_tf = self._to_giskard_pose(pose)
        
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        move_task = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=target_tf,
            name="MoveToPose"
        )
        msc.add_node(move_task)
        
        # 2. Handle Collision Avoidance
        collision_entries = []
        
        if object_to_allow_collision:
            # If we are moving to grasp an object, we must allow collision with it
            tip_body = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
            obj_body = self.giskard.world.get_kinematic_structure_entity_by_name(object_to_allow_collision)
            
            collision_entries.append(
                CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=[tip_body],
                    body_group2=[obj_body]
                )
            )
            
            # Allow object to touch environment (e.g. table)
            for env_name in environment_objects_to_allow_collision:
                try:
                    env_body = self.giskard.world.get_kinematic_structure_entity_by_name(env_name)
                    collision_entries.append(
                        CollisionRequest(
                            type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                            body_group1=[obj_body],
                            body_group2=[env_body]
                        )
                    )
                    print(f"DEBUG: Allowing collision between {object_to_allow_collision} and {env_name}")
                except Exception as e:
                    print(f"WARNING: Could not find environment object {env_name} for collision allowance: {e}")

        if collision_entries:
            allow_col = CollisionAvoidance(collision_entries=collision_entries)
            allow_col.start_condition = cas.TrinaryTrue
            msc.add_node(allow_col)

        # 3. End Condition
        end = EndMotion()
        end.start_condition = move_task.observation_variable
        msc.add_node(end)
        
        # 4. Execute
        self.giskard.execute(msc)
        print("Motion complete.")

    def _get_gripper_links(self) -> List[Body]:
        """
        Heuristic to get all gripper links for collision avoidance.
        Finds all bodies with 'gripper' or 'finger' in their name.
        """
        all_bodies = self.giskard.world.bodies_with_enabled_collision
        gripper_bodies = []
        for body in all_bodies:
            name = str(body.name)
            # Adjust keywords based on your robot's URDF naming convention
            if "gripper" in name or "finger" in name:
                gripper_bodies.append(body)
        
        # Ensure tip link is included
        tip_body = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        if tip_body not in gripper_bodies:
            gripper_bodies.append(tip_body)
            
        print(f"DEBUG: Found {len(gripper_bodies)} gripper links for collision avoidance: {[str(b.name) for b in gripper_bodies]}")
        return gripper_bodies

    def execute_smooth_sequence(self, poses: List[PoseStamped], object_to_allow_collision: Optional[str] = None, environment_objects_to_allow_collision: List[str] = []):
        """
        Executes a sequence of poses (e.g. Pre-Grasp -> Grasp) smoothly.
        Giskard will plan a continuous motion through these waypoints.
        """
        print(f"Executing sequence of {len(poses)} poses...")
        
        msc = MotionStatechart()
        
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        tasks = []

        # 1. Create a list of tasks
        for i, pose in enumerate(poses):
            target_tf = self._to_giskard_pose(pose)
            task = CartesianPose(
                root_link=root_entity,
                tip_link=tip_entity,
                goal_pose=target_tf,
                name=f"Waypoint_{i}"
            )
            tasks.append(task)
            
        # 2. Use Sequence Template
        # This automatically handles start/end conditions so tasks run one after another
        seq = Sequence(nodes=tasks)
        msc.add_node(seq)

        # 3. Handle Collision Avoidance (Global for the sequence)
        collision_entries = []
        
        if object_to_allow_collision:
            # Allow gripper to touch object
            gripper_bodies = self._get_gripper_links()
            obj_body = self.giskard.world.get_kinematic_structure_entity_by_name(object_to_allow_collision)
            
            collision_entries.append(
                CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=gripper_bodies,
                    body_group2=[obj_body]
                )
            )
            
            # Allow object to touch environment (e.g. table)
            for env_name in environment_objects_to_allow_collision:
                try:
                    env_body = self.giskard.world.get_kinematic_structure_entity_by_name(env_name)
                    collision_entries.append(
                        CollisionRequest(
                            type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                            body_group1=[obj_body],
                            body_group2=[env_body]
                        )
                    )
                    print(f"DEBUG: Allowing collision between {object_to_allow_collision} and {env_name}")
                except Exception as e:
                    print(f"WARNING: Could not find environment object {env_name} for collision allowance: {e}")

        if collision_entries:
            allow_col = CollisionAvoidance(collision_entries=collision_entries)
            allow_col.start_condition = cas.TrinaryTrue
            msc.add_node(allow_col)
        # 4. End Condition
        # The sequence is done when the Sequence node is satisfied
        end = EndMotion()
        end.start_condition = seq.observation_variable
        msc.add_node(end)
        
        # 4. Execute
        self.giskard.execute(msc)
        print("Sequence complete.")

def main():
    # Usage Example
    engine = GiskardMotionEngine()
    
    # Example: Receive poses from PyCram (simulated here)
    pose_A = PoseStamped()
    pose_A.header.frame_id = "map"
    pose_A.pose.position.x = 0.4
    pose_A.pose.position.z = 0.5
    pose_A.pose.orientation.w = 1.0
    
    pose_B = PoseStamped()
    pose_B.header.frame_id = "map"
    pose_B.pose.position.x = 0.5
    pose_B.pose.position.z = 0.4 # Lower/Forward
    pose_B.pose.orientation.w = 1.0

    # Execute Smooth Sequence (e.g. Approach -> Grasp)
    # We pass "test_cube" to allow collision with it during this motion
    engine.execute_smooth_sequence([pose_A, pose_B], object_to_allow_collision="test_cube")
    
    # At this point, the robot is at pose_B.
    # The Manipulation team (you) signals the Gripper system to close.
    # Then you call engine.move_to_pose() to lift/retreat.

    engine.giskard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
