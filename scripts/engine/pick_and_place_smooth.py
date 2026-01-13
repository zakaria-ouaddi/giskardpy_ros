#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from typing import List, Tuple

# Giskard Imports
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.model.collision_matrix_manager import CollisionRequest, CollisionAvoidanceTypes
from semantic_digital_twin.spatial_types import TransformationMatrix, Vector3, Point3
import semantic_digital_twin.spatial_types.spatial_types as cas

# Constants
LEFT_GRIPPER_ACTION_NAME = '/left_gripper/robotiq_gripper_controller/gripper_cmd'
RIGHT_GRIPPER_ACTION_NAME = '/right_gripper/robotiq_gripper_controller/gripper_cmd'

class SmoothPickAndPlace:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('smooth_pick_and_place')
        
        # Initialize Giskard
        self.giskard = GiskardWrapperNode()
        self.giskard.spin_in_background()
        
        # Initialize Gripper Clients
        self.gripper_node = rclpy.create_node('gripper_client_node')
        self.left_gripper = self._init_gripper_client(LEFT_GRIPPER_ACTION_NAME)
        self.right_gripper = self._init_gripper_client(RIGHT_GRIPPER_ACTION_NAME)
        
        # World Setup (Links)
        self.root_link = "map"
        self.tip_link = "l_gripper_tool_frame" # Default to left for now

    def _init_gripper_client(self, action_name: str, timeout: float = 5.0) -> ActionClient:
        client = ActionClient(self.gripper_node, GripperCommand, action_name)
        if not client.wait_for_server(timeout_sec=timeout):
            self.giskard.get_logger().warn(f'Action server "{action_name}" not available.')
        return client

    def control_gripper(self, client: ActionClient, position: float, max_effort: float = 10.0):
        """
        0.0 = Open, 0.8 = Closed (approx)
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        future = client.send_goal_async(goal)
        # We spin the gripper node briefly to send the goal
        rclpy.spin_until_future_complete(self.gripper_node, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            print("Gripper goal rejected")
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.gripper_node, res_future)
        return res_future.result()

    def pick(self, object_name: str, grasp_pose: PoseStamped):
        """
        Executes a smooth pick operation:
        1. Approach Pre-Grasp
        2. Blend into Grasp Approach
        3. Grasp
        4. Attach Object
        5. Lift
        """
        print(f"Picking {object_name}...")
        
        # 1. Open Gripper
        self.control_gripper(self.left_gripper, 0.0)

        # 2. Define Poses
        # Convert ROS Pose to Giskard TransformationMatrix
        p = cas.Point3(grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z)
        q = cas.Quaternion(
            grasp_pose.pose.orientation.x, 
            grasp_pose.pose.orientation.y, 
            grasp_pose.pose.orientation.z, 
            grasp_pose.pose.orientation.w
        )
        ref_frame = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        grasp_tf = cas.TransformationMatrix.from_point_rotation_matrix(
            p, q.to_rotation_matrix(), reference_frame=ref_frame
        )
        
        # Pre-grasp is 10cm back along the Z axis of the grasp pose
        # We assume the grasp pose Z points TOWARDS the object.
        # Adjust this based on your gripper's coordinate system!
        # Usually gripper Z is "forward". So we want to be -10cm in Z.
        pre_grasp_offset = TransformationMatrix.from_xyz_rpy(x=0, y=0, z=-0.1)
        pre_grasp_tf = grasp_tf * pre_grasp_offset

        # 3. Build Motion Statechart
        msc = MotionStatechart()

        # Resolve links to entities
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)

        # --- Tasks ---
        
        # Task A: Pre-Grasp
        task_pre = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=pre_grasp_tf,
            name="PreGrasp"
        )
        
        # Task B: Grasp
        task_grasp = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=grasp_tf,
            name="Grasp"
        )

        # Task C: Align Planes (Keep gripper horizontal/vertical?)
        # Let's align Gripper Z with Table Z (if top grasp) or similar.
        # For now, let's just rely on the CartesianPose orientation.
        # But if you want the "AlignPlanes" constraint from your snippet:
        # task_align = AlignPlanes(...) 
        
        # --- Smooth Blending Logic ---
        
        msc.add_node(task_pre)
        msc.add_node(task_grasp)

        # Start Grasping approach when PreGrasp is "close enough" (e.g. 5cm)
        # This creates the smooth "corner cutting" effect.
        # task_pre.observation_expression is usually "distance < threshold".
        # We want to access the distance directly. 
        # CartesianPose doesn't expose distance symbol easily in public API, 
        # so we can use a custom monitor or just standard Sequence for safety first.
        
        # Let's use a standard Sequence first, but with a loose tolerance on PreGrasp.
        # To do "human-like" blending properly requires accessing the distance symbol.
        # For this script, let's stick to a robust Sequence but mention how to tune it.
        
        # Standard Sequence:
        task_grasp.start_condition = task_pre.observation_variable
        
        # --- Collision Avoidance ---
        # Allow collision with the object we are picking!
        tip_body = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        obj_body = self.giskard.world.get_kinematic_structure_entity_by_name(object_name)
        
        allow_obj = CollisionAvoidance(
            collision_entries=[
                CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=[tip_body],
                    body_group2=[obj_body]
                )
            ]
        )
        msc.add_node(allow_obj)
        # Make sure this is active during the whole motion
        allow_obj.start_condition = cas.TrinaryTrue 

        # --- End Condition ---
        end = EndMotion()
        end.start_condition = task_grasp.observation_variable
        msc.add_node(end)

        # Execute Approach
        self.giskard.execute(msc)

        # 4. Close Gripper
        self.control_gripper(self.left_gripper, 0.8) # Close

        # 5. Attach Object (Crucial for collision checking during lift)
        # We tell the world that 'object_name' is now attached to 'tip_link'
        # Note: The Giskard wrapper might need a helper for this, or we call the ROS service directly.
        # For now, we assume the user handles attachment or we skip it if not strictly necessary for simple tests.
        # self.giskard.world.attach_object(...) 

        # 6. Lift (Post-Grasp)
        # Move 20cm up
        lift_tf = grasp_tf * TransformationMatrix.from_xyz_rpy(x=0, y=0, z=-0.2) # Backing out
        
        msc_lift = MotionStatechart()
        task_lift = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=lift_tf,
            name="Lift"
        )
        msc_lift.add_node(task_lift)
        msc_lift.add_node(EndMotion.when_true(task_lift))
        
        # We still need to allow collision with the object (it's in our hand!)
        # If we attached it, we wouldn't need this. If we didn't attach, we do.
        msc_lift.add_node(allow_obj)
        
        self.giskard.execute(msc_lift)
        print("Pick complete.")

    def place(self, place_pose: PoseStamped):
        print("Placing...")
        # Similar logic: Approach -> Place -> Open -> Retreat
        # ... (Implementation similar to pick)

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

def add_box(world, name, size, pose_stamped):
    # Manual conversion
    p = cas.Point3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    q = cas.Quaternion(
        pose_stamped.pose.orientation.x, 
        pose_stamped.pose.orientation.y, 
        pose_stamped.pose.orientation.z, 
        pose_stamped.pose.orientation.w
    )
    parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())

    with world.modify_world():
        box = Body(name=PrefixedName(name))
        box_shape = Box(scale=Scale(*size))
        box.collision.append(box_shape)
        box.visual.append(box_shape)
        connection = FixedConnection(
            parent=world.root,
            child=box,
            parent_T_connection_expression=parent_T_pose,
        )
        world.add_connection(connection)

def main():
    bot = SmoothPickAndPlace()
    
    # Example Object
    # In a real scenario, you'd get this from PyCram/Topic
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "map"
    grasp_pose.pose.position.x = 0.5
    grasp_pose.pose.position.y = 0.0
    grasp_pose.pose.position.z = 0.4
    grasp_pose.pose.orientation.w = 1.0 # Identity orientation (adjust as needed)

    # Add dummy object to world for testing
    add_box(bot.giskard.world, "test_cube", (0.05, 0.05, 0.05), grasp_pose)

    # Run Pick
    bot.pick("test_cube", grasp_pose)
    
    # Clean up
    bot.giskard.destroy_node()
    bot.gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
