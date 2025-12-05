#!/usr/bin/env python3
import rclpy
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.model.collision_matrix_manager import CollisionRequest, CollisionAvoidanceTypes
from giskardpy.motion_statechart.graph_node import EndMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time

# Import our Motion Engine
# Note: This assumes you run this script from the same directory, 
# or that motion_engine is in your PYTHONPATH.
from motion_engine import GiskardMotionEngine

# --- Gripper Controller (The "Other System") ---
class GripperController:
    def __init__(self, action_server='/right_gripper/robotiq_gripper_controller/gripper_cmd', node_name='gripper_test_client'):
        self.node = rclpy.create_node(node_name)
        # Adjust these topic names to match your real robot
        self.left_client = ActionClient(self.node, GripperCommand, action_server)
        
    def command(self, position: float, effort: float = 20.0):
        """ 0.0 = Open, 0.8 = Closed """
        print(f"Gripper: Moving to {position}...")
        if not self.left_client.wait_for_server(timeout_sec=2.0):
            print("Gripper server not found! Skipping gripper command.")
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        
        future = self.left_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        # In a real script, wait for result...
        time.sleep(1.0) # Simulate wait
        print("Gripper: Done.")

    def destroy(self):
        self.node.destroy_node()

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.spatial_types import TransformationMatrix

# ... Helper to create poses ...
def create_pose(x, y, z, roll=0, pitch=0, yaw=0, frame="map2"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    q = quaternion_from_euler(roll, pitch, yaw)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p

import semantic_digital_twin.spatial_types.spatial_types as cas

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
    rclpy.init()
    # 1. Setup
    # Use Left Gripper
    gripper = GripperController(action_server="/right_gripper/gripper_cmd")
    
    # Use Left Arm Tip Link
    motion = GiskardMotionEngine(tip_link="r_gripper_tool_frame")

    # 2. Add Object to World (Left side)
    obj_name = "milk_box"
    obj_pose = create_pose(0.8, -0.2, 0.9) # y=0.4 is on the left
    
    # Add box (using GiskardTester utility or similar logic if available, 
    # but here we use the motion engine's world interface if we had one, 
    # or just assume it's there. 
    # For this test script, we just define the pose for the robot to go to.)
    # In a real test, we might want to spawn it. 
    # But let's assume the user just wants the robot to move to these coords.
    add_box(motion.giskard.world, obj_name, (0.05, 0.05, 0.05), obj_pose)
    
    # 3. Define Poses (Simulating "Planning Team" Output)
    # Pick Sequence (Left side)
    # Pre-pick: 20cm above object
    # Pitch=3.14 for top-down (pointing down, rotated 180 deg relative to roll=3.14)
    pose_pre_pick = create_pose(0.8, -0.2, 1.1, pitch=3.14)
    # Pick: At object
    pose_pick     = create_pose(0.8, -0.2, 0.9, pitch=3.14)
    
    # Place Sequence (Left side)
    pose_lift      = create_pose(0.8, -0.2, 1.2, pitch=3.14)
    pose_pre_place = create_pose(0.8, 0.2, 1.2, pitch=3.14)
    pose_place     = create_pose(0.8, 0.2, 1.0, pitch=3.14)
    pose_retreat   = create_pose(0.8, 0.2, 1.2, pitch=3.14)

    try:
        print("--- STARTING TEST ---")

        # A. Open Gripper
        print("Gripper: Moving to 0.0...")
        gripper.command(0.0)

        # B. Move to Pick (Smooth Sequence: Pre -> Pick)
        print("Motion: Executing Pick Sequence (Pre -> Pick)...")
        # We tell it to allow collision with 'milk_box' because we are touching it
        motion.execute_smooth_sequence([pose_pre_pick, pose_pick], object_to_allow_collision=obj_name)
        # execute_with_all_collisions(motion, [pose_pre_pick, pose_pick])
        print("Motion: Pick Sequence Complete.")

        # C. Close Gripper
        print("Gripper: Closing...")
        gripper.command(0.8)
        
        # Attach object in Giskard World so it moves with us
        motion.attach_object(obj_name, motion.tip_link)

        # D. Lift & Move to Place (Split into steps for debugging)
        print("Motion: Executing Lift...")
        # Allow collision with "table" because the box is sitting on it when we start lifting
        motion.move_to_pose(
            pose_lift, 
            object_to_allow_collision=obj_name,
            environment_objects_to_allow_collision=["table"]
        )
        # execute_with_all_collisions(motion, [pose_lift])
        
        print("Motion: Executing Pre-Place...")
        motion.move_to_pose(pose_pre_place, object_to_allow_collision=obj_name)
        # execute_with_all_collisions(motion, [pose_pre_place])
        
        print("Motion: Executing Place...")
        motion.move_to_pose(
            pose_place, 
            object_to_allow_collision=obj_name,
            environment_objects_to_allow_collision=["table"]
        )
        # execute_with_all_collisions(motion, [pose_place])
        print("Motion: Place Sequence Complete.")

        # E. Open Gripper
        print("Gripper: Opening...")
        gripper.command(0.0)
        
        # Detach
        motion.detach_object(obj_name)

        # F. Retreat
        print("Motion: Retreating...")
        motion.move_to_pose(pose_retreat)
        # execute_with_all_collisions(motion, [pose_retreat])

        print("--- TEST COMPLETE ---")

    except KeyboardInterrupt:
        pass
    finally:
        gripper.destroy()
        motion.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
