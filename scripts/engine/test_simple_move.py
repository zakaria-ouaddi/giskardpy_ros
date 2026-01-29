#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from giskardpy_ros.ros2 import rospy
from motion_engine import GiskardMotionEngine
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

def add_box(world, name, size, pose_stamped):
    """
    Helper to add a box to the Giskard world.
    """
    p = cas.Point3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    q = cas.Quaternion(
        pose_stamped.pose.orientation.x, 
        pose_stamped.pose.orientation.y, 
        pose_stamped.pose.orientation.z, 
        pose_stamped.pose.orientation.w
    )
    parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())

    with world.modify_world():
        obj = Body(name=PrefixedName(name))
        shape = Box(scale=Scale(*size))
        obj.collision.append(shape)
        obj.visual.append(shape)
        connection = FixedConnection(
            parent=world.root,
            child=obj,
            parent_T_connection_expression=parent_T_pose,
        )
        try:
             world.add_connection(connection)
        except:
             print(f"Object {name} might already exist.")

def spawn_objects(engine):
    print("Spawning Test Objects...")
    
    # 1. Small Part (for complex grasp)
    p1 = PoseStamped()
    p1.header.frame_id = "world"
    p1.pose.position.x = 0.9
    p1.pose.position.y = -0.2
    p1.pose.position.z = 0.95 # Table height approx
    p1.pose.orientation.w = 1.0
    add_box(engine.giskard.world, "small_part", (0.05, 0.05, 0.05), p1)

def test_small_pick(engine):
    print("\n--- TEST: Small Object Grasp (Complex Approach) - RIGHT ARM ---")
    
    # 1. Allow Table Collision globally
    # Use allow_all_gripper_collisions to include knuckles/fingers found by heuristic
    # 1. Collision Rules
    # Note: We removed 'table' allowance because it caused the robot to crash into table.
    # The default collision avoidance is safer.
    
    # 1b. Allow Gripper Self-Collision (Internal parts)
    engine.allow_gripper_self_collision() 
    
    # 1c. Constrain Left Arm (Passive)
    # Move/Hold Left arm to a safe "Side" configuration
    left_arm_home = {
        'left_shoulder_pan_joint': 1.57,  # Point Left
        'left_shoulder_lift_joint': -1.5, # Upwards/Back
        'left_elbow_joint': 1.5,          # Bent
        'left_wrist_1_joint': -1.5,
        'left_wrist_2_joint': -1.5,
        'left_wrist_3_joint': 0.0
    }
    engine.hold_joints(left_arm_home) 
    
    # 2. Define Grasp Pose for 'small_part'
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "world"
    # User requested 0.9, -0.2
    grasp_pose.pose.position.x = 0.9
    grasp_pose.pose.position.y = -0.2
    grasp_pose.pose.position.z = 0.95
    # Orientation: Vertical down grasp.
    grasp_pose.pose.orientation.x = 1.0
    grasp_pose.pose.orientation.y = 0.0
    grasp_pose.pose.orientation.z = 0.0
    grasp_pose.pose.orientation.w = 0.0
    
    # 3. Execute Complex Grasp
    # Approach from 10cm above (Z is -0.1 in Gripper Frame usually)
    print("Executing Complex Grasp...")
    # Explicitly use RIGHT gripper (configured in engine)
    engine.execute_complex_grasp(grasp_pose, "small_part", approach_offset=[0, 0, -0.1])
    
    # 4. Simulate Close & Lift
    print("Simulating Gripper Close...")
    # Attach to RIGHT griper
    engine.attach_object("small_part", "tracy/r_gripper_tool_frame") 
    
    lift_pose = PoseStamped()
    lift_pose.header.frame_id = "world"
    lift_pose.pose.position.x = 0.9
    lift_pose.pose.position.y = -0.2
    lift_pose.pose.position.z = 1.1 # Lift up
    lift_pose.pose.orientation = grasp_pose.pose.orientation
    
    print("Lifting...")
    engine.move_cartesian_constrained(lift_pose) # Keep orientation
    print("Test Complete.")

def main():
    # Use right gripper as default tip link
    engine = GiskardMotionEngine(tip_link="tracy/r_gripper_tool_frame")
    
    # Spawn
    spawn_objects(engine)
    
    try:
        test_small_pick(engine)
    except Exception as e:
        print(f"Test Failed: {e}")
        import traceback
        traceback.print_exc()

    engine.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
