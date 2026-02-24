#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from giskardpy_ros.ros2 import rospy
from motion_engine import GiskardMotionEngine
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Cylinder
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
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())

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
        # Check if exists, remove? But simpler to assume unique names for test.
        try:
             world.add_connection(connection)
        except:
             print(f"Object {name} might already exist.")

def add_cylinder(world, name, size, pose_stamped):
    """
    Helper to add a cylinder (for dual arm test).
    """
    p = cas.Point3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    q = cas.Quaternion(
        pose_stamped.pose.orientation.x, 
        pose_stamped.pose.orientation.y, 
        pose_stamped.pose.orientation.z, 
        pose_stamped.pose.orientation.w
    )
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())

    with world.modify_world():
        obj = Body(name=PrefixedName(name))
        # Cylinder size is (radius, height). Scale uses (x, y, z)? No, Cylinder usually takes radius/height kwargs.
        # But Giskard Cylinder might be different. Let's use Box for simplicity if cylinder fails, 
        # but let's try Cylinder geometry.
        # Checking imports... from semantic_digital_twin.world_description.geometry import Cylinder
        # It likely takes (radius, length).
        shape = Cylinder(radius=size[0], length=size[1])
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
    # Pose: Accessible on table
    p1 = PoseStamped()
    p1.header.frame_id = "world"
    p1.pose.position.x = 0.8
    p1.pose.position.y = 0.2
    p1.pose.position.z = 0.95 # Table height approx
    p1.pose.orientation.w = 1.0
    add_box(engine.giskard.world, "small_part", (0.02, 0.02, 0.02), p1)
    
    # 2. Long Bar (for dual arm)
    # Pose: Centered in front
    p2 = PoseStamped()
    p2.header.frame_id = "world"
    p2.pose.position.x = 0.8
    p2.pose.position.y = 0.0
    p2.pose.position.z = 1.0 # Lifted slightly? Or on table? Let's say 0.95 + radius (0.02)
    p2.pose.orientation.w = 1.0 
    # Let's assume Box is X-aligned.
    add_box(engine.giskard.world, "long_bar", (0.04, 0.4, 0.04), p2) 
    
    # 3. Screw Hole (for spiral search)
    p3 = PoseStamped()
    p3.header.frame_id = "world"
    p3.pose.position.x = 0.8
    p3.pose.position.y = -0.2
    p3.pose.position.z = 0.95
    p3.pose.orientation.w = 1.0
    add_box(engine.giskard.world, "screw_hole", (0.1, 0.1, 0.01), p3)

def test_small_pick(engine):
    print("\n--- TEST 1: Small Object Grasp (Complex Approach) ---")
    
    # 1. Allow Table Collision globally (Persistent)
    engine.allow_collision(['map2/l_gripper_tool_frame', 'map2/r_gripper_tool_frame'], 'table') 
    
    # 2. Define Grasp Pose for 'small_part'
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "world"
    grasp_pose.pose.position.x = 0.8
    grasp_pose.pose.position.y = 0.2
    grasp_pose.pose.position.z = 0.95
    # Orientation: Vertical down grasp.
    grasp_pose.pose.orientation.x = 1.0
    grasp_pose.pose.orientation.y = 0.0
    grasp_pose.pose.orientation.z = 0.0
    grasp_pose.pose.orientation.w = 0.0
    
    # 3. Execute Complex Grasp
    # Approach from 10cm above (Z is -0.1 in Gripper Frame usually)
    engine.execute_complex_grasp(grasp_pose, "small_part", approach_offset=[0, 0, -0.1])
    
    # 4. Simulate Close & Lift
    print("Simulating Gripper Close...")
    engine.attach_object("small_part", "map2/l_gripper_tool_frame") # Left arm default
    
    lift_pose = PoseStamped()
    lift_pose.header.frame_id = "world"
    lift_pose.pose.position.x = 0.8
    lift_pose.pose.position.y = 0.2
    lift_pose.pose.position.z = 1.1 # Lift up
    lift_pose.pose.orientation = grasp_pose.pose.orientation
    
    engine.move_cartesian_constrained(lift_pose) # Keep orientation
    print("Test 1 Complete.")

def test_dual_manipulation(engine):
    print("\n--- TEST 2: Dual-Arm Rotation ---")
    
    # 1. Define Grasp Poses for Left and Right ends of "long_bar"
    # Bar is at (0.8, 0.0, 1.0), length 0.4 (Y-width).
    # Left End: (0.8, 0.15, 1.0)
    # Right End: (0.8, -0.15, 1.0)
    
    left_pose = PoseStamped()
    left_pose.header.frame_id = "world"
    left_pose.pose.position.x = 0.8
    left_pose.pose.position.y = 0.15
    left_pose.pose.position.z = 1.0
    left_pose.pose.orientation.x = 1.0 # Down
    
    right_pose = PoseStamped()
    right_pose.header.frame_id = "world"
    right_pose.pose.position.x = 0.8
    right_pose.pose.position.y = -0.15
    right_pose.pose.position.z = 1.0
    right_pose.pose.orientation.x = 1.0 # Down
    
    # 2. Move to Grasp
    engine.move_dual_arm(left_pose, right_pose)
    
    # 3. Simulate Attach
    engine.attach_object("long_bar", "map2/l_gripper_tool_frame") 
    
    # 4. Rotate! 
    # Move Left UP, Right DOWN.
    left_rot = PoseStamped()
    left_rot.header.frame_id = "world"
    left_rot.pose.position.x = 0.8
    left_rot.pose.position.y = 0.15
    left_rot.pose.position.z = 1.1
    left_rot.pose.orientation.x = 1.0
    
    right_rot = PoseStamped()
    right_rot.header.frame_id = "world"
    right_rot.pose.position.x = 0.8
    right_rot.pose.position.y = -0.15
    right_rot.pose.position.z = 0.9
    right_rot.pose.orientation.x = 1.0
    
    engine.move_dual_arm(left_rot, right_rot)
    
    # Detach
    engine.detach_object("long_bar")
    print("Test 2 Complete.")

def test_handover(engine):
    print("\n--- TEST 3: Handover (Pass Left -> Right) ---")
    
    # 1. Pick object with Left (simulated, assume already holding something or just move)
    # Move Left to Center
    center_pose = PoseStamped()
    center_pose.header.frame_id = "world"
    center_pose.pose.position.x = 0.6
    center_pose.pose.position.y = 0.0
    center_pose.pose.position.z = 1.0
    center_pose.pose.orientation.x = 1.0
    
    engine.move_to_pose(center_pose) # Left arm default
    
    # 2. Move Right to meet it (offset slightly)
    meet_pose = PoseStamped()
    meet_pose.header.frame_id = "world"
    meet_pose.pose.position.x = 0.6
    meet_pose.pose.position.y = -0.05 # Slightly to the right
    meet_pose.pose.position.z = 1.0
    meet_pose.pose.orientation.x = 1.0
    
    # This requires moving Right Arm specifically. 
    # Use move_dual_arm to hold Left still and move Right.
    engine.move_dual_arm(center_pose, meet_pose)
    
    print("Test 3 Complete.")

def test_screwing(engine):
    print("\n--- TEST 4: Screwing (Spiral Search) ---")
    
    # 1. Move to "Search Start" above screw hole
    start_pose = PoseStamped()
    start_pose.header.frame_id = "world"
    start_pose.pose.position.x = 0.8
    start_pose.pose.position.y = -0.2
    start_pose.pose.position.z = 1.0
    start_pose.pose.orientation.x = 1.0
    
    # 2. Execute Spiral Search
    # Push down 5cm
    engine.execute_spiral_search(start_pose, max_radius=0.03, duration=5.0, push_depth=0.05)
    
    # 3. Simulate "Screwing" Rotation
    # Move wrist joint
    # engine.move_joints({'tracy_left_arm_wrist_3_joint': 3.14}) # Example joint name
    print("Test 4 Complete.")

def main():
    engine = GiskardMotionEngine(tip_link="map2/l_gripper_tool_frame")
    
    # Spawn
    spawn_objects(engine)
    
    try:
        test_small_pick(engine)
        test_dual_manipulation(engine)
        test_handover(engine)
        test_screwing(engine)
    except Exception as e:
        print(f"Test Failed: {e}")
        import traceback
        traceback.print_exc()

    engine.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
