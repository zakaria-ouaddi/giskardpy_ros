#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time

# Giskard Engine
from motion_engine import GiskardMotionEngine
# Helper imports
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

# --- Helpers ---
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

def add_cube(engine, name, size, pose):
    """ Spawns a box in the Giskard world """
    p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    q = cas.Quaternion(
        pose.pose.orientation.x, 
        pose.pose.orientation.y, 
        pose.pose.orientation.z, 
        pose.pose.orientation.w
    )
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    with engine.giskard.world.modify_world():
        obj = Body(name=PrefixedName(name))
        shape = Box(scale=Scale(*size))
        obj.collision.append(shape)
        obj.visual.append(shape)
        connection = FixedConnection(
            parent=engine.giskard.world.root,
            child=obj,
            parent_T_connection_expression=parent_T_pose,
        )
        try:
            engine.giskard.world.add_connection(connection)
        except:
            print(f"Object {name} might already exist.")

class DualArmTest:
    def __init__(self):
        # We initialize with a default tip, but we'll specific tips for dual arm
        self.engine = GiskardMotionEngine(tip_link="tracy/r_gripper_tool_frame")
        
        # Gripper Controllers (Stub/Simulated or Real via ROS)
        self.gripper_node = rclpy.create_node('gripper_client_dual')
        # In a real scenario, use ActionClients. Here we just print.
    
    def gripper_cmd(self, side, value):
        # 0.0 = Open, 0.8 = Closed
        print(f"[{side.upper()} GRIPPER] Command: {value}")
        # Add sleep to simulate gripper movement time
        time.sleep(0.5)

    def run(self):
        print("--- STARTING DUAL-ARM HANDOVER TEST ---")
        
        # 0. Setup
        cube_name = "handover_cube"
        cube_pose = create_pose(0.9, -0.2, 0.95, frame="map2")
        add_cube(self.engine, cube_name, (0.05, 0.05, 0.05), cube_pose)
        
        # Explicitly allow collision with the cube and self-collision globally
        self.engine.allow_gripper_self_collision()
        self.engine.allow_all_gripper_collisions(cube_name)
        
        # 1. Right Arm Pick
        print("\n[PHASE 1] Right Arm Pick")
        self.gripper_cmd("right", 0.0) # Open
        
        # Pre-Pick (Above)
        pose_pre_pick = create_pose(0.9, -0.2, 1.15, pitch=3.14, yaw=1.57) # Vertical
        print("Moving Right to Pre-Pick...")
        self.engine.move_to_pose(pose_pre_pick, object_to_allow_collision=cube_name)
        
        # Pick (Down)
        pose_pick = create_pose(0.9, -0.2, 0.95, pitch=3.14, yaw=1.57)
        print("Moving Right to Pick...")
        # Allow table collision for pick
        self.engine.move_to_pose(pose_pick, object_to_allow_collision=cube_name, environment_objects_to_allow_collision=["table"])
        
        self.gripper_cmd("right", 0.8) # Close
        self.engine.attach_object(cube_name, "tracy/r_gripper_tool_frame")
        
        # Lift
        print("Lifting Right...")
        self.engine.move_to_pose(pose_pre_pick, object_to_allow_collision=cube_name)
        
        # 2. Dual Arm Meet (Exchange Point)
        print("\n[PHASE 2] Dual Arm Meeting")
        
        # Exchange Point (Center, High)
        # Exchange Point (Center)
        # Right Arm holds object at (0.65, -0.05, 1.1)
        # Orientation: Pointing +Y (Left). 
        # Roll=-1.57 (Z -> +Y). 
        pose_exchange_right = create_pose(0.65, -0.05, 1.1, roll=-1.57, pitch=0.0, yaw=0.0)
        
        # Left Arm Approaches from LEFT side (Y+)
        # Orientation: Pointing -Y (Right).
        # Roll=1.57 (Z -> -Y).
        # ORTHOGONAL: To grasp Top/Bottom instead of Front/Back?
        # If Right Arm (Roll=-1.57) has fingers Vertical (along Z world? No, along X world?).
        # We need to rotate Left Arm around its approach axis (Z).
        # If Approach is -Y. Rotating around Y.
        # Let's add Pitch=1.57 to separate fingers.
        
        # Pre-Exchange: 20cm away (Y=0.25)
        pose_exchange_left_pre = create_pose(0.65, 0.25, 1.1, roll=1.57, pitch=1.57, yaw=0.0) 
        
        print("Moving Both Arms to Exchange Init...")
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_pre,
            right_pose=pose_exchange_right,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        # 3. Handover Interaction
        print("\n[PHASE 3] Handover Approach")
        
        # Left Arm moves to Cube (0.65, -0.05, 1.1)
        # Same orientation (Roll=1.57, Pitch=1.57, Yaw=0)
        pose_exchange_left_grasp = create_pose(0.65, -0.05, 1.1, roll=1.57, pitch=1.57, yaw=0.0)
        
        # CRITICAL: We use move_dual_arm to keep Right Arm FIXED while Left moves.
        # Right Goal = Same as before.
        print("Left Arm Approaching...")
        
        # Allow Left Gripper to touch Cube (held by Right) collision
        # Also Allow Left Gripper to touch Right Gripper (Near miss)
        # We need to use `allow_collision` manually or via helper?
        # The engine methods take collision args but usually for the moving tip.
        # Does move_dual_arm support collision allowances?
        # Checking implementation... It does NOT take object_to_allow_collision.
        # We must set global allowances manually.
        
        # Allow Left Gripper <-> Cube
        # Also allow Gripper <-> Gripper (Left touches Right)
        self.engine.allow_gripper_self_collision()
        self.engine.allow_all_gripper_collisions(cube_name)
        
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_grasp,
            right_pose=pose_exchange_right, # Logic: Hold Right Still
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        # 4. Exchange
        print("\n[PHASE 4] Exchange")
        self.gripper_cmd("left", 0.8) # Close Left
        
        # Update World State
        self.engine.detach_object(cube_name) # Detach from Right
        self.engine.attach_object(cube_name, "tracy/l_gripper_tool_frame") # Attach to Left
        
        self.gripper_cmd("right", 0.0) # Open Right
        
        # 5. Separation
        print("\n[PHASE 5] Separation")
        
        # Right Arm Retreats (Right)
        # Left Arm Holds (Stationary)
        pose_retreat_right = create_pose(0.65, -0.2, 1.1, roll=-1.57, pitch=0.0, yaw=0.0)
        # Left Arm Goal = Same (Hold)
        
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_grasp, # Hold
            right_pose=pose_retreat_right,      # Retreat
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        
        # 6. Final Place (Left Arm)
        print("\n[PHASE 6] Left Arm Place")
        # Move Left Arm away from center
        pose_place_left = create_pose(0.65, 0.4, 1.1, roll=1.57, pitch=0.0, yaw=0.0)
        # Right Arm stays safe
        self.engine.move_dual_arm(
            left_pose=pose_place_left,
            right_pose=pose_retreat_right,  # Keep Right in retreat position
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        
        print("--- TEST COMPLETE ---")

    def destroy(self):
        self.engine.destroy()
        self.gripper_node.destroy_node()

def main():
    rclpy.init()
    test = DualArmTest()
    try:
        test.run()
    except Exception as e:
        print(f"Test Failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        test.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
