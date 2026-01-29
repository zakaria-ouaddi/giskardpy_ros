#!/usr/bin/env python3
import rclpy
import time
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

# Custom Imports
from motion_engine import GiskardMotionEngine
from handover import HandoverManager, create_pose
import time

# --- SPEED CONFIGURATION ---
LINEAR_SPEED = 0.05
ANGULAR_SPEED = 0.1
JOINT_SPEED = 0.8


class DualArmTestSmart:
    def __init__(self):
        # Initialize Engine with a default tip (doesn't matter much for dual arm as we override it)
        self.engine = GiskardMotionEngine(tip_link="tracy/r_gripper_tool_frame")
        self.gripper_node = rclpy.create_node('gripper_client_dual')
        
        # Initialize the Smart Skill
        # We pass our local gripper_cmd function as the callback
        self.handover_manager = HandoverManager(self.engine, self.gripper_cmd)
    
    def gripper_cmd(self, side, value):
        """
        Callback to simulate or control real grippers.
        value: 0.0 (Open) to 0.8 (Closed)
        """
        print(f"[{side.upper()} GRIPPER] Command: {value}")
        time.sleep(0.5) # Simulate actuation time

    def run(self):
        print("--- SETUP ---")
        cube_name = "smart_cube"
        cube_pose = create_pose(0.9, -0.2, 0.95, frame="map2")
        self.spawn_cube(cube_name, cube_pose)
        
        # 1. PREP: Pick up the cube with the RIGHT arm
        print("\n--- STEP 1: PICKING UP CUBE (Right Arm) ---")
        self.engine.allow_all_gripper_collisions(cube_name)
        
        pose_pick = create_pose(0.9, -0.2, 0.95, pitch=3.14, yaw=1.57)
        pose_lift = create_pose(0.9, -0.2, 1.15, pitch=3.14, yaw=1.57)
        
        self.gripper_cmd("right", 0.0)
        self.engine.move_to_pose(pose_pick, object_to_allow_collision=cube_name, linear_speed=LINEAR_SPEED, angular_speed=ANGULAR_SPEED)
        self.gripper_cmd("right", 0.8)
        self.engine.attach_object(cube_name, "tracy/r_gripper_tool_frame")
        self.engine.move_to_pose(pose_lift, linear_speed=LINEAR_SPEED, angular_speed=ANGULAR_SPEED)

        # 2. EXECUTE HANDOVER (Right -> Left)
        # We simply define the meeting point. The manager detects that Right has the cube.
        # Original script used y=-0.05, approach_offset=0.30 (from 0.25), retreat_offset=0.15 (to -0.20)
        meeting_pose = create_pose(0.65, -0.05, 1.1) 
        
        print("\n--- STEP 2: HANDOVER (Right -> Left) ---")
        success = self.handover_manager.execute_handover(meeting_pose, approach_offset=0.30, retreat_offset=0.15, linear_speed=LINEAR_SPEED, angular_speed=ANGULAR_SPEED)
        
        if success:
             # 3. PLACE (Left Arm)
             # To match original script checks:
             # Left moves to (0.65, 0.4, 1.1) to place.
             # Right stays at retreat pos (0.65, -0.2, 1.1).
             print("\n--- STEP 3: PLACE (Left Arm) ---")
             
             # Calculate poses
             pose_place_left = create_pose(0.65, 0.4, 1.1, roll=1.57, pitch=0.0, yaw=0.0)
             # We need the Right arm's current retreat pose to hold it there.
             # Based on retreat_offset=0.15 from -0.05, that's -0.20.
             pose_retreat_right = create_pose(0.65, -0.2, 1.1, roll=-1.57, pitch=0.0, yaw=0.0)
             
             self.engine.move_dual_arm(
                left_pose=pose_place_left,
                right_pose=pose_retreat_right,
                left_tip="tracy/l_gripper_tool_frame",
                right_tip="tracy/r_gripper_tool_frame",
                linear_speed=LINEAR_SPEED,
                angular_speed=ANGULAR_SPEED
            )
             
             # Detach? Original script didn't explicitly detach in the code shown (it ended), 
             # but usually one would open gripper.
             # Original just moved there. We'll add open gripper for completeness?
             # User asked to "double check my codes to see if I have missed something".
             # Original code:
             # 195: print("\n[PHASE 6] Left Arm Place")
             # ...
             # 207: print("--- TEST COMPLETE ---")
             # It didn't open the gripper! I will stick to exact behavior: Move only.
             # But a 'Place' usually implies releasing. I'll stick to moving.

    def spawn_cube(self, name, pose):
        """Spawns a box in the Giskard world model."""
        p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        q = cas.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
        
        with self.engine.giskard.world.modify_world():
            obj = Body(name=PrefixedName(name))
            shape = Box(scale=Scale(0.05, 0.05, 0.05))
            obj.collision.append(shape)
            obj.visual.append(shape)
            conn = FixedConnection(self.engine.giskard.world.root, obj, parent_T_pose)
            try:
                self.engine.giskard.world.add_connection(conn)
            except: 
                print(f"Object {name} might already exist.")
        time.sleep(0.5) # Wait for spawn propagation

    def destroy(self):
        self.engine.destroy()
        self.gripper_node.destroy_node()

def main():
    rclpy.init()
    test = DualArmTestSmart()
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