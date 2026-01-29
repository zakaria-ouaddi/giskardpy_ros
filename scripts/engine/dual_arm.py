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
        # Spawn 3 cubes stacked on top of each other
        # Right Arm Start Position: (0.9, -0.2)
        base_x, base_y, base_z = 0.9, -0.2, 0.95
        cube_height = 0.05
        
        cubes = []
        for i in range(3):
            name = f"cube_{i}"
            z = base_z + (i * cube_height)
            pose = create_pose(base_x, base_y, z, frame="map2")
            self.spawn_cube(name, pose)
            cubes.append({"name": name, "pick_z": z})
            
        print(f"Spawned {len(cubes)} cubes.")
        
        # Place Target (Left Arm side)
        place_x, place_y, place_z_base = 0.65, 0.4, 0.95
        
        # We iterate from Top Cube to Bottom Cube
        # Top cube is the last one in the list
        for i in range(len(cubes) - 1, -1, -1):
            cube = cubes[i]
            print(f"\n=== PROCESSING {cube['name']} ===")
            
            pick_z = cube["pick_z"]
            # New Stack Height: 0 for first placed, 1 for second...
            # The 'i' index counts down: 2, 1, 0.
            # We want placed index: 0, 1, 2.
            placed_idx = (len(cubes) - 1) - i
            place_z = place_z_base + (placed_idx * cube_height)
            
            self.process_cube(cube['name'], pick_z, place_z, [c['name'] for c in cubes])

    def process_cube(self, name, pick_z, place_z, all_cubes):
        """
        Full lifecycle for one cube: Pick (Right) -> Handover -> Place (Left)
        """
        print(f"[{name}] Picking at z={pick_z:.3f}, Placing at z={place_z:.3f}")
        
        other_cubes = [c for c in all_cubes if c != name]
        
        # --- STEP 1: PICK (Right Arm) ---
        print(f"[{name}] Phase 1: Picking (Right)")
        self.engine.allow_all_gripper_collisions(name)
        # Allow gripper to touch other cubes in the stack (e.g. while picking top one)
        for other in other_cubes:
            self.engine.allow_all_gripper_collisions(other)
        
        # Pick Poses
        pose_pick = create_pose(0.9, -0.2, pick_z, pitch=3.14, yaw=1.57) # Grip at center
        # Lift slightly higher to clear the stack below (if any)
        pose_lift = create_pose(0.9, -0.2, pick_z + 0.05, pitch=3.14, yaw=1.57)
        
        self.gripper_cmd("right", 0.0) # Open
        # Allow moving to pick while touching other cubes (if tight stack)
        self.engine.move_to_pose(pose_pick, object_to_allow_collision=name, 
                                 environment_objects_to_allow_collision=other_cubes,
                                 linear_speed=LINEAR_SPEED, angular_speed=ANGULAR_SPEED)
        time.sleep(2)
        self.gripper_cmd("right", 0.8) # Close
        self.engine.attach_object(name, "tracy/r_gripper_tool_frame")
        
        # When lifting, allowed to touch other cubes (dragging effect)
        self.engine.move_to_pose(pose_lift, 
                                 object_to_allow_collision=name,
                                 environment_objects_to_allow_collision=other_cubes,
                                 linear_speed=LINEAR_SPEED, angular_speed=ANGULAR_SPEED)
        time.sleep(2)
        # --- STEP 2: HANDOVER ---
        print(f"[{name}] Phase 2: Handover")
        meeting_pose = create_pose(0.65, -0.05, 1.1)
        success = self.handover_manager.execute_handover(
            meeting_pose, 
            approach_offset=0.30, 
            retreat_offset=0.15,
            linear_speed=LINEAR_SPEED,
            angular_speed=ANGULAR_SPEED
        )
        time.sleep(2)
        if not success:
            print("Handover failed! Aborting.")
            return

        # --- STEP 3: PLACE (Left Arm) ---
        print(f"[{name}] Phase 3: Placing (Left)")
        
        # Calculate Poses
        # Place Pose: The target location
        pose_place = create_pose(0.65, 0.4, place_z, roll=1.57, pitch=0.0, yaw=0.0)
        # Approach/Retreat for Left Arm (above place)
        pose_place_pre = create_pose(0.65, 0.4, place_z + 0.10, roll=1.57, pitch=0.0, yaw=0.0)
        
        # Right Arm Retreat: Hold it at its safe spot (from handover retreat)
        # Handover retreat was at y = -0.05 - 0.15 = -0.20
        pose_right_safe = create_pose(0.65, -0.2, 1.1, roll=-1.57, pitch=0.0, yaw=0.0)
        
        # Move to Pre-Place
        self.engine.move_dual_arm(
            left_pose=pose_place_pre,
            right_pose=pose_right_safe,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame",
            linear_speed=LINEAR_SPEED,
            angular_speed=ANGULAR_SPEED
        )
        time.sleep(2)
        # Move to Place
        # Allow collision with other cubes? 
        # If we are stacking, we might touch the cube below.
        # We should allow collision with ALL previously placed cubes.
        # Simple fix: Allow collision with "cube_0", "cube_1", "cube_2".
        for c_env in other_cubes:
             # Allow the HELD object (name) to touch the existing stack (c_env)
             self.engine.allow_collision([name], c_env)
        
        self.engine.move_dual_arm(
            left_pose=pose_place,
            right_pose=pose_right_safe,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame",
            linear_speed=LINEAR_SPEED,
            angular_speed=ANGULAR_SPEED
        )
        time.sleep(2)
        # Release
        self.gripper_cmd("left", 0.0)
        self.engine.detach_object(name)
        
        # Retreat Left Arm
        self.engine.move_dual_arm(
            left_pose=pose_place_pre,
            right_pose=pose_right_safe,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame",
            linear_speed=LINEAR_SPEED,
            angular_speed=ANGULAR_SPEED
        )
        time.sleep(2)
        # Cleanup Collision Allows to prevent state bloat/conflict in next iteration
        self.engine.clear_collision_allowance() 

    def spawn_cube(self, name, pose):
        """Spawns a box in the Giskard world model."""
        p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        q = cas.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
        
        with self.engine.giskard.world.modify_world():
            # Robustly remove existing object if it exists
            try:
                existing = self.engine.giskard.world.get_body_by_name(name)
                self.engine.giskard.world.remove_connection(existing.parent_connection)
            except:
                pass
            
            obj = Body(name=PrefixedName(name))
            shape = Box(scale=Scale(0.05, 0.05, 0.05))
            obj.collision.append(shape)
            obj.visual.append(shape)
            conn = FixedConnection(self.engine.giskard.world.root, obj, parent_T_pose)
            self.engine.giskard.world.add_connection(conn)
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