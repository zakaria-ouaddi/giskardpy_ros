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
from config import DualArmConfig
from skills import PickSkill, PlaceSkill, HandoverSkill, GripperController
from skills.utils import create_pose

class DualArmOrchestrator:
    def __init__(self):
        self.config = DualArmConfig()
        # Initialize Engine
        self.engine = GiskardMotionEngine(tip_link=self.config.RIGHT_TIP) # Default tip
        self.gripper_node = rclpy.create_node('gripper_client_dual_main')
        
        # Initialize Gripper Controller
        self.gripper_controller = GripperController(node=self.gripper_node)

        # Initialize Skills
        self.picker = PickSkill(self.engine, self.gripper_cmd)
        self.placer = PlaceSkill(self.engine, self.gripper_cmd)
        self.hander = HandoverSkill(self.engine, self.gripper_cmd)
        
    def gripper_cmd(self, side, value, effort=10.0):
        """
        Callback to control real grippers using GripperController.
        value: 0.0 (Open) to 0.8 (Closed)
        """
        self.gripper_controller.command(side, value, effort=effort)

    def spawn_cube(self, name, x, y, z):
        """Spawns a box in the Giskard world model."""
        pose = create_pose(x, y, z, frame=self.config.ROOT_FRAME)
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
        time.sleep(0.5)

    def run_sequence(self):
        print("--- SETUP ---")
        
        cubes = []
        # Spawn logic derived from old script
        for i in range(3):
            name = f"cube_{i}"
            z = self.config.BASE_Z + (i * self.config.CUBE_HEIGHT)
            self.spawn_cube(name, self.config.RIGHT_ARM_START_X, self.config.RIGHT_ARM_START_Y, z)
            cubes.append({"name": name, "pick_z": z})
            
        print(f"Spawned {len(cubes)} cubes.")
        
        # We iterate from Top Cube to Bottom Cube
        for i in range(len(cubes) - 1, -1, -1):
            cube = cubes[i]
            cube_name = cube['name']
            pick_z = cube['pick_z']
            
            # Destination Logic
            placed_idx = (len(cubes) - 1) - i
            place_z = self.config.PLACE_Z_BASE + (placed_idx * self.config.CUBE_HEIGHT)
            
            print(f"\n=== PROCESSING {cube_name} ===")
            
            all_cube_names = [c['name'] for c in cubes]
            other_cubes = [c for c in all_cube_names if c != cube_name] # Environment for collision
            
            # 1. Pick (Right)
            pick_pose = create_pose(
                self.config.RIGHT_ARM_START_X, 
                self.config.RIGHT_ARM_START_Y, 
                pick_z, 
                pitch=3.14, yaw=1.57 # Grip at center
            )
            
            self.picker.execute_with_pose(
                cube_name, 
                pick_pose, 
                arm="right", 
                environment_objects=other_cubes
            )
            
            # 2. Handover (Right -> Left)
            self.hander.execute(giver="right", receiver="left", environment_objects=other_cubes)
            
            # 3. Place (Left)
            # Place Pose: The target location
            # Pitch=3.14 ensures vertical (downward) gripper
            pose_place = create_pose(
                self.config.PLACE_X, 
                self.config.PLACE_Y, 
                place_z, 
                roll=0.0, pitch=3.14, yaw=1.57
            )
            
            # We allow collision with "other_cubes" because we are stacking ON TOP of previously placed ones?
            # Wait, `other_cubes` contains ALL other cubes.
            # If we are placing "cube_2" (top of source), we might stack on "cube_?" (placed stack).
            # The logic in original script was: allow collision with ALL previously placed cubes.
            # `other_cubes` effectively captures all other cubes in the world, which is safe enough.
            
            self.placer.execute(
                cube_name, 
                pose_place, 
                arm="left", 
                stack_on=other_cubes
            )

    def destroy(self):
        self.engine.destroy()
        self.gripper_node.destroy_node()

def main():
    import sys
    
    # Check for --step or --confirm-steps flag
    step_mode = "--step" in sys.argv or "--confirm-steps" in sys.argv
    
    rclpy.init()
    orchestrator = DualArmOrchestrator()
    
    if step_mode:
        print("\n[STEP MODE ENABLED] Execution will pause before every goal. Press 'y' to continue.\n")
        
        # Monkey-patch the execute method of the inner giskard object
        original_execute = orchestrator.engine.giskard.execute
        
        def stepped_execute(msc):
            print(">>> About to send goal to Giskard...")
            while True:
                user_input = input(">>> Press 'y' to execute, 'n' to abort: ").strip().lower()
                if user_input == 'y':
                    break
                elif user_input == 'n':
                    print("Aborting execution based on user input.")
                    raise KeyboardInterrupt("User aborted execution.")
            return original_execute(msc)
            
        orchestrator.engine.giskard.execute = stepped_execute

    try:
        orchestrator.run_sequence()
    except Exception as e:
        print(f"Sequence Failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        orchestrator.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
