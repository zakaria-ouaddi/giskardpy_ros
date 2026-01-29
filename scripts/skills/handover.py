from typing import Tuple, Optional
import time
from geometry_msgs.msg import PoseStamped
from .base import BaseSkill, CollisionContext
from .utils import create_pose
from semantic_digital_twin.world_description.world_entity import Body

class HandoverSkill(BaseSkill):
    """
    A specialized skill class to handle dual-arm handovers robustly.
    It automatically detects which arm is the 'Giver' and which is the 'Receiver'.
    """
    
    def detect_holding_state(self) -> Tuple[Optional[str], Optional[str]]:
        """
        Scans the world to see which gripper is holding an object.
        Returns: (holding_arm_side, object_name) or (None, None)
        """
        left_tip_body = self.engine._resolve_entity_name(self.config.LEFT_TIP)
        right_tip_body = self.engine._resolve_entity_name(self.config.RIGHT_TIP)
        
        holding_side = None
        held_object = None

        # Iterate over all entities in the world to find one attached to a gripper
        # Using the engine's access to world
        for entity in self.engine.giskard.world.kinematic_structure_entities:
            if not isinstance(entity, Body):
                continue
            try:
                # Check the parent of the entity
                parent = entity.parent_connection.parent
                if parent == left_tip_body:
                    holding_side = "left"
                    held_object = str(entity.name)
                    break 
                elif parent == right_tip_body:
                    holding_side = "right"
                    held_object = str(entity.name)
                    break
            except AttributeError:
                continue
        
        return holding_side, held_object

    def execute(self, giver: str = None, receiver: str = None, environment_objects: list = []):
        """
        Executes the handover sequence.
        Note: The 'giver' and 'receiver' args can be used to FORCE a direction.
        environment_objects: List of object names to allow collision with during the process.
        """
        
        meeting_point = create_pose(*self.config.HANDOVER_MEETING_POINT, frame="map2")
        approach_offset = self.config.HANDOVER_APPROACH_OFFSET
        retreat_offset = self.config.HANDOVER_RETREAT_OFFSET
        
        with CollisionContext(self.engine):
            print("\n--- INITIATING SMART HANDOVER ---")
            
            # 1. Detect State (Who is Giver, Who is Receiver)
            holder, obj_name = self.detect_holding_state()
            
            if not holder:
                print("ERROR: No object detected in either hand. Cannot perform handover.")
                return False
            
            # Validation if arguments provided
            if giver and giver != holder:
                print(f"WARNING: Requested giver '{giver}' does not match detected holder '{holder}'. proceeding with detected holder.")
            
            execution_giver = holder
            execution_receiver = "left" if holder == "right" else "right"
            
            print(f"Status: {execution_giver.upper()} is holding '{obj_name}'.")
            print(f"Action: Handover {execution_giver.upper()} -> {execution_receiver.upper()}")

            # 2. Define Geometry (Relative to Meeting Point)
            frame_id = meeting_point.header.frame_id
            x = meeting_point.pose.position.x
            y = meeting_point.pose.position.y
            z = meeting_point.pose.position.z

            # --- ORIENTATION CONFIGURATION ---
            # Offset Logic: Right is -Y side, Left is +Y side
            # We shift them away from zero by HANDOVER_OFFSET
            
            if execution_giver == "right":
                # GIVER (Right): Holds object at center
                # Target Y = y - OFFSET
                pose_giver = create_pose(x, y - self.config.HANDOVER_OFFSET, z, roll=-1.57, pitch=3.14, yaw=0, frame=frame_id)
                # RECEIVER (Left): Approaches from +Y (Left side)
                # Target Y = y + OFFSET
                pose_receiver_grasp = create_pose(x, y + self.config.HANDOVER_OFFSET, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id) 
                pose_receiver_pre = create_pose(x, y + self.config.HANDOVER_OFFSET + approach_offset, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)
                
                giver_tip = self.config.RIGHT_TIP
                receiver_tip = self.config.LEFT_TIP
                
            else: # execution_giver == "left"
                # GIVER (Left): Target Y = y + OFFSET
                pose_giver = create_pose(x, y + self.config.HANDOVER_OFFSET, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)
                # RECEIVER (Right): Target Y = y - OFFSET
                pose_receiver_grasp = create_pose(x, y - self.config.HANDOVER_OFFSET, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
                pose_receiver_pre = create_pose(x, y - self.config.HANDOVER_OFFSET - approach_offset, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
                
                giver_tip = self.config.LEFT_TIP
                receiver_tip = self.config.RIGHT_TIP

            # 3. Setup Collisions
            self.engine.allow_gripper_self_collision()
            self.engine.allow_all_gripper_collisions(obj_name)
            
            # Allow collision between the two grippers specifically (left and right tips)
            self.engine.allow_collision(
                robot_links=[self.config.LEFT_TIP], 
                body_name=self.config.RIGHT_TIP
            )
            # Just to be safe, reverse direction too (simmetry usually handled but good to be explicit)
            self.engine.allow_collision(
                robot_links=[self.config.RIGHT_TIP], 
                body_name=self.config.LEFT_TIP
            )
            
            # Allow collision with environment objects (e.g. other cubes) trying to be safe
            for env_obj in environment_objects:
                try:
                    # Allow collision between Held Object <-> Environment Object
                    self.engine.allow_collision(robot_links=[obj_name], body_name=env_obj)
                    # Also allow Grippers <-> Environment Object (in case we are close)
                    self.engine.allow_all_gripper_collisions(env_obj)
                except Exception as e:
                    print(f"Warning: Could not allow collision for {env_obj}: {e}")
            
            # 4. Phase 1: Move to Meeting (Pre-Grasp)
            print("Phase 1: Moving to Meeting Point...")
            self.open_gripper(execution_receiver) # Ensure receiver is open
            
            self.engine.move_dual_arm(
                left_pose=pose_giver if execution_giver == "left" else pose_receiver_pre,
                right_pose=pose_giver if execution_giver == "right" else pose_receiver_pre,
                left_tip=self.config.LEFT_TIP,
                right_tip=self.config.RIGHT_TIP,
                linear_speed=self.config.LINEAR_SPEED,
                angular_speed=self.config.ANGULAR_SPEED
            )
            
            # 5. Phase 2: Approach (Receiver moves to Object)
            print("Phase 2: Receiver Approach...")
            self.engine.move_dual_arm(
                left_pose=pose_giver if execution_giver == "left" else pose_receiver_grasp,
                right_pose=pose_giver if execution_giver == "right" else pose_receiver_grasp,
                left_tip=self.config.LEFT_TIP,
                right_tip=self.config.RIGHT_TIP,
                linear_speed=self.config.LINEAR_SPEED,
                angular_speed=self.config.ANGULAR_SPEED
            )
            
            # 6. Phase 3: Transfer Logic
            print("Phase 3: Transferring (Coordinated Release)...")
            
            # Step A: Receiver Close (Start closing)
            # We command it to close, but we don't block yet (managed by controller or we assume async behavior)
            # Note: gripper_cmd is async on the wire, but our wrapper is simple.
            # To be safe and rapid, we call them sequentially but without explicit sleeps between.
            
            print(f"  -> Receiver {execution_receiver} Closing...")
            if self.gripper_callback:
                self.gripper_callback(execution_receiver, self.config.GRIPPER_CLOSE, effort=self.config.GRIPPER_EFFORT_DEFAULT)
            
            # Step B: Giver Slack (Immediately Relax/Slight Open)
            # "Open slightly" -> If 0.0 is Open and 0.4 is Closed, 0.3 is slightly open.
            # We use a slightly looser position to allow slip, but maintain some grasp (don't drop).
            # We also reduce effort to minimize fighting force.
            # Default Close is 0.4. Let's try 0.35 (Just a bit looser) + Low Effort.
            SLIGHT_OPEN_VAL = 0.33 
            
            print(f"  -> Giver {execution_giver} Loosening to {SLIGHT_OPEN_VAL}...")
            if self.gripper_callback:
                self.gripper_callback(execution_giver, SLIGHT_OPEN_VAL, effort=self.config.GRIPPER_EFFORT_SOFT)
            
            # Step C: Wait for transfer to settle
            time.sleep(1.0)
            
            # Kinematic Switch
            self.engine.detach_object(obj_name)
            self.engine.attach_object(obj_name, receiver_tip)
            
            # Step D: Full Release
            print(f"  -> Giver {execution_giver} Opening Full...")
            self.open_gripper(execution_giver)
            
            # 7. Phase 4: Separation (Giver Retreats)
            print("Phase 4: Separation...")
            
            if execution_giver == "right":
                 # Right moves away to -Y
                 pose_giver_retreat = create_pose(x, y - retreat_offset, z, roll=-1.57, pitch=1.57, yaw=0, frame=frame_id)
            else:
                 # Left moves away to +Y
                 pose_giver_retreat = create_pose(x, y + retreat_offset, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)

            self.engine.move_dual_arm(
                left_pose=pose_giver_retreat if execution_giver == "left" else pose_receiver_grasp, 
                right_pose=pose_giver_retreat if execution_giver == "right" else pose_receiver_grasp,
                left_tip=self.config.LEFT_TIP,
                right_tip=self.config.RIGHT_TIP,
                linear_speed=self.config.LINEAR_SPEED,
                angular_speed=self.config.ANGULAR_SPEED
            )

            print("Handover Complete.")
            return True
