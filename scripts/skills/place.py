from typing import List
import time
from .base import BaseSkill, CollisionContext
from .utils import create_pose
from geometry_msgs.msg import PoseStamped

class PlaceSkill(BaseSkill):
    def execute(self, object_name: str, target_pose: PoseStamped, arm: str = "left", 
                stack_on: List[str] = [], retreat_distance: float = 0.1, force_vertical: bool = False):
        """
        Executes a place sequence.
        :param force_vertical: If True, overrides the target_pose orientation to be vertical (downward).
        """
        tip_link = self.resolve_tip(arm)
        
        # Enforce vertical orientation if requested using config defaults or hardcoded 3.14 pitch
        if force_vertical:
            # Create a vertical orientation (Pitch=3.14, Yaw=1.57 as standard)
            # We preserve the position of target_pose
            vertical_pose = create_pose(
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                roll=0.0, pitch=3.14, yaw=1.57,
                frame=target_pose.header.frame_id
            )
            target_pose = vertical_pose
        
        with CollisionContext(self.engine):
            print(f"[{object_name}] Phase 3: Placing ({arm})")
            
            # Calculate Pre-Place (Retreat) Pose
            # We assume retreat is purely shifting Z up for now based on the previous script's logic, 
            # BUT the previous script did specific things.
            # "pose_place_pre = create_pose(..., place_z + 0.10 ...)"
            # So yes, approach/retreat from above.
            
            pre_place_pose = create_pose(
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z + self.config.PLACE_RETREAT_HEIGHT,
                frame=target_pose.header.frame_id
            )
            pre_place_pose.pose.orientation = target_pose.pose.orientation

            # Setup safe spot for the OTHER arm?
            # The previous script had: "Right Arm Retreat: Hold it at its safe spot"
            # In a pure atomic skill, "Place" shouldn't necessarily know about the other arm's safety unless we enforce dual arm move.
            # However, Giskard's `move_dual_arm` was used to hold the other arm steady. 
            # If we just use `move_to_pose` for the active arm, the other arm *should* stay put if not commanded, 
            # BUT Giskard might behave better with full body constraint or explicit keep.
            # FOR NOW: I will rely on `move_to_pose` for the single arm, assuming the other arm is idle/stationary.
            # IF the user wants the explicit dual arm behavior from the script (holding right arm safe), 
            # we might need to handle that. But "Atomic Skill" usually implies focusing on the task.
            # Let's stick to `move_to_pose` for the active arm to be truly modular.
            
            # 1. Approach (Pre-Place)
            self.engine.move_to_pose(
                pre_place_pose, 
                linear_speed=self.config.LINEAR_SPEED, 
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link
            )
            time.sleep(1)

            # 2. Move to Place
            # Allow collision with the stack we are placing on
            # Logic: We are holding 'object_name', and we want to place it on 'stack_on' objects.
            
            # IMPLICITLY ALLOW TABLE: "tracy/table" or "table"
            # It's a place action, usually on a table.
            local_stack_on = list(stack_on)
            if "table" not in local_stack_on:
                 local_stack_on.append("table")
            if "table" not in local_stack_on:
                 local_stack_on.append("table")
            
            # Robustness Fix: Allow Gripper to touch Stack
            for stack_obj in local_stack_on:
                try:
                    self.engine.allow_all_gripper_collisions(stack_obj)
                except Exception:
                    pass
            
            self.engine.move_to_pose(
                target_pose,
                object_to_allow_collision=object_name,
                environment_objects_to_allow_collision=local_stack_on,
                linear_speed=self.config.LINEAR_SPEED, 
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link,
                allow_gripper_to_object=False
            )
            time.sleep(2)
            
            # 3. Open Gripper (Release)
            self.open_gripper(arm)
            self.engine.detach_object(object_name, pose=target_pose)
            
            # 4. Retreat
            # We MUST allow collision with the object we just released, and potentially the stack,
            # as we are likely still touching them or very close.
            self.engine.move_to_pose(
                pre_place_pose,
                object_to_allow_collision=object_name,
                environment_objects_to_allow_collision=local_stack_on,
                linear_speed=self.config.LINEAR_SPEED, 
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link
            )
            time.sleep(2)
            
        print(f"[{object_name}] Place Complete.")
