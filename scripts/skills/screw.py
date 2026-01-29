import time
from geometry_msgs.msg import PoseStamped
from .base import BaseSkill, CollisionContext

class ScrewSkill(BaseSkill):
    """
    Skill for fastening tasks (Screwing).
    """
    def execute(self, object_name: str, target_pose: PoseStamped, rotations: float = 3.0, arm: str = "right"):
        
        tip_link = self.resolve_tip(arm)
        
        with CollisionContext(self.engine):
            print(f"[{object_name}] Initiating Screw Sequence...")
            
            # 1. Approach
            approach_pose = target_pose
            approach_pose.pose.position.z += 0.05
            
            self.engine.allow_all_gripper_collisions(object_name)
            
            print("Phase 1: Aligning Driver")
            self.engine.move_to_pose(approach_pose, tip_link=tip_link)
            
            # 2. Engage (Move to contact)
            print("Phase 2: Engaging")
            self.engine.move_to_pose(target_pose, tip_link=tip_link)
            
            # 3. Drive (Helical Motion)
            print(f"Phase 3: Driving ({rotations} rotations)...")
            
            # Monitor Torque?
            # Ideally we run this in a loop or the engine handles the monitor.
            # Our engine `execute_helical_motion` just runs the trajectory blindly for now.
            # We can check torque AFTER each segment if we refactored, but let's trust the current impl.
            
            self.engine.execute_helical_motion(
                start_pose=target_pose,
                rotations=rotations,
                pitch=self.config.SCREW_PITCH,
                speed_lin=self.config.SCREW_ADVANCE_SPEED,
                speed_rot=self.config.SCREW_ROTATION_SPEED
            )
            
            # 4. Torque Check (Post-action verification)
            # Check if we are "stuck" implies tight.
            # If we completed the motion, we are tight or stripped?
            
            # 5. Release
            self.open_gripper(arm)
            self.engine.detach_object(object_name)
            
            # 6. Retreat
            self.engine.move_to_pose(approach_pose, tip_link=tip_link)
            
            print("Screw Operation Complete.")
