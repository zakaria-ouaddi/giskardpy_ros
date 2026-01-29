from typing import Optional
import time
from geometry_msgs.msg import PoseStamped
from .base import BaseSkill, CollisionContext
from .utils import create_pose

class InsertionSkill(BaseSkill):
    """
    Skill for Peg-in-Hole insertion tasks.
    """
    def execute(self, object_name: str, hole_pose: PoseStamped, arm: str = "right", 
                spiral_radius: float = 0.02, spiral_duration: float = 10.0):
        
        tip_link = self.resolve_tip(arm)
        
        with CollisionContext(self.engine):
            print(f"[{object_name}] Initiating Insertion Sequence...")
            
            # 1. Approach (Hover above hole)
            approach_pose = create_pose(
                hole_pose.pose.position.x,
                hole_pose.pose.position.y,
                hole_pose.pose.position.z + 0.05, # 5cm above
                frame=hole_pose.header.frame_id
            )
            approach_pose.pose.orientation = hole_pose.pose.orientation
            
            self.engine.allow_all_gripper_collisions(object_name)
            # Find the hole object? Assume user handled environment collision allows or we pass it
            
            print("Phase 1: Approach")
            self.engine.move_to_pose(approach_pose, tip_link=tip_link)
            
            # 2. Touch / Spiral Search
            # We try to move DOWN to the hole Z
            # If we hit something (Z position doesn't reach goal or Force spikes), we Spiral.
            # Giskard's `execute_spiral_search` does both: touches down + spirals
            
            print("Phase 2: Spiral Search & Insert")
            
            # We pass the HOLE pose as center. The engine will try to push down while spiraling.
            self.engine.execute_spiral_search(
                center_pose=hole_pose, 
                max_radius=spiral_radius,
                duration=spiral_duration,
                push_depth=0.02 # Try to go 2cm deep
            )
            
            # 3. Final Push / Seat
            # Once spiral is done (time expired or condition met), we might perform a final robust push
            # or we assume we are in.
            
            # 4. Release
            self.open_gripper(arm)
            self.engine.detach_object(object_name)
            
            # 5. Retreat
            self.engine.move_to_pose(approach_pose, tip_link=tip_link)
            
            print("Insertion Complete.")
            return True
