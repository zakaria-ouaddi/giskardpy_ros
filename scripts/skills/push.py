from .base import BaseSkill, CollisionContext
from .utils import create_pose

class PushSkill(BaseSkill):
    """
    Skill for pushing or sliding objects (e.g. into a fixture).
    """
    def execute(self, start_pose, end_pose, arm: str = "right", force_limit: float = None):
        tip_link = self.resolve_tip(arm)
        
        limit = force_limit if force_limit else self.config.PUSH_FORCE_LIMIT
        
        with CollisionContext(self.engine):
            print("Initiating Push Sequence...")
            
            # 1. Approach Start
            pre_start = create_pose(
                start_pose.pose.position.x, 
                start_pose.pose.position.y,
                start_pose.pose.position.z + 0.1,
                frame=start_pose.header.frame_id
            )
            pre_start.pose.orientation = start_pose.pose.orientation
            
            self.engine.move_to_pose(pre_start, tip_link=tip_link)
            self.engine.move_to_pose(start_pose, tip_link=tip_link)
            
            # 2. Push to End
            # We use Cartesian Constrained move (Linear)
            print(f"Pushing to target (Force Limit: {limit}N)...")
            
            # TODO: Integrate force monitoring in the loop?
            # For now, standard move. Giskard handles impedance if configured in constraints.
            
            self.engine.move_to_pose(
                end_pose, 
                linear_speed=self.config.PUSH_SPEED,
                tip_link=tip_link
            )
            
            # 3. Retreat
            pre_end = create_pose(
                end_pose.pose.position.x, 
                end_pose.pose.position.y,
                end_pose.pose.position.z + 0.1,
                frame=end_pose.header.frame_id
            )
            pre_end.pose.orientation = end_pose.pose.orientation
            
            self.engine.move_to_pose(pre_end, tip_link=tip_link)
            print("Push Complete.")
