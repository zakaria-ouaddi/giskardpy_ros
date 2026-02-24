from typing import List, Optional
import time
from .base import BaseSkill, CollisionContext
from .utils import create_pose

class PickSkill(BaseSkill):
    def execute(self, object_name: str, arm: str = "right", lift_height: float = 0.05, 
                environment_objects: List[str] = [], approach_yaw: float = 1.57):
        """
        Executes a pick sequence.
        """
        print(f"[{object_name}] Initiating Pick Sequence with {arm.upper()} arm.")
        
        # Determine Poses
        # We need the object's position. The previous script hardcoded the x/y for picking based on loop.
        # Ideally we should get the object pose from the world, but for this refactor we might pass specific coords 
        # OR we can assume the user passes the logic.
        # The user's request said Inputs: Target object name, grasp pose, approach offset.
        
        # Wait, the user prompt said: "Encapsulate the logic of approach -> open gripper -> move to grasp -> close gripper -> lift."
        # And inputs: Target object name, grasp pose, approach offset.
        
        # But in the main dual_arm.py, it calculated pose based on "pick_z".
        # I will accept `pick_z` and calculate the pose here using the config constants, OR accept the full pose.
        # To be most flexible, I should accept the `grasp_pose`.
        
        raise NotImplementedError("This method is intended to be called with a specific pose. Use execute_with_pose instead.")

    def execute_with_pose(self, object_name: str, grasp_pose, lift_height: float = 0.05, 
                          arm: str = "right", environment_objects: List[str] = []):
        
        tip_link = self.resolve_tip(arm)
        
        # Ensure 'table' is always in environment_objects to allow picking from it
        if "table" not in environment_objects:
            environment_objects = list(environment_objects)
            environment_objects.append("table")
        
        with CollisionContext(self.engine):
            print(f"[{object_name}] Phase 1: Picking ({arm})")
            
            # Setup Collisions
            self.engine.allow_all_gripper_collisions(object_name)
            for other in environment_objects:
                self.engine.allow_all_gripper_collisions(other)
            
            # Calculate Lift Pose
            # We assume grasp_pose is a PoseStamped
            lift_pose = create_pose(
                grasp_pose.pose.position.x,
                grasp_pose.pose.position.y,
                grasp_pose.pose.position.z + lift_height,
                frame=grasp_pose.header.frame_id
            )
            # Copy orientation from grasp_pose
            lift_pose.pose.orientation = grasp_pose.pose.orientation

            # Calculate Approach Pose (High & Vertical)
            approach_height = self.config.PICK_APPROACH_HEIGHT
            approach_pose = create_pose(
                grasp_pose.pose.position.x,
                grasp_pose.pose.position.y,
                grasp_pose.pose.position.z + approach_height,
                roll=0.0, pitch=3.14, yaw=1.57, # Force Vertical
                frame=grasp_pose.header.frame_id
            )
            
            # 0. Approach (High & Vertical)
            self.engine.move_to_pose(
                approach_pose,
                object_to_allow_collision=object_name,
                environment_objects_to_allow_collision=environment_objects,
                linear_speed=self.config.LINEAR_SPEED,
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link
            )
            time.sleep(1)

            # 1. Open Gripper
            self.open_gripper(arm)
            
            # 2. Move to Grasp
            # Allow moving to pick while touching other cubes
            self.engine.move_to_pose(
                grasp_pose, 
                object_to_allow_collision=object_name, 
                environment_objects_to_allow_collision=environment_objects,
                linear_speed=self.config.LINEAR_SPEED, 
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link
            )
            time.sleep(2)
            
            # 3. Close Gripper & Attach
            self.close_gripper(arm)
            self.engine.attach_object(object_name, tip_link)
            self.engine.allow_all_object_collisions(object_name)
            
            # 4. Lift
            # Start of fix constraint: "Ensure the 'Lifting' phase of the pick allows collision with the stack"
            self.engine.move_to_pose(
                lift_pose, 
                object_to_allow_collision=object_name,
                environment_objects_to_allow_collision=environment_objects,
                linear_speed=self.config.LINEAR_SPEED, 
                angular_speed=self.config.ANGULAR_SPEED,
                tip_link=tip_link,
                allow_gripper_to_object=False
            )
            time.sleep(2)
            
        print(f"[{object_name}] Pick Complete.")
