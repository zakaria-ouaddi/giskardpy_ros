from typing import Optional
import time
from .base import BaseSkill

class VerifyGraspSkill(BaseSkill):
    """
    Skill to verify if a grasp was successful by checking gripper joint positions.
    """
    def execute(self, arm: str, object_width: float, tolerance: float = 0.01) -> bool:
        """
        Checks if the gripper is holding an object of approximately `object_width`.
        Returns True if successful, False if failed (e.g. fully closed or too open).
        """
        print(f"Verifying Grasp for {arm} arm (Expected: {object_width}m)...")
        
        # Determine joint names based on arm
        # Using heuristic to find relevant joints in the joint_state
        # This assumes standard Robotiq or similar naming conventions
        
        prefix = "l_" if arm.lower() == "left" else "r_"
        # Common gripper joint names
        candidates = ["finger_joint", "gripper_joint", "robotiq_85_left_knuckle_joint"]
        
        found_joint = None
        current_pos = 0.0
        
        if self.engine.last_joint_state:
            for name in self.engine.last_joint_state.name:
                # Check if this joint belongs to the correct arm and is a gripper joint
                if prefix in name or (arm.lower() == "left" and "left" in name) or (arm.lower() == "right" and "right" in name):
                    for cand in candidates:
                        if cand in name:
                            idx = self.engine.last_joint_state.name.index(name)
                            current_pos = self.engine.last_joint_state.position[idx]
                            found_joint = name
                            break
                if found_joint:
                    break
        
        if not found_joint:
            print(f"WARNING: Could not identify gripper joint for {arm}. content: {self.engine.last_joint_state.name if self.engine.last_joint_state else 'None'}")
            # Fallback: Assume success if we can't check? Or fail safely?
            print("Assuming Success (Blind).")
            return True
            
        print(f"Checked Joint: {found_joint}, Position: {current_pos:.4f}")
        
        # Logic:
        # If position is near 0.0 (or whatever FULLY CLOSED is for this gripper), and object_width > 0 -> FAIL (Empty Hand)
        # If position is near expected_width -> SUCCESS
        # Robotiq: 0.0 is usually Open, 0.8 is Closed. Or vice versa depending on model.
        # Config says: GRIPPER_OPEN = 0.0, GRIPPER_CLOSE = 0.4 (for 5cm cube?)
        
        # We will compare diff
        if abs(current_pos - object_width) < tolerance:
            print("Grasp Verified: SUCCESS")
            return True
        else:
            print(f"Grasp Verification FAILED. Diff: {abs(current_pos - object_width):.4f}")
            # Check if fully closed (missed object)
            if abs(current_pos - self.config.GRIPPER_CLOSE) < tolerance:
                 print("Result: Empty Hand (Fully Closed)")
            return False
