#!/usr/bin/env python3
import sys
import os
import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from engine.motion_engine import GiskardMotionEngine
from skills.verify_grasp import VerifyGraspSkill
from config import DualArmConfig

def main():
    rclpy.init()
    engine = GiskardMotionEngine()
    skill = VerifyGraspSkill(engine)
    
    print("--- Testing Verify Grasp ---")
    
    # 1. Test success case (assuming holding something or simulating)
    # Since we can't physically actuate, this will read actual sensor data
    success = skill.execute("left", object_width=0.05) # Assume 5cm cube
    print(f"Verify Result (5cm): {success}")
    
    # 2. Test failure case (Empty)
    success = skill.execute("left", object_width=DualArmConfig.GRIPPER_OPEN) 
    print(f"Verify Result (Open): {success}")
    
    engine.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
