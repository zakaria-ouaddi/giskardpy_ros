#!/usr/bin/env python3
import sys
import os
import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from engine.motion_engine import GiskardMotionEngine
from skills.insertion import InsertionSkill
from skills.utils import create_pose
from config import DualArmConfig

def main():
    rclpy.init()
    engine = GiskardMotionEngine()
    skill = InsertionSkill(engine)
    
    # Mock Hole Pose
    hole_pose = create_pose(0.6, 0.0, 0.9, frame=DualArmConfig.ROOT_FRAME)
    
    print("--- Testing Insertion Skill ---")
    skill.execute("test_peg", hole_pose, arm="right")
    
    engine.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
