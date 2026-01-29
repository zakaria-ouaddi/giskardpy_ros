#!/usr/bin/env python3
import sys
import os
import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from engine.motion_engine import GiskardMotionEngine
from skills.screw import ScrewSkill
from skills.utils import create_pose
from config import DualArmConfig

def main():
    rclpy.init()
    engine = GiskardMotionEngine()
    skill = ScrewSkill(engine)
    
    # Mock Screw Pose
    target = create_pose(0.6, 0.2, 0.95, roll=0, pitch=3.14, yaw=0, frame=DualArmConfig.ROOT_FRAME)
    
    print("--- Testing Screw Skill ---")
    skill.execute("test_screw", target, rotations=2.0, arm="right")
    
    engine.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
