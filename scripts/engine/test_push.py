#!/usr/bin/env python3
import sys
import os
import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from engine.motion_engine import GiskardMotionEngine
from skills.push import PushSkill
from skills.utils import create_pose
from config import DualArmConfig

def main():
    rclpy.init()
    engine = GiskardMotionEngine()
    skill = PushSkill(engine)
    
    start = create_pose(0.6, 0.0, 0.92, frame=DualArmConfig.ROOT_FRAME)
    end = create_pose(0.7, 0.0, 0.92, frame=DualArmConfig.ROOT_FRAME)
    
    print("--- Testing Push Skill ---")
    skill.execute(start, end, arm="left")
    
    engine.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
