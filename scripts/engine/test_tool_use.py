#!/usr/bin/env python3
import sys
import os
import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from engine.motion_engine import GiskardMotionEngine
from skills.tool_use import ToolUseSkill

def main():
    rclpy.init()
    engine = GiskardMotionEngine()
    skill = ToolUseSkill(engine)
    
    print("--- Testing Tool Use Skill ---")
    skill.execute("screwdriver")
    
    engine.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
