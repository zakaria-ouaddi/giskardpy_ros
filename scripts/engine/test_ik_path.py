import rclpy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from motion_engine import GiskardMotionEngine
from config import DualArmConfig
from skills.utils import create_pose

def main():
    rclpy.init()
    config = DualArmConfig()
    engine = GiskardMotionEngine(tip_link=config.RIGHT_TIP)

    pose1 = create_pose(0.9, -0.2, 0.97, pitch=3.14, yaw=1.57) # cube 1 grasp
    pose2 = create_pose(0.9, -0.2, 1.02, pitch=3.14, yaw=1.57) # cube 1 lift
    
    print("\n--- Moving to 0.97 ---")
    try:
        engine.move_to_pose(pose1, linear_speed=config.LINEAR_SPEED, angular_speed=config.ANGULAR_SPEED)
        print("✅ Reached 0.97")
    except Exception as e:
        print(f"❌ Aborted: -> {e}")
        
    print("\n--- Moving to 1.02 (Lifting straight up) ---")
    try:
        engine.move_to_pose(pose2, linear_speed=config.LINEAR_SPEED, angular_speed=config.ANGULAR_SPEED)
        print("✅ Reached 1.02")
    except Exception as e:
        print(f"❌ Aborted: -> {e}")

    engine.destroy()

if __name__ == "__main__":
    main()
