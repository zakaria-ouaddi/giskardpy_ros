import rclpy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from motion_engine import GiskardMotionEngine
from config import DualArmConfig
from skills.utils import create_pose
import time

def main():
    rclpy.init()
    config = DualArmConfig()
    engine = GiskardMotionEngine(tip_link=config.RIGHT_TIP)

    cubes = [
        ("cube_2 approach", 1.22),
        ("cube_2 grasp", 1.02),
        ("cube_2 lift", 1.07),
        ("cube_1 approach", 1.17),
        ("cube_1 grasp", 0.97),
        ("cube_1 lift", 1.02),
    ]

    for name, z in cubes:
        print(f"\n--- Testing reachability to {name} at Z={z} ---")
        pose = create_pose(
            config.RIGHT_ARM_START_X, 
            config.RIGHT_ARM_START_Y, 
            z, 
            pitch=3.14, yaw=1.57 # Grip at center
        )
        try:
            engine.move_to_pose(pose, linear_speed=config.LINEAR_SPEED, angular_speed=config.ANGULAR_SPEED)
            print(f"✅ Reachable: {name} (Z={z})")
        except Exception as e:
            print(f"❌ Aborted: {name} (Z={z}) -> {e}")

    engine.destroy()

if __name__ == "__main__":
    main()
