#!/usr/bin/env python3
import sys
import os
import time
import argparse
import rclpy

# Add parent directory to path to allow importing skills from sibling directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

try:
    from skills.gripper_controller import GripperController
    # Import config to use the defined open/close values
    from config import DualArmConfig
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit(1)

def main():
    # Initialize ROS 2
    rclpy.init()
    
    parser = argparse.ArgumentParser(description="Simple Utility to Control Grippers")
    parser.add_argument('action', choices=['open', 'close', 'test'], help="Action to perform")
    parser.add_argument('--arm', choices=['left', 'right', 'both'], default='both', help="Which arm to control (default: both)")
    
    args = parser.parse_args()
    
    # Create the controller
    # We rely on it to create its own node since we passed None
    controller = GripperController()
    
    try:
        # Determine arms to control
        arms_to_control = ['left', 'right'] if args.arm == 'both' else [args.arm]
        
        # Determine positions based on config
        pos_open = DualArmConfig.GRIPPER_OPEN
        pos_close = DualArmConfig.GRIPPER_CLOSE
        diff_effort = DualArmConfig.GRIPPER_EFFORT_DEFAULT

        if args.action == 'test':
            print("--- Running Gripper Test Sequence ---")
            for arm in arms_to_control:
                print(f"\nTesting {arm.upper()} Arm:")
                
                print(f"  Opening {arm} ({pos_open})...")
                controller.command(arm, pos_open, diff_effort)
                time.sleep(2.0)
                
                print(f"  Closing {arm} ({pos_close})...")
                controller.command(arm, pos_close, diff_effort)
                time.sleep(2.0)
                
                print(f"  Opening {arm} ({pos_open})...")
                controller.command(arm, pos_open, diff_effort)
                
            print("\nTest Sequence Complete.")
            
        elif args.action == 'open':
            for arm in arms_to_control:
                print(f"Opening {arm.upper()} gripper...")
                controller.command(arm, pos_open, diff_effort)
                
        elif args.action == 'close':
            for arm in arms_to_control:
                print(f"Closing {arm.upper()} gripper...")
                controller.command(arm, pos_close, diff_effort)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Shutting down...")
        controller.destroy()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
