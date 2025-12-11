#!/usr/bin/env python3
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

# Giskard / Semantic Digital Twin Imports
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body
import semantic_digital_twin.spatial_types.spatial_types as cas

# Local Motion Engine Import
try:
    from motion_engine import GiskardMotionEngine
except ImportError:
    print("Error: 'motion_engine.py' not found. Ensure it is in the PYTHONPATH or local dir.")
    sys.exit(1)

# --- CONFIGURATION ---
# UPDATED: Using the RIGHT gripper topic
GRIPPER_TOPIC = '/right_gripper/robotiq_gripper_controller/gripper_cmd'
ROBOT_TIP_LINK = "r_gripper_tool_frame"
WORLD_FRAME = "map2"

OBJECT_NAME = "milk_box"
TABLE_NAME = "table"

# --- UPDATED ROBUST GRIPPER CONTROLLER ---
class RobustGripperController:
    def __init__(self, node: Node, action_topic: str):
        self.node = node
        self.client = ActionClient(self.node, GripperCommand, action_topic)
        self.logger = self.node.get_logger()
        
        self.logger.info(f"Waiting for gripper server: {action_topic}...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"Error: Server '{action_topic}' not found. Is the driver running?")
            # We don't exit here to allow the script to try anyway, but it's risky
        else:
            self.logger.info("Gripper server connected.")

    def command(self, position: float, effort: float = 50.0):
        """
        Sends command and checks if we stalled (grasped object) or reached goal (air).
        """
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)

        self.logger.info(f"Gripper: Sending command Pos={position}, Effort={effort}")

        # 1. Send Goal
        goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.logger.error("Gripper Command REJECTED by server.")
            return False

        # 2. Wait for Result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        # 3. Analyze Result
        success = False
        self.logger.info(f"Gripper Finished. Pos: {result.position:.4f}")
        
        if result.stalled:
            self.logger.info("SUCCESS: Gripper stalled (Object detected!)")
            success = True
        elif result.reached_goal:
            # If we wanted to close fully (0.8), this is success.
            # If we wanted to grasp an object at 0.55, but reached 0.55 without stalling, 
            # it might mean the object is smaller than expected or missing.
            if position > 0.05: # If not opening
                self.logger.warning("WARNING: Gripper reached target without stalling. Missed object?")
            else:
                self.logger.info("Gripper opened successfully.")
            success = True
        
        return success

# --- HELPER FUNCTIONS ---
def create_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, frame=WORLD_FRAME) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    q = quaternion_from_euler(roll, pitch, yaw)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p

def add_box_to_giskard(world, name, size, pose_stamped: PoseStamped):
    p = cas.Point3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    q = cas.Quaternion(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, 
                       pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)
    parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    with world.modify_world():
        box = Body(name=PrefixedName(name))
        box_shape = Box(scale=Scale(*size))
        box.collision.append(box_shape)
        box.visual.append(box_shape)
        connection = FixedConnection(parent=world.root, child=box, parent_T_connection_expression=parent_T_pose)
        world.add_connection(connection)

# --- MAIN ---
def main():
    rclpy.init()
    test_node = rclpy.create_node('pick_place_test_node')
    
    try:
        # 1. Setup
        # Initialize the NEW Robust Gripper Controller
        gripper = RobustGripperController(test_node, action_topic=GRIPPER_TOPIC)
        motion = GiskardMotionEngine(tip_link=ROBOT_TIP_LINK)

        # 2. Add Object
        obj_pose = create_pose(0.8, -0.2, 1.0) 
        add_box_to_giskard(motion.giskard.world, OBJECT_NAME, (0.05, 0.05, 0.05), obj_pose)
        
        # 3. Define Poses
        common_pitch = 3.14
        common_yaw = 1.57

        pose_pre_pick = create_pose(0.8, -0.2, 1.20, pitch=common_pitch, yaw=common_yaw)
        pose_pick     = create_pose(0.8, -0.2, 0.95, pitch=common_pitch, yaw=common_yaw)
        pose_lift     = create_pose(0.8, -0.2, 1.20, pitch=common_pitch, yaw=common_yaw)
        pose_place    = create_pose(0.8, -0.3, 0.95, pitch=common_pitch, yaw=common_yaw)

        print("--- STARTING TEST ---")

        # A. Open Gripper
        gripper.command(position=0.0, effort=50.0)

        # B. Move to Pick
        print("Motion: Moving to Pick...")
        motion.execute_smooth_sequence([pose_pre_pick, pose_pick], object_to_allow_collision=OBJECT_NAME)

        # C. Close Gripper (Using your specific settings)
        print("Gripper: Closing...")
        # 0.55 = Target (Strong Grip), 50.0 = Force
        grasped = gripper.command(position=0.55, effort=50.0)
        
        if not grasped:
            print("CRITICAL WARNING: Gripper did not grasp correctly. Aborting lift?")
            # In a real scenario, you might want to stop here:
            # return 

        motion.attach_object(OBJECT_NAME, ROBOT_TIP_LINK)

        # D. Lift & Place
        print("Motion: Lifting...")
        motion.move_to_pose(pose_lift, object_to_allow_collision=OBJECT_NAME, environment_objects_to_allow_collision=[TABLE_NAME])
        
        print("Motion: Placing...")
        motion.move_to_pose(pose_place, object_to_allow_collision=OBJECT_NAME, environment_objects_to_allow_collision=[TABLE_NAME])

        # E. Release
        print("Gripper: Releasing...")
        gripper.command(position=0.0, effort=50.0)
        motion.detach_object(OBJECT_NAME)

        print("--- TEST COMPLETE ---")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'motion' in locals(): motion.destroy()
        if 'test_node' in locals(): test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()#main