#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

# --- Giskard / Semantic Digital Twin ---
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body
import semantic_digital_twin.spatial_types.spatial_types as cas

from motion_engine import GiskardMotionEngine

# ================= CONFIG =================
LEFT_GRIPPER_TOPIC  = '/left_gripper/robotiq_gripper_controller/gripper_cmd'
RIGHT_GRIPPER_TOPIC = '/right_gripper/robotiq_gripper_controller/gripper_cmd'
LEFT_TIP_LINK  = "l_gripper_tool_frame"
RIGHT_TIP_LINK = "r_gripper_tool_frame"
WORLD_FRAME = "map2"
BOX_SIZE = (0.05, 0.05, 0.05)

# ---------- Robust Gripper ----------
class RobustGripperController:
    def __init__(self, node: Node, action_topic: str, name: str):
        self.node = node
        self.name = name
        self.client = ActionClient(self.node, GripperCommand, action_topic)

        self.node.get_logger().info(f"[{self.name}] Waiting for {action_topic} ...")
        self.connected = self.client.wait_for_server(timeout_sec=5.0)
        if not self.connected:
            self.node.get_logger().error(f"[{self.name}] Server not found!")

    def command(self, position: float, effort: float = 50.0):
        if not self.connected:
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            print(f"[{self.name}] Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        print(f"[{self.name}] Done. pos={result.position:.3f} stalled={result.stalled}")
        return True

# ---------- Helpers ----------
def create_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, frame=WORLD_FRAME):
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

def add_box(world, name, size, pose_stamped):
    # Manual conversion
    p = cas.Point3(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
    q = cas.Quaternion(
        pose_stamped.pose.orientation.x, 
        pose_stamped.pose.orientation.y, 
        pose_stamped.pose.orientation.z, 
        pose_stamped.pose.orientation.w
    )
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    with world.modify_world():
        existing_box = None
        try:
            existing_box = world.get_body_by_name(name)
        except Exception:
            pass

        if existing_box:
            box = existing_box
            if box.parent_connection:
                world.remove_connection(box.parent_connection)
        else:
            box = Body(name=PrefixedName(name))
            box_shape = Box(scale=Scale(*size))
            box.collision.append(box_shape)
            box.visual.append(box_shape)
            
            # Register the new body
            registered = False
            for fn_name in ("add_body", "add_world_entity", "add_object", "add_link", "add_node", "add_model"):
                if hasattr(world, fn_name):
                    getattr(world, fn_name)(box)
                    registered = True
                    break
            
            if not registered:
                candidates = [m for m in dir(world) if m.startswith("add_")]
                raise RuntimeError(f"World has no known 'add_*' API. Candidates: {candidates}")

        # Create & add the connection (works for both new and existing)
        connection = FixedConnection(
            parent=world.root,
            child=box,
            parent_T_connection_expression=parent_T_pose,
        )
        world.add_connection(connection)

def main():
    rclpy.init()
    node = rclpy.create_node("left_arm_lift")

    try:
        # --- Grippers ---
        left_gripper  = RobustGripperController(node, LEFT_GRIPPER_TOPIC,  "LeftGripper")
        right_gripper = RobustGripperController(node, RIGHT_GRIPPER_TOPIC, "RightGripper")

        # --- Motion Engines ---
        # We need both engines sharing the same Giskard instance
        motion_left = GiskardMotionEngine(default_tip_link=LEFT_TIP_LINK)
        
        # Helper class to share Giskard instance (copied from dual_arm_transfer.py)
        class LinkedGiskardMotionEngine(GiskardMotionEngine):
            def __init__(self, existing_engine: GiskardMotionEngine, tip_link: str):
                self.giskard = existing_engine.giskard
                self.root_link = existing_engine.root_link
                self.default_tip_link = tip_link
                
        motion_right = LinkedGiskardMotionEngine(motion_left, tip_link=RIGHT_TIP_LINK)
        
        print("Waiting for Giskard world to be ready...")
        while motion_left.giskard.world is None or motion_left.giskard.world.root is None:
            time.sleep(0.5)
        print("World is ready.")
        world = motion_left.giskard.world

        # --- Box ---
        box_l_base = "box_l_base"
        # Move closer to ensure reachability (was 0.8)
        pose_l_base = create_pose(0.6, 0.4, 0.92)
        print(f"DEBUG: Adding box {box_l_base}...")
        add_box(world, box_l_base, BOX_SIZE, pose_l_base)
        
        # Force disable collision for the box immediately to avoid issues
        print(f"DEBUG: Force disabling collision for {box_l_base}...")
        with world.modify_world():
            try:
                box_body = world.get_body_by_name(box_l_base)
                box_body.collision_config.disabled = True
                print("DEBUG: Box collision disabled.")
            except Exception as e:
                print(f"WARNING: Could not disable box collision: {e}")
        
        # --- Open Grippers ---
        left_gripper.command(0.0)
        right_gripper.command(0.0)

        # --- Pick Sequence (Left Arm) ---
        print(f"================ Handling {box_l_base} ================")
        
        # Pre-pick (10cm above)
        pre_pick = create_pose(pose_l_base.pose.position.x,
                               pose_l_base.pose.position.y,
                               pose_l_base.pose.position.z + 0.1,
                               yaw=-1.57, pitch=3.14)
                               
        # Pick (at object)
        pick = create_pose(pose_l_base.pose.position.x,
                           pose_l_base.pose.position.y,
                           pose_l_base.pose.position.z,
                           yaw=-1.57, pitch=3.14)

        # Execute Pick (Split into steps)
        print("DEBUG: Moving to Pre-Pick...")
        motion_left.move_to_pose(pre_pick, object_to_allow_collision=box_l_base)
        
        print("DEBUG: Moving to Pick...")
        motion_left.move_to_pose(pick, object_to_allow_collision=box_l_base)
        
        # Close Gripper
        left_gripper.command(0.55)
        motion_left.attach_object(box_l_base, LEFT_TIP_LINK)

        # Lift
        motion_left.move_to_pose(pre_pick, object_to_allow_collision=box_l_base)
        
        # --- Handover Sequence ---
        print("DEBUG: Moving to Handover...")
        
        # Handover Point (Meeting in the middle)
        meet_x = 0.6
        meet_y = 0.0
        meet_z = 1.0
        
        # Left Arm Pose: Gripper HORIZONTAL (pitch=1.57), making box VERTICAL
        # yaw=-1.57 faces right
        pose_transfer_l = create_pose(meet_x, meet_y, meet_z, yaw=-1.57, pitch=1.57) 
        
        # Right Arm Pose: Gripper HORIZONTAL (pitch=1.57), to grasp vertical box
        # yaw=1.57 faces left
        pose_transfer_r = create_pose(meet_x, meet_y, meet_z, yaw=1.57, pitch=1.57)
        
        # Right Arm Pre-Pose (Approach from side)
        pose_pre_r = create_pose(meet_x, meet_y - 0.2, meet_z, yaw=1.57, pitch=1.57)
        
        # 1. Move Right to Pre-Handover (Clear space for Left Arm)
        print("DEBUG: Right arm moving to pre-handover...")
        motion_right.move_to_pose(pose_pre_r, object_to_allow_collision=box_l_base)

        # 2. Move Left (with box) to handover point
        print("DEBUG: Left arm moving to handover point...")
        motion_left.move_to_pose(pose_transfer_l, object_to_allow_collision=box_l_base)
        
        # 3. Move Right to Grasp (Approach)
        print("DEBUG: Right arm approaching to grasp...")
        # Allow collision with box (it's about to grab it) AND Left Gripper
        # We need to find the left gripper body name to allow collision with it
        # Usually "l_gripper_tool_frame" is the tip, but we need the body.
        # For now, we allow collision with EVERYTHING to ensure it doesn't stop 1cm away.
        #all_bodies = list(motion_right.giskard.world.bodies_with_enabled_collision)
        motion_right.move_to_pose(pose_transfer_r, 
                                object_to_allow_collision=box_l_base)
                                #,environment_objects_to_allow_collision=all_bodies)
        
        # Execute Handover
        right_gripper.command(0.55)  # Right grabs
        
        motion_left.detach_object(box_l_base)
        motion_right.attach_object(box_l_base, RIGHT_TIP_LINK)
        
        left_gripper.command(0.0)    # Left releases
        
        # Retreat Left to starting position (pre-pick)
        print("DEBUG: Moving left arm back to starting position...")
        pose_retreat_l = create_pose(0.6, 0.4, 1.02, yaw=-1.57, pitch=3.14)
        motion_left.move_to_pose(pose_retreat_l)

        print("\n================ ALL DONE ================")
        print("\nHolding position. Press ENTER to shutdown and release arms...")
        input()  # Wait for user input before destroying motion engines

    finally:
        try:
            motion_left.destroy()
            motion_right.destroy()
        except:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
