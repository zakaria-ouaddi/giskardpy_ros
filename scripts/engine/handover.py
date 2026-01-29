#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from semantic_digital_twin.world_description.world_entity import Body

def create_pose(x, y, z, roll=0, pitch=0, yaw=0, frame="map2"):
    """Helper to create a ROS PoseStamped message."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    q = quaternion_from_euler(roll, pitch, yaw)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p

def create_pose_with_frame(frame_id, x, y, z, roll=0, pitch=0, yaw=0):
    """Helper to create a ROS PoseStamped message with specific frame."""
    return create_pose(x, y, z, roll, pitch, yaw, frame=frame_id)

class HandoverManager:
    """
    A specialized skill class to handle dual-arm handovers robustly.
    It automatically detects which arm is the 'Giver' and which is the 'Receiver'.
    """
    def __init__(self, engine, gripper_control_cb):
        """
        :param engine: Instance of GiskardMotionEngine
        :param gripper_control_cb: A callback function(side, value) to control grippers
        """
        self.engine = engine
        self.gripper_control_cb = gripper_control_cb 
        self.left_tip = "tracy/l_gripper_tool_frame"
        self.right_tip = "tracy/r_gripper_tool_frame"

    def detect_holding_state(self):
        """
        Scans the world to see which gripper is holding an object.
        Returns: (holding_arm_side, object_name) or (None, None)
        """
        left_tip_body = self.engine._resolve_entity_name(self.left_tip)
        right_tip_body = self.engine._resolve_entity_name(self.right_tip)
        
        holding_side = None
        held_object = None

        # Iterate over all entities in the world to find one attached to a gripper
        for entity in self.engine.giskard.world.kinematic_structure_entities:
            if not isinstance(entity, Body):
                continue
            try:
                # Check the parent of the entity
                parent = entity.parent_connection.parent
                if parent == left_tip_body:
                    holding_side = "left"
                    held_object = str(entity.name)
                    break 
                elif parent == right_tip_body:
                    holding_side = "right"
                    held_object = str(entity.name)
                    break
            except AttributeError:
                continue
        
        return holding_side, held_object

    def execute_handover(self, meeting_point: PoseStamped, approach_offset=0.20, retreat_offset=0.20, linear_speed=0.2, angular_speed=0.2):
        """
        Executes the handover sequence at the specified meeting point.
        :param meeting_point: PoseStamped of the exchange point
        :param approach_offset: Distance (m) for the approaching arm to stop before grasping
        :param retreat_offset: Distance (m) for the giving arm to retreat after release
        """
        print("\n--- INITIATING SMART HANDOVER ---")
        
        # 1. Detect State (Who is Giver, Who is Receiver)
        holder, obj_name = self.detect_holding_state()
        
        if not holder:
            print("ERROR: No object detected in either hand. Cannot perform handover.")
            return False
        
        receiver = "left" if holder == "right" else "right"
        print(f"Status: {holder.upper()} is holding '{obj_name}'.")
        print(f"Action: Handover {holder.upper()} -> {receiver.upper()}")

        # 2. Define Geometry (Relative to Meeting Point)
        frame_id = meeting_point.header.frame_id
        x = meeting_point.pose.position.x
        y = meeting_point.pose.position.y
        z = meeting_point.pose.position.z

        # --- ORIENTATION CONFIGURATION ---
        # Right Arm: Roll=-1.57 (Standard side grasp)
        # Left Arm:  Roll=1.57, Pitch=-1.57 (Flipped 180 to face Right Arm)
        
        if holder == "right":
            # GIVER (Right): Holds object at center
            pose_giver = create_pose(x, y, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
            
            # RECEIVER (Left): Approaches from +Y (Left side)
            pose_receiver_grasp = create_pose(x, y, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id) 
            
            # Receiver Pre-Grasp: offset away in +Y
            pose_receiver_pre = create_pose(x, y + approach_offset, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)
            
            giver_tip = self.right_tip
            receiver_tip = self.left_tip
            
        else: # holder == "left"
            # GIVER (Left): Holds object at center
            pose_giver = create_pose(x, y, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)
            
            # RECEIVER (Right): Approaches from -Y (Right side)
            pose_receiver_grasp = create_pose(x, y, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
            
            # Receiver Pre-Grasp: offset away in -Y
            pose_receiver_pre = create_pose(x, y - approach_offset, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
            
            giver_tip = self.left_tip
            receiver_tip = self.right_tip

        # 3. Setup Collisions
        # We must allow the grippers to touch each other and the object
        self.engine.allow_gripper_self_collision()
        self.engine.allow_all_gripper_collisions(obj_name)
        
        # 4. Phase 1: Move to Meeting (Pre-Grasp)
        # Both arms move from their starting locations (far away) to the meeting vicinity.
        print("Phase 1: Moving to Meeting Point...")
        self.gripper_control_cb(receiver, 0.0) # Ensure receiver is open
        
        # Move both arms: Giver goes to center, Receiver goes to Pre-Grasp
        self.engine.move_dual_arm(
            left_pose=pose_giver if holder == "left" else pose_receiver_pre,
            right_pose=pose_giver if holder == "right" else pose_receiver_pre,
            left_tip=self.left_tip,
            right_tip=self.right_tip,
            linear_speed=linear_speed,
            angular_speed=angular_speed
        )
        
        # 5. Phase 2: Approach (Receiver moves to Object)
        # Giver holds position (we re-send the same pose) while Receiver closes the gap.
        print("Phase 2: Receiver Approach...")
        self.engine.move_dual_arm(
            left_pose=pose_giver if holder == "left" else pose_receiver_grasp, # Giver holds, Receiver moves
            right_pose=pose_giver if holder == "right" else pose_receiver_grasp,
            left_tip=self.left_tip,
            right_tip=self.right_tip,
            linear_speed=linear_speed,
            angular_speed=angular_speed
        )
        
        # 6. Phase 3: Transfer Logic
        print("Phase 3: Transferring...")
        self.gripper_control_cb(receiver, 0.8) # Close Receiver
        
        # Kinematic Switch: Detach from Giver, Attach to Receiver
        self.engine.detach_object(obj_name)
        self.engine.attach_object(obj_name, receiver_tip)
        
        self.gripper_control_cb(holder, 0.0)   # Open Giver
        
        # 7. Phase 4: Separation (Giver Retreats)
        print("Phase 4: Separation...")
        
        # Calculate retreat pose for Giver
        if holder == "right":
             # Right moves away to -Y
             pose_giver_retreat = create_pose(x, y - retreat_offset, z, roll=-1.57, pitch=0, yaw=0, frame=frame_id)
        else:
             # Left moves away to +Y
             pose_giver_retreat = create_pose(x, y + retreat_offset, z, roll=1.57, pitch=-1.57, yaw=0, frame=frame_id)

        # Move both arms: Receiver holds still, Giver retreats
        self.engine.move_dual_arm(
            left_pose=pose_giver_retreat if holder == "left" else pose_receiver_grasp, 
            right_pose=pose_giver_retreat if holder == "right" else pose_receiver_grasp,
            left_tip=self.left_tip,
            right_tip=self.right_tip,
            linear_speed=linear_speed,
            angular_speed=angular_speed
        )

        print("Handover Complete.")
        return True