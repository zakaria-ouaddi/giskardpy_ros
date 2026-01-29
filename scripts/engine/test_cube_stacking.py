#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time

# Giskard Engine
from motion_engine import GiskardMotionEngine
from motion_engine import GiskardWrapperNode
# Helper imports
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

# --- Helpers ---
def create_pose(x, y, z, roll=0, pitch=0, yaw=0, frame="map2"):
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

def add_cube(engine, name, size, pose):
    """ Spawns a box in the Giskard world """
    p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    q = cas.Quaternion(
        pose.pose.orientation.x, 
        pose.pose.orientation.y, 
        pose.pose.orientation.z, 
        pose.pose.orientation.w
    )
    parent_T_pose = cas.TransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    with engine.giskard.world.modify_world():
        obj = Body(name=PrefixedName(name))
        shape = Box(scale=Scale(*size))
        obj.collision.append(shape)
        obj.visual.append(shape)
        connection = FixedConnection(
            parent=engine.giskard.world.root,
            child=obj,
            parent_T_connection_expression=parent_T_pose,
        )
        try:
            engine.giskard.world.add_connection(connection)
        except:
            print(f"Object {name} might already exist.")

class CubeStackingDemo:
    def __init__(self):
        self.engine = GiskardMotionEngine(tip_link="tracy/r_gripper_tool_frame")
        self.gripper_node = rclpy.create_node('gripper_client_stacking')
        
        # Cube parameters
        self.cube_size = 0.05  # 5cm cubes
        self.cube_names = ["cube_A", "cube_B"]
        
        # Positions (using PROVEN working area from handover test)
        # Cube A at 0.9 (Exact match to working test)
        # Cube B at 1.05 (15cm further)
        self.pick_positions = [
            (0.90, -0.2, 0.95),  # Cube A (Proven working position)
            (1.05, -0.2, 0.95),  # Cube B (Further back)
        ] 
        self.stack_base = (0.5, 0.3, 0.95)
        self.exchange_point = (0.65, -0.05, 1.1)
    
    def gripper_cmd(self, side, value):
        print(f"[{side.upper()} GRIPPER] Command: {value}")
        time.sleep(0.5)
    
    def setup_scene(self):
        """Spawn 2 cubes on right side of table"""
        print("--- SETTING UP SCENE ---")
        size = (self.cube_size, self.cube_size, self.cube_size)
        
        for i, (name, pos) in enumerate(zip(self.cube_names, self.pick_positions)):
            cube_pose = create_pose(*pos, frame="map2")
            add_cube(self.engine, name, size, cube_pose)
            print(f"Spawned {name} at {pos}")
            time.sleep(1.0)  # Wait for world update
            
            # Persist collision allowance for this cube (Like handover test)
            try:
                self.engine.allow_all_gripper_collisions(name)
                print(f"Enabled persistent collision allowance for {name}")
            except Exception as e:
                print(f"Warning: Could not enable collision for {name}: {e}")
        
        # Global collision allowances
        self.engine.allow_gripper_self_collision()
    
    def pick_cube(self, cube_name, pick_pos):
        """Right arm picks a cube - LOGIC FROM HANDOVER TEST"""
        print(f"\n[RIGHT ARM] Picking {cube_name}...")
        self.gripper_cmd("right", 0.0)  # Open
        
        # Pre-pick
        pose_pre_pick = create_pose(pick_pos[0], pick_pos[1], pick_pos[2] + 0.2, pitch=3.14, yaw=1.57)
        self.engine.move_to_pose(pose_pre_pick, object_to_allow_collision=cube_name)
        
        # Pick
        pose_pick = create_pose(*pick_pos, pitch=3.14, yaw=1.57)
        self.engine.move_to_pose(pose_pick, object_to_allow_collision=cube_name, 
                                environment_objects_to_allow_collision=["table"])
        
        self.gripper_cmd("right", 0.8)  # Close
        self.engine.attach_object(cube_name, "tracy/r_gripper_tool_frame")
        
        # Lift - Exact logic from handover test, but with ADDED table safety
        self.engine.move_to_pose(pose_pre_pick, object_to_allow_collision=cube_name, 
                                environment_objects_to_allow_collision=["table"])
    
    def handover_cube(self, cube_name):
        """Execute face-to-face handover"""
        print(f"\n[HANDOVER] {cube_name}...")
        
        # Exchange poses
        pose_exchange_right = create_pose(self.exchange_point[0], self.exchange_point[1], 
                                         self.exchange_point[2], roll=-1.57, pitch=0.0, yaw=0.0)
        pose_exchange_left_pre = create_pose(self.exchange_point[0], self.exchange_point[1] + 0.25, 
                                            self.exchange_point[2], roll=1.57, pitch=1.57, yaw=0.0)
        
        # Move to exchange init
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_pre,
            right_pose=pose_exchange_right,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        # Left approaches
        pose_exchange_left_grasp = create_pose(self.exchange_point[0], self.exchange_point[1], 
                                              self.exchange_point[2], roll=1.57, pitch=1.57, yaw=0.0)
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_grasp,
            right_pose=pose_exchange_right,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        # Exchange
        self.gripper_cmd("left", 0.8)  # Close
        self.engine.detach_object(cube_name)
        self.engine.attach_object(cube_name, "tracy/l_gripper_tool_frame")
        self.gripper_cmd("right", 0.0)  # Open
        
        # Separate
        pose_retreat_right = create_pose(0.65, -0.2, 1.1, pitch=3.14, yaw=1.57)
        self.engine.move_dual_arm(
            left_pose=pose_exchange_left_grasp,
            right_pose=pose_retreat_right,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
    
    def place_cube(self, cube_name, stack_height):
        """Left arm places cube on stack"""
        target_z = self.stack_base[2] + stack_height * self.cube_size
        print(f"\n[LEFT ARM] Placing {cube_name} at z={target_z:.3f}...")
        
        # Pre-place (10cm above) - gripper pointing DOWN
        pose_pre_place = create_pose(self.stack_base[0], self.stack_base[1], target_z + 0.1, 
                                    pitch=3.14, yaw=0.0)
        
        # Place - gripper pointing DOWN
        pose_place = create_pose(self.stack_base[0], self.stack_base[1], target_z, 
                                pitch=3.14, yaw=0.0)
        
        # We need to change tip_link for left arm single motion
        self.engine.tip_link = "tracy/l_gripper_tool_frame"
        
        self.engine.move_to_pose(pose_pre_place, object_to_allow_collision=cube_name)
        self.engine.move_to_pose(pose_place, object_to_allow_collision=cube_name,
                                environment_objects_to_allow_collision=["table"] + self.cube_names)
        
        self.gripper_cmd("left", 0.0)  # Open
        self.engine.detach_object(cube_name)
        
        # Retreat
        self.engine.move_to_pose(pose_pre_place)
        
        # Reset tip_link to right
        self.engine.tip_link = "tracy/r_gripper_tool_frame"
    
    def parallel_pick_and_place(self, cube_name_to_pick, pick_pos, cube_name_to_place, stack_height):
        """SIMPLIFIED PARALLEL: Travel together, but pick/place sequentially"""
        print(f"\n[PARALLEL TRAVEL] Moving arms to positions...")
        
        # Right arm goals (pre-pick position)
        pick_z = pick_pos[2]
        pose_right_pre_pick = create_pose(pick_pos[0], pick_pos[1], pick_z + 0.2, pitch=3.14, yaw=1.57)
        
        # Left arm goals (pre-place position)
        target_z = self.stack_base[2] + stack_height * self.cube_size
        pose_left_pre_place = create_pose(self.stack_base[0], self.stack_base[1], target_z + 0.1, 
                                         pitch=3.14, yaw=0.0)
        
        # PARALLEL: Move to pre-positions simultaneously
        self.engine.move_dual_arm(
            left_pose=pose_left_pre_place,
            right_pose=pose_right_pre_pick,
            left_tip="tracy/l_gripper_tool_frame",
            right_tip="tracy/r_gripper_tool_frame"
        )
        
        # SEQUENTIAL: Left places first
        print(f"[LEFT ARM] Placing {cube_name_to_place}...")
        pose_left_place = create_pose(self.stack_base[0], self.stack_base[1], target_z, 
                                     pitch=3.14, yaw=0.0)
        
        self.engine.tip_link = "tracy/l_gripper_tool_frame"
        self.engine.move_to_pose(pose_left_place, object_to_allow_collision=cube_name_to_place,
                                environment_objects_to_allow_collision=["table"] + self.cube_names)
        
        self.gripper_cmd("left", 0.0)  # Open left (release)
        self.engine.detach_object(cube_name_to_place)
        self.engine.move_to_pose(pose_left_pre_place)  # Retreat
        
        # SEQUENTIAL: Right picks next
        print(f"[RIGHT ARM] Picking {cube_name_to_pick}...")
        self.engine.tip_link = "tracy/r_gripper_tool_frame"
        self.gripper_cmd("right", 0.0)  # Open right
        
        pose_right_pick = create_pose(*pick_pos, pitch=3.14, yaw=1.57)
        self.engine.move_to_pose(pose_right_pick, object_to_allow_collision=cube_name_to_pick,
                                environment_objects_to_allow_collision=["table"])
        
        self.gripper_cmd("right", 0.8)  # Close right (grasp)
        self.engine.attach_object(cube_name_to_pick, "tracy/r_gripper_tool_frame")
        
        # Lift - Exact logic from handover test, but with ADDED table safety
        self.engine.move_to_pose(pose_right_pre_pick, object_to_allow_collision=cube_name_to_pick,
                                environment_objects_to_allow_collision=["table"])
    
    def run(self):
        print("\n" + "="*50)
        print("CUBE STACKING DEMO - DUAL ARM COORDINATION")
        print("="*50)
        
        self.setup_scene()
        
        # CYCLE 1: Cube A (Sequential)
        print("\n" + "="*50)
        print("CYCLE 1: Cube A")
        print("="*50)
        self.pick_cube(self.cube_names[0], self.pick_positions[0])
        self.handover_cube(self.cube_names[0])
        self.place_cube(self.cube_names[0], stack_height=0)
        
        # CYCLE 2: Cube B (PARALLEL: Pick B while placing A)
        print("\n" + "="*50)
        print("CYCLE 2: Cube B (PARALLEL OPERATIONS)")
        print("="*50)
        self.parallel_pick_and_place(
            cube_name_to_pick=self.cube_names[1],
            pick_pos=self.pick_positions[1],
            cube_name_to_place=self.cube_names[0],
            stack_height=0
        )
        self.handover_cube(self.cube_names[1])
        self.place_cube(self.cube_names[1], stack_height=1)
        
        print("\n" + "="*50)
        print("DEMO COMPLETE - 2 CUBES STACKED!")
        print("="*50)
    
    def destroy(self):
        self.engine.destroy()
        self.gripper_node.destroy_node()

def main():
    rclpy.init()
    demo = CubeStackingDemo()
    try:
        demo.run()
    except Exception as e:
        print(f"Demo Failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        demo.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
