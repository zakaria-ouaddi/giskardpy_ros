import rclpy
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import ExecutionType, Arms
from pycram.process_module import ProcessModuleManager
from pycram.language import SequentialPlan
from pycram.robot_plans.motions.gripper import MoveTCPMotion

import pycram
print(f"DEBUG: pycram file: {pycram.__file__}")

def main():
    # Initialize ROS
    rclpy.init()
    node = rclpy.create_node("pycram_demo")

    # Start a background thread to spin the node
    # This is required for action clients (GiskardWrapper) AND Visualization Publishers
    import threading
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    
    # ProcessModuleManager needs the node for Giskard communication (if needed by sim?)
    # ProcessModuleManager.node = node

    # GiskardWrapper uses 'giskardpy_ros.ros2.rospy.node' globally
    # We must set it to our node
    from giskardpy_ros.ros2 import rospy
    rospy.node = node

    # Initialize World
    world = World()

    # --- EXECUTION ---
    print("Parsing URDF from local file and merging into World...")
    sdt_base = "/home/zakaria/workspace/ros/src/cognitive_robot_abstract_machine/semantic_digital_twin/resources"
    urdf_path = f"{sdt_base}/urdf/tracy.urdf"
    
    from semantic_digital_twin.adapters.urdf import URDFParser
    # Use URDFParser.from_file if available, or just read file content and pass to URDFParser(urdf=content)
    try:
        urdf_parser = URDFParser.from_file(urdf_path)
    except AttributeError:
        with open(urdf_path, 'r') as f:
            urdf_string = f.read()
        urdf_parser = URDFParser(urdf=urdf_string)
        
    # Simplify: parse returns a World. Merge it.
    parsed_world = urdf_parser.parse()
    world.merge_world(parsed_world)

    print("Loading Tracy Semantic Annotation...")
    tracy = Tracy.from_world(world)
    
    # --- VISUALIZATION SETUP ---
    print("Setting up Visualization Publishers...")
    # TF Publisher to show robot/frames in RViz
    tf_publisher = TFPublisher(node=node, world=world)
    # Viz Marker Publisher to show objects/links
    viz_publisher = VizMarkerPublisher(world=world, node=node)
    
    print("Visualization Active.")
    print("In RViz:")
    print(f"  1. Set Fixed Frame to: {world.root.name}")
    print("  2. Add MarkerArray display on topic: /semworld/viz_marker")
    print("  3. Set Durability Policy to TRANSIENT_LOCAL")
    
    # --- CREATE CONTEXT ---
    context = Context(world, tracy, ros_node=node)

    # --- DEMO LOGIC ---
    from pycram.process_module import simulated_robot
    
    # 1. Move Right Arm to Safe High Pose
    pose_right = PoseStamped.from_list(
        [0.6, -0.2, 1.1],
        [0, 0, 0, 1],
        frame="map"
    )
    
    # 2. Move Left Arm to "Table" Pose (Collision Test: z=0.85, Table is 0.88)
    pose_left = PoseStamped.from_list(
        [0.6, 0.2, 0.85], 
        [0, 0, 0, 1],
        frame="map"
    )

    print("Defining Collision Test Plan...")
    plan = SequentialPlan(
        context,
        # Move Right Arm High
        MoveTCPMotion(target=pose_right, arm=Arms.RIGHT),
        # Move Left Arm Low (Should trigger avoidance/failure)
        MoveTCPMotion(target=pose_left, arm=Arms.LEFT)
    )

    print("Performing Collision Test Plan (Simulated)...")

    # Debug: Print Execution State
    print(f"DEBUG: Execution Type: {ProcessModuleManager.execution_type}")
    manager = ProcessModuleManager().get_manager(tracy)
    print(f"DEBUG: Manager for Tracy: {manager}")
    if manager:
        print(f"DEBUG: MoveTCP Module: {manager.move_tcp()}")
    
    # Debug: Print current TCP pose
    l_arm_pose = world.get_body_by_name("left_robotiq_85_left_finger_tip_link").global_pose
    print(f"Initial Left Arm Pose: {l_arm_pose}")

    with simulated_robot:
        plan.perform()
    
    # Debug: Print new TCP pose
    l_arm_pose_new = world.get_body_by_name("left_robotiq_85_left_finger_tip_link").global_pose
    print(f"Final Left Arm Pose: {l_arm_pose_new}")
    
    print("Action completed.")
    
    input("Press Enter to exit...")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
