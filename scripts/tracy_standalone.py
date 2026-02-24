from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy_ros.ros2 import rospy
from rclpy import Parameter

from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import StandAloneBTConfig
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.iai_robots.tracy import (
    WorldWithTracyConfig,
    TracyStandAloneRobotInterfaceConfig,
)
from giskardpy_ros.utils.utils import load_xacro
from giskardpy_ros.tree.blackboard_utils import GiskardBlackboard
import traceback

INITIAL_JOINT_POSITIONS = {
    "left_shoulder_pan_joint": 2.92,
    "right_shoulder_pan_joint": 3.35,
    "left_shoulder_lift_joint": -1.24,
    "right_shoulder_lift_joint": -1.90,
    "left_elbow_joint": 1.90,
    "right_elbow_joint": -1.90,
    "left_wrist_1_joint": -1.57,
    "right_wrist_1_joint": -1.57,
    "left_wrist_2_joint": -0.5,
    "right_wrist_2_joint": 0.5,
    "left_wrist_3_joint": 0.0,
    "right_wrist_3_joint": 0.0,
}

def main():
    rospy.init_node("giskard")
    rospy.node.declare_parameters(
        namespace="", parameters=[("robot_description", Parameter.Type.STRING)]
    )
    robot_description = rospy.node.get_parameter_or("robot_description").value
    # robot_description = load_xacro("package://iai_tracy_description/urdf/tracy.urdf.xacro")

    giskard = Giskard(
        world_config=WorldWithTracyConfig(urdf=robot_description),
        robot_interface_config=TracyStandAloneRobotInterfaceConfig(),
        behavior_tree_config=StandAloneBTConfig(publish_tf=True, debug_mode=True),
        qp_controller_config=QPControllerConfig(target_frequency=33),
        collision_checker_id=CollisionCheckerLib.bpb,
    )
    
    try:
        giskard.setup()
        
        # Set initial positions
        initial_joint_state = {}
        for joint_name, position in INITIAL_JOINT_POSITIONS.items():
            try:
                joint = giskard.world_config.world.get_connection_by_name(joint_name)
                initial_joint_state[joint] = position
            except Exception as e:
                print(f"Skipping initial pos for {joint_name}: {e}")
                
        if initial_joint_state:
            giskard.world_config.world.set_positions_1DOF_connection(initial_joint_state)
            print("Initial joint positions set.")
            
        GiskardBlackboard().tree.live()
        
    except Exception as e:
        traceback.print_exc()
        import rclpy
        rclpy.shutdown()


if __name__ == "__main__":
    main()