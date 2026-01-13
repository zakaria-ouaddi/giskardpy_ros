from time import sleep

import rclpy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy import Parameter
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback

from giskardpy.model.collision_matrix_manager import CollisionRequest
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.payload_monitors import CountSeconds
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy_ros.python_interface.python_interface import (
    GiskardWrapper,
)
from giskardpy_ros.ros2 import rospy
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import WorldEntityNotFoundError
from semantic_digital_twin.spatial_types import TransformationMatrix
import semantic_digital_twin.spatial_types.spatial_types as cas


class InteractiveMarkerNode:
    def __init__(self) -> None:
        super().__init__()
        self.giskard = GiskardWrapper(
            node_handle=rospy.node, giskard_node_name="giskard"
        )

        # Create an interactive marker server
        self.server = InteractiveMarkerServer(rospy.node, "cartesian_goals")

        # We define the kinematic chains we want to control here
        self.chains_config = [
            {"root": "map2", "tip": "r_gripper_tool_frame"},
            {"root": "map2", "tip": "l_gripper_tool_frame"},
        ]

        # This dictionary will store the Giskard body objects for each marker
        # Structure: { 'marker_name': {'root_body': obj, 'tip_body': obj, 'marker_msg': msg} }
        self.active_markers = {}

        for config in self.chains_config:
            self.create_marker_for_chain(config["root"], config["tip"])

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def create_marker_for_chain(self, root_link_name, tip_link_name):
        """Helper function to create a marker for a specific root/tip pair"""

        # 1. Get Giskard Body Objects
        root_body = None
        tip_body = None

        for i in range(100):
            try:
                root_body = self.giskard.world.get_kinematic_structure_entity_by_name(root_link_name)
                tip_body = self.giskard.world.get_kinematic_structure_entity_by_name(tip_link_name)
                break
            except WorldEntityNotFoundError as e:
                sleep(0.5)
                self.giskard.node_handle.get_logger().error(
                    f"Waiting for bodies {root_link_name} or {tip_link_name}..."
                )
        else:
            self.giskard.node_handle.get_logger().error(
                f"Could not find bodies for chain {root_link_name}->{tip_link_name}")
            return

        # 2. Create the Interactive Marker
        marker_name = f"{root_link_name}/{tip_link_name}"

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = tip_link_name  # Marker starts attached to the tip
        int_marker.name = marker_name
        int_marker.scale = 0.25
        int_marker.pose.orientation.w = 1.0

        # Visual Cube
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.175
        box_marker.scale.y = 0.175
        box_marker.scale.z = 0.175
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.5

        # Control Box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(box_control)

        # 6-DOF Controls
        self.add_control(int_marker, "move_x", InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, "move_y", InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, "move_z", InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0, 1.0)
        self.add_control(int_marker, "rotate_x", InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, "rotate_y", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, "rotate_z", InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0, 1.0)

        # 3. Store Data and Insert
        self.active_markers[marker_name] = {
            "root_body": root_body,
            "tip_body": tip_body,
            "marker_msg": int_marker
        }

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.giskard.node_handle.get_logger().info(f"Created interactive marker for {marker_name}")

    def add_control(self, int_marker, name, interaction_mode, x, y, z, w):
        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = interaction_mode
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        int_marker.controls.append(control)

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:

            # Retrieve the correct body objects based on which marker was moved
            marker_name = feedback.marker_name
            if marker_name not in self.active_markers:
                return

            chain_data = self.active_markers[marker_name]
            root_body = chain_data["root_body"]
            tip_body = chain_data["tip_body"]
            original_marker = chain_data["marker_msg"]

            self.giskard.node_handle.get_logger().info(f"Sending goal for {marker_name}")

            # Calculate Goal
            goal = TransformationMatrix.from_xyz_quaternion(
                pos_x=feedback.pose.position.x,
                pos_y=feedback.pose.position.y,
                pos_z=feedback.pose.position.z,
                quat_x=feedback.pose.orientation.x,
                quat_y=feedback.pose.orientation.y,
                quat_z=feedback.pose.orientation.z,
                quat_w=feedback.pose.orientation.w,
                reference_frame=self.giskard.world.get_kinematic_structure_entity_by_name(
                    feedback.header.frame_id
                ),
            )

            # Build Motion Statechart
            msc = MotionStatechart()
            cart_goal = CartesianPose(
                root_link=root_body,
                tip_link=tip_body,
                goal_pose=goal,
            )
            msc.add_node(cart_goal)
            max_traj = CountSeconds(seconds=20)
            msc.add_node(max_traj)
            collision_avoidance = CollisionAvoidance(
                collision_entries=[CollisionRequest.avoid_all_collision()],
            )
            msc.add_node(collision_avoidance)
            end = EndMotion()
            msc.add_node(end)
            end.start_condition = cas.trinary_logic_or(cart_goal.observation_variable, max_traj.observation_variable)

            self.giskard.execute_async(msc)

            # Reset marker pose to 0 (relative to tip) so it snaps back to the hand
            original_marker.pose.position.x = 0.0
            original_marker.pose.position.y = 0.0
            original_marker.pose.position.z = 0.0
            original_marker.pose.orientation.x = 0.0
            original_marker.pose.orientation.y = 0.0
            original_marker.pose.orientation.z = 0.0
            original_marker.pose.orientation.w = 1.0

            self.server.insert(original_marker)
            self.server.applyChanges()


def main(args: None = None) -> None:
    rospy.init_node("interactive_marker")
    node = InteractiveMarkerNode()
    node.giskard.node_handle.get_logger().info("interactive marker server running")
    rospy.spinner_thread.join()
    rclpy.shutdown()


if __name__ == "__main__":
    main()