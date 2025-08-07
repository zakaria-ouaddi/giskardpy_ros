import rclpy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.duration import Duration
from rclpy.time import Time
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback

import giskardpy_ros.ros2.tfwrapper as tf
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy


class InteractiveMarkerNode:
    def __init__(self) -> None:
        super().__init__()
        self.giskard = GiskardWrapperNode('interactive_cartesian_goals')
        tf.init(self.giskard.node_handle)

        self.root_link = 'map'
        self.tip_links = ['r_gripper_tool_frame', 'l_gripper_tool_frame']

        self.server = InteractiveMarkerServer(self.giskard.node_handle, 'cartesian_goals')
        self.int_markers = {}

        for tip_link in self.tip_links:
            int_marker = self.create_interactive_marker(tip_link)
            self.int_markers[tip_link] = int_marker
            self.server.insert(int_marker)
            self.server.setCallback(int_marker.name, self.process_feedback)

        self.server.applyChanges()

    def create_interactive_marker(self, tip_link: str) -> InteractiveMarker:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = tip_link
        int_marker.name = f'{self.root_link}/{tip_link}'
        int_marker.scale = 0.25
        int_marker.pose.orientation.w = 1.0

        # Cube visualization
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.175
        box_marker.scale.y = 0.175
        box_marker.scale.z = 0.175
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.5

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(box_control)

        # Add 6DOF controls
        self.add_control(int_marker, 'move_x', InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, 'move_y', InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, 'move_z', InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0, 1.0)

        self.add_control(int_marker, 'rotate_x', InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0, 1.0)
        self.add_control(int_marker, 'rotate_y', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0, 1.0)
        self.add_control(int_marker, 'rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0, 1.0)

        return int_marker

    def add_control(self, int_marker: InteractiveMarker, name: str, interaction_mode: int,
                    x: float, y: float, z: float, w: float) -> None:
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
            tip_link = feedback.header.frame_id
            self.giskard.node_handle.get_logger().info(f"Marker feedback received for {tip_link}")
            goal = PoseStamped()
            goal.header = feedback.header
            goal.pose = feedback.pose
            self.giskard.motion_goals.add_cartesian_pose(goal_pose=goal,
                                                         tip_link=tip_link,
                                                         root_link=self.root_link)
            self.giskard.motion_goals.allow_all_collisions()
            self.giskard.add_default_end_motion_conditions()
            self.giskard.execute_async()

            # Reset marker pose
            marker = self.int_markers[tip_link]
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            self.server.insert(marker)
            self.server.applyChanges()


def main(args: None = None) -> None:
    rospy.init_node('interactive_marker')
    node = InteractiveMarkerNode()
    node.giskard.node_handle.get_logger().info('interactive marker server running')
    rospy.spinner_thread.join()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

