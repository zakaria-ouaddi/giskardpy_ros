from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

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
