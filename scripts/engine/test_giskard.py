import sys
import rclpy
from geometry_msgs.msg import PoseStamped
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.goals.templates import CartesianPose
from semantic_digital_twin.spatial_types import Point3, Quaternion, HomogeneousTransformationMatrix

def main():
    rclpy.init()
    g = GiskardWrapper("test_giskard_abort")
    
    # Try sending something identical to the failing goal
    msc = MotionStatechart()
    
    #... We don't need to rebuild everything here.
    # It would be easier to just catch the exception in motion_engine.py and print the result!
