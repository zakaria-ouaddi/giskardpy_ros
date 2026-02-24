import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from giskard_msgs.msg import ExecutionState
import sys

# Provide explicit paths if needed
sys.path.append('/home/zakaria/workspace/ros/src/giskardpy_ros/scripts/engine')
from motion_engine import GiskardMotionEngine

rclpy.init()
engine = GiskardMotionEngine()
import time
time.sleep(2)
world = engine.giskard.world
table = world.get_body_by_name("table")
gripper = world.get_body_by_name("r_gripper_tool_frame")

tf_expr = world.compute_forward_kinematics(gripper, table)
print("Expression:", type(tf_expr))

# How do we evaluate it?
if hasattr(tf_expr, 'evaluate'):
    print("Has evaluate:", tf_expr.evaluate())
if hasattr(tf_expr, 'get_value'):
    print("Has get_value:", tf_expr.get_value())
    
# Let's check state dependencies
print("State vars:", world.state.get_values())

try:
    numeric_tf = tf_expr.eval(world.state.get_values())
    print("Eval with state:", numeric_tf)
except Exception as e:
    print("eval error:", e)

