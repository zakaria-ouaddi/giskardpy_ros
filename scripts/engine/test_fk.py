import sys
# Provide explicit paths if needed
sys.path.append('/home/zakaria/workspace/ros/install/giskardpy_ros/lib/python3.12/site-packages')
sys.path.append('/home/zakaria/workspace/ros/install/semantic_digital_twin/lib/python3.12/site-packages')
sys.path.append('/home/zakaria/workspace/ros/install/krrood/lib/python3.12/site-packages')

from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy

rospy.init_node('test_fk')
g = GiskardWrapperNode()
import time
time.sleep(2)

world = g.world
table = world.get_body_by_name("table")
gripper = world.get_body_by_name("r_gripper_tool_frame")

expr = world.compute_forward_kinematics(gripper, table)
import casadi as ca
import numpy as np

vars_list = world.state.get_variables()
var_sx = ca.vertcat(*[v.casadi_sx for v in vars_list])
val_sx = np.concatenate([
    world.state.positions,
    world.state.velocities,
    world.state.accelerations,
    world.state.jerks
])
f = ca.Function('f', [var_sx], [expr.casadi_sx])
numeric_mat = f(val_sx).full()
print("EVALUATED MATRIX gripper -> table:")
print(numeric_mat)
