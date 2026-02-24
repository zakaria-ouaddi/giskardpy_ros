import sys
# try standard paths
sys.path.append("/home/zakaria/workspace/ros/install/giskardpy_ros/lib/python3.12/site-packages")
sys.path.append("/home/zakaria/workspace/ros/install/semantic_digital_twin/lib/python3.12/site-packages")
sys.path.append("/home/zakaria/workspace/ros/install/krrood/lib/python3.12/site-packages")
import rclpy
try:
    from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
    rclpy.init()
    g = GiskardWrapperNode()
    import time
    time.sleep(1)
    
    world = g.world
    p = world.get_body_by_name("r_gripper_tool_frame")
    c = world.get_body_by_name("table")
    
    expr = world.compute_forward_kinematics(root=p, tip=c)
    print("Has expr:", type(expr.casadi_sx))
    
    import casadi as ca
    import numpy as np
    
    vars = world.state.get_variables()
    var_sx = ca.vertcat(*[v.casadi_sx for v in vars])
    val_sx = world.state.positions
    
    f = ca.Function('f', [var_sx], [expr.casadi_sx])
    numeric_mat = f(val_sx).full()
    print("SUCCESS: \n", numeric_mat)
except Exception as e:
    print("FAIL:", e)

