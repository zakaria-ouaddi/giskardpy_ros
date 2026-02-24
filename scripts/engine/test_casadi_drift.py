import rclpy
import sys
import os
import time
import numpy as np
import casadi as ca
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from motion_engine import GiskardMotionEngine
from config import DualArmConfig
from skills.utils import create_pose
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body

def spawn_cube(engine, name, x, y, z):
    pose = create_pose(x, y, z, frame="map2")
    p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    q = cas.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    with engine.giskard.world.modify_world():
        obj = Body(name=PrefixedName(name))
        shape = Box(scale=Scale(0.05, 0.05, 0.05))
        obj.collision.append(shape)
        obj.visual.append(shape)
        
        root_link = engine._resolve_entity_name("map2")
        conn = FixedConnection(parent=root_link, child=obj, parent_T_connection_expression=parent_T_pose)
        engine.giskard.world.add_connection(conn)

def evaluate_casadi(engine, name):
    obj_body = engine.giskard.world.get_body_by_name(name)
    parent_link = engine._resolve_entity_name("r_gripper_tool_frame")
    
    expr = engine.giskard.world.compute_forward_kinematics(root=parent_link, tip=obj_body)
    
    vars_list = engine.giskard.world.state.get_variables()
    if len(vars_list) > 0:
        var_sx = ca.vertcat(*[v.casadi_sx for v in vars_list])
        val_sx = np.concatenate([
            engine.giskard.world.state.positions,
            engine.giskard.world.state.velocities,
            engine.giskard.world.state.accelerations,
            engine.giskard.world.state.jerks
        ])
        f = ca.Function('f', [var_sx], [expr.casadi_sx])
        numeric_mat = f(val_sx).full()
    else:
        numeric_mat = np.eye(4)
        
    print(f"\n[{name}] offset from r_gripper_tool_frame to {name}:\n{numeric_mat}")
    
    # Calculate offset distance (Frobenius norm of translation, rotation deviation)
    translation_mag = np.linalg.norm(numeric_mat[0:3, 3])
    rotation_trace = np.trace(numeric_mat[0:3, 0:3])
    print(f"[{name}] Translation Mag: {translation_mag:.5f}, Rot Trace: {rotation_trace:.5f}")
    return numeric_mat

def main():
    rclpy.init()
    config = DualArmConfig()
    engine = GiskardMotionEngine(tip_link=config.RIGHT_TIP)

    cubes = [
        ("cube_2", config.BASE_Z + 2 * config.CUBE_HEIGHT),
        ("cube_1", config.BASE_Z + 1 * config.CUBE_HEIGHT)
    ]

    for name, z in cubes:
        spawn_cube(engine, name, config.RIGHT_ARM_START_X, config.RIGHT_ARM_START_Y, z)
        
    for name, z in cubes:
        print(f"\n=== Evaluating {name} ===")
        pose = create_pose(
            config.RIGHT_ARM_START_X, 
            config.RIGHT_ARM_START_Y, 
            z, 
            pitch=3.14, yaw=1.57
        )
        engine.move_to_pose(pose, linear_speed=config.LINEAR_SPEED, angular_speed=config.ANGULAR_SPEED)
        time.sleep(1.0)
        
        with engine.giskard.world.modify_world():
            evaluate_casadi(engine, name)

    engine.destroy()

if __name__ == "__main__":
    main()
