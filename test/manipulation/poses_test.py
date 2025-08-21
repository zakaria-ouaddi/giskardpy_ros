import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy
import tf_transformations
import sys

def load_grasps_from_yaml(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
        return data['grasps']

def dict_to_pose_stamped(grasp, frame="world"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = grasp['position']['x']
    pose.pose.position.y = grasp['position']['y']
    pose.pose.position.z = grasp['position']['z']
    pose.pose.orientation = Quaternion(
        x=grasp['orientation']['x'],
        y=grasp['orientation']['y'],
        z=grasp['orientation']['z'],
        w=grasp['orientation']['w']
    )
    return pose

def transform_pose_relative_to_cube(grasp, cube_pose):
    # grasp: dict with position & orientation
    # cube_pose: (x, y, z, qx, qy, qz, qw)
    p = grasp['position']
    q = grasp['orientation']
    # Convert to transformation matrices
    m_grasp = tf_transformations.quaternion_matrix([q['x'], q['y'], q['z'], q['w']])
    m_grasp[0:3, 3] = [p['x'], p['y'], p['z']]
    m_cube = tf_transformations.quaternion_matrix(cube_pose[3:])
    m_cube[0:3, 3] = cube_pose[:3]
    # Compose transforms: world_T_grasp = world_T_cube * cube_T_grasp
    m_out = np.dot(m_cube, m_grasp)
    out_pos = m_out[0:3, 3]
    out_quat = tf_transformations.quaternion_from_matrix(m_out)
    return {
        'position': {'x': out_pos[0], 'y': out_pos[1], 'z': out_pos[2]},
        'orientation': {'x': out_quat[0], 'y': out_quat[1], 'z': out_quat[2], 'w': out_quat[3]}
    }

def main(yaml_path, cube_pose):
    rospy.init_node('validate_transformed_grasps_giskard')
    giskard = GiskardWrapperNode('validate_transformed_grasps')
    giskard.spin_in_background()

    # --- Spawn the cube ---
    cube_size = (0.09, 0.09, 0.09)  # Adjust if needed
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = cube_pose[0]
    pose.pose.position.y = cube_pose[1]
    pose.pose.position.z = cube_pose[2]
    pose.pose.orientation.x = cube_pose[3]
    pose.pose.orientation.y = cube_pose[4]
    pose.pose.orientation.z = cube_pose[5]
    pose.pose.orientation.w = cube_pose[6]
    giskard.world.add_box(name='target_cube', size=cube_size, pose=pose)
    giskard.world.dye_group(group_name='target_cube', rgba=(1.0, 0.5, 0.0, 1.0))

    # --- Load grasps ---
    grasps = load_grasps_from_yaml(yaml_path)
    valid_ids = []

    for grasp in grasps:
        transformed = transform_pose_relative_to_cube(grasp, cube_pose)
        grasp_pose = dict_to_pose_stamped(transformed)
        giskard.motion_goals.clear()
        giskard.motion_goals.avoid_all_collisions()
        giskard.add_default_end_motion_conditions()
        giskard.motion_goals.add_cartesian_pose(
            name='test_grasp',
            goal_pose=grasp_pose,
            tip_link='r_gripper_tool_frame',
            root_link='world',
            end_condition='test_grasp'
        )
        try:
            res = giskard.execute()
            if hasattr(res, "success") and res.success:
                valid_ids.append(grasp['id'])
            elif res is True:
                valid_ids.append(grasp['id'])
            else:
                print(f"Grasp {grasp['id']} failed.")
        except Exception as e:
            print(f"Grasp {grasp['id']} invalid: {e}")

    print("Valid (collision-free) grasp IDs:", valid_ids)
    return valid_ids

if __name__ == '__main__':
    # Provide the YAML file path directly here
    yaml_path = "/home/zakaria/workspace/ros/src/giskardpy_ros/test/manipulation/Cube_Pad_grasps.yaml"
    # Provide the cube pose directly here: (x, y, z, qx, qy, qz, qw)
    cube_pose = (0.7, 0.0, 0.95, 0, 0, 0, 1)
    main(yaml_path, cube_pose)