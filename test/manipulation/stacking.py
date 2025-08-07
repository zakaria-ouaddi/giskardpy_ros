from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Quaternion
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy

# --------------------------- Robot joint poses ---------------------------
ready_pose = {
    # Right arm
    'right_shoulder_pan_joint': 0.07298205015556784,
    'right_shoulder_lift_joint': -1.5451006079298082,
    'right_elbow_joint': 1.6973897170940788,
    'right_wrist_1_joint': 1.2240374339159892,
    'right_wrist_2_joint': 0.9671098455282143,
    'right_wrist_3_joint': -2.867628177785656,
    # Right gripper
    'right_robotiq_85_left_knuckle_joint':  4.3396027437163125e-18,
    'right_robotiq_85_left_finger_tip_joint': 0.0,
    'right_robotiq_85_right_knuckle_joint': 0.0,
    'right_robotiq_85_right_finger_tip_joint': 0.0,
    'right_robotiq_85_left_inner_knuckle_joint': 0.0,
    'right_robotiq_85_right_inner_knuckle_joint': 0.0,
}

tucked_pose = {
    # Right arm
    'right_shoulder_pan_joint': 0.04,
    'right_shoulder_lift_joint': -1.75,
    'right_elbow_joint': 1.32,
    'right_wrist_1_joint': 1.3,
    'right_wrist_2_joint': 1.9,
    'right_wrist_3_joint': -0.6,
    # Right gripper
    'right_robotiq_85_left_knuckle_joint': 2.6037616462297832e-17,
    'right_robotiq_85_right_knuckle_joint': 0.0,
    'right_robotiq_85_left_inner_knuckle_joint': 0.0,
    'right_robotiq_85_right_inner_knuckle_joint': 0.0,
    'right_robotiq_85_left_finger_tip_joint': 0.0,
    'right_robotiq_85_right_finger_tip_joint': 0.0,
}

right_closed = {
    "right_robotiq_85_right_finger_tip_joint": 0.0,
    "right_robotiq_85_left_finger_tip_joint": 0.0,
}
right_open = {
    "right_robotiq_85_right_finger_tip_joint": 0.8,
    "right_robotiq_85_left_finger_tip_joint": 0.8,
}

# --------------------------- Initialization ---------------------------
rospy.init_node('stack_cups_giskard')
giskard = GiskardWrapperNode('stack_cups')
giskard.spin_in_background()

giskard.motion_goals.add_joint_position(tucked_pose, name='tuck', end_condition='tuck')
giskard.motion_goals.add_joint_position(ready_pose, start_condition='tuck')
giskard.motion_goals.avoid_all_collisions()
giskard.add_default_end_motion_conditions()

# --------------------------- Cup properties ---------------------------
cup_names = ['blue_cup', 'red_cup', 'green_cup']
cup_colors = [(0.0, 0.0, 1.0, 1.0), (1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0)]
cup_poses = [
    (0.7, 0.0, 0.95),  # blue
    (0.8, 0.0, 0.95),  # red
    (0.9, 0.0, 0.95)   # green
]
cup_size = (0.09, 0.09, 0.09)  # x, y, z (height = 9cm)
stack_base_pose = (1, -0.5, 0.95)  # Where to stack the cups

for i, (name, color, pose_xyz) in enumerate(zip(cup_names, cup_colors, cup_poses)):
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pose_xyz
    pose.pose.orientation.w = 1.0
    giskard.world.add_box(name=name, size=cup_size, pose=pose)
    giskard.world.dye_group(group_name=name, rgba=color)

def make_quat(x, y, z, w):
    q = Quaternion()
    q.x = x
    q.y = y
    q.z = z
    q.w = w
    return q

giskard.execute()

# --------------------------- Helper function ---------------------------
def set_collision_for_current_target(current_cup_name):
    # Reset to avoid all collisions
    giskard.motion_goals.avoid_all_collisions()
    # Allow collision only with the current cup
    giskard.motion_goals.allow_collision(group1='robot', group2=current_cup_name)
    # Allow robot self-collision (optional but recommended)
    giskard.motion_goals.allow_self_collision()

# --------------------------- Main stacking loop ---------------------------
for i, cup in enumerate(cup_names):
    # Allow collision only with the current box before approaching it
    set_collision_for_current_target(cup)

    # Step 1: Move above the cup (pre-grasp)
    pre_grasp_pose = PoseStamped()
    pre_grasp_pose.header.frame_id = cup
    pre_grasp_pose.pose.orientation = make_quat(0, 1, 0, 0)
    pre_grasp_pose.pose.position.z = 0.20
    pregrasp_name = f'{cup}_pregrasp'
    giskard.motion_goals.add_cartesian_pose(
        name=pregrasp_name,
        goal_pose=pre_grasp_pose,
        tip_link='r_gripper_tool_frame',
        root_link='world',
        end_condition=pregrasp_name
    )

    # Step 2: Move down to grasp
    grasp_pose = deepcopy(pre_grasp_pose)
    grasp_pose.pose.position.z -= 0.17  # Adjust height for grasping
    grasp_name = f'{cup}_grasp'
    giskard.motion_goals.add_cartesian_pose(
        name=grasp_name,
        goal_pose=grasp_pose,
        tip_link='r_gripper_tool_frame',
        root_link='world',
        start_condition=pregrasp_name,
        end_condition=grasp_name
    )

    # Step 3: Close gripper
    close_name = f'{cup}_close'
    giskard.motion_goals.add_joint_position(
        name=close_name,
        goal_state=right_closed,
        start_condition=grasp_name,
        end_condition=close_name
    )

    # Step 4: Execute pick sequence
    giskard.monitors.add_end_motion(start_condition=f'{grasp_name} and {close_name}')
    # Collision config already set by set_collision_for_current_target
    giskard.execute()

    # Step 4.1: Attach cup to gripper
    giskard.world.update_parent_link_of_group(name=cup, parent_link='r_gripper_tool_frame')
    print(f'Attach {cup} to gripper')

    # Step 5: Move above stacking position
    stack_z = stack_base_pose[2] + i * cup_size[2]
    stack_pre_place = PoseStamped()
    stack_pre_place.header.frame_id = 'world'
    stack_pre_place.pose.position.x = stack_base_pose[0]
    stack_pre_place.pose.position.y = stack_base_pose[1]
    stack_pre_place.pose.position.z = stack_z + 0.15
    stack_pre_place.pose.orientation = make_quat(0, 1, 0, 0)
    preplace_name = f'{cup}_preplace'
    giskard.motion_goals.add_cartesian_pose(
        name=preplace_name,
        goal_pose=stack_pre_place,
        tip_link='r_gripper_tool_frame',
        root_link='world',
        end_condition=preplace_name
    )
    print(f'Move above stacking position for {cup}')

    # Step 6: Lower to stack position (place)
    place_pose = deepcopy(stack_pre_place)
    place_pose.pose.position.z -= 0.15
    place_name = f'{cup}_place'

    # Allow collision only with the current box and the boxes already stacked
    giskard.motion_goals.avoid_all_collisions()
    # Allow collision with the current box
    giskard.motion_goals.allow_collision(group1='robot', group2=cup)
    # Allow collision with previously stacked boxes (to allow gripper to contact them)
    for prev_cup in cup_names[:i]:
        giskard.motion_goals.allow_collision(group1='robot', group2=prev_cup)
    giskard.motion_goals.allow_self_collision()

    giskard.motion_goals.add_cartesian_pose(
        name=place_name,
        goal_pose=place_pose,
        tip_link='r_gripper_tool_frame',
        root_link='world',
        start_condition=preplace_name,
        end_condition=place_name
    )
    print(f'Lower to stack position for {cup}')

    # Step 7: Open gripper to release
    open_name = f'{cup}_open'
    giskard.motion_goals.add_joint_position(
        name=open_name,
        goal_state=right_open,
        start_condition=place_name,
        end_condition=open_name
    )
    print(f'Open gripper to release {cup}')

    giskard.monitors.add_end_motion(start_condition=f'{place_name} and {open_name}')
    giskard.motion_goals.allow_collision(group1='robot', group2=cup)  # Keep allowing collision with placed box during release
    giskard.monitors.add_check_trajectory_length()
    giskard.motion_goals.allow_self_collision()

    giskard.execute()

    # Step 8: Detach cup from gripper, parent to world
    giskard.world.update_parent_link_of_group(name=cup, parent_link='world')
    print(f'Detach {cup} from gripper, parent to world')

# Step 9: Return to tucked pose after stacking
giskard.motion_goals.add_joint_position(tucked_pose, start_condition=f'{cup}_open', end_condition='done')
giskard.execute()

print("All cups stacked successfully!")
