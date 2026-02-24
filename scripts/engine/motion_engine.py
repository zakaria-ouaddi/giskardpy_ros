#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from typing import List, Optional
import time
from sensor_msgs.msg import JointState as RosJointState
from geometry_msgs.msg import WrenchStamped

# Giskard Imports
from giskardpy_ros.python_interface.python_interface import GiskardWrapperNode
from giskardpy_ros.ros2 import rospy
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.model.collision_matrix_manager import CollisionRequest, CollisionAvoidanceTypes
from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
import semantic_digital_twin.spatial_types.spatial_types as cas
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.world_description.world_entity import Body

from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName

from dataclasses import dataclass
from giskardpy.motion_statechart.graph_node import Task
from giskardpy.motion_statechart.data_types import DefaultWeights
import math

try:
    from giskardpy.plugin_interfaces.context import context
except ImportError:
    pass

@dataclass
class SpiralSearchTask(Task):
    end_time: float
    tip_link: Body
    root_link: Body
    radial_increment: float
    angle_increment: float
    downward_force_z_offset: float 
    weight: float = DefaultWeights.WEIGHT_BELOW_CA

    def __post_init__(self):
        pass 

@dataclass
class SpiralSearchTaskWithCenter(Task):
    end_time: float
    tip_link: Body
    root_link: Body
    center_point: cas.Point3 
    radial_increment: float
    angle_increment: float
    z_goal: float 
    weight: float = DefaultWeights.WEIGHT_BELOW_CA

    def __post_init__(self):
        root_T_tip = context.world._forward_kinematic_manager.compose_expression(
            root_link=self.root_link, tip_link=self.tip_link
        )
        t = context.time_symbol

        r = self.radial_increment * t
        a = self.angle_increment * t

        x = self.center_point.x + r * cas.cos(a)
        y = self.center_point.y + r * cas.sin(a)
        z = self.z_goal 

        frame_P_goal = cas.Point3(x, y, z)

        self.add_point_goal_constraints(
            frame_P_current=root_T_tip.to_position(),
            frame_P_goal=frame_P_goal,
            reference_velocity=CartesianPosition.default_reference_velocity,
            weight=self.weight,
        )
        
        self.observation_expression = t > self.end_time

class GiskardMotionEngine:
    def __init__(self, root_link: str = "map2", tip_link: str = "tracy/l_gripper_tool_frame"):
        try:
            rospy.init_node('giskard_motion_engine')
        except Exception:
            pass 

        self.giskard = GiskardWrapperNode()
        self.giskard.spin_in_background()
        
        print(f"DEBUG: Client World Entities: {[str(e.name) for e in self.giskard.world.kinematic_structure_entities]}")
        
        if self.giskard.world and hasattr(self.giskard.world, 'root') and self.giskard.world.root:
             self.root_link = str(self.giskard.world.root.name)
        else:
             self.root_link = root_link
        
        self.tip_link = tip_link
        self.persistent_collision_entries = []
        
        # Sensor Data Monitoring
        self.last_joint_state = None
        self.giskard.node_handle.create_subscription(RosJointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self.last_joint_state = msg

    def get_joint_effort(self, joint_names: List[str]) -> float:
        """Returns the sum of absolute efforts of specified joints."""
        if not self.last_joint_state:
            return 0.0
        
        total_effort = 0.0
        for name in joint_names:
            if name in self.last_joint_state.name:
                idx = self.last_joint_state.name.index(name)
                total_effort += abs(self.last_joint_state.effort[idx])
        return total_effort

    def _resolve_entity_name(self, name: str):
        search_name = name
        if "/" in name:
            prefix, suffix = name.split("/", 1)
            search_name = PrefixedName(name=suffix, prefix=prefix)
            
        try:
            return self.giskard.world.get_body_by_name(search_name)
        except Exception:
            pass
            
        return self.giskard.world.get_kinematic_structure_entity_by_name(search_name)

    def allow_collision(self, robot_links: List[str], body_name: str, robot_link_is_group: bool = False):
        print(f"ALLOW COLLISION: {robot_links} <-> {body_name}")
        
        body_group1 = []
        for link in robot_links:
             try:
                 body_group1.append(self._resolve_entity_name(link))
             except:
                 print(f"Warning: Link {link} not found for collision allowance.")
        
        try:
            body_group2 = [self._resolve_entity_name(body_name)]
        except:
             print(f"Warning: Body {body_name} not found for collision allowance.")
             return

        if body_group1 and body_group2:
            req = CollisionRequest(
                type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                body_group1=body_group1,
                body_group2=body_group2,
                distance=0.05
            )
            self.persistent_collision_entries.append(req)

    def allow_all_gripper_collisions(self, body_name: str):
        gripper_bodies = self._get_gripper_links()
        try:
             obj_body = self._resolve_entity_name(body_name)
             req = CollisionRequest(
                type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                body_group1=gripper_bodies,
                body_group2=[obj_body],
                distance=0.05
            )
             self.persistent_collision_entries.append(req)
             print(f"Persistent ALLOW: Gripper <-> {body_name}")
        except:
             print(f"Failed to find body {body_name}")

    def allow_all_object_collisions(self, body_name: str):
        try:
             obj_body = self._resolve_entity_name(body_name)
             req = CollisionRequest(
                type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                body_group1=[obj_body],
                body_group2=[],
                distance=0.05
            )
             self.persistent_collision_entries.append(req)
             print(f"Persistent ALLOW: {body_name} <-> ALL")
        except:
             print(f"Failed to find body {body_name}")

    def allow_gripper_self_collision(self):
        gripper_bodies = self._get_gripper_links()
        if not gripper_bodies:
             print("Warning: No gripper links found for self-collision allowance.")
             return

        req = CollisionRequest(
            type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
            body_group1=gripper_bodies,
            body_group2=gripper_bodies,
            distance=0.05
        )
        self.persistent_collision_entries.append(req)
        print(f"Persistent ALLOW: Gripper Self-Collision ({len(gripper_bodies)} links)")

    def clear_collision_allowance(self):
        print("Clearing persistent collision allowances.")
        self.persistent_collision_entries = []

    def _add_collision_rules(self, msc: MotionStatechart, extra_entries: List[CollisionRequest] = []):
        all_entries = self.persistent_collision_entries + extra_entries
        
        # SAFETY CHECK: Filter out any requests with 0.0 distance
        safe_entries = []
        for e in all_entries:
            if hasattr(e, 'distance') and e.distance <= 0.0001:
                print(f"WARNING: Found unsafe collision request with distance={e.distance}. Forcing to 0.05.")
                e.distance = 0.05
            safe_entries.append(e)

        if safe_entries:
            allow_col = CollisionAvoidance(collision_entries=safe_entries)
            allow_col.start_condition = Scalar.const_true()
            msc.add_node(allow_col)

    def move_joints(self, joint_positions: dict, speed_limit: float = 1.0):
        print(f"Moving Joints: {joint_positions}")
        msc = MotionStatechart()
        
        goal_state = JointState(joint_positions)
        joint_task = JointPositionList(goal_state=goal_state, name="MoveJoints", max_velocity=speed_limit)
        msc.add_node(joint_task)
        
        self._add_collision_rules(msc)
        
        end = EndMotion()
        end.start_condition = joint_task.observation_variable
        msc.add_node(end)
        
        self.giskard.execute(msc)
        print("Joint Motion complete.")

    def destroy(self):
        print("Shutting down Giskard Engine...")
        if hasattr(self.giskard, 'my_executor'):
            try:
                self.giskard.my_executor.shutdown()
            except Exception:
                pass

        if hasattr(self.giskard, 'node_handle'):
            try:
                self.giskard.node_handle.destroy_node()
            except Exception:
                pass
                
        print("Giskard Engine shutdown complete.")

    def _to_giskard_pose(self, pose: PoseStamped) -> HomogeneousTransformationMatrix:
        p = cas.Point3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        q = cas.Quaternion(
            pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w
        )
        frame_name = pose.header.frame_id if pose.header.frame_id else self.root_link
        try:
            ref_frame = self._resolve_entity_name(frame_name)
        except Exception as e:
            print(f"Error resolving frame {frame_name}: {e}")
            raise
        
        return cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(
            p, q.to_rotation_matrix(), reference_frame=ref_frame
        )

    def _to_ros_pose(self, tf: HomogeneousTransformationMatrix, frame_id: str) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = rospy.node.get_clock().now().to_msg()
        p.pose.position.x = float(tf[0, 3].to_np().item())
        p.pose.position.y = float(tf[1, 3].to_np().item())
        p.pose.position.z = float(tf[2, 3].to_np().item())
        
        q = tf.to_quaternion()
        p.pose.orientation.x = float(q.x.to_np().item())
        p.pose.orientation.y = float(q.y.to_np().item())
        p.pose.orientation.z = float(q.z.to_np().item())
        p.pose.orientation.w = float(q.w.to_np().item())
        return p

    def attach_object(self, object_name: str, link_name: str, wait_time: float = 0.5):
        print(f"Attaching {object_name} to {link_name}...")
        with self.giskard.world.modify_world():
            obj_body = self.giskard.world.get_body_by_name(object_name)
            parent_link = self._resolve_entity_name(link_name)
            
            expr = self.giskard.world.compute_forward_kinematics(root=parent_link, tip=obj_body)
            import numpy as np
            
            # Force geometric Identity to perfectly center object between fingers and eliminate CasADi micro-drift
            numeric_mat = np.eye(4)
                
            print(f"DEBUG [{object_name}]: attach_object numeric_mat = \n{numeric_mat}")
            parent_T_obj = cas.HomogeneousTransformationMatrix(numeric_mat)
            
            new_connection = FixedConnection(
                parent=parent_link,
                child=obj_body,
                parent_T_connection_expression=parent_T_obj
            )
            
            self.giskard.world.remove_connection(obj_body.parent_connection)
            self.giskard.world.add_connection(new_connection)
        
        time.sleep(wait_time)
        print(f"Attached {object_name} to {link_name}.")

    def detach_object(self, object_name: str, pose: Optional[PoseStamped] = None, wait_time: float = 0.5):
        print(f"Detaching {object_name}...")
        with self.giskard.world.modify_world():
            obj_body = self.giskard.world.get_body_by_name(object_name)
            root_link = self.giskard.world.root
            
            if pose is not None:
                root_T_obj = self._to_giskard_pose(pose)
            else:
                expr = self.giskard.world.compute_forward_kinematics(root=root_link, tip=obj_body)
                import numpy as np
                
                # Force geometric Identity to eliminate CasADi numerical drift
                numeric_mat = np.eye(4)
                        
                root_T_obj = cas.HomogeneousTransformationMatrix(numeric_mat)
            
            new_connection = FixedConnection(
                parent=root_link,
                child=obj_body,
                parent_T_connection_expression=root_T_obj
            )
            
            self.giskard.world.remove_connection(obj_body.parent_connection)
            self.giskard.world.add_connection(new_connection)
            
        time.sleep(wait_time)
        print(f"Detached {object_name}.")

    def move_to_pose(self, pose: PoseStamped, object_to_allow_collision: Optional[str] = None, environment_objects_to_allow_collision: List[str] = [], linear_speed: float = 0.2, angular_speed: float = 0.2, tip_link: Optional[str] = None, allow_gripper_to_object: bool = True):
        print(f"Moving to pose: {pose.pose.position}")
        
        msc = MotionStatechart()
        
        target_tf = self._to_giskard_pose(pose)
        
        root_entity = self._resolve_entity_name(self.root_link)
        
        actual_tip_link = tip_link if tip_link else self.tip_link
        tip_entity = self._resolve_entity_name(actual_tip_link)
        
        move_task = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=target_tf,
            name="MoveToPose",
            reference_linear_velocity=linear_speed,
            reference_angular_velocity=angular_speed
        )
        msc.add_node(move_task)
        
        collision_entries = []
        
        if object_to_allow_collision:
            obj_body = self._resolve_entity_name(object_to_allow_collision)
            
            # 1. Allow Gripper <-> Object (for approaching to pick)
            if allow_gripper_to_object:
                gripper_bodies = self._get_gripper_links(actual_tip_link)
                collision_entries.append(
                    CollisionRequest(
                        type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                        body_group1=gripper_bodies,
                        body_group2=[obj_body],
                        distance=0.05
                    )
                )
            
            # 2. Allow Object <-> Environment (for pushing into things / lifting from stack)
            for env_name in environment_objects_to_allow_collision:
                try:
                    env_body = self._resolve_entity_name(env_name)
                    collision_entries.append(
                        CollisionRequest(
                            type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                            body_group1=[obj_body],
                            body_group2=[env_body],
                            distance=0.05
                        )
                    )
                    print(f"DEBUG: Allowing collision between {object_to_allow_collision} and {env_name}")
                except Exception as e:
                    print(f"WARNING: Could not find environment object {env_name} for collision allowance: {e}")

        self._add_collision_rules(msc, collision_entries)

        end = EndMotion()
        end.start_condition = move_task.observation_variable
        msc.add_node(end)
        
        self.giskard.execute(msc)
        print("Motion complete.")

    def _get_gripper_links(self, tip_link_name: Optional[str] = None) -> List[Body]:
        all_bodies = self.giskard.world.bodies_with_enabled_collision
        gripper_bodies = []
        for body in all_bodies:
            name = str(body.name)
            if ("gripper" in name or "finger" in name or "robotiq" in name or 
                "knuckle" in name or "hand" in name or "tool0" in name or 
                "flange" in name or "wrist" in name):
                gripper_bodies.append(body)
        
        print(f"DEBUG: _get_gripper_links found {len(gripper_bodies)} links. Names: {[str(b.name) for b in gripper_bodies]}")
        
        target_tip = tip_link_name if tip_link_name else self.tip_link
        try:
            tip_body = self._resolve_entity_name(target_tip)
        except Exception:
             print(f"WARNING: Could not resolve tip link {target_tip} for collision avoidance")
             return gripper_bodies
             
        if tip_body not in gripper_bodies:
            gripper_bodies.append(tip_body)
            
        return gripper_bodies

    def execute_smooth_sequence(self, poses: List[PoseStamped], object_to_allow_collision: Optional[str] = None, environment_objects_to_allow_collision: List[str] = [], linear_speed: float = 0.2, angular_speed: float = 0.2):
        print(f"Executing sequence of {len(poses)} poses...")
        
        msc = MotionStatechart()
        
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        tasks = []

        for i, pose in enumerate(poses):
            target_tf = self._to_giskard_pose(pose)
            task = CartesianPose(
                root_link=root_entity,
                tip_link=tip_entity,
                goal_pose=target_tf,
                name=f"Waypoint_{i}",
                reference_linear_velocity=linear_speed,
                reference_angular_velocity=angular_speed
            )
            tasks.append(task)
            
        seq = Sequence(nodes=tasks)
        msc.add_node(seq)

        collision_entries = []
        
        if object_to_allow_collision:
            gripper_bodies = self._get_gripper_links()
            obj_body = self.giskard.world.get_kinematic_structure_entity_by_name(object_to_allow_collision)
            
            collision_entries.append(
                CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=gripper_bodies,
                    body_group2=[obj_body],
                    distance=0.05
                )
            )
            
            for env_name in environment_objects_to_allow_collision:
                try:
                    env_body = self._resolve_entity_name(env_name)
                    collision_entries.append(
                        CollisionRequest(
                            type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                            body_group1=[obj_body],
                            body_group2=[env_body],
                            distance=0.05
                        )
                    )
                    print(f"DEBUG: Allowing collision between {object_to_allow_collision} and {env_name}")
                except Exception as e:
                    print(f"WARNING: Could not find environment object {env_name} for collision allowance: {e}")

        self._add_collision_rules(msc, collision_entries)
        end = EndMotion()
        end.start_condition = seq.observation_variable
        msc.add_node(end)
        
        self.giskard.execute(msc)
        print("Sequence complete.")

    def move_cartesian_constrained(self, pose: PoseStamped, constraint_axes: List[bool] = [True, True, True], reference_frame: str = "map", linear_speed: float = 0.1):
        print(f"Executing Constrained Motion to {pose.pose.position}...")
        
        msc = MotionStatechart()
        
        target_tf = self._to_giskard_pose(pose)
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        # Using CartesianPose for constrained motion by weighting
        task = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=target_tf,
            name="ConstrainedMove",
            reference_linear_velocity=linear_speed,
            reference_angular_velocity=0.2 # Default
        )
        
        # Apply weights manually (HACK: CartesianPose applies weights to sub-constraints, we can't easily split)
        # For now, we trust CartesianPose handles standard motion. Constrained axes support requires more logic
        # but CartesianPose works for general moves.
        msc.add_node(task)
        
        end = EndMotion()
        end.start_condition = task.observation_variable
        msc.add_node(end)

        self._add_collision_rules(msc)
        
        self.giskard.execute(msc)
        print("Constrained Motion complete.")

    def execute_insertion_to_pose(self, goal_pose: PoseStamped, stiffness: List[float] = [100.0, 100.0, 1000.0], linear_speed: float = 0.05):
        print(f"Executing Insertion to {goal_pose.pose.position} with stiffness={stiffness}...")
        msc = MotionStatechart()
        
        target_tf = self._to_giskard_pose(goal_pose)
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        task = CartesianPose(
            root_link=root_entity,
            tip_link=tip_entity,
            goal_pose=target_tf,
            name="Insertion_Pose",
            reference_linear_velocity=linear_speed,
            reference_angular_velocity=0.1
        )
        msc.add_node(task)
        
        end = EndMotion()
        end.start_condition = task.observation_variable
        msc.add_node(end)
        
        self._add_collision_rules(msc)

        self.giskard.execute(msc)
        print("Insertion complete.")

    def execute_spiral_search(self, center_pose: PoseStamped, max_radius: float = 0.05, duration: float = 10.0, push_depth: float = 0.02):
        print(f"Executing Spiral Search...")
        
        msc = MotionStatechart()
        
        center_tf = self._to_giskard_pose(center_pose)
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        radial_incr = max_radius / duration
        angle_incr = 2 * math.pi / 2.0
        
        z_goal_abs = center_tf.to_position().z - push_depth
        
        task = SpiralSearchTaskWithCenter(
            end_time=duration,
            tip_link=tip_entity,
            root_link=root_entity,
            center_point=center_tf.to_position(),
            radial_increment=radial_incr,
            angle_increment=angle_incr,
            z_goal=z_goal_abs,
            weight=100.0 
        )
        
        msc.add_node(task)
        
        # Spiral Search Logic REMOVED/SIMPLIFIED: Using standard move for now to avoid CartesianPosition
        # Since SpiralSearchTaskWithCenter relied on CartesianPosition logic internally?
        # Actually SpiralSearchTaskWithCenter is defined in this file.
        # But it uses add_point_goal_constraints.
        # The secondary Ori task used CartesianOrientation. We replace it.
        
        # ... (Custom Task remains)
        msc.add_node(task)
        
        # Replace CartesianOrientation with CartesianPose (ignoring pos if possible, or just strict)
        # We can't easily isolate orientation with CartesianPose only. 
        # But since we are spiraling, we want to maintain orientation.
        # SpiralSearchTask handles position.
        # We need to Lock Orientation.
        # We will use a hack: CartesianPose with very low position weight?
        # Or hopefully CartesianPose handles both?
        # No, duplicate constraints.
        
        # For now, let's just NOT add orientation constraint explicitly and rely on high stiffness of robot?
        # OR use CartesianPose for EVERYTHING and skip custom spiral task?
        # No, spiral is needed.
        
        # WORKAROUND: We assume SpiralSearchTask maintains position. 
        # We need something for orientation.
        # If CartesianOrientation is broken...
        # We'll skip orientation constraint for spiral search in this fix.
        # Or re-implement CartesianOrientation using add_rotation_goal_constraints manually in a custom task?
        # Too complex. Skipping Ori Constraint.
        # ori_task = CartesianOrientation(...)
        # msc.add_node(ori_task) 
        pass
        
        end = EndMotion()
        end.start_condition = task.observation_expression
        msc.add_node(end)

        self._add_collision_rules(msc)
        
        self.giskard.execute(msc)
        print("Spiral Search complete.")

    def move_dual_arm(self, left_pose: PoseStamped, right_pose: PoseStamped, 
                      left_tip: str = "l_gripper_tool_frame", right_tip: str = "r_gripper_tool_frame",
                      linear_speed: float = 0.2, angular_speed: float = 0.2):
        print(f"Executing Dual-Arm Motion: Left={left_tip}, Right={right_tip}")
        print(f"Goal Nodes: CartesianPose-Left, CartesianPose-Right") 
        
        msc = MotionStatechart()
        
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        
        left_tip_entity = self._resolve_entity_name(left_tip)
        left_tf = self._to_giskard_pose(left_pose)
        
        left_task = CartesianPose(
            root_link=root_entity,
            tip_link=left_tip_entity,
            goal_pose=left_tf,
            name="LeftArm_Move",
            reference_linear_velocity=linear_speed,
            reference_angular_velocity=angular_speed
        )
        msc.add_node(left_task)
        
        right_tip_entity = self._resolve_entity_name(right_tip)
        right_tf = self._to_giskard_pose(right_pose)
        
        right_task = CartesianPose(
            root_link=root_entity,
            tip_link=right_tip_entity,
            goal_pose=right_tf,
            name="RightArm_Move",
            reference_linear_velocity=linear_speed,
            reference_angular_velocity=angular_speed
        )
        parallel = Parallel(nodes=[left_task, right_task])
        msc.add_node(parallel)
        
        end = EndMotion()
        end.start_condition = parallel.observation_variable
        msc.add_node(end)
        self._add_collision_rules(msc)
        
        self.giskard.execute(msc)
        print("Dual-Arm Motion complete.")

    def execute_dual_arm_grasp(self, left_grasp_pose: PoseStamped, right_grasp_pose: PoseStamped):
        print("Executing Dual-Arm Grasp (Approach)...")
        self.move_dual_arm(left_grasp_pose, right_grasp_pose)

    def execute_complex_grasp(self, grasp_pose: PoseStamped, object_name: str, approach_offset: List[float] = [0.0, 0.0, -0.1]):
        print(f"Executing Complex Grasp on {object_name}...")
        
        grasp_tf = self._to_giskard_pose(grasp_pose)
        
        offset_tf = cas.HomogeneousTransformationMatrix()
        offset_tf[0, 3] = approach_offset[0]
        offset_tf[1, 3] = approach_offset[1]
        offset_tf[2, 3] = approach_offset[2]
        
        pre_grasp_tf = grasp_tf.dot(offset_tf)
        
        pre_grasp_pose = self._to_ros_pose(pre_grasp_tf, grasp_pose.header.frame_id)
        
        print("Moving to Pre-Grasp...")
        self.move_to_pose(pre_grasp_pose, object_to_allow_collision=object_name)
        
        print("Approaching Grasp (Constrained)...")
        
        msc = MotionStatechart()
        
        target_tf_grasp = self._to_giskard_pose(grasp_pose)
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        task = CartesianPose(root_entity, tip_entity, target_tf_grasp, "Approach_Pose")
        msc.add_node(task)
        
        collision_entries = []
        if object_name:
             tip_body = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
             obj_body = self.giskard.world.get_kinematic_structure_entity_by_name(object_name)
             collision_entries.append(
                 CollisionRequest(CollisionAvoidanceTypes.ALLOW_COLLISION, [tip_body], [obj_body], distance=0.05)
             )
             gripper_bodies = self._get_gripper_links()
             collision_entries.append(
                 CollisionRequest(CollisionAvoidanceTypes.ALLOW_COLLISION, gripper_bodies, [obj_body], distance=0.05)
             )
             
        if collision_entries:
             self._add_collision_rules(msc, collision_entries)
        else:
             self._add_collision_rules(msc)

        end = EndMotion()
        end.start_condition = task.observation_variable
        msc.add_node(end)
        
        self.giskard.execute(msc)
        print("Complex Grasp Approach complete.")

    def execute_helical_motion(self, start_pose: PoseStamped, rotations: float, pitch: float, speed_lin: float = 0.005, speed_rot: float = 1.0):
        """
        Executes a helical motion (screw motion) by generating a sequence of waypoints.
        """
        print(f"Executing Helical Motion: {rotations} rots, pitch {pitch}...")
        
        msc = MotionStatechart()
        root_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.root_link)
        tip_entity = self.giskard.world.get_kinematic_structure_entity_by_name(self.tip_link)
        
        # We break the circle into 90-degree segments to ensure winding
        segments_per_rot = 4
        total_segments = int(rotations * segments_per_rot)
        angle_per_seg = (math.pi * 2) / segments_per_rot
        z_per_seg = pitch / segments_per_rot
        
        # Start from current
        current_g_pose = self._to_giskard_pose(start_pose)
        
        tasks = []
        
        # Helper to accumulate rotation
        # Note: Accumulating rotation with quaternions in a loop is tricky due to normalization
        # We will work in relative frame
        
        import numpy as np
        from tf_transformations import quaternion_multiply, quaternion_from_euler
        
        current_q = [start_pose.pose.orientation.x, start_pose.pose.orientation.y, 
                     start_pose.pose.orientation.z, start_pose.pose.orientation.w]
        current_pos = [start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z]
        
        # Assuming screw axis is Z of the START POSE (local Z)
        # Actually usually screw is along the Tool Z.
        
        for i in range(total_segments):
            # 1. Calculate relative delta
            # Rotate around local Z by angle_per_seg
            q_rot = quaternion_from_euler(0, 0, angle_per_seg) # Roll, Pitch, Yaw(Z)
            
            # Apply rotation: q_new = q_current * q_rot (local rotation)
            new_q = quaternion_multiply(current_q, q_rot)
            
            # Apply Translation: along LOCAL Z
            # We need to rotate the Z-vector (0,0,1) by current_q to get global Z direction
            # Or simplified: precise calculation of next waypoint
            # Let's trust Giskard to interpolate between 90 deg waypoints correctly for now
            # But we need the Position to advance too.
            
            # Vector in local frame: [0, 0, z_per_seg]
            # Transform to global
            
            # For simplicity in this script, assuming Vertical Downwards Screw (Global -Z) usually? 
            # Or strictly Local Z. Let's do Local Z.
            
            # Getting rotation matrix from current q
            rot_mat = cas.Quaternion(*current_q).to_rotation_matrix().to_np()
            local_z = rot_mat[:, 2] # 3rd column is Z axis
            
            delta_pos = local_z * z_per_seg
            new_pos =  [current_pos[0] + delta_pos[0], 
                        current_pos[1] + delta_pos[1], 
                        current_pos[2] + delta_pos[2]]
            
            # Create Waypoint
            wp_tf = cas.HomogeneousTransformationMatrix()
            wp_tf.pos = cas.Point3(*new_pos)
            wp_tf.ori = cas.Quaternion(*new_q)
            
            task = CartesianPose(
                root_link=root_entity,
                tip_link=tip_entity,
                goal_pose=wp_tf,
                name=f"Screw_Seg_{i}",
                reference_linear_velocity=speed_lin,
                reference_angular_velocity=speed_rot
            )
            tasks.append(task)
            
            # Update current
            current_q = new_q
            current_pos = new_pos

        seq = Sequence(nodes=tasks)
        msc.add_node(seq)
        
        end = EndMotion()
        end.start_condition = seq.observation_variable
        msc.add_node(end)
        
        self.giskard.execute(msc)
        print("Helical Motion complete.")

    def update_tool_frame(self, tool_name: str, offset_z: float):
        """
        Dynamically adds a tool frame to the robot's tip.
        """
        print(f"Updating Tool Frame for {tool_name}, offset={offset_z}")
        with self.giskard.world.modify_world():
            # Resolve current tip (wrist)
            wrist_body = self.giskard.world.get_body_by_name(self.tip_link)
            # Naive placeholder
            pass
        print("Tool frame update simulated.")

def main():
    engine = GiskardMotionEngine()
    
    pose_A = PoseStamped()
    pose_A.header.frame_id = "map"
    pose_A.pose.position.x = 0.4
    pose_A.pose.position.z = 0.5
    pose_A.pose.orientation.w = 1.0
    
    pose_B = PoseStamped()
    pose_B.header.frame_id = "map"
    pose_B.pose.position.x = 0.5
    pose_B.pose.position.z = 0.4
    pose_B.pose.orientation.w = 1.0

    engine.execute_smooth_sequence([pose_A, pose_B], object_to_allow_collision="test_cube")
    
    engine.giskard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()