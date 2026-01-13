# Giskard: Zero to Hero Tutorial

Welcome to the comprehensive guide on using **Giskard**, a powerful constraint-based motion planning and control framework for ROS. This tutorial will take you from the basic concepts to advanced usage, enabling you to control robots with precision and flexibility.

## Table of Contents
1. [Introduction](#introduction)
2. [Key Concepts](#key-concepts)
3. [Setup & Boilerplate](#setup--boilerplate)
4. [Level 1: Joint Motions](#level-1-joint-motions)
5. [Level 2: Cartesian Motions](#level-2-cartesian-motions)
6. [Level 3: Advanced Tasks (Pointing, Aligning)](#level-3-advanced-tasks-pointing-aligning)
7. [Level 4: Combining Motions (Sequence & Parallel)](#level-4-combining-motions-sequence--parallel)
8. [Level 5: Constraints & Collision Avoidance](#level-5-constraints--collision-avoidance)
9. [Level 6: Interactive Control](#level-6-interactive-control)

---

## Introduction

Giskard is not just a path planner; it's a **constraint-based controller**. Instead of simply asking for a path from A to B, you define a set of **constraints** and **objectives** that the robot must satisfy.

*   **Objectives (Tasks)**: "Move hand to position X", "Look at object Y".
*   **Constraints**: "Don't hit the table", "Keep the cup upright", "Don't exceed joint limits".

Giskard solves these constraints in real-time to generate robot motions.

## Key Concepts

*   **`GiskardWrapper`**: The main Python interface to communicate with the Giskard ROS node.
*   **`MotionStatechart`**: The container for your motion plan. It's a state machine where each state is a motion phase.
*   **`Task`**: A node that adds **constraints** to the optimization problem (e.g., "Joints must be at X").
*   **`Monitor`**: A node that **observes** the world but adds no constraints (e.g., "Is the door open?").
*   **`Goal`**: A container node that manages other nodes (e.g., `Sequence`, `Parallel`).
*   **`World`**: The digital twin of the environment, containing the robot and objects.

---

## Setup & Boilerplate

To use Giskard, you need a running Giskard ROS node (the server) and your client script.

### 1. Imports
Standard imports you'll need:

```python
import rclpy
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.graph_node import EndMotion
```

### 2. Initialization
Initialize the ROS node and the Giskard wrapper:

```python
def main():
    rospy.init_node("my_giskard_client")
    
    # Connect to the Giskard node
    giskard = GiskardWrapper(node_handle=rospy.node, giskard_node_name="giskard")
    
    # Your code here...
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## Level 1: Joint Motions

The simplest motion is moving joints to specific positions.

**Key Class**: `JointPositionList`

```python
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState

# Define the goal joint positions
goal_joints = {
    "shoulder_pan_joint": 1.5,
    "elbow_flex_joint": -0.5
}

# Create the motion plan
msc = MotionStatechart()

# Create the task
joint_task = JointPositionList(
    goal_state=JointState.from_str_dict(goal_joints, giskard.world)
)

# Add task to the plan
msc.add_node(joint_task)

# Define when to stop: When the task is satisfied (joints reached)
end = EndMotion()
end.start_condition = joint_task.observation_variable
msc.add_node(end)

# Execute
giskard.execute(msc)
```

---

## Level 2: Cartesian Motions

Moving the end-effector (hand/gripper) to a specific 3D pose in space.

**Key Class**: `CartesianPose`

```python
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.spatial_types import TransformationMatrix

# Get robot links
root_link = giskard.world.get_kinematic_structure_entity_by_name("map")
tip_link = giskard.world.get_kinematic_structure_entity_by_name("gripper_tool_frame")

# Define the goal pose (x=0.5, y=0.0, z=0.8)
goal_pose = TransformationMatrix.from_xyz_rpy(
    x=0.5, y=0.0, z=0.8, 
    roll=0, pitch=0, yaw=0,
    reference_frame=root_link
)

# Create the task
cart_task = CartesianPose(
    root_link=root_link,
    tip_link=tip_link,
    goal_pose=goal_pose
)

# Build statechart
msc = MotionStatechart()
msc.add_node(cart_task)
msc.add_node(EndMotion.when_true(cart_task)) # Helper for simple end condition

# Execute
giskard.execute(msc)
```

---

## Level 3: Advanced Tasks (Pointing, Aligning)

Giskard allows more abstract goals than just "go to pose".

### Pointing
"Point the camera at the apple."

**Key Class**: `Pointing`

```python
from giskardpy.motion_statechart.tasks.pointing import Pointing
from semantic_digital_twin.spatial_types import Point3, Vector3

# Define target point
target_point = Point3(x=1.0, y=0.5, z=0.5, reference_frame=root_link)

# Define which axis of the tip points (e.g., Z-axis for cameras)
pointing_axis = Vector3.Z(reference_frame=tip_link)

pointing_task = Pointing(
    root_link=root_link,
    tip_link=tip_link,
    goal_point=target_point,
    pointing_axis=pointing_axis
)
```

### Align Planes
"Keep the tray flat" or "Align gripper with table surface."

**Key Class**: `AlignPlanes`

```python
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes

# Align tip's Z-axis with World's Z-axis (keep upright)
align_task = AlignPlanes(
    root_link=root_link,
    tip_link=tip_link,
    tip_normal=Vector3.Z(reference_frame=tip_link),
    goal_normal=Vector3.Z(reference_frame=root_link)
)
```

---

## Level 4: Combining Motions (Sequence & Parallel)

You can combine tasks to create complex behaviors.

### Parallel
"Look at the object WHILE moving towards it."

```python
from giskardpy.motion_statechart.goals.templates import Parallel

parallel_motion = Parallel([
    pointing_task,
    cart_task
])

msc.add_node(parallel_motion)
msc.add_node(EndMotion.when_true(parallel_motion))
```

### Sequence
"Go to A, THEN go to B."

```python
from giskardpy.motion_statechart.goals.templates import Sequence

sequence_motion = Sequence([
    task_a,
    task_b
])

msc.add_node(sequence_motion)
msc.add_node(EndMotion.when_true(sequence_motion))
```

---

## Level 5: Constraints & Collision Avoidance

Constraints are tasks that *must* be satisfied. Collision avoidance is a common constraint.

**Key Class**: `CollisionAvoidance`

```python
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.model.collision_matrix_manager import CollisionRequest

# Define collision avoidance
# Avoid collisions between everything
collision_task = CollisionAvoidance(
    collision_entries=[CollisionRequest.avoid_all_collision()]
)

# Add to your motion
# Usually added in Parallel with your main task
final_motion = Parallel([
    cart_task,
    collision_task
])

msc.add_node(final_motion)
```

**Note**: Giskard often adds default collision avoidance if configured, but adding it explicitly gives you control (e.g., allowing specific collisions for grasping).

---

## Level 6: Interactive Control

For dynamic applications, you might want to update the goal in real-time (e.g., following a moving target or interactive marker).

Instead of `giskard.execute()`, use `giskard.execute_async()` and update the goal.

See `scripts/tools/interactive_marker.py` in the codebase for a full example of this pattern.


---

## Level 7: Custom Monitors

Sometimes the built-in monitors aren't enough. You can create your own by inheriting from `Monitor` or `PayloadMonitor`.

### Example: A Random Monitor
This monitor randomly decides to be True or False.

```python
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from giskardpy.motion_statechart.data_types import ObservationStateValues
import random

class RandomMonitor(MotionStatechartNode):
    def __init__(self, probability=0.5, name=None):
        super().__init__(name=name)
        self.probability = probability

    def on_tick(self, context=None):
        # This function is called every control cycle
        # Return True if the condition is met
        if random.random() < self.probability:
             return ObservationStateValues.TRUE
        return ObservationStateValues.FALSE

# Usage
my_monitor = RandomMonitor(probability=0.1)
msc.add_node(my_monitor)
```

### Example: Expression Monitor
For high-performance checks involving robot state, use symbolic expressions (CasADi).

```python
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
import giskardpy.spatial_types as cas

class JointLimitMonitor(MotionStatechartNode):
    def __init__(self, joint_name, limit, name=None):
        super().__init__(name=name)
        # Get the symbol for the joint position
        joint_pos = giskard.world.get_joint(joint_name).position
        
        # Define the expression (True if pos > limit)
        self.observation_expression = cas.greater(joint_pos, limit)

# Usage
limit_monitor = JointLimitMonitor("elbow_joint", 1.5)
msc.add_node(limit_monitor)
```

---

## Level 8: Custom Goals

You can also create custom goals by combining existing tasks or defining new constraints.

### Example: Custom Pose Goal
A goal that combines a position goal and an orientation goal.

```python
from giskardpy.motion_statechart.graph_node import Goal
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPosition, CartesianOrientation

class MyCustomPose(Goal):
    def __init__(self, root_link, tip_link, target_pose, name=None):
        super().__init__(name=name)
        
        # 1. Create sub-goals
        pos_goal = CartesianPosition(root_link, tip_link, target_pose.position)
        ori_goal = CartesianOrientation(root_link, tip_link, target_pose.orientation)
        
        # 2. Add them to this goal
        self.add_node(pos_goal)
        self.add_node(ori_goal)
        
        # 3. Define when this goal is "done"
        # It's done when both sub-goals are done
        self.end_condition = cas.logic_and(
            pos_goal.end_condition, 
            ori_goal.end_condition
        )

# Usage
custom_goal = MyCustomPose(root, tip, pose)
msc.add_node(custom_goal)
```

---

## Important Note on Wiki vs. Codebase

If you check the [GiskardPy Wiki](https://github.com/SemRoCo/giskardpy/wiki), you might see examples using `giskard.monitors.add_...` or `giskard.motion_goals.add_...`. 

**That is an alternative/older API.** 

In this codebase (`giskardpy_ros`), we use the **`MotionStatechart`** approach described in this tutorial. It offers more explicit control over the motion graph and is the recommended pattern for this project.

### Summary Checklist

1.  **Define World Objects**: Get `root` and `tip` links.
2.  **Define Goal**: Create a `TransformationMatrix` or `JointState`.
3.  **Create Task**: Instantiate `CartesianPose`, `JointPositionList`, etc.
4.  **Create Statechart**: Add the task to a `MotionStatechart`.
5.  **Define End Condition**: When should it stop? (`EndMotion`).
6.  **Execute**: Run it!

Happy Coding with Giskard!
