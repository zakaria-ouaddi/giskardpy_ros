# Motion Statechart Manager: A Deep Dive

This tutorial provides an in-depth look at the **Motion Statechart**, the brain behind Giskard's reactive control. We will explore its internal structure, lifecycle, and how to use it effectively.

## 1. Core Concepts

The `MotionStatechart` is a **directed graph** where:
*   **Nodes** (`MotionStatechartNode`) represent states of behavior.
*   **Edges** represent transitions triggered by **Conditions**.

### 1.1. Node Lifecycle
Every node in the chart has a **Life Cycle State**:
1.  **NOT_STARTED**: The node is inactive.
2.  **RUNNING**: The node is active. Its constraints (if any) are added to the QP solver.
3.  **PAUSED**: The node is temporarily inactive but retains its state.
4.  **DONE**: The node has finished execution.

### 1.2. Node Observation
Every node also has an **Observation State** (Trinary Logic):
*   **True**: The node's objective is satisfied (e.g., target reached).
*   **False**: The objective is not satisfied.
*   **Unknown**: The state cannot be determined yet.

### 1.3. Conditions (The Wiring)
Transitions between lifecycle states are controlled by four conditions (CasADi expressions):
*   **`start_condition`**: `NOT_STARTED` -> `RUNNING`
*   **`pause_condition`**: `RUNNING` <-> `PAUSED`
*   **`end_condition`**: `RUNNING`/`PAUSED` -> `DONE`
*   **`reset_condition`**: Any -> `NOT_STARTED`

**Priority**: Reset > End > Pause > Start.

## 2. Anatomy of a Node

All nodes inherit from `MotionStatechartNode`.

### 2.1. Task (`Task`)
A **Task** adds **constraints** to the robot's motion.
*   *Example*: `JointPositionList`, `CartesianPose`.
*   *Behavior*: When `RUNNING`, it tells the solver "I want to be at X".

### 2.2. Monitor (`Monitor`)
A **Monitor** only observes the world. It does **not** add constraints.
*   *Example*: `ThreadPayloadMonitor` (checks external sensors).
*   *Behavior*: Used to trigger conditions for other nodes.

### 2.3. Goal (`Goal`)
A **Goal** is a container for other nodes. It helps organize complex behaviors.
*   *Example*: `Sequence`, `Parallel`.
*   *Behavior*: It manages the lifecycle of its children. For example, a `Sequence` goal ensures child B only starts after child A is done.

## 3. How It Works (The Loop)

When you call `giskard.execute(msc)`, the following happens in a loop:

1.  **Tick**:
    *   **Update Observations**: Every `RUNNING` node checks the world (e.g., "Am I at the target?").
    *   **Update Lifecycle**: Based on the new observations, conditions are evaluated. Nodes might start, stop, or pause.
2.  **Collect Constraints**:
    *   Giskard collects constraints from all **RUNNING** Tasks.
3.  **Solve**:
    *   A Quadratic Program (QP) solver finds the optimal joint velocities.
4.  **Command**:
    *   Velocities are sent to the robot.

## 4. Line-by-Line Usage Guide

Let's build a complex behavior: "Move to a pose, but stop if a laser sensor triggers."

### Step 1: Setup
```python
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.graph_node import EndMotion, MotionStatechartNode
import giskardpy.spatial_types as cas

msc = MotionStatechart()
```

### Step 2: Define the Task
```python
# Create a task to move the robot
move_task = CartesianPose(
    root_link=..., 
    tip_link=..., 
    goal_pose=...
)
msc.add_node(move_task)
```

### Step 3: Define a Safety Monitor
Let's create a monitor that watches a sensor value.
```python
# Assume 'sensor_value' is available in the world model
sensor_val = giskard.world.get_variable("laser_distance")

class SafetyMonitor(MotionStatechartNode):
    def __init__(self, threshold, name=None):
        super().__init__(name=name)
        # True if distance < threshold (DANGER!)
        self.observation_expression = cas.less(sensor_val, threshold)

safety_check = SafetyMonitor(threshold=0.5)
msc.add_node(safety_check)
```

### Step 4: Wire the Logic (Pause Condition)
We want the robot to **PAUSE** if the safety check is True.

```python
# The task pauses when safety_check observes "True"
move_task.pause_condition = safety_check.observation_variable
```

### Step 5: Define Completion
The entire behavior ends when the move task is done.

```python
end_node = EndMotion()
end_node.start_condition = move_task.observation_variable
msc.add_node(end_node)
```

### Step 6: Execute
```python
giskard.execute(msc)
```

## 5. Using Templates

Giskard provides `Sequence` and `Parallel` goals to simplify wiring.

### Sequence
Automatically wires `start_condition` of node `i` to `observation_variable` of node `i-1`.
```python
from giskardpy.motion_statechart.goals.templates import Sequence

seq = Sequence([task_A, task_B])
# task_B.start_condition = task_A.observation_variable (Automatic!)
```

### Parallel
Runs all nodes together. The `Parallel` goal itself is "Done" when **ALL** children are "Done".
```python
from giskardpy.motion_statechart.goals.templates import Parallel

par = Parallel([task_A, task_B])
# par.observation_variable becomes True only when BOTH A and B are True.
```

## 6. Summary

*   **MotionStatechart**: The engine driving the robot.
*   **Nodes**: The logic units (Tasks, Monitors, Goals).
*   **Conditions**: The logic gates (`start`, `pause`, `end`, `reset`).
*   **Observation**: The state of the world (True/False).

By mastering these concepts, you can create highly reactive and complex robot behaviors that go far beyond simple path planning.
