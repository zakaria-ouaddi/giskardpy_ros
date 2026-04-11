# PyCRAM ↔ Giskard Pipeline — Research Summary

## Overview

The repo (`cognitive_robot_abstract_machine`) is a monorepo. The key folders for the pipeline are:

| Folder | Role |
|---|---|
| `pycram/src/pycram/` | Core PyCRAM library |
| `pycram/src/pycram/robot_plans/` | High-level actions & low-level motions |
| `pycram/src/pycram/motion_executor.py` | **Bridge** between PyCRAM and Giskard |
| `giskardpy/` | Motion planning engine |
| `semantic_digital_twin/` | World model, robot view, spatial types |
| `test/pycram_test/` | All PyCRAM integration tests |
| `test/conftest.py` | Shared pytest fixtures (world, robots) |

---

## How the Pipeline Works

```mermaid
flowchart TD
    A["ActionDescription\n(e.g. PickUpAction)"] -->|calls| B["SequentialPlan\n(language.py)"]
    B -->|chains| C["BaseMotion subclasses\n(MoveTCPMotion, MoveGripperMotion, MoveMotion...)"]
    C -->|builds| D["giskardpy Task\n(CartesianPose, JointPositionList, Sequence...)"]
    D -->|passed to| E["MotionExecutor\n(motion_executor.py)"]
    E -->|builds| F["MotionStatechart\n(giskardpy)"]
    F -->|sim| G["Ros2Executor.tick_until_end()"]
    F -->|real| H["GiskardWrapper.execute()"]
```

### 1. Motions → Giskard Tasks

Every motion in `robot_plans/motions/` is a `BaseMotion` dataclass that exposes a `_motion_chart` property returning a giskardpy `Task`:

| Motion class | Giskard Task produced |
|---|---|
| `MoveMotion` | `CartesianPose` (base navigation) |
| `MoveTCPMotion` | `CartesianPose` or `CartesianPosition` |
| `MoveTCPWaypointsMotion` | `Sequence` of `CartesianPose` |
| `MoveGripperMotion` | `JointPositionList` |
| `ReachMotion` | `Sequence` of `CartesianPose` |

### 2. MotionExecutor — the Bridge

`MotionExecutor` (in `motion_executor.py`) takes a list of `Task` objects, wraps them in a `Sequence` → `EndMotion` `MotionStatechart`, then executes:

- **`ExecutionType.SIMULATED`** → `Ros2Executor(world, QPControllerConfig(...)).tick_until_end(timeout=2000)`
- **`ExecutionType.REAL`** → `GiskardWrapper(ros_node).execute(msc)`
- **`ExecutionType.NO_EXECUTION`** → no-op (for plan graph tests)

The execution type is set via context managers:
```python
from pycram.motion_executor import simulated_robot, real_robot, no_execution

with simulated_robot:
    SequentialPlan(context, NavigateActionDescription(target_pose)).perform()
```

### 3. High-Level Actions

Actions in `robot_plans/actions/core/`:

| Action | What it does |
|---|---|
| `NavigateAction` | `MoveMotion` to target pose |
| `PickUpAction` | Open gripper → `ReachAction` → Close gripper → Lift |
| `PlaceAction` | Move TCP + open gripper + retract |
| `ReachAction` | `MoveTCPMotion` pre-pose → grasp pose |
| `GraspingAction` | Pre-pose → open → grasp → close |
| `MoveBodyAction` (robot_body.py) | Move torso / park arms |
| `TransportingAction` (composite/) | Navigate + PickUp + Navigate + Place |

---

## Test Infrastructure

### `test/conftest.py` — Global Fixtures

| Fixture | What it provides |
|---|---|
| `pr2_world_setup` | PR2 URDF world (session-scoped) |
| `apartment_world_setup` | Apartment URDF + milk + cereal (session) |
| `pr2_apartment_world` | PR2 merged into apartment |
| `simple_pr2_world_setup` | PR2 + simple box apartment |
| `hsr_apartment_world` | HSRB world |
| `tracy_world` | Tracy robot world |
| `rclpy_node` | ROS2 node (function-scoped) |

### `test/pycram_test/conftest.py` — PyCRAM Fixtures

| Fixture | What it provides |
|---|---|
| `immutable_model_world` | PR2 + apartment, state reset after each test |
| `mutable_model_world` | Deep copy, freely modifiable |
| `immutable_simple_pr2_world` | PR2 + simple apartment, state reset |

### `test/pycram_test/` — Key Test Files

| Test file | What it tests |
|---|---|
| `test_plan.py` | Plan graph structure + full robot action execution |
| `test_goal_validator.py` | Pose validators, tolerance checking |
| `test_costmaps.py` | Costmaps for location designators |
| `test_designator/` | Action, motion designators |
| `test_language.py` | SequentialPlan, ParallelPlan |

---

## How to Write a PyCRAM-Giskard Pipeline Test

### Minimal Example (no ROS, simulated)

```python
import pytest
from pycram.datastructures.dataclasses import Context
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import PickUpAction, NavigateActionDescription
from pycram.language import SequentialPlan
from pycram.datastructures.enums import Arms
from pycram.datastructures.grasp import GraspDescription

def test_pick_up_pipeline(immutable_simple_pr2_world):
    world, robot_view, context = immutable_simple_pr2_world

    # Get the milk object from the world
    milk = world.get_body_by_name("milk.stl")
    
    with simulated_robot:
        SequentialPlan(
            context,
            # Navigate near the object
            NavigateActionDescription(target_location=...),
            # Pick it up
            PickUpAction.description(
                object_designator=milk,
                arm=Arms.LEFT,
                grasp_description=GraspDescription.top(),
            ),
        ).perform()
```

### Running Existing Tests

```bash
# From the workspace root
cd /home/zakaria/workspace/ros/src/cognitive_robot_abstract_machine

# Run all pycram tests (no ROS required for pure simulation tests)
poetry run pytest test/pycram_test/ -v

# Run only plan tests
poetry run pytest test/pycram_test/test_plan.py -v

# Run tests that need ROS (set ROS_VERSION=2 first)
ROS_VERSION=2 poetry run pytest test/pycram_test/test_plan.py -v -k "simulated"
```

---

## Key Imports for Writing Pipeline Tests

```python
# World setup
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.world import World
from pycram.datastructures.dataclasses import Context

# Execution contexts
from pycram.motion_executor import simulated_robot, real_robot, no_execution

# Plans
from pycram.language import SequentialPlan, ParallelPlan
from pycram.robot_plans import (
    NavigateAction, PickUpAction, PlaceAction,
    MoveGripperMotion, MoveTCPMotion, MoveMotion,
)

# Giskard Tasks (used internally, but useful to know)
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose, CartesianPosition
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.ros_executor import Ros2Executor
```

---

## Resources Available

```
pycram/resources/
  robots/    → pr2.urdf, hsrb.urdf, stretch_description.urdf, ...
  worlds/    → apartment.urdf, ...
  objects/   → milk.stl, breakfast_cereal.stl, ...
```

These are used directly in the test fixtures.
