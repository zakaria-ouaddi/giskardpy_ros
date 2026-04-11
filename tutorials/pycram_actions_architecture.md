# PyCRAM Action & Motion Architecture

Reference guide for understanding existing skills and adding new ones.

---

## 1. Three-Layer Design

```
SequentialPlan / ParallelPlan
        │
        ▼
  ActionDescription          ← "what" (high-level intent, pre/post conditions)
        │  calls
        ▼
  BaseMotion / XxxMotion     ← "how" (Giskard task graph node)
        │  wraps
        ▼
  Giskard Task               ← QP solver primitive (JointPositionList, CartesianPose, …)
```

The [MotionExecutor](cognitive_robot_abstract_machine/pycram/src/pycram/motion_executor.py#22-104) picks up the Giskard tasks and runs them through the QP controller (simulated or real robot).

---

## 2. Class Hierarchy

```
DesignatorDescription   (pycram/designator.py)
        │
        └── ActionDescription   (robot_plans/actions/base.py)
                │
                └── ParkArmsAction, PickUpAction, PlaceAction, …   (core/*.py)

BaseMotion              (robot_plans/motions/base.py)
        │
        └── MoveJointsMotion, MoveTCPMotion, MoveGripperMotion, …   (motions/*.py)
```

### How [world](cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#107-116), [robot_view](cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#97-106), [context](cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#117-120) are injected

[DesignatorDescription](cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#79-166) **does not** store world/robot. They come from the `Plan` at execution time:

```python
@property
def world(self) -> World:
    return self.plan_node.plan.world     # set by SequentialPlan when perform() runs

@property
def robot_view(self) -> AbstractRobot:
    return self.plan.robot

@property
def context(self) -> Context:
    return Context(self.world, self.robot_view, self.plan)
```

→ You **never** pass [world](cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#107-116) or [robot](/cognitive_robot_abstract_machine/pycram/src/pycram/designator.py#97-106) to an [ActionDescription](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/base.py#15-78). They're resolved at runtime.

---

## 3. [ActionDescription](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/base.py#15-78) Contract

Every action must implement **three** abstract methods:

```python
@dataclass
class MyAction(ActionDescription):

    # --- fields (dataclass) ---
    my_param: SomeType

    # --- REQUIRED ---
    def execute(self) -> None:
        """The actual plan. Call motions or sub-actions via SequentialPlan."""
        SequentialPlan(self.context, SomeMotion(...)).perform()

    def validate_precondition(self):
        """Check world state BEFORE execute(). Raise a Failure if not met."""
        pass   # or raise SomeFailure(...)

    def validate_postcondition(self, result=None):
        """Check world state AFTER execute(). Raise a Failure if not achieved."""
        pass   # or raise SomeFailure(...)

    # --- REQUIRED for use in SequentialPlan ---
    @classmethod
    def description(cls, my_param: ...) -> PartialDesignator["MyAction"]:
        return PartialDesignator[MyAction](MyAction, my_param=my_param)


# Module-level alias (convention used everywhere):
MyActionDescription = MyAction.description
```

> **Note:** [base.py](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/base.py) routes [validate_precondition](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/base.py#55-61) and [validate_postcondition](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/base.py#62-68) through
> [perform()](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/gripper.py#150-152) automatically. Some older actions use a single [validate()](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/robot_body.py#155-174) method —
> that signature still works if you override `validate_precondition/postcondition` to call it.

---

## 4. [BaseMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/base.py#52-87) Contract

Motions expose a single [_motion_chart](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/navigation.py#29-36) property that returns a **Giskard Task**:

```python
@dataclass
class MyMotion(BaseMotion):
    target_pose: PoseStamped
    arm: Arms

    def perform(self):
        return   # intentionally empty — MotionExecutor handles execution

    @property
    def _motion_chart(self):
        # Return a giskardpy Task object
        return CartesianPose(
            root_link=...,
            tip_link=...,
            goal_pose=self.target_pose.to_spatial_type(),
        )
```

Available Giskard primitives (from `giskardpy.motion_statechart.tasks`):
| Primitive | Use |
|-----------|-----|
| `JointPositionList` | Set a list of joints to target angles |
| `JointState.from_mapping` | Build a joint-state dict for JointPositionList |
| `CartesianPose` | Move TCP to a 6D pose |
| `Pointing` | Orient a link axis toward a 3D point |
| `Sequence` | Chain multiple tasks sequentially |

---

## 5. Catalogue of Existing Actions

### Core actions (`robot_plans/actions/core/`)

| Class | Alias | Key parameters | What it does |
|-------|-------|---------------|--------------|
| [ParkArmsAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/robot_body.py#124-180) | `ParkArmsActionDescription` | `arm: Arms` | Moves arms to `StaticJointState.PARK` |
| [SetGripperAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/robot_body.py#81-122) | `SetGripperActionDescription` | `gripper: Arms`, `motion: GripperState` | Opens / closes gripper |
| [MoveTorsoAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/robot_body.py#27-79) | `MoveTorsoActionDescription` | `torso_state: TorsoState` | Moves torso to a named state |
| [CarryAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/robot_body.py#182-304) | `CarryActionDescription` | [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `align`, `tip_link`, `tip_axis` | Parks arms + optional axis alignment |
| [ReachAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/pick_up.py#33-122) | `ReachActionDescription` | `target_pose`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `grasp_description` | Moves TCP through pre-pose → pose |
| [PickUpAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/pick_up.py#124-212) | `PickUpActionDescription` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `grasp_description` | Open gripper → Reach → Close → Lift |
| [GraspingAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/pick_up.py#214-275) | `GraspingActionDescription` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `grasp_description` | Pre-pose → Open → Grasp → Close |
| [PlaceAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/placing.py#31-153) | `PlaceActionDescription` | `object_designator`, `target_location`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113) | Reach (reverse) → Open → Retract |
| [NavigateAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/navigation.py#22-65) | `NavigateActionDescription` | `target_location: PoseStamped` | Move robot base |
| [LookAtAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/navigation.py#67-107) | `LookAtActionDescription` | `target: PoseStamped`, `camera?` | Aim camera at a point |
| [DetectAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/misc.py#21-103) | `DetectActionDescription` | `technique`, `object_sem_annotation?`, `region?` | Run perception query |
| [OpenAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/container.py#28-92) | `OpenActionDescription` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113) | Grasp handle and open |
| [CloseAction](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/container.py#94-158) | `CloseActionDescription` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113) | Grasp handle and close |

### Composite actions (`robot_plans/actions/composite/`)
- `TransportAction` — navigate + pick + transport + place
- `SearchAction` — navigate + detect
- `FacingAction` — turn robot to face a target
- `ToolBasedAction` — actions using a tool attached to the robot

---

## 6. Catalogue of Existing Motions

### `robot_plans/motions/`

| Class | Key parameters | Giskard primitive |
|-------|---------------|-------------------|
| [MoveJointsMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/robot_body.py#14-58) | `names: list`, `positions: list` | `JointPositionList` |
| [LookingMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/robot_body.py#60-88) | `target: PoseStamped`, `camera: Camera` | `Pointing` |
| [MoveMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/navigation.py#10-36) | `target_location: PoseStamped` | `CartesianPose` |
| [MoveGripperMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/gripper.py#93-125) | `gripper: Arms`, `motion: GripperState` | `JointPositionList` |
| [MoveTCPMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/gripper.py#127-177) | `target_pose`, [arm](/cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `movement_type` | `CartesianPose` |
| [ReachMotion](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/motions/gripper.py#24-91) | `target_pose`, [arm](/cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113), `pre_pose_distance` | Sequence of `CartesianPose` |
| `OpeningMotion` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113) | Joint trajectory |
| `ClosingMotion` | `object_designator`, [arm](cognitive_robot_abstract_machine/test/pycram_test/test_designator/test_tracy_action_designator.py#90-113) | Joint trajectory |

---

## 7. Recipe: Adding a New Skill

### Step 1 — Create the Motion file (if new Giskard primitive needed)

```python
# pycram/src/pycram/robot_plans/motions/my_motion.py
from dataclasses import dataclass
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from .base import BaseMotion
from ...datastructures.pose import PoseStamped
from ...datastructures.enums import Arms

@dataclass
class MyNewMotion(BaseMotion):
    target_pose: PoseStamped
    arm: Arms

    def perform(self):
        return   # empty — MotionExecutor handles it

    @property
    def _motion_chart(self):
        # return a Giskard Task
        from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
        manipulator = self.robot_view.get_arm(self.arm).manipulator
        return CartesianPose(
            root_link=self.robot_view.root,
            tip_link=manipulator.tip,
            goal_pose=self.target_pose.to_spatial_type(),
        )
```

### Step 2 — Create the Action file

```python
# pycram/src/pycram/robot_plans/actions/core/my_action.py
from __future__ import annotations
from dataclasses import dataclass
from typing_extensions import Union, Optional, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ...motions.my_motion import MyNewMotion      # or any existing motion


@dataclass
class MyNewAction(ActionDescription):
    """One-line description of what the action does."""

    target_pose: PoseStamped
    """Target 6D pose for the action."""

    arm: Arms
    """Which arm to use."""

    def execute(self) -> None:
        SequentialPlan(
            self.context,
            MyNewMotion(self.target_pose, self.arm),
        ).perform()

    def validate_precondition(self):
        pass   # add world-state checks if needed

    def validate_postcondition(self, result: Optional[Any] = None):
        pass   # add goal-achievement checks if needed

    @classmethod
    def description(
        cls,
        target_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms] = None,
    ) -> PartialDesignator["MyNewAction"]:
        return PartialDesignator[MyNewAction](MyNewAction, target_pose=target_pose, arm=arm)


MyNewActionDescription = MyNewAction.description
```

### Step 3 — Export from [__init__.py](cognitive_robot_abstract_machine/test/__init__.py)

```python
# pycram/src/pycram/robot_plans/actions/core/__init__.py
from .my_action import *          # add this line
```

Also add it to the top-level [robot_plans/__init__.py](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/__init__.py) if it exists.

### Step 4 — Use it in a demo or test

```python
from pycram.robot_plans import MyNewActionDescription
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot

with simulated_robot:
    SequentialPlan(
        context,
        MyNewActionDescription(target_pose=some_pose, arm=Arms.LEFT),
    ).perform()
```

---

## 8. Key Conventions

| Convention | Rule |
|-----------|------|
| **Class name** | `XxxAction(ActionDescription)` |
| **Alias** | `XxxActionDescription = XxxAction.description` at module level |
| **[description()](cognitive_robot_abstract_machine/pycram/src/pycram/robot_plans/actions/core/pick_up.py#260-275) return** | Always `PartialDesignator[XxxAction](XxxAction, ...)` |
| **World access** | Use `self.world`, `self.robot_view`, `self.context` — never constructor args |
| **Sub-actions** | Compose with `SequentialPlan(self.context, SubActionDescription(...)).perform()` |
| **Giskard access** | Via `BaseMotion._motion_chart` property — return a Task, not execute it |
| **File location** | Atomic: `core/`, Multi-step: `composite/` |
| **Imports** | Use relative imports (`....language`, `...motions.gripper`, etc.) |
