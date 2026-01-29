import rclpy
from typing import Optional, List, Any, ContextManager
from motion_engine import GiskardMotionEngine
from config import DualArmConfig

class CollisionContext:
    """
    Context manager to ensure collision allowances are cleared after a block of code.
    """
    def __init__(self, engine: GiskardMotionEngine):
        self.engine = engine

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Always clear collisions, even if an exception occurred
        print("Context: Clearing collision allowances.")
        self.engine.clear_collision_allowance()

class BaseSkill:
    def __init__(self, engine: GiskardMotionEngine, gripper_callback=None):
        self.engine = engine
        self.config = DualArmConfig()
        self.gripper_callback = gripper_callback

    def open_gripper(self, arm: str):
        if self.gripper_callback:
            # Open is usually low effort or standard
            self.gripper_callback(arm, self.config.GRIPPER_OPEN, self.config.GRIPPER_EFFORT_DEFAULT)

    def close_gripper(self, arm: str, effort: Optional[float] = None):
        target_effort = effort if effort is not None else self.config.GRIPPER_EFFORT_DEFAULT
        if self.gripper_callback:
            self.gripper_callback(arm, self.config.GRIPPER_CLOSE, target_effort)

    def resolve_tip(self, arm: str) -> str:
        if arm.lower() == "left":
            return self.config.LEFT_TIP
        elif arm.lower() == "right":
            return self.config.RIGHT_TIP
        else:
            raise ValueError(f"Unknown arm: {arm}")
