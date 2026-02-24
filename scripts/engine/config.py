from dataclasses import dataclass

@dataclass
class DualArmConfig:
    # Frame Names
    LEFT_TIP: str = "l_gripper_tool_frame"
    RIGHT_TIP: str = "r_gripper_tool_frame"
    ROOT_FRAME: str = "map2"

    # Motion Parameters
    LINEAR_SPEED: float = 0.05
    ANGULAR_SPEED: float = 0.1
    JOINT_SPEED: float = 0.8
    
    # Gripper Limits
    GRIPPER_OPEN: float = 0.0
    GRIPPER_CLOSE: float = 0.35
    GRIPPER_EFFORT_DEFAULT: float = 10.0
    GRIPPER_EFFORT_SOFT: float = 2.0

    # Scenario Constants
    CUBE_HEIGHT: float = 0.05
    BASE_Z: float = 0.92
    
    # Starting Positions
    RIGHT_ARM_START_X: float = 0.9
    RIGHT_ARM_START_Y: float = -0.2
    
    # Place Positions
    PLACE_X: float = 0.65
    PLACE_Y: float = 0.4
    PLACE_Z_BASE: float = 0.94

    # Handover
    HANDOVER_MEETING_POINT = (0.75, 0, 1.3)
    HANDOVER_APPROACH_OFFSET: float = 0.30
    HANDOVER_RETREAT_OFFSET: float = 0.15
    HANDOVER_OFFSET: float = 0.02 # Margin from center for each arm

    # Refined Heights
    PICK_APPROACH_HEIGHT: float = 0.20
    PLACE_RETREAT_HEIGHT: float = 0.20

    # Screw / Assembly Config
    SCREW_TORQUE_LIMIT: float = 5.0 # Nm, needs tuning based on robot
    SCREW_PITCH: float = 0.001 # 1mm per rotation
    SCREW_ADVANCE_SPEED: float = 0.005 # m/s
    SCREW_ROTATION_SPEED: float = 1.0 # rad/s

    # Push / Snap Config
    PUSH_FORCE_LIMIT: float = 20.0 # N
    PUSH_DISTANCE: float = 0.05
    PUSH_SPEED: float = 0.02

    # Tool Config
    # Default offset from gripper tip to tool tip (e.g. screwdriver length)
    TOOL_OFFSET_Z: float = 0.15 
