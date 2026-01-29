from .base import BaseSkill

class ToolUseSkill(BaseSkill):
    """
    Skill for changing the active TCP (Tool Center Point) by attaching a virtual tool.
    """
    def execute(self, tool_name: str, offset_z: float = None):
        offset = offset_z if offset_z else self.config.TOOL_OFFSET_Z
        print(f"Equipping Tool: {tool_name} (Offset: {offset}m)...")
        
        # Call engine to update frame
        self.engine.update_tool_frame(tool_name, offset)
        
        # In a real system, we might also switch the active tip link variable in the engine
        # self.engine.tip_link = f"{tool_name}_tip"
        
        print(f"Tool {tool_name} equipped. Active TCP updated.")
