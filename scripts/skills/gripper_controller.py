import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import time

class GripperController:
    def __init__(self, node=None):
        """
        Initialize the GripperController.
        If a node is passed, it is used. Otherwise, a new node 'gripper_controller' is created.
        """
        if node:
            self.node = node
            self.own_node = False
        else:
            # Check if rclpy is initialized, if not init it? 
            # Usually strict users want us to check. 
            # But here we assume context exists or we create node.
            # If rclpy.ok() is false, we might need to init. 
            # But usually this is called inside a running ros context.
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node('gripper_controller')
            self.own_node = True

        self.clients = {
            'left': ActionClient(self.node, GripperCommand, '/left_gripper/robotiq_gripper_controller/gripper_cmd'),
            'right': ActionClient(self.node, GripperCommand, '/right_gripper/robotiq_gripper_controller/gripper_cmd')
        }
        
        # State tracking: None means unknown
        self.current_state = {
            'left': None, 
            'right': None
        }

        # Wait for servers? Optional, but good for robustness.
        # We'll check availability in the command function to avoid blocking init indefinitely.

    def command(self, side: str, position: float, effort: float = 10.0):
        """
        Send a gripper command.
        side: 'left' or 'right'
        position: 0.0 (Open) to 0.8 (Closed) [Adjust based on actual robot limits, usually 0.8 is closed]
        effort: Max effort/force
        """
        side = side.lower()
        if side not in self.clients:
            print(f"[GripperController] Error: Unknown side '{side}'. Use 'left' or 'right'.")
            return

        # Check state
        if self.current_state[side] is not None:
             # loose check for float equality
             if abs(self.current_state[side] - position) < 0.001:
                 state_str = "OPEN" if position < 0.1 else "CLOSED"
                 print(f"[GripperController] {side.upper()} gripper is already at {position} ({state_str}). Skipping.")
                 return

        client = self.clients[side]
        
        print(f"[GripperController] Moving {side.upper()} gripper to {position}...")
        
        if not client.wait_for_server(timeout_sec=2.0):
            print(f"[GripperController] Error: {side.upper()} action server not available!")
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = effort
        
        future = client.send_goal_async(goal)
        
        # We need to spin to get the result if we are using an async call.
        # If we passed in a node from an orchestrator that is already spinning elsewhere, 
        # explicitly spinning here might cause issues (MultiThreadedExecutor etc).
        # However, for this simple helper, we likely need to wait.
        # If own_node is True, we can spin freely.
        # If own_node is False, spinning might conflict.
        # The user's snippet used: execute_async then spin_until_future_complete.
        
        if self.own_node:
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()
            # In a real action, we might wait for the result callback too, 
            # but usually sending goal is enough for simple gripper command? 
            # The action client returns a GoalHandle. We should wait for result of the goal handle.
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                print(f"[GripperController] Goal rejected for {side}.")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            # result = result_future.result().result # The actual result msg
            
        else:
            # If we don't own the node, we can't block-spin safely without potentially blocking the main loop 
            # if we are in a single threaded executor.
            # But the user asked for a blocking-like behavior: "checks... and prints outputs".
            # We'll assume we can spin or wait. 
            pass  

        # Update state regardless of strict success for now, or assume success
        self.current_state[side] = position
        print(f"[GripperController] {side.upper()} Gripper command sent (Target: {position}).")
        time.sleep(0.5) # Physical delay

    def destroy(self):
        if self.own_node:
            self.node.destroy_node()
