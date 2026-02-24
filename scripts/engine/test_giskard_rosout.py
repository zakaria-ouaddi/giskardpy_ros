import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

class RosoutListener(Node):
    def __init__(self):
        super().__init__('rosout_listener')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            100)

    def listener_callback(self, msg):
        if msg.level >= Log.ERROR or 'giskard' in msg.name.lower():
            if msg.level >= Log.WARN:
                print(f"[{msg.name}] {msg.msg}")
                with open('giskard_errors.log', 'a') as f:
                    f.write(f"[{msg.name}] {msg.msg}\n")

if __name__ == '__main__':
    rclpy.init()
    node = RosoutListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
