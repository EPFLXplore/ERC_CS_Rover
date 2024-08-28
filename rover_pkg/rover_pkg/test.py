import rclpy
from rclpy.action import ActionServer
from custom_msg.action import NAVReachGoal  # Ensure this matches your action
from rclpy.node import Node

# Define a simple Node class that includes the ActionServer
class TestActionServer(Node):
def __init__(self):
    super().__init__('test_action_server')
    
    # Create an ActionServer for NAVReachGoal
    self._action_server = ActionServer(
        self,
        NAVReachGoal,
        'test_nav_reach_goal',  # Use the correct action name here
        execute_callback=self.execute_callback
    )

    self.get_logger().info('Action server for NAVReachGoal has been created successfully.')

    # Dummy execute callback just for testing purposes
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing action...')
        goal_handle.succeed()
        return NAVReachGoal.Result()

# Run the test only if this is the first time the ROS context is initialized
if not rclpy.ok():
    # Initialize the ROS 2 Python environment
    rclpy.init()

    # Create an instance of the TestActionServer node
    node = TestActionServer()

    # Spin the node to keep it alive and processing callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()
else:
    print("ROS 2 context is already initialized.")
