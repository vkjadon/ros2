from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 2 second timer
        self.timer = self.create_timer(5.0, self.timer_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self.timer.cancel()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self.get_logger().info('Sending goal request...')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = MinimalActionClient()

        action_client.send_goal()

        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()