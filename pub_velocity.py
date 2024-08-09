import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def control_loop(self):
        cmd = Twist()
        # Compute control commands
        self.publisher_.publish(cmd)
        self.get_logger().info('Publishing velocity command')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
