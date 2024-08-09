import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.check_health)  # 1 Hz

    def check_health(self):
        status = String()
        # Perform health checks and populate status
        status.data = 'All systems operational'
        self.publisher_.publish(status)
        self.get_logger().info('Publishing robot status')

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
