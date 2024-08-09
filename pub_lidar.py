import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'lidar_scan', 10)
        self.timer = self.create_timer(0.1, self.publish_lidar_data)  # 10 Hz

    def publish_lidar_data(self):
        msg = LaserScan()
        # Populate msg with lidar data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing lidar data')

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
