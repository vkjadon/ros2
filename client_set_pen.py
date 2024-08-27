import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen


class SetPenClient(Node):
    def __init__(self):
        super().__init__('set_pen_client')
        
        # Create a service client for the /turtle1/set_pen service
        self.client = self.create_client(SetPen, '/turtle1/set_pen')
        
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/set_pen service...')
        
        # Once service is available, send a request
        self.send_request(255, 0, 0, 3, 0)

    def send_request(self, r, g, b, width, off):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Pen properties set successfully.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SetPenClient()    
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
