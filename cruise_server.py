#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import CruiseSpeed
import time

class CruiseActionServerNode(Node):

    def __init__(self):
        super().__init__('cruise_node')

        self.cruse_action_server=ActionServer(self, CruiseSpeed, "cruise_speed", execute_callback=self.callback_cruise_server)
        self.get_logger().info("Action Server Node Started !! Waiting For request")
        self.speed=70

    def callback_cruise_server(self, handle):
        cruise_speed = handle.request.cruise_speed
        cruise_step = handle.request.cruise_step
        current_speed = self.speed

        for i in range (self.speed, cruise_speed, cruise_step):
            current_speed=current_speed + cruise_step 
            self.get_logger().info(f"Execute Callback Invoked !! {current_speed}")
            time.sleep(1)
            
        handle.succeed()

        result = CruiseSpeed.Result()
        result.final_speed = current_speed
        return result

def main():
    rclpy.init()

    try:
        node = CruiseActionServerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()