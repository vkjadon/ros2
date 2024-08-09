# python my_node.py --ros-args --node-name my_custom_node --namespace my_namespace

import rclpy

def main(args=None):
    rclpy.init(args=args)
    # Rest of your node code

if __name__ == '__main__':
    import sys
    main(sys.argv)
