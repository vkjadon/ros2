# python my_node.py --ros-args -r /old_topic:=/new_topic
import rclpy

def main(args=None):
    rclpy.init(args=args)
    # Rest of your node code

if __name__ == '__main__':
    import sys
    main(sys.argv)
