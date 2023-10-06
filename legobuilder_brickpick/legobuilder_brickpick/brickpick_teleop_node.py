#!/usr/bin/env python


''' Brickpick Teleop Node

Key bindings:
Down: left plate down
Up: left plate up
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import random

class BrickPickTeleopNode(Node):

    def __init__(self):
        super().__init__('brickpick_teleop_node')

        self.cmd_publisher = self.create_publisher(
            String, 
            "brickpick_cmd",
            10
            )
        timer_period = 1.0 # second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = random.choice(["Up", "Down"])
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Published "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    brickpick_teleop_node = BrickPickTeleopNode()
    
    rclpy.spin(brickpick_teleop_node)
 
    brickpick_teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()