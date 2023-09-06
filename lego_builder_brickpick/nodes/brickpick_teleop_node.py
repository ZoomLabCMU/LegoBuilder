#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class BrickPickTeleopNode(Node):
    def __init__(self):
        super().__init__('brickpick_teleop_node')
        self.cmd_publisher = self.create_publisher(String, 'topic', 10)
        

def main(args=None):
    rclpy.init(args=args)
    brickpick_teleop_node = BrickPickTeleopNode()
    while rclpy.ok():
        msg = String()
        cmd = input("Enter command: ")
        msg.data = cmd
        brickpick_teleop_node.cmd_publisher.publish(msg)
        brickpick_teleop_node.get_logger().info(f"[BrickPick Teleop] Published {cmd}")
    brickpick_teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()