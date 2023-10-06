#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
#import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class BrickPickAdapterNode(Node):
    def __init__(self):
        super().__init__('brickpick_adapter_node')

        # Command subscriber
        self.cmd_subscriber = self.create_subscription(
            String,
            'brickpick_cmd',
            self.recv_cmd_callback,
            10
        )
        #self.brickpick_adapter = lego_builder_brickpick.brickpick_adapter.BrickPickAdapter()

    def recv_cmd_callback(self, msg : String):
        self.get_logger().info(f'Recieved "{msg.data}"')
        #self.brickpick_adapter.push(msg.data)


def main(args=None):
    rclpy.init(args=args)

    brickpick_adapter_node = BrickPickAdapterNode()

    rclpy.spin(brickpick_adapter_node)

    brickpick_adapter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()