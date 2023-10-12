#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
#import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class BrickPickAdapterNode(Node):
    def __init__(self):
        super().__init__("brickpick_adapter_node")

        # Command subscriber
        self.cmd_subscriber = self.create_service(
            AddTwoInts,
            'brickpick_cmd',
            self.recv_cmd_callback
        )
        #self.brickpick_adapter = lego_builder_brickpick.brickpick_adapter.BrickPickAdapter()

    def recv_cmd_callback(self, request, response):
        # push command to brickpick_adapter
        a = request.a
        b = request.b
        # wait for command to finish/terminate
        sum = a + b
        # set response
        response.sum = sum
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')

        return response


def main(args=None):
    rclpy.init(args=args)

    brickpick_adapter_node = BrickPickAdapterNode()

    rclpy.spin(brickpick_adapter_node)

    brickpick_adapter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()