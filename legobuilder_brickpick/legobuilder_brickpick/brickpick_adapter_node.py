#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
#import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from legobuilder_interfaces.srv import BrickpickCommand
from legobuilder_brickpick.brickpick_adapter import BrickPickAdapter

class BrickPickAdapterNode(Node):
    def __init__(self):
        super().__init__("brickpick_adapter_node")

        # Command subscriber
        self.cmd_subscriber = self.create_service(
            BrickpickCommand,
            'brickpick_cmd',
            self.recv_cmd_callback
        )
        self.brickpick_adapter = BrickPickAdapter()

    def recv_cmd_callback(self, request, response):
        # push command to brickpick_adapter
        short_vel = request.short_vel
        long_vel = request.long_vel
        plunger_down = request.plunger_down
        # wait for command to finish/terminate
        status = self.brickpick_adapter.push_velocity(short_vel, long_vel, plunger_down)
        # set response
        response.status = status
        self.get_logger().info(
            f'Incoming request\nshort_vel: {request.short_vel} long_vel: {request.long_vel} plunger_down: {request.plunger_down}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    brickpick_adapter_node = BrickPickAdapterNode()

    rclpy.spin(brickpick_adapter_node)

    brickpick_adapter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()