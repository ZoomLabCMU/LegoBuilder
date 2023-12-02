#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
#import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from legobuilder_interfaces.srv import BrickpickCommand
from legobuilder_drivers.brickpick_adapter import BrickPickAdapter

ip_address = "172.26.185.38"
ip_address = "192.168.2.101"

class BrickPickAdapterNode(Node):
    def __init__(self):
        super().__init__("brickpick_adapter_node") #type: ignore

        # Command subscriber
        self.cmd_subscriber = self.create_service(
            BrickpickCommand,
            'brickpick_cmd',
            self.recv_cmd_req_callback
        )
        self.brickpick_adapter = BrickPickAdapter(ip_address)

    def recv_cmd_req_callback(self, request : BrickpickCommand.Request, response : BrickpickCommand.Response):
        # push command to brickpick_adapter
        self.get_logger().info(
            f'Incoming BrickpickCommand request: {request}'
        )
        # wait for command to finish/terminate
        self.get_logger().info(f"Pushing HTTP request to BrickPick@{ip_address}")
        status = self.brickpick_adapter.push_cmd(request)
        self.get_logger().info(f"Recieved HTTP response from BrickPick@{ip_address}:\n{status}")
        # set response
        response.status = status
        self.get_logger().info(
            f'Returning BrickpickCommand response: {response}'
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
