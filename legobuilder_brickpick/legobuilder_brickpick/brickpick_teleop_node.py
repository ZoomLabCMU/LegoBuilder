#!/usr/bin/env python


''' Brickpick Teleop Node

Key bindings:
Down: left plate down
Up: left plate up
'''

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

import sys

class BrickPickTeleopNode(Node):

    def __init__(self):
        super().__init__('brickpick_teleop_node') # type: ignore

        self.cmd_client = self.create_client(
            AddTwoInts, 
            "brickpick_cmd"
            )
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cmd_request = AddTwoInts.Request()

    def send_cmd_request(self, a: int, b: int) -> int | None:
        self.cmd_request.a = a
        self.cmd_request.b = b
        self.future = self.cmd_client.call_async(self.cmd_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    brickpick_teleop_node = BrickPickTeleopNode()
    
    #rclpy.spin(brickpick_teleop_node)
    response = brickpick_teleop_node.send_cmd_request(int(sys.argv[1]), int(sys.argv[2]))
    brickpick_teleop_node.get_logger().info(
        f'Command status: {response}'
    )

    brickpick_teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()