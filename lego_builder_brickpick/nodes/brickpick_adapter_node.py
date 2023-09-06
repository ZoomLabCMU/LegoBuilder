#!/usr/bin/env python


''' Brickpick Interface Node

Node to handle the lego_builder interface to the BrickPick end effector
'''
import lego_builder_brickpick.brickpick_adapter

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class BrickPickAdapterNode(Node):
    def __init__(self):
        super().__init__('brickpick_adapter_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('recv_brickpick_cmd_topic', rclpy.Parameter.Type.STRING),
                ('execute_cmd_interval_s', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        self.brickpick_adapter = lego_builder_brickpick.brickpick_adapter.BrickPickAdapter()
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Callback for receiving commands
        self.brickpick_cmd_sub = self.create_subscription(
            String,
            rclpy.get_parameter("recv_cmd_topic"),
            self.recv_cmd_callback,
            10
        )

        # Timer for executing commands
        self.execute_cmd_timer = self.create_timer(
            rclpy.get_parameter("execute_cmd_interval_s"),
            self.execute_cmd_callback
        )

    def recv_cmd_callback(self, msg : String):
        self.get_logger().info(f"[BrickPickAdapter] Recieved {msg.data}")
        self.brickpick_adapter.push(msg.data)

    def execute_cmd_callback(self):
        self.brickpick_adapter.execute_next_cmd()
        self.get_logger().info(f"[BrickPickAdapter] Executing next command")


def main(args=None):
    rclpy.init(args=args)
    brickpick_adapter_node = BrickPickAdapterNode()
    rclpy.spin(brickpick_adapter_node)
    brickpick_adapter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()