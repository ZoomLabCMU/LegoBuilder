#!/usr/bin/env python


''' Brickpick Teleop Node

Heavily adapted from ros-teleop/teleop_twist_keyboard
https://github.com/ros2/teleop_twist_keyboard/blob/dashing/teleop_twist_keyboard.py

Key bindings:
Down: short_vel down
Up: short_vel up
Right: long_vel up
Left: long_vel down
Space pressed: plunger_down True
Space released: plunger_down False
'''

import rclpy
from rclpy.node import Node

from legobuilder_interfaces.srv import BrickpickCommand

import threading
import sys

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


help_msg = """
This node takes keypresses from the keyboard and publishes them as BrickpickCommand service requests
-------------------------
Velocity Control:
short_vel    ('Up', 'Down')
long_vel     ('Right', 'Left')
plunger_down ('Space')

CTRL-C to quit
"""

# velocity state space
# [short_vel, long_vel, plunger_down]
velocityBindings = {
    '[A': {'command': '/set_long_ctrl', 'u': -1},   #Up
    '[B': {'command': '/set_long_ctrl', 'u': 1},    #Down
    '[C': {'command': '/set_short_ctrl', 'u': -1},  #Right
    '[D': {'command': '/set_short_ctrl', 'u': 1},   #Left
}

positionBindings = {
    '1': {'command': '/set_long_target_brick', 'target_brick': 1},  #Up
    '2': {'command': '/set_long_target_brick', 'target_brick': 2},  #Down
    '3': {'command': '/set_long_target_brick', 'target_brick': 3},  #Right
    '4': {'command': '/set_long_target_brick', 'target_brick': 4},  #Left
    '5': {'command': '/set_long_target_brick', 'target_brick': 5},  #Space
}


class BrickPickTeleopNode(Node):

    def __init__(self):
        super().__init__('brickpick_teleop_node') # type: ignore

        self.cmd_client = self.create_client(
            BrickpickCommand, 
            "brickpick_cmd"
            )
        self.velocity_scalar = 65535
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_cmd_request(self, cmd_request : BrickpickCommand.Request) -> int | None:
        #Blocking and waiting???

        #self.future = self.cmd_client.call_async(self.cmd_request)
        #rclpy.spin_until_future_complete(self, self.future)
        #return self.future.result()

        return self.cmd_client.call(cmd_request)

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b': #Escape keys (fn, arrow, ctrl, etc)
            key = sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    settings = saveTerminalSettings()
    rclpy.init(args=args)

    brickpick_teleop_node = BrickPickTeleopNode()
    
    #executor = rclpy.executors.MultiThreadedExecutor()
    spinner = threading.Thread(target=rclpy.spin, args=(brickpick_teleop_node,))
    spinner.start()

    try:
        print(help_msg)
        velocity_scalar = 65535
        while True:
            key = getKey(settings)
            if key in velocityBindings.keys():
                command = velocityBindings[key]['command']
                u = velocityBindings[key]['u'] * velocity_scalar

                command_request = BrickpickCommand.Request()
                command_request.command = command
                command_request.u = u
            elif key in positionBindings.keys():
                command = positionBindings[key]['command']
                target_brick = positionBindings[key]['target_brick']

                command_request = BrickpickCommand.Request()
                command_request.command = command
                command_request.target_brick = target_brick
            else:
                command_request = BrickpickCommand.Request()
                command_request.command = '/stop'
                if (key == '\x03'):
                    break
            print(f"Requested controls [{key}]: (request: {command_request}")

            response = brickpick_teleop_node.send_cmd_request(command_request)
            brickpick_teleop_node.get_logger().info(
                f'Command status: {response}'
            )
    except Exception as e:
        print(e)

    finally:
        #CHANGE THIS TO STOP LATER
        command_request = BrickpickCommand.Request()
        command_request.command = '/stop'
        response = brickpick_teleop_node.send_cmd_request(command_request)
        brickpick_teleop_node.get_logger().info(
            f'Command status: {response}'
        )
        brickpick_teleop_node.destroy_node()
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()