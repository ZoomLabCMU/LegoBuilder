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
    '[A': (0, -1, False), #Up
    '[B': (0, 1, False),  #Down
    '[C': (-1, 0, False), #Right
    '[D': (1, 0, False),  #Left
    ' ': (0, 0, True),        #Space
}


class BrickPickTeleopNode(Node):

    def __init__(self):
        super().__init__('brickpick_teleop_node') # type: ignore

        self.cmd_client = self.create_client(
            BrickpickCommand, 
            "brickpick_cmd"
            )
        while not self.cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cmd_request = BrickpickCommand.Request()

    def send_cmd_request(self, short_vel: int, long_vel: int, plunger_down: bool) -> int | None:
        self.cmd_request.short_vel = short_vel
        self.cmd_request.long_vel = long_vel
        self.cmd_request.plunger_down = plunger_down

        #self.future = self.cmd_client.call_async(self.cmd_request)
        #rclpy.spin_until_future_complete(self, self.future)
        #return self.future.result()

        return self.cmd_client.call(self.cmd_request)

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

        while True:
            key = getKey(settings)
            if key in velocityBindings.keys():
                short_vel = velocityBindings[key][0]
                long_vel = velocityBindings[key][1]
                plunger_down = velocityBindings[key][2]
            else:
                short_vel = 0
                long_vel = 0
                plunger_down = False
                if (key == '\x03'):
                    break
            print(f"Requested controls [{key}]: (short_vel: {short_vel}, long_vel: {long_vel}, plunger_down: {plunger_down})")

            response = brickpick_teleop_node.send_cmd_request(short_vel, long_vel, plunger_down)
            brickpick_teleop_node.get_logger().info(
                f'Command status: {response}'
            )
    except Exception as e:
        print(e)

    finally:
        short_vel = 0
        long_vel = 0
        plunger_down = False
        response = brickpick_teleop_node.send_cmd_request(short_vel, long_vel, plunger_down)
        brickpick_teleop_node.get_logger().info(
            f'Command status: {response}'
        )
        brickpick_teleop_node.destroy_node()
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()