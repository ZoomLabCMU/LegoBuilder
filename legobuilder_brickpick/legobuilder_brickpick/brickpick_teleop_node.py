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

Position Control:
plunger ('p', 'o')

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
    '0': {'command': '/set_long_target_mm', 'target_mm': 0.0}, # Zero
    '1': {'command': '/goto_long_brick', 'target_brick': 1},  # Brick 1
    '2': {'command': '/goto_long_brick', 'target_brick': 2},  # Brick 2
    '3': {'command': '/goto_long_brick', 'target_brick': 3},  # Brick 3
    '4': {'command': '/goto_long_brick', 'target_brick': 4},  # Brick 4
    '5': {'command': '/goto_long_brick', 'target_brick': 5},  # Brick 5
    ')': {'command': '/set_short_target_mm', 'target_mm': 0.0}, # Zero
    '!': {'command': '/goto_short_brick', 'target_brick': 1},  # Brick 1
    '@': {'command': '/goto_short_brick', 'target_brick': 2},  # Brick 2
    '#': {'command': '/goto_short_brick', 'target_brick': 3},  # Brick 3
    '$': {'command': '/goto_short_brick', 'target_brick': 4},  # Brick 4
    '%': {'command': '/goto_short_brick', 'target_brick': 5},  # Brick 5
    'o': {'command': '/goto_plunger', 'plunger_target': 0}, #plunger up
    'p': {'command': '/goto_plunger', 'plunger_target': 1} #plunger down
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

    def send_cmd_request(self, cmd_request : BrickpickCommand.Request) -> BrickpickCommand.Response:
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
        print(f"key0: {key}")
        if key == '\x1b': #Escape keys (fn, arrow, ctrl, etc)
            key = sys.stdin.read(2)
            print(f"key1: {key}")
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
        velocity_scalar = 65535 / 4
        while True:
            key = getKey(settings)
            if key in velocityBindings.keys():
                command = velocityBindings[key]['command']
                u = velocityBindings[key]['u'] * velocity_scalar

                command_request = BrickpickCommand.Request()
                command_request.command = command
                command_request.u = int(u)
            elif key in positionBindings.keys():
                command = positionBindings[key]['command']
                command_request = BrickpickCommand.Request()
                command_request.command = command
                if 'target_brick' in positionBindings[key]:
                    target_brick = positionBindings[key]['target_brick']
                    command_request.target_brick = target_brick
                if 'plunger_target' in positionBindings[key]:
                    plunger_target = positionBindings[key]['plunger_target']
                    command_request.plunger_target = plunger_target

            else:
                command_request = BrickpickCommand.Request()
                command_request.command = '/stop'
                if (key == '\x03'):
                    break
            print(f"Requested controls [{key}]: (request: {command_request}")

            response = brickpick_teleop_node.send_cmd_request(command_request)
            brickpick_teleop_node.get_logger().info(
                f'Command status: {response.status}'
            )
    except Exception as e:
        print(e)

    finally:
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