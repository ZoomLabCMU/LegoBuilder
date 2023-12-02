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
from legobuilder_control.Controllers import BrickPickController

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
Stop: 'Space'
Long plate:  /\ Up 
             ||
             \/ Down

            Up  Down
Short plate: <===>

Position Control:
plunger     ('p', 'o')              [Plunger down/up]
long plate  ('0', #        , '5')   [Go to brick #]
short plate (')', shift + #, '%')   [Go to brick #]

CTRL-C to quit
"""


class BrickPickTeleopNode(Node):

    def __init__(self):
        super().__init__('brickpick_teleop_node') # type: ignore

        self.brickpick_cli = self.create_client(
            BrickpickCommand, 
            "brickpick_cmd"
        )
        while not self.brickpick_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


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
    bp_controller = BrickPickController(brickpick_teleop_node.brickpick_cli)

    bindings = {
        # Velocity Controls 
        '[A': lambda: bp_controller.set_long_ctrl(-1),          # Up     
        '[B': lambda: bp_controller.set_long_ctrl(1),           # Down
        '[C': lambda: bp_controller.set_short_ctrl(-1),         # Right
        '[D': lambda: bp_controller.set_short_ctrl(1),          # Left

        # Long plate PID
        '0': lambda: bp_controller.set_long_target_mm(0.0),     # 0
        '1': lambda: bp_controller.long_goto_brick(1),          # Brick 1
        '2': lambda: bp_controller.long_goto_brick(2),          # Brick 2
        '3': lambda: bp_controller.long_goto_brick(3),          # Brick 3
        '4': lambda: bp_controller.long_goto_brick(4),          # Brick 4
        '5': lambda: bp_controller.long_goto_brick(5),          # Brick 5

        # Short plate PID
        ')': lambda: bp_controller.set_short_target_mm(0.0),     # 0
        '!': lambda: bp_controller.short_goto_brick(1),          # Brick 1
        '@': lambda: bp_controller.short_goto_brick(2),          # Brick 2
        '#': lambda: bp_controller.short_goto_brick(3),          # Brick 3
        '$': lambda: bp_controller.short_goto_brick(4),          # Brick 4
        '%': lambda: bp_controller.short_goto_brick(5),          # Brick 5

        # Plunger PID
        'o': lambda: bp_controller.plunger_up(),                 # Plunger up
        'p': lambda: bp_controller.plunger_down(),               # Plunger down
    }
    
    spinner = threading.Thread(target=rclpy.spin, args=(brickpick_teleop_node,))
    spinner.start()

    try:
        print(help_msg)
        velocity_scalar = 65535 / 4
        while True:
            key = getKey(settings)
            if key in bindings.keys():
                status = bindings[key]()
            else:
                status = bp_controller.stop()
                if (key == '\x03'):
                    break
            brickpick_teleop_node.get_logger().info(
                f'Command status: {status}'
            )
    except Exception as e:
        print(e)

    finally:
        status = bp_controller.stop()
        brickpick_teleop_node.get_logger().info(
            f'Command status: {status}'
        )
        brickpick_teleop_node.destroy_node()
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()