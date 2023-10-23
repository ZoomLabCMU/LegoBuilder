import numpy as np
import serial
from collections import deque

from std_msgs.msg import String
import requests


IP_ADDRESS = "192.168.2.113"

class BrickPickAdapter(object):
    def __init__(self):
        # TODO - Kinematics (load from config)
        self.TCP = np.array([8.0, -16.0, 238.6]) #Coordinate transform in mm to corner of fixed brick

        # Comms

        # Geometry/limits
        #joints: [q1: long_plate, q2: short_plate, q3: plunger]
        # ALL UNITS IN mm
        self.joint_lengths = np.array([50, 50, 10]) 
        self.joint_limits = np.array([[1.5,29.5], [1.5,29.5], [1.5,8.5]])
        
        # Commands
        self.command_queue = deque([], maxlen=10)

        # TODO - Payload
        self.brick_stack = [] # 0-idx corresponds to brick nearest base
        self.stack_height = 0


    ### ROS node callback functions ###
    def push_velocity(self, short_vel: float, long_vel: float, plunger_down: bool) -> String:
        # TMP for demo
        if short_vel > 0:
            response = requests.get(f"http://{IP_ADDRESS}/Down")
        if short_vel < 0:
            response = requests.get(f"http://{IP_ADDRESS}/Up")
        if short_vel == 0:
            response = requests.get(f"http://{IP_ADDRESS}/Stop")
        return "Null"

    def push(self, msg: String):
        cmd = msg # Extract command from message buffer
        # Push to right side of deque
        self.command_queue.append(cmd)

    def execute_next_cmd(self):
        # Pop from left side of deque
        cmd = self.command_queue.popleft()
        self.ser.write(cmd.encode())
        pass

    ########### Serial Commands ##############
    def help(self):
        self.ser.write(b'H\r\n')
        print("<INSERT HELP MESSAGE HERE>")
        return

    def goto(self, joint_lengths):
        # joint_lengths: np.ndarray([q1, q2, q3])
        msg = "G {} {} {}\r\n".format(int(joint_lengths[0]), int(joint_lengths[1]), int(joint_lengths[2]))
        print(msg)
        self.ser.write(msg.encode())
        return
    
    def brick_goto(self, plate, brick):
        # plate: <0: long_plate, 1: short_plate>
        # brick: <0, 1, 2, 3> plate moves to base of brick
        if plate in {0, 'long', 'LONG', 'l', 'L'}:
            plate = 0
        if plate in {1, 'short', 'SHORT', 's', 'S'}:
            plate  = 1
        print(plate)
        print(brick)
        msg = "B {} {}\r\n".format(int(plate), int(brick))
        print(msg)
        self.ser.write(msg.encode())
        return
    
    def toggle_plotting(self):
        # WIP, enable serial plotting from BP
        self.ser.write(b'P\r\n')
        return
    
    def deposit(self):
        # Deposit the stack of bricks
        self.ser.write(b'D\r\n')
        return
    
    def raise_plunger(self):
        # Raise the plunger
        self.ser.write(b'R\r\n')
        return
    
    ########### Brick Manipulation #################

    def push_to_brick_stack(self, brick):
        # brick: Brick object
        # TODO - add checks against plate lengths and stack height
        self.brick_stack.append(brick)
        self.stack_height += brick.bb_height
        return
    
    def pop_from_brick_stack(self):
        # Pops the last Brick object from the stack and
        brick = self.brick_stack.pop()
        self.stack_height -= brick.bb_height
        return brick
    
    def deposit_brick_stack(self):
        stack = self.brick_stack
        self.brick_stack = []
        self.stack_height = 0
        return stack

