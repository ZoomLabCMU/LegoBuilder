#from UR5 import UR5
from BrickPick import BrickPick
#import rospy
#from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
#from std_msgs.msg import Float32, Bool, Float64MultiArray
import numpy as np
import time
#import sensor_comm as sensor
#import pandas as pd
import os
import serial.tools.list_ports


BP_COM_LINUX = "/dev/tty/ACM2"
BP_COM_PC = "COM10"
BP_BAUDRATE = 115200

class LegoBuilder:
    def __init__(self, BP_info):
        #self.ur5 = UR5()

        BP_COM_port = BP_info[0]
        BP_baudrate = BP_info[1]
        self.BP = BrickPick(BP_COM_port, BP_baudrate) #TODO - Double check COM port

        #TODO - implement BrickPick.reset_joints()
        #self.BP.reset_joints()
        #rospy.Subscriber('/ur_pose', PoseStamped, self.ur5.callback_pos)
        #rospy.Subscriber('/ur_joints', Float64MultiArray, self.ur5.joint_callback)
        #rospy.Subscriber('/target_joints', Float64MultiArray, self.ur5.tgt_jt_callback)
        #rospy.sleep(0.5)
        # self.ur5.go_home()
        # self.ur5.set_ur_wrist_joint_val(0)
        # rospy.sleep(10)
        
        #TODO - Establish brick measurements (1x1 unit brick)
        #       ___---___   , stud_height
        #       |        |   |
        #       |        |   |
        #       |        |   | brick_height
        #       |________|   |
        #        brick_width
        self.stud_height = 2
        self.brick_width = 8
        self.brick_height = 9.6

        self.sandbox_origin = np.array([0.0, 0.0, 0.0]) #TODO - Establish sandbox_origin
        self.sandbox_brickWidth = 0     #X
        self.sandbox_brickLength = 0    #Y
        self.sandbox_width = self.brick_width * self.sandbox_brickWidth
        self.sandbox_length = self.brick_width * self.sandbox_brickLength

        self.ultra_safe_speed = 0.3
        self.ultra_safe_acc = 0.7
        self.safe_spd_mult = 0.6
        self.safe_acc = 0.30
        self.sensitive_spd_mult = 3.0
        self.sensitive_acc = 0.30

        #self.ur5.set_ur_speed(0.6)
        #self.ur5.set_ur_acceleration(0.35)

    def BP_help(self):
        self.BP.help()
    
    def BP_goto(self, joint_lengths):
        self.BP.goto(joint_lengths)

    def BP_brick_goto(self, plate, brick):
        self.BP.brick_goto(plate, brick)

    def BP_toggle_plotting(self):
        self.BP.toggle_plotting()

    def BP_deposit(self):
        self.BP.deposit()

    def BP_raise(self):
        self.BP.raise_plunger()

    


def main():
    print("Available COM Ports")
    serial.tools.list_ports.comports()

    BP_info = (BP_COM_PC, BP_BAUDRATE)
    robot = LegoBuilder(BP_info)

    cmd_dict = {"H": robot.BP_help,
                "G": robot.BP_goto,
                "B": robot.BP_brick_goto,
                "P": robot.BP_toggle_plotting,
                "D": robot.BP_deposit,
                "R": robot.BP_raise}
    while True:
        cmd = input("Enter Command:")
        cmd_word = cmd[0]
        if cmd_word not in cmd_dict:
            print("Command word \'%s\' not found".format(cmd_word))
            robot.BP_help()
        else:
            fn = cmd_dict[cmd_word]
            if cmd_word in {"H", "D", "R", "P"}:
                fn()
            elif cmd_word in {"G"}:
                vals = cmd[2:].split(" ")
                joint_lengths = [int(x) for x in vals]
                fn(joint_lengths)
            elif cmd_word in {"B"}:
                vals = cmd[2:].split(" ")
                args = tuple([int(x) for x in vals])
                fn(*args)


        

if __name__ == '__main__':
    main()