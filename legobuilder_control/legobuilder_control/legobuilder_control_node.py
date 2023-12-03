#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import tf2_ros
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from legobuilder_interfaces.srv import BrickpickCommand

import os
import time
import signal
import sys
import argparse
from legobuilder_control.Controllers import URController, BrickPickController
#from legobuilder_perception.LegoDetector import Detectron2

import random
from legobuilder_control.utils import save_rgb, save_proprioceptive

import pickle

clicks = []
tool_height = 0.08
block_height = 0.0096
block_width = 0.0158
block_length = 0.0318

class LegoBuilderControlNode(Node):
    def __init__(self):
        super().__init__('Legobuilder_control_node') # type: ignore

        # Workspace camera subscriber
        self.workspace_camera_sub = self.create_subscription(
            Image,
            "/workspace_img",
            self.recv_workspace_camera_img,
            10
        )
        # BrickPick adapter client
        self.brickpick_adapter_cli = self.create_client(
            BrickpickCommand,
            "/brickpick_command"
        )
        # UR5e driver publisher
        self.ur_driver_pub = self.create_publisher(
            String,
            "/urscript_interface/script_command",
            10
        )

        self.ur_wrench_sub = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.recv_ur_wrench,
            10
        )

        self.ur_joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.recv_ur_joint_states,
            10
        )
        '''
        # Detectron service node client
        self.lego_inference_client = self.create_client(
            LegoImageDetection, 
            "lego_img_detection"
        )

        while not self.lego_inference_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Lego detector service not available, waiting again...')

        # Realsense subscriber
        self.ee_camera_sub = self.create_client(
            ???, 
            "???"
        )
        '''

        self.bp_controller = BrickPickController(self.brickpick_adapter_cli)
        self.ur_controller = URController(self.ur_driver_pub)

    def recv_workspace_camera_img(self, img : Image):
        return None
    
    def recv_ur_wrench(self, wrench : WrenchStamped):
        self.ur_controller.update_ee_wrench(wrench)
        return None
    
    def recv_ur_joint_states(self, joint_states : JointState):
        self.ur_controller.update_joint_states(joint_states)
        return None

def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if input("\nAre you sure you want to quit? (y/n)> ").lower().startswith('y'):
            sys.exit(1)

    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)


def demo1(args=None):
    num_trials = 10
    directory_path = '/home/klz/Documents/lego_ur5e/chris_data/'
    rclpy.init(args=args)
    legobuilder_control_node = LegoBuilderControlNode()

    ur_controller = URController(legobuilder_control_node.ur_driver_pub)
    bp_controller = BrickPickController(legobuilder_control_node.brickpick_adapter_cli)
    #detectron = Detectron2()

    ur_controller.set_tcp([0, 0, tool_height, 0, 0, 0])

    ur_controller.home_arm(10)

    print('Free Drive enabled')

    ur_controller.freedrive(axes=[1,1,1,0,0,0], time=15)

    #block_pose = ur_controller.get_ee_pose()
    block_pose = [0.12859122581391358, -0.5526477938788351, 0.015143098090133963, -2.9024186136539276, 1.2023166270087013, -0.000011600763336924778]
    print(block_pose)

    above_block_pose = block_pose.copy()
    above_block_pose[2] += 0.1

    ur_controller.move_ee(above_block_pose, 5)

    #detectron.set_top_bbox()

    #detectron.set_bottom_bbox()

    #top_bbox = detectron.get_top_bbox()
    #bottom_bbox = detectron.get_bottom_bbox()
    top_bbox = 0
    bottom_bbox = 0
    starting_top = 0
    starting_bottom = 0
    ending_top = 0
    ending_bottom = 0

    direction_actions = ['front', 'back', 'left', 'right']
    skill_actions = ['pick', 'place']

    for skill in skill_actions:
        if not os.path.exists(directory_path + skill):
            os.makedirs(directory_path + skill)
    for direction in direction_actions: 
        if not os.path.exists(directory_path + skill + '/' + direction):
            os.makedirs(directory_path + skill + '/' + direction)


    for i in range(num_trials):
        for skill in skill_actions:
            #starting_top, starting_bottom = detectron.evaluate_start_image()

            if skill == 'pick' and starting_bottom == 0:
                continue
            if skill == 'place' and starting_top == 0:
                continue

            if skill == 'pick':
                num_blocks = random.randint(1,starting_bottom)
            elif skill == 'place':
                num_blocks = random.randint(1,starting_top)


            random_angle = random.random() * 45

            random_direction = random.randint(0,3)
            #random_z_offset = random.uniform(-0.005, 0.005)
            random_z_offset = 0

            if skill == 'pick':
                starting_z = tool_height + (starting_top + num_blocks) * block_height
                #random_direction = 0
            elif skill == 'place':
                starting_z = tool_height + (starting_top - num_blocks) * block_height
                #random_direction = random.randint(0,3)

            ur_controller.zero_ft_sensor()

            if random_direction == 0:
                # Front
                #random_x_offset = random.uniform(-0.005, 0.005)
                random_x_offset = 0
                
                
                starting_x = -block_width/2
                starting_y = 0

                print(skill + ' ' + str(num_blocks) + ' blocks using front tilt.')

                if not os.path.exists(directory_path + skill + '/front/' + str(starting_top)):
                    os.makedirs(directory_path + skill + '/front/' + str(starting_top))
                if not os.path.exists(directory_path + skill + '/front/' + str(starting_top) + '/' + str(starting_bottom)):
                    os.makedirs(directory_path + skill + '/front/' + str(starting_top) + '/' + str(starting_bottom))
                if not os.path.exists(directory_path + skill + '/front/' + str(starting_top) + '/' + str(starting_bottom) + '/' + str(num_blocks)):
                    os.makedirs(directory_path + skill + '/front/' + str(starting_top) + '/' + str(starting_bottom) + '/' + str(num_blocks))

                path = directory_path + skill + '/front/' + str(starting_top) + '/' + str(starting_bottom) + '/' + str(num_blocks) + '/'
                folder_num = str(len(os.listdir(path)))
                os.mkdir(path+folder_num)

                trial_dict = {'block_pose':block_pose,
                              'top_bbox':top_bbox,
                              'bottom_bbox':bottom_bbox,
                              'starting_top':starting_top, 
                              'starting_bottom':starting_bottom,
                              'random_angle':random_angle,
                              'direction':'front',
                              'num_blocks':num_blocks,
                              'tcp':[starting_x + random_x_offset, starting_y, starting_z + random_z_offset],
                              'random_offset':[random_x_offset, 0, random_z_offset]}

                with open(path+folder_num+'/trial_dict.pkl', 'wb') as handle:
                    pickle.dump(trial_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

                rgb_p = save_rgb(path+folder_num+'/', 'rgb')
                bag_p = save_proprioceptive(path+folder_num+'/', 'bag')

                ur_controller.move_ee(block_pose, 5, force_thresholds=[80,80,80])

                ur_controller.set_tcp([starting_x + random_x_offset, starting_y, starting_z + random_z_offset, 0, 0, 0])

                ur_controller.rotate_ee_degrees([1,0,0], -random_angle, 3, force_thresholds=[80,80,80])

                ur_controller.set_tcp([0, 0, tool_height, 0, 0, 0])

                ur_controller.move_ee(above_block_pose, 5)

                rgb_p.terminate()
                bag_p.terminate()

                #starting_top, starting_bottom, ending_top, ending_bottom = detectron.evaluate_end_image()

                end_trial_dict = {'ending_top':ending_top,
                                  'ending_bottom':ending_bottom}

                with open(path+folder_num+'/end_trial_dict.pkl', 'wb') as handle:
                    pickle.dump(end_trial_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

    ur_controller.home_arm(10)


if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_trials', '-n', type=int, required=True)
    args = parser.parse_args()

    demo1(args)
