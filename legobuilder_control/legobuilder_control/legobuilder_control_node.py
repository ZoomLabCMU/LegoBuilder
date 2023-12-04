#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import tf2_ros
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, Wrench, Twist
from sensor_msgs.msg import JointState

from legobuilder_interfaces.msg import GotoGoal, TwistGoal
from legobuilder_interfaces.srv import BrickpickCommand
from legobuilder_interfaces.action import BasicControl, GotoControl, TCPTwistControl

from rclpy.action import ActionServer, GoalResponse, CancelResponse

import os
import time
import signal
import sys
import argparse
from legobuilder_control.Controllers import URController, BrickPickController, utils
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
        self.motion_completion_thresh = 99.5

        # State Estimation
        self.joint_states = JointState()
        self.ee_wrench = WrenchStamped()
        self.tf_buffer = tf2_ros.Buffer()

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
        # Joint state subscriber
        self.ur_joint_states_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.recv_ur_joint_states,
            10
        )
        # End effector force-torque subscriber
        self.ur_wrench_sub = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.recv_ur_wrench,
            10
        )
        # Transform buffer subscriber
        self.tf_buffer_sub = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )
        '''
        # Realsense subscriber
        self.ee_camera_sub = self.create_client(
            ???, 
            "???"
        )
        '''

        ### Action Servers ###
        # Basic actions
        self.basic_goal = BasicControl.Goal()
        self.basic_action_server = ActionServer(
            self,
            BasicControl,
            'basic_control',
            execute_callback=self.basic_execute_callback,
            goal_callback=self.basic_goal_callback,
            cancel_callback=self.cancel_callback
        )
        # Goto actions
        self.goto_goal = GotoControl.Goal()
        self.goto_action_server = ActionServer(
            self,
            GotoControl,
            'goto_control',
            execute_callback=self.goto_execute_callback,
            goal_callback=self.goto_goal_callback,
            cancel_callback=self.cancel_callback
        )
        # TCP twist actions
        self.tcp_twist_goal = TCPTwistControl.Goal()
        self.tcp_twist_action_server = ActionServer(
            self,
            TCPTwistControl,
            'tcp_twist_control',
            execute_callback=self.tcp_twist_execute_callback,
            goal_callback=self.tcp_twist_goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Actuation
        self.bp_controller = BrickPickController(self.brickpick_adapter_cli)
        self.ur_controller = URController(self.ur_driver_pub)

    
    def recv_ur_wrench(self, wrench : WrenchStamped):
        self.ee_wrench = wrench
        return True
    
    def recv_ur_joint_states(self, joint_states : JointState):
        self.joint_states = joint_states
        return True
    
    def get_ee_pose(self):
        trans = self.tf_buffer.lookup_transform('base', 'tool0_controller', rclpy.Time())

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def basic_goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Recieved (basic) goal request')
        self.basic_goal = goal_request
        return GoalResponse.ACCEPT

    def basic_execute_callback(self, goal_handle):
        self.get_logger().info('Executing basic control goal...')
        cmd = self.basic_goal.goal
        if cmd == "freedrive":
            self.ur_controller.freedrive()
        elif cmd == "stop":
            self.ur_controller.stop()
            self.bp_controller.stop()
        elif cmd == "home_arm":
            self.ur_controller.home_arm()
        elif cmd == "home_ee":
            self.bp_controller.reset()
        elif cmd == "activate_FT":
            task_frame=None
            selection_vector=None
            wrench=None
            limits=None
            time=None
            self.ur_controller.force_mode(task_frame, selection_vector, wrench, limits, time)
        elif cmd == "deactivate_FT":
            self.ur_controller.end_force_mode()
        elif cmd == "zero_FT":
            self.ur_controller.zero_ft_sensor()
        else:
            raise NotImplementedError
        goal_handle.succeed()

        result = BasicControl.Result()
        result.result = "complete"
        return result

    def goto_goal_callback(self, goal_request):
        self.get_logger().info('Received (goto) goal request')
        self.goto_goal = goal_request
        return GoalResponse.ACCEPT
    
    def goto_execute_callback(self, goal_handle):
        # TODO - separate time and timeout?
        goto_msg = self.goto_goal.goal # type: GotoGoal
        goal_ee_pose = goto_msg.ee_pose
        goal_joint_states = goto_msg.joint_states
        policy = goto_msg.policy
        timeout = goto_msg.timeout
        use_timeout = goto_msg.use_timeout
        wrench_thresh = goto_msg.wrench_thresh
        use_ft = goto_msg.use_ft
        # Read policy and time to generate ur command
        if policy == "move_ee":
            if use_timeout:
                self.ur_controller.move_ee(goal_ee_pose, time=timeout)
            else:
                self.ur_controller.move_ee(goal_ee_pose)
        elif policy == "move_joints":
            if use_timeout:
                self.ur_controller.move_joints_in_radians(goal_joint_states, time=timeout)
            else:
                self.ur_controller.move_joints_in_radians(goal_joint_states)
        else:
            raise NotImplementedError
        
        start_state = self.joint_states
        end_state = goal_joint_states
        result = GotoControl.Result()
        t_start = time.time()
        # Loop monitoring time, ee_wrench, and motion progress for exit
        # Publish motion progress as feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                return result
            # Timeout exit condition
            if use_timeout and ((time.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                return result

            curr_state = self.joint_states
            # Mean percent completed accross all joints
            completion_percent = 100.0 * (1/6) * sum((curr_state-start_state)**2/(end_state-start_state)**2)
            feedback_msg = GotoControl.Feedback()

            # Move complete exit condition
            if completion_percent > self.motion_completion_thresh:
                result.result = "Motion complete"
                break
            feedback_msg.feedback = completion_percent
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        return result
    
    def tcp_twist_goal_callback(self, goal_request):
        self.get_logger().info('Received (tcp_twist) goal request')
        self.tcp_twist_goal = goal_request
        return GoalResponse.ACCEPT
    
    def tcp_twist_execute_callback(self, goal_handle):
        # Displace the tcp by a given twist in the world coordinate frame
        tcp_twist_msg = self.tcp_twist_goal.goal # type: TwistGoal
        displacement = tcp_twist_msg.displacement
        axis = tcp_twist_msg.axis
        angle = tcp_twist_msg.angle
        policy = tcp_twist_msg.policy
        wrench_thresh = tcp_twist_msg.wrench_thresh
        use_ft = tcp_twist_msg.use_ft

        ee_pos = self.get_ee_pose()

        if policy == "move":
            new_ee_pos = [ee_pos[0] + displacement[0],
                          ee_pos[1] + displacement[1],
                          ee_pos[2] + displacement[2],
                          ee_pos[3], ee_pos[4], ee_pos[5]]
            self.ur_controller.move_ee(new_ee_pos)
        elif policy == "rotate":
            axis_angle = [ee_pos[3], ee_pos[4], ee_pos[5]]
            final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)
            new_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0], final_axis_angle[1], final_axis_angle[2]]
            self.ur_controller.move_ee(new_ee_pos)
        else:
            raise NotImplementedError
        
        start_state = ee_pos
        end_state = new_ee_pos
        result = GotoControl.Result()
        # Loop monitoring ee_wrench and motion progress for exit
        # Publish motion progress as feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                return result

            curr_state = self.joint_states
            # Mean percent completed accross all joints
            completion_percent = 100.0 * (1/6) * sum((curr_state-start_state)**2/(end_state-start_state)**2)
            feedback_msg = GotoControl.Feedback()

            # Move complete exit condition
            if completion_percent > self.motion_completion_thresh:
                result.result = "Motion complete"
                break
            feedback_msg.feedback = completion_percent
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    lb_control_node = LegoBuilderControlNode()

    rclpy.spin(lb_control_node)

    lb_control_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
