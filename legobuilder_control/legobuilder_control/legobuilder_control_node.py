#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.time

import tf2_ros
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped, Wrench, Twist
from sensor_msgs.msg import JointState

# from legobuilder_interfaces.msg import 
from legobuilder_interfaces.srv import BrickpickCommand
from legobuilder_interfaces.action import LegobuilderCommand

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import time as T
import threading
import signal
import sys
import argparse
from legobuilder_control.Controllers import URController, BrickPickController, utils


clicks = []
tool_height = 0.08
block_height = 0.0096
block_width = 0.0158
block_length = 0.0318

class LegoBuilderControlNode(Node):
    def __init__(self):
        super().__init__('Legobuilder_control_node') # type: ignore
        self.motion_completion_thresh = 99.5
        self.motion_tolerance_thresh_m = 0.0005 # m
        self.motion_tolerance_thresh_rad = 0.001

        # State Estimation
        self.joint_states = JointState()
        self.ee_wrench = WrenchStamped()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=1))

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
            self,
        )
        '''
        # Realsense subscriber
        self.ee_camera_sub = self.create_client(
            ???, 
            "???"
        )
        '''

        ### Action Servers ###
        # Control actions
        self.control_goal = LegobuilderCommand.Goal()
        self.basic_action_server = ActionServer(
            self,
            LegobuilderCommand,
            'legobuilder_command',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup()
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
        trans = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())
        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    def get_joint_positions(self):
        joint_states = self.joint_states
        D = dict()
        for i in range(len(joint_states.name)):
            D[joint_states.name[i]] = joint_states.position[i]
        joint_positions = [
            D['shoulder_pan_joint'], D['shoulder_lift_joint'], D['elbow_joint'],\
            D['wrist_1_joint'], D['wrist_2_joint'], D['wrist_3_joint']
        ]
        return joint_positions

    def cancel_cb(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def goal_cb(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info(f'Recieved goal request: ???')
        self.control_goal = goal_request
        return GoalResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        self.get_logger().info(f'Executing control goal...')
        goal = self.control_goal
        cmd = goal.cmd
        if cmd == "enable_freedrive":
            self.ur_controller.enable_freedrive()
            result = LegobuilderCommand.Result()
            goal_handle.succeed()
            result.result = "Complete"
        elif cmd == "disable_freedrive":
            self.ur_controller.disable_freedrive()
            result = LegobuilderCommand.Result()
            goal_handle.succeed()
            result.result = "Complete"
        elif cmd == "stop":
            self.ur_controller.stop()
            self.bp_controller.stop()
            result = LegobuilderCommand.Result()
            goal_handle.succeed()
            result.result = "Complete"
        elif cmd == "home":
            # self.bp_controller.reset()
            home_joint_angles = [90.0, -90.0, 90.0, -90.0, -90.0, -135.0]
            self.control_goal.joint_positions = home_joint_angles
            self.control_goal.use_time = True
            self.control_goal.time = 10.0
            self.control_goal.use_ft = False
            result = self.goto_joints_deg_execution(goal_handle)
        elif cmd == "activate_FT":
            task_frame=None
            selection_vector=None
            wrench=None
            limits=None
            time=None
            self.ur_controller.force_mode(task_frame, selection_vector, wrench, limits, time)
            goal_handle.succeed()
            result = LegobuilderCommand.Result()
            result.result = "Complete"
        elif cmd == "deactivate_FT":
            self.ur_controller.end_force_mode()
            goal_handle.succeed()
            result = LegobuilderCommand.Result()
            result.result = "Complete"
        elif cmd == "zero_FT":
            self.ur_controller.zero_ft_sensor()
            goal_handle.succeed()
            result = LegobuilderCommand.Result()
            result.result = "Complete"
        elif cmd == "goto_joints_deg":
            result = self.goto_joints_deg_execution(goal_handle)
        elif cmd == "goto_TCP":
            result = self.goto_TCP_execution(goal_handle)
        elif cmd == "rotate_TCP":
            result = self.rotate_TCP_execution(goal_handle)
        elif cmd == "move_TCP":
            result = self.move_TCP_execution(goal_handle)
        else:
            raise NotImplementedError
        return result

    def goto_joints_deg_execution(self, goal_handle):
        self.get_logger().info("Executing 'goto_joints_deg' control goal...")
        # Read goal parameters
        goal_msg = self.control_goal # type: LegobuilderCommand.Goal
        goal_joint_positions_deg = goal_msg.joint_positions
        time = goal_msg.time
        wrench_thresh = goal_msg.wrench_thresh
        use_time = goal_msg.use_time
        use_ft = goal_msg.use_ft
        
        # Publish UR move script
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_joints_in_degrees(goal_joint_positions_deg, time=time)
        else:
            self.ur_controller.move_joints_in_degrees(goal_joint_positions_deg)
        
        start_state = np.asarray(self.get_joint_positions())
        end_state = np.asarray(goal_joint_positions_deg) * np.pi/180
        start_dist = np.sqrt(np.sum((start_state[0:4] - end_state[0:4])**2)) + 10E-7

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                break
            # Timeout exit condition
            if use_time and ((T.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                break

            curr_state = np.asarray(self.get_joint_positions())
            curr_dist = np.sqrt(np.sum((curr_state[0:4] - end_state[0:4])**2))
            completion_percent = 100.0 * (start_dist - curr_dist)/start_dist

            # Move complete exit condition
            if curr_dist < self.motion_tolerance_thresh_rad:
                result.result = "Motion complete"
                break

            feedback_msg = LegobuilderCommand.Feedback()
            feedback_msg.completion_percent = completion_percent
            goal_handle.publish_feedback(feedback_msg)

            T.sleep(0.1)
        goal_handle.succeed()
        return result

    def goto_TCP_execution(self, goal_handle):
        self.get_logger().info("Executing 'goto_TCP' control goal...")
        # Read goal parameters
        goal_msg = self.control_goal # type: LegobuilderCommand.Goal
        goal_ee_pose = goal_msg.ee_pose
        time = goal_msg.time
        wrench_thresh = goal_msg.wrench_thresh
        use_time = goal_msg.use_time
        use_ft = goal_msg.use_ft
        
        # Publish UR move script
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_ee(goal_ee_pose, time=time)
        else:
            self.ur_controller.move_ee(goal_ee_pose)
        
        start_state = np.asarray(self.get_ee_pose())
        end_state = np.asarray(goal_ee_pose)
        start_dist = np.sqrt(np.sum((start_state[0:3] - end_state[0:3])**2))

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                break
            # Timeout exit condition
            if use_time and ((T.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                break

            curr_state = np.asarray(self.get_ee_pose())
            curr_dist = np.sqrt(np.sum((curr_state[0:3] - end_state[0:3])**2))
            completion_percent = 100.0 * (start_dist - curr_dist) / start_dist

            # Move complete exit condition
            if curr_dist < self.motion_tolerance_thresh_m:
                result.result = "Motion complete"
                break

            feedback_msg = LegobuilderCommand.Feedback()
            feedback_msg.completion_percent = completion_percent
            goal_handle.publish_feedback(feedback_msg)

            T.sleep(0.1)
        goal_handle.succeed()
        return result
    
    def rotate_TCP_execution(self, goal_handle):
        self.get_logger().info("Executing 'rotate_TCP' control goal...")
        # Get goal parameters
        goal_msg = self.control_goal
        axis = goal_msg.axis
        angle = goal_msg.angle
        time = goal_msg.time
        wrench_thresh = goal_msg.wrench_thresh
        use_time = goal_msg.use_time
        use_ft = goal_msg.use_ft

        # Get current pose
        ee_pos = self.get_ee_pose()
        axis_angle = np.asarray([ee_pos[3], ee_pos[4], ee_pos[5]])

        # Compute new pose
        final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)
        goal_ee_pose = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0][0], final_axis_angle[1][0], final_axis_angle[2][0]]

        # Publish UR move script
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_ee(goal_ee_pose, time=time)
        else:
            self.ur_controller.move_ee(goal_ee_pose)
        
        start_state = np.asarray(self.get_ee_pose())
        end_state = np.asarray(goal_ee_pose)
        start_dist = np.sqrt(np.sum((start_state[3:6] - end_state[3:6])**2)) + 10E-7

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                break
            # Timeout exit condition
            if use_time and ((T.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                break

            curr_state = np.asarray(self.get_ee_pose())
            curr_dist = np.sqrt(np.sum((curr_state[3:6] - end_state[3:6])**2))

            self.get_logger().info(f"\n{start_state[3:6]}\n{curr_state[3:6]}\n{end_state[3:6]}\n{curr_dist}\n{start_dist}")
            completion_percent = 100.0 * (start_dist - curr_dist) / start_dist
            # Move complete exit condition
            if curr_dist < self.motion_tolerance_thresh_rad:
                result.result = "Motion complete"
                break

            feedback_msg = LegobuilderCommand.Feedback()
            feedback_msg.completion_percent = completion_percent
            goal_handle.publish_feedback(feedback_msg)

            T.sleep(0.1)
        goal_handle.succeed()
        return result

    def move_TCP_execution(self, goal_handle):
        self.get_logger().info("Executing 'move_TCP' control goal...")
        # Get goal parameters
        goal_msg = self.control_goal
        displacement = goal_msg.displacement
        time = goal_msg.time
        wrench_thresh = goal_msg.wrench_thresh
        use_time = goal_msg.use_time
        use_ft = goal_msg.use_ft

        # Get current pose
        ee_pose = self.get_ee_pose()
        # Compute new pose
        goal_ee_pose = [ee_pose[0] + displacement[0],
                        ee_pose[1] + displacement[1], 
                        ee_pose[2] + displacement[2], 
                        ee_pose[3], ee_pose[4], ee_pose[5]]
        
        start_state = np.asarray(self.get_ee_pose())
        end_state = np.asarray(goal_ee_pose)
        start_dist = np.sqrt(np.sum((start_state[0:3] - end_state[0:3])**2)) + 10E-7

        # Publish UR move script
        self.get_logger().info(f"\n{ee_pose}\n{goal_ee_pose}")
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_ee(goal_ee_pose, time=time)
        else:
            self.ur_controller.move_ee(goal_ee_pose)
    
        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft and (abs(self.ee_wrench) > wrench_thresh):
                self.ur_controller.stop()
                result.result = "Force-Torque limit exceeded"
                break
            # Timeout exit condition
            if use_time and ((T.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                break

            curr_state = np.asarray(self.get_ee_pose())
            curr_dist = np.sqrt(np.sum((curr_state[0:3] - end_state[0:3])**2))
            completion_percent = 100.0 * (start_dist - curr_dist) / start_dist

            feedback_msg = LegobuilderCommand.Feedback()
            feedback_msg.completion_percent = completion_percent
            goal_handle.publish_feedback(feedback_msg)

            # Move complete exit condition
            if curr_dist < self.motion_tolerance_thresh_m:
                result.result = "Motion complete"
                break

            T.sleep(0.1)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    lb_control_node = LegoBuilderControlNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(lb_control_node, executor=executor)

    lb_control_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
