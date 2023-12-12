#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped, Pose
from sensor_msgs.msg import JointState

from legobuilder_interfaces.srv import BrickpickCommand
from legobuilder_interfaces.action import LegobuilderCommand

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import time as T

from legobuilder_control.Controllers import URController, BrickPickController
from legobuilder_control import utils


clicks = []
tool_height = 0.08
block_height = 0.0096
block_width = 0.0158
block_length = 0.0318

class LegoBuilderControlNode(Node):
    def __init__(self):
        super().__init__('Legobuilder_control_node') # type: ignore
        self.motion_tolerance_thresh_m = 0.0005 # m
        self.motion_tolerance_thresh_rad = 0.001 # rad
        
        # The TCP offset from the tool0 (flange) pose
        # This is due to the fact that altering the TCP using the teach pad or ur script interface does not change
        # the tool0 transform and the tool0_controller frame does not exist with my current ur_robot_driver version (humble)
        self.tcp_pose = Pose()

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
            cancel_callback=self.cancel_cb
        )

        # Actuation
        self.bp_controller = BrickPickController(self.brickpick_adapter_cli)
        self.ur_controller = URController(self.ur_driver_pub)

    
    def recv_ur_wrench(self, wrench : WrenchStamped):
        self.ee_wrench = wrench
    
    def recv_ur_joint_states(self, joint_states : JointState):
        self.joint_states = joint_states
    
    def get_ee_pose(self):
        # TODO - add support for angled TCPs but not super necessary
        tool0_trans = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())
        
        axis_angle = utils.quat_to_axis_angle([tool0_trans.transform.rotation.x, tool0_trans.transform.rotation.y,
                                                tool0_trans.transform.rotation.z, tool0_trans.transform.rotation.w])

        tcp_pose = tf2_geometry_msgs.do_transform_pose(self.tcp_pose, tool0_trans)
        tcp_pose = [tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z,
                      axis_angle[0], axis_angle[1], axis_angle[2]]
        return tcp_pose

    def get_joint_positions(self):
        joint_states = self.joint_states
        D = dict()
        for i in range(len(joint_states.name)):
            D[joint_states.name[i]] = joint_states.position[i] # type: ignore
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

    def execute_cb(self, goal_handle):
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
        elif cmd == "set_TCP":
            goal_msg = self.control_goal
            tcp_pose = goal_msg.ee_pose

            # Set the internal tcp PoseStamped
            self.tcp_pose.position.x = tcp_pose[0]
            self.tcp_pose.position.y = tcp_pose[1]
            self.tcp_pose.position.z = tcp_pose[2]
            quat = utils.axis_angle_to_quaternion([tcp_pose[3], tcp_pose[4], tcp_pose[5]])
            self.tcp_pose.orientation.x = quat[0]
            self.tcp_pose.orientation.y = quat[1]
            self.tcp_pose.orientation.z = quat[2]
            self.tcp_pose.orientation.w = quat[3]

            # Set the TCP on the ur system
            self.ur_controller.set_tcp(tcp_pose)
            
            goal_handle.succeed()
            result = LegobuilderCommand.Result()
            result.result = "Complete"
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
            timeout = 30.0
            self.ur_controller.move_joints_in_degrees(goal_joint_positions_deg)
        
        start_state = np.asarray(self.get_joint_positions())
        end_state = np.asarray(goal_joint_positions_deg) * np.pi/180
        start_dist = np.sqrt(np.sum((start_state - end_state)**2)) + 10E-7

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft : # and self.ee_wrench > wrench_thresh
                ee_wrench =[
                    self.ee_wrench.wrench.force.x, 
                    self.ee_wrench.wrench.force.y,
                    self.ee_wrench.wrench.force.z,
                    self.ee_wrench.wrench.torque.x,
                    self.ee_wrench.wrench.torque.y,
                    self.ee_wrench.wrench.torque.z,
                ]
                overloaded_axis = -1
                for i in range(len(ee_wrench)):
                    if ee_wrench[i] > wrench_thresh[i]:
                        overloaded_axis = i
                        break
                if overloaded_axis >= 0:
                    self.ur_controller.stop()
                    idx2ax = {0: "Fx", 1: "Fy", 2: "Fz", 3: "Rx", 4: "Ry", 5: "Rz"}
                    result.result = f"Force-Torque limit exceeded: {idx2ax[overloaded_axis]} @ {ee_wrench[overloaded_axis]:0.2f}"
                    break
            # Timeout exit condition
            if use_time and ((T.time() - t_start) > timeout):
                self.ur_controller.stop()
                result.result = f"Move timed out ({timeout:0.1f} s)"
                break

            curr_state = np.asarray(self.get_joint_positions())
            curr_dist = np.sqrt(np.sum((curr_state - end_state)**2))
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
            timeout = 30.0
            self.ur_controller.move_ee(goal_ee_pose)
        
        start_state = np.asarray(self.get_ee_pose())
        end_state = np.asarray(goal_ee_pose)
        start_dist = np.sqrt(np.sum((start_state[0:3] - end_state[0:3])**2))

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft : # and self.ee_wrench > wrench_thresh
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
        goal_msg = self.control_goal # type: LegobuilderCommand.Goal
        axis = goal_msg.axis
        angle = goal_msg.angle
        time = goal_msg.time
        wrench_thresh = goal_msg.wrench_thresh
        use_time = goal_msg.use_time
        use_ft = goal_msg.use_ft

        # Get current pose
        ee_pose = self.get_ee_pose()
        axis_angle = [ee_pose[3], ee_pose[4], ee_pose[5]]

        # Compute new pose
        final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)
        goal_ee_pose = [ee_pose[0], ee_pose[1], ee_pose[2], final_axis_angle[0][0], final_axis_angle[1][0], final_axis_angle[2][0]]

        # Publish UR move script
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_ee(goal_ee_pose, time=time)
        else:
            timeout = 30.0
            self.ur_controller.move_ee(goal_ee_pose)
        
        start_state = np.asarray(self.get_ee_pose())
        end_state = np.asarray(goal_ee_pose)
        start_dist = np.sqrt(np.sum((start_state[3:6] - end_state[3:6])**2)) + 10E-7

        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft : # and self.ee_wrench > wrench_thresh
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
        if use_time:
            timeout = 1.5 * time
            self.ur_controller.move_ee(goal_ee_pose, time=time)
        else:
            timeout = 30.0
            self.ur_controller.move_ee(goal_ee_pose)
    
        result = LegobuilderCommand.Result()
        t_start = T.time()
        # Loop and spin node for feedback
        while (rclpy.ok()):
            # Force-Torque exit condition
            if use_ft : # and self.ee_wrench > wrench_thresh
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
