import rclpy
import rclpy.time
import time as T
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.node import Node

import numpy as np
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench, Twist
from legobuilder_interfaces.action import LegobuilderCommand

from legobuilder_control import utils

class LegoBuilderPlannerNode(Node):

    def __init__(self):
        super().__init__('legobuilder_planner_node')
        self.lb_control_cli = ActionClient(self, LegobuilderCommand, 'legobuilder_command')
        self.lb_control_send_goal_future = None # type: Future
        self.lb_control_get_result_future = None # type: Future


        # Transform buffer subscriber
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_buffer_sub = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

    def get_ee_pose(self):
        trans = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())

        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    ### Goto Action Client ###
    def lb_control_send_goal(self, goal_msg : LegobuilderCommand.Goal):
        self.lb_control_goal_terminated = False
        
        self.lb_control_cli.wait_for_server()

        self.goto_send_goal_future = self.lb_control_cli.send_goal_async(
            goal_msg, 
            feedback_callback=self.lb_control_feedback_cb
        )        
        self.goto_send_goal_future.add_done_callback(self.lb_control_goal_response_cb)


    def lb_control_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal (Goto) rejected')
            return

        self.get_logger().info('Goal (Goto) accepted')

        self.goto_get_result_future = goal_handle.get_result_async()
        self.goto_get_result_future.add_done_callback(self.lb_control_get_result_cb)

    def lb_control_get_result_cb(self, future):
        self.lb_control_goal_terminated = True
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

    def lb_control_feedback_cb(self, feedback_msg : LegobuilderCommand.Feedback):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Completion percent: {feedback.completion_percent:0.2f}%')

    ### High level commands ###
    def home(self):
        self.get_logger().info(f'Homing robot')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "home"
        self.lb_control_send_goal(goal)
        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def enable_freedrive(self):
        self.get_logger().info(f'Enabling freedrive')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "enable_freedrive"
        self.lb_control_send_goal(goal)
        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def disable_freedrive(self):
        self.get_logger().info(f'Disabling freedrive')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "disable_freedrive"
        self.lb_control_send_goal(goal)
        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def goto_joints_deg(self, joint_positions_deg : list[float], wrench_thresh=None, time=None):
        self.get_logger().info(f"Goto {[f'{val:0.2f}' for val in joint_positions_deg]}")
        goal = LegobuilderCommand.Goal()
        goal.cmd = 'goto_joints_deg'
        goal.joint_positions = joint_positions_deg
        goal.wrench_thresh = Wrench()
        goal.use_ft = False
        goal.time = 0.0
        goal.use_time = False

        if wrench_thresh is not None:
            goal.use_ft = True
            goal.wrench_thresh = wrench_thresh
        if time is not None:
            goal.time = time
            goal.use_time = True
        self.lb_control_send_goal(goal)

        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def goto_TCP(self, ee_pose : list[float], wrench_thresh=None, time=None):
        self.get_logger().info(f"Goto {[f'{val:0.2f}' for val in ee_pose]}")
        goal = LegobuilderCommand.Goal()
        goal.cmd = 'goto_TCP'
        goal.ee_pose = ee_pose
        goal.wrench_thresh = Wrench()
        goal.use_ft = False
        goal.time = 0.0
        goal.use_time = False

        if wrench_thresh is not None:
            goal.use_ft = True
            goal.wrench_thresh = wrench_thresh
        if time is not None:
            goal.time = time
            goal.use_time = True
        self.lb_control_send_goal(goal)

        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def rotate_TCP_deg(self, axis : list[float], angle_deg : float, wrench_thresh=None, time=None):
        self.get_logger().info(f'Rotate {axis}, {angle_deg}')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "rotate_TCP"
        goal.axis = axis
        goal.angle = angle_deg * (np.pi/180)
        
        if wrench_thresh is not None:
            goal.use_ft = True
            goal.wrench_thresh = wrench_thresh
        if time is not None:
            goal.time = time
            goal.use_time = True

        self.lb_control_send_goal(goal)

        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def move_TCP(self, displacement : list[float], wrench_thresh=None, time=None):
        # in meters
        self.get_logger().info(f'Move {displacement}')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "move_TCP"
        goal.displacement = displacement

        if wrench_thresh is not None:
            goal.use_ft = True
            goal.wrench_thresh = wrench_thresh
        if time is not None:
            goal.time = time
            goal.use_time = True

        self.lb_control_send_goal(goal)

        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)


def demo1(args=None):
    BRICK_HEIGHT = 0.0096
    registration_pose_preset = [0.15, -0.50, 0.04, \
                                -2.9, 1.2, -0.0]
    num_trials = 5
    # directory_path = 'path/to/datastore'
    rclpy.init(args=args)

    lb_planner_node = LegoBuilderPlannerNode()

    # Set TCP
    # TODO - implement set_TCP

    # Home system
    lb_planner_node.home()

    # Freedrive to registration pose
    # TODO - Improve freedrive
    lb_planner_node.enable_freedrive()
    print("Freedrive enabled, move end effector to registration brick")
    lb_planner_node.goto_TCP(
        registration_pose_preset,
        wrench_thresh=None,
        time=10.0
    )
    _ = input("Press 'Enter' to continue")
    lb_planner_node.disable_freedrive()
    T.sleep(1)

    # Record registration pose
    registration_pose = lb_planner_node.get_ee_pose()
    block_pose = registration_pose
    # Move to raised position
    waypoint_1 = [block_pose[0], block_pose[1], block_pose[2] + 5*BRICK_HEIGHT,
                  block_pose[3], block_pose[4], block_pose[5]]
    lb_planner_node.goto_TCP(waypoint_1)
    T.sleep(1)
    
    # for trial in num_trials:
    for trial in range(num_trials):
        print(F"### Beginning trial {trial} ###")
        # get lego bboxes
        # planning...
        
        # Move to pick location
        lb_planner_node.goto_TCP(block_pose)
        T.sleep(1)
        # brickpick srv: set moment plate
        # basic: set_tcp
        # Rotate TCP to break
        lb_planner_node.rotate_TCP_deg(
            axis=[1.0, 0.0, 0.0],
            angle_deg=-30.0,
            time=5.0
        )
        # Raise TCP to pick
        lb_planner_node.move_TCP(
            displacement=[0.0, 0.0, 0.0200],
            time=5.0
        )
        lb_planner_node.goto_TCP(waypoint_1, time=2.0)
        T.sleep(5)
        # brickpick srv: withdraw for pick

        # perception srv: get lego bboxes
        # planning...
        # goto: move for place
        # brickpick srv: set moment plate
        # basic: set tcp
        # tcp twist: break for place
        # goto: withdraw for place
        # brickpick srv: withdraw for place

        # save imgs and anlyize...
    # basic: home_arm
    # basic: home_ee


    lb_planner_node.home()





if __name__ == '__main__':
    demo1()