import rclpy
import rclpy.time
import time as T
from rclpy.action import ActionClient
from rclpy.node import Node

import numpy as np
import cv2
import cv_bridge
import tf2_ros
import threading

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Image

from legobuilder_interfaces.action import LegobuilderCommand
from legobuilder_interfaces.srv import BrickpickCommand

from legobuilder_control import utils
from legobuilder_control.Controllers import BrickPickController

# Constants
STUD_HEIGHT = 0.0096 # m
STUD_WIDTH = 0.008 # m
TCP_BASE = np.asarray([-2 * STUD_WIDTH, 1 * STUD_WIDTH, 0.140, 0, 0, 0])
REGISTRATION_POSE_PRESET = [0.15, -0.50, 0.04, -2.9, 1.2, -0.0]

class LegoBuilderPlannerNode(Node):

    def __init__(self):
        super().__init__("legobuilder_planner_node") # type: ignore
        self.bridge = cv_bridge.CvBridge()
        self.ws_img = Image()
        self.ee_img = Image()

        self.lb_control_cli = ActionClient(self, LegobuilderCommand, 'legobuilder_command')

        self.bp_control_cli = self.create_client(
            BrickpickCommand,
            '/brickpick_command'
        )
        self.bp_controller = BrickPickController(self.bp_control_cli)

        # Transform buffer subscriber
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_buffer_sub = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.workspace_img_sub = self.create_subscription(
            Image,
            '/workspace_img',
            self.recv_ws_img_cb,
            10
        )

        self.ee_img_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.recv_ee_img_cb,
            10  # QoS profile, adjust as needed
        )

    def recv_ws_img_cb(self, img : Image):
        self.ws_img = img

    def recv_ee_img_cb(self, img : Image):
        self.ee_img = img

    def get_ee_pose(self):
        trans = self.tf_buffer.lookup_transform('base', 'tool0', rclpy.time.Time())

        axis_angle = utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
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
        feedback = feedback_msg.feedback # type: ignore
        self.get_logger().info(f'Completion percent: {feedback.completion_percent:0.2f}%')

    ### High level commands ###
    def home(self):
        self.get_logger().info(f'Homing robot')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "home"
        self.lb_control_send_goal(goal)
        while not self.lb_control_goal_terminated:
            rclpy.spin_once(self)

    def set_TCP(self, tcp_pose : list[float]):
        self.get_logger().info(f"Setting UR TCP to {[f'{val:0.2f}' for val in tcp_pose]}")
        goal = LegobuilderCommand.Goal()
        goal.cmd = "set_TCP"
        goal.ee_pose = tcp_pose
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
        goal.wrench_thresh = []
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
        goal.wrench_thresh = []
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
        goal.wrench_thresh = []
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

    def move_TCP(self, displacement : list[float], wrench_thresh=None, time=None):
        # in meters
        self.get_logger().info(f'Move {displacement}')
        goal = LegobuilderCommand.Goal()
        goal.cmd = "move_TCP"
        goal.displacement = displacement
        goal.wrench_thresh = []
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

    ### Composite Commands ###
    # approach() : approach a target pose along a given normal vector
    #   Call this from somewhere near/reasonable to generate a trajectory and it should:
    #   1. rapid to a point {approach buffer} along the normal away from the approached surface
    #   2. slowly engage the studs of the approached surface
    #       2.a FT limiting?
    #       2.b servoing motion?
    #       2.c Remote Center of Compliance?
    #   3. stop and release majority of pressure
    def approach(self, target_pose : list[float], engage=True, jog_height=0.300):
        '''
        Approach a target pose by motion to a jog plane, motion accross a jog plane, and a path terminating tangent to the final orientation

        
        target_pose : Pose to approach to
        engage : Whether or not to engage the studs on the brick or hover over
        jog_height : Height of the jog plane in m
        '''
        # Move directly up to jog plane
        jog_pose = self.get_ee_pose()
        jog_pose[2] = jog_height
        self.goto_TCP(jog_pose)

        # Move directly over target pose
        hover_pose = target_pose[:]
        hover_pose[2] = jog_height
        self.goto_TCP(hover_pose)

        # Approach the brick at the target pose
        approach_pose = target_pose[:]
        approach_pose[2] += STUD_HEIGHT
        self.goto_TCP(approach_pose)

        # Engage the brick at the target pose
        if engage:
            self.goto_TCP(target_pose, time=2.0)


    def pick(self, brick : int, direction='long'):
        '''
        Break away the end effector brick payload ending at a given brick [stud height] from the base
        
        brick : Set the TCP to brick * STUD_HEIGHT from the base
        direction : Specify moment plate / axis of rotation for removal

        TODO - Add a normal vector arg for pullback
        TODO - Axis of rotation should be in EE coordinates
        '''
        BRICK_MAX = 5
        ROTATION_ANGLE_DEG = 30.0
        RELEASE_VEC = [0.0, 0.0, STUD_HEIGHT]

        brick = min(max(brick, 1), BRICK_MAX)

        # Set the TCP to the brick corner
        TCP_offset = np.asarray([0.0, 0.0, brick * STUD_HEIGHT, 0.0, 0.0, 0.0])
        tcp_pose = (TCP_BASE + TCP_offset).tolist()
        self.set_TCP(tcp_pose)

        # Deploy the BrickPick moment plate
        if direction == 'long':
            result_future = self.bp_controller.long_goto_brick(brick)
            rclpy.spin_until_future_complete(self, result_future)
        elif direction == 'short':
            result_future = self.bp_controller.short_goto_brick(brick)
            rclpy.spin_until_future_complete(self, result_future)
        else:
            raise NotImplementedError
        
        # Rotate about the axis direction 
        if direction == 'long':
            axis = [-1.0, 0.0, 0.0]
        elif direction == 'short':
            axis = [0.0, -1.0, 0.0]
        else:
            raise NotImplementedError
        self.rotate_TCP_deg(axis, ROTATION_ANGLE_DEG)

        # Pull up to complete pick
        self.move_TCP(RELEASE_VEC)

        # Re-zero moment plates
        if direction == 'long':
            result_future = self.bp_controller.set_long_target_mm(0.0)
            rclpy.spin_until_future_complete(self, result_future)
        elif direction == 'short':
            result_future = self.bp_controller.set_short_target_mm(0.0)
            rclpy.spin_until_future_complete(self, result_future)
        else:
            raise NotImplementedError

    def deposit(self):
        '''
        Deposit the full end effector payload
        '''

        plunger_future = self.bp_controller.plunger_down()
        self.move_TCP([0.0, 0.0, STUD_HEIGHT])
        rclpy.spin_until_future_complete(self, plunger_future)
        plunger_future = self.bp_controller.plunger_up()
        rclpy.spin_until_future_complete(self, plunger_future)

    ### Perception ###
    def display_ws_img(self, time_ms=5000, title='Workspace'):
        cv_image = self.bridge.imgmsg_to_cv2(self.ws_img, desired_encoding='bgr8')
        # Display the image
        cv2.imshow(title, cv_image)
        cv2.waitKey(time_ms)
        cv2.destroyAllWindows()

    def display_ee_img(self, time_ms=5000, title='End Effector'):
        cv_image = self.bridge.imgmsg_to_cv2(self.ee_img, desired_encoding='bgr8')
        # Display the image
        cv2.imshow(title, cv_image)
        cv2.waitKey(time_ms)
        cv2.destroyAllWindows()

def demo1(args=None):
    # args
    num_trials = 5
    # directory_path = 'path/to/datastore'

    # ROS
    rclpy.init(args=args)
    lb_planner_node = LegoBuilderPlannerNode()

    # Set TCP
    TCP_offset = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tcp_pose = (TCP_BASE + TCP_offset).tolist()
    lb_planner_node.set_TCP(tcp_pose)

    # Home system
    lb_planner_node.home()

    # Freedrive to registration pose
    lb_planner_node.enable_freedrive()
    print("Freedrive enabled, move end effector to registration brick")
    lb_planner_node.goto_TCP(
        REGISTRATION_POSE_PRESET,
        wrench_thresh=None,
        time=10.0
    )
    lb_planner_node.display_ws_img()
    _ = input("Press 'Enter' to continue")
    lb_planner_node.disable_freedrive()
    T.sleep(1)

    # Record registration pose
    registration_pose = lb_planner_node.get_ee_pose()
    block_pose = registration_pose[:] # clone

    # Move to safe pose
    waypoint_1 = [block_pose[0], block_pose[1], block_pose[2] + 10*STUD_HEIGHT,
                  block_pose[3], block_pose[4], block_pose[5]]
    lb_planner_node.goto_TCP(waypoint_1, time=5.0)
    
    # Execute the skills in each trial
    for trial in range(num_trials):
        brick = trial + 1
        # direction = 'long' if trial % 2 == 0 else 'short'
        direction = 'long'

        print(F"### Beginning trial {trial} ###")
        T.sleep(5)

        # Anylize workspace
        # detections = lb_planner_node.get_ws_detections()
        lb_planner_node.display_ws_img()
        # planning...
        
        lb_planner_node.approach(block_pose)
        T.sleep(1)
        lb_planner_node.pick(brick, direction=direction)
        T.sleep(1)
        lb_planner_node.goto_TCP(waypoint_1, time=2.0)
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