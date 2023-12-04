import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench, Twist
from legobuilder_interfaces.msg import GotoGoal, TwistGoal
from legobuilder_interfaces.action import BasicControl, GotoControl, TCPTwistControl

from legobuilder_control import utils

class LegoBuilderPlannerNode(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self.basic_action_client = ActionClient(self, BasicControl, 'basic_control')
        self.goto_control_client = ActionClient(self, GotoControl, 'goto_control')
        self.tcp_twist_control_client = ActionClient(self, TCPTwistControl, 'tcp_twist_control')

        # Transform buffer subscriber
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_buffer_sub = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

    def get_ee_pose(self):
        trans = self.tf_buffer.lookup_transform('base', 'tool0_controller', rclpy.Time())

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    ### Basic Action Client ###
    def basic_send_goal(self, goal : str):
        goal_msg = BasicControl.Goal()
        goal_msg.goal = goal

        self.basic_action_client.wait_for_server()

        return self.basic_action_client.send_goal_async(goal_msg)

    def basic_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal (basic) rejected')
            return
        
        self.get_logger().info('Goal (basic) accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_basic_result_callback)

    def basic_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

    # def basic_feedback_callback(self, feedback_msg):
    #   expect no feedback

    ### Goto Action Client ###
    def goto_send_goal(self, goal : GotoGoal):
        goal_msg = GotoControl.Goal()
        goal_msg.goal = goal
        
        self.goto_action_client.wait_for_server()

        return self.goto_action_client.send_goal_async(goal_msg)

    def goto_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal (basic) rejected')
            return
        
        self.get_logger().info('Goal (basic) accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_basic_result_callback)
      

    def goto_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

    def goto_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


    ### TCP Twist Action Client ###
    def tcp_twist_send_goal(self, goal : TwistGoal):
        goal_msg = GotoControl.Goal()
        goal_msg.goal = goal
        
        self.goto_action_client.wait_for_server()

        return self.goto_action_client.send_goal_async(goal_msg)

    def tcp_twist_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal (basic) rejected')
            return
        
        self.get_logger().info('Goal (basic) accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_basic_result_callback)
      

    def tcp_twist_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

    def tcp_twist_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    ### High level commands ###
    def home(self):
        self.get_logger().info(f'Homing robot')
        future = self.basic_send_goal("home_arm")
        rclpy.spin_until_future_complete(self, future)
        #future = self.basic_send_goal("home_ee")
        #rclpy.spin_until_future_complete(future)

    def freedrive(self):
        self.get_logger().info(f'Enabling freedrive')
        future = self.basic_send_goal("freedrive")
        rclpy.spin_until_future_complete(self, future)

    def goto_pose(self, ee_pose : list[float], wrench_thresh=None, timeout=None):
        self.get_logger().info(f'Goto {ee_pose}')
        goal = GotoGoal()
        goal.ee_pose = ee_pose
        goal.joint_states = JointState()
        goal.wrench_thresh = Wrench()
        goal.use_ft = False
        goal.timeout = 0
        goal.use_ft = False
        if wrench_thresh is not None:
            goal.use_ft = True
            goal.wrench_thresh = wrench_thresh
        if timeout is not None:
            goal.timeout = timeout
            goal.use_timeout = True
        future = self.goto_send_goal(goal)
        rclpy.spin_until_future_complete(self, future)

    def rotate_TCP(self, axis, angle):
        self.get_logger().info(f'Rotate {axis}, {angle}')
        goal = TwistGoal()
        goal.policy = "rotate"
        goal.axis = axis
        goal.angle = angle
        future = self.tcp_twist_send_goal(goal)
        rclpy.spin_until_future_complete(self, future)


def demo1(args=None):
    BRICK_HEIGHT = 0.096
    num_trials = 10
    # directory_path = 'path/to/datastore'
    rclpy.init(args=args)

    lb_planner_node = LegoBuilderPlannerNode()

    # basic: set_tcp
    # basic: home_arm
    lb_planner_node.home()
    # basic: freedrive
    lb_planner_node.freedrive()
    # ioctl: wait for human to move robot to origin blocks
    print("Freedrive enabled, move end effector to registration brick")
    _ = input("Press 'Enter' to continue")
    # state sub: record registration pose
    registration_pose = lb_planner_node.get_ee_pose()
    block_pose = registration_pose
    # override without real freedrive
    block_pose = [0.12859122581391358, -0.5526477938788351, 0.015143098090133963, -2.9024186136539276, 1.2023166270087013, -0.000011600763336924778]
    # goto: move to raised position
    waypoint_1 = registration_pose + [0, 0, 5*BRICK_HEIGHT, 0, 0, 0]
    lb_planner_node.goto_pose(waypoint_1)
    
    # for trial in num_trials:
    for trial in num_trials:
        # perception srv: get lego bboxes
        # planning...
        # goto: move for pick
        lb_planner_node.goto_pose(block_pose)
        # brickpick srv: set moment plate
        # basic: set_tcp
        # tcp twist: break for pick
        lb_planner_node.rotate_TCP(
            axis=[0, 1, 0],
            angle=30
        )
        # goto: withdraw for pick
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


    future = lb_planner_node.send_basic_goal('home')
    rclpy.spin_until_future_complete(lb_planner_node, future)





if __name__ == '__main__':
    demo1()