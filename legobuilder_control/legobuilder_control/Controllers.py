#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Client

from legobuilder_interfaces.srv import BrickpickCommand

import tf2_ros
import numpy as np
import time as T
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import legobuilder_control.utils as utils

class BrickPickController:
    def __init__(self, brickpick_adapter_cli : Client):
        self.brickpick_adapter_cli = brickpick_adapter_cli

        self.u_max_long = 65535
        self.u_max_short = 65535
        self.u_max_plunger = 1023

    # Non-Blocking services
    def set_long_ctrl(self, u : float) -> str:
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_long)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_long_ctrl'
        command_request.u = u
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status

    def set_short_ctrl(self, u : float) -> str:
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_short)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_short_ctrl'
        command_request.u = u
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status
    
    def set_plunger_ctrl(self, u : float) -> str:
        # TODO - Add set_plunger_ctrl to brickpick embedded
        raise NotImplementedError
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_plunger)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_plunger_ctrl'
        command_request.u = u
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status

    def stop(self) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/stop'
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status
    
    def set_long_target_mm(self, target_mm : float) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_long_target_mm'
        command_request.target_mm = target_mm
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status

    def set_short_target_mm(self, target_mm : float) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_short_target_mm'
        command_request.target_mm = target_mm
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status

    # Blocking services
    def plunger_up(self) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_plunger'
        command_request.plunger_target = 0
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status

    def plunger_down(self) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_plunger'
        command_request.plunger_target = 1
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status
    
    def long_goto_brick(self, brick : int) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_long_brick'
        command_request.target_brick = brick
        response = self.brickpick_adapter_cli.call(command_request) # type: BrickpickCommand.Response
        return response.status
    
    def short_goto_brick(self, brick : int) -> str:
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_short_brick'
        command_request.target_brick = brick
        response = self.brickpick_adapter_cli.call(command_request) # type: BrickpickCommand.Response
        return response.status
    
    def reset(self) -> str:
        # TODO - Add a reset command to brickpick embedded
        raise NotImplementedError
        command_request = BrickpickCommand.Request()
        command_request.command = '/reset'
        response = self.brickpick_adapter_cli.call(command_request)  # type: BrickpickCommand.Response
        return response.status
    

class URController:
    def __init__(self, ur_driver_pub):
        self.current_arm_position = [0.0, 0.0, 0.0]

        self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.ur_driver_pub = ur_driver_pub
        T.sleep(1)

    def set_tcp(self, tcp_pose):
        set_tcp_string = String()

        set_tcp_string.data =  'set_tcp(p[' + ','.join(str(element) for element in tcp_pose) + '])'

        self.ur_driver_pub.publish(set_tcp_string)
        self.ur_driver_pub.publish(set_tcp_string)

        T.sleep(1)

        return True

    def home_arm(self, time=None):
        home_joint_angles = [90, -90, 90, -90, -90, -135]

        self.move_joints_in_degrees(home_joint_angles, time)

        return True

    def freedrive(self, axes=[1,1,1,1,1,1], time=10):
        freedrive_mode_string = String()
        
        freedrive_mode_string.data = 'def freedrive_control():\n    zero_ftsensor()\n'+\
                                      '    freedrive_mode(freeAxes=[' + ','.join(str(axis) for axis in axes) + '], feature="tool")\n'+\
                                      '    sleep('+str(time)+')\n    end_freedrive_mode()\nend'

        self.ur_driver_pub.publish(freedrive_mode_string)
        self.ur_driver_pub.publish(freedrive_mode_string)

        T.sleep(time)

        return True

    def zero_ft_sensor(self):
        zero_ft_sensor_string = String()

        zero_ft_sensor_string.data = 'zero_ftsensor()'

        self.ur_driver_pub.publish(zero_ft_sensor_string)
        self.ur_driver_pub.publish(zero_ft_sensor_string)

        T.sleep(1)

        return True

    def end_force_mode(self):
        end_force_mode = String()

        end_force_mode.data = 'end_force_mode()'

        self.ur_driver_pub.publish(end_force_mode)
        self.ur_driver_pub.publish(end_force_mode)

        T.sleep(1)

        return True

    def force_mode(self, task_frame, selection_vector, wrench, limits, time):
        force_mode_string = String()
        
        force_mode_string.data = 'def force_control():\n    zero_ftsensor()\n    force_mode(p[' + ','.join(str(frame) for frame in task_frame) + '],' +\
                                             '[' + ','.join(str(selection) for selection in selection_vector) + '],' + \
                                             '[' + ','.join(str(force) for force in wrench) + '], 2, ' + \
                                             '[' + ','.join(str(limit) for limit in limits) + '])\n    sleep('+str(time)+')\n    end_force_mode()\nend'

        self.ur_driver_pub.publish(force_mode_string)
        self.ur_driver_pub.publish(force_mode_string)

        T.sleep(time)

        return True

    def move_joints_in_degrees(self, desired_arm_angles, time=None, 
                               force_thresholds=None,
                               torque_thresholds=None):

        return self.move_joints_in_radians([(angle * np.pi/180.0) for angle in desired_arm_angles], time, force_thresholds, torque_thresholds)

    def move_joints_in_radians(self, desired_arm_angles, time=None,
                               force_thresholds=None,
                               torque_thresholds=None):
        move_joints_string = String()

        if time is not None:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '],t='+str(time)+')'
        else:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '])'

        self.ur_driver_pub.publish(move_joints_string)
        self.ur_driver_pub.publish(move_joints_string)

        if force_thresholds is not None or torque_thresholds is not None:
            start_time = T.time()
            current_time = T.time()
            while current_time-start_time < time:
                wrench_msg = rospy.wait_for_message('/wrench', WrenchStamped)
                if force_thresholds is not None:
                    if abs(wrench_msg.wrench.force.x) > force_thresholds[0] or \
                       abs(wrench_msg.wrench.force.y) > force_thresholds[1] or \
                       abs(wrench_msg.wrench.force.z) > force_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopj(2)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                if torque_thresholds is not None:
                    if abs(wrench_msg.wrench.torque.x) > torque_thresholds[0] or \
                       abs(wrench_msg.wrench.torque.y) > torque_thresholds[1] or \
                       abs(wrench_msg.wrench.torque.z) > torque_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopj(2)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                current_time = T.time()

        else:
            if time is not None:
                T.sleep(time)
            else:
                T.sleep(1)

        return True

    def move_ee(self, ee_pose, time=None,
                force_thresholds=None,
                torque_thresholds=None):
        move_ee_string = String()

        if time is not None:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '],t='+str(time)+')'
        else:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '])'

        self.ur_driver_pub.publish(move_ee_string)
        self.ur_driver_pub.publish(move_ee_string)

        if force_thresholds is not None or torque_thresholds is not None:
            start_time = T.time()
            current_time = T.time()
            while current_time-start_time < time:
                wrench_msg = rospy.wait_for_message('/wrench', WrenchStamped)
                if force_thresholds is not None:
                    if abs(wrench_msg.wrench.force.x) > force_thresholds[0] or \
                       abs(wrench_msg.wrench.force.y) > force_thresholds[1] or \
                       abs(wrench_msg.wrench.force.z) > force_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopl(20)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                if torque_thresholds is not None:
                    if abs(wrench_msg.wrench.torque.x) > torque_thresholds[0] or \
                       abs(wrench_msg.wrench.torque.y) > torque_thresholds[1] or \
                       abs(wrench_msg.wrench.torque.z) > torque_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopl(20)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                current_time = T.time()

        else:
            if time is not None:
                T.sleep(time)
            else:
                T.sleep(1)

        return True

    def rotate_ee(self, axis, angle, time=None,
                  force_thresholds=None,
                  torque_thresholds=None):

        ee_pos = self.get_ee_pose()

        axis_angle = [ee_pos[3], ee_pos[4], ee_pos[5]]

        final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)

        new_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0], final_axis_angle[1], final_axis_angle[2]]

        move_ee_string = String()

        if time is not None:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in new_ee_pos) + '],t='+str(time)+')'
        else:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in new_ee_pos) + '])'

        self.ur_driver_pub.publish(move_ee_string)
        self.ur_driver_pub.publish(move_ee_string)

        if force_thresholds is not None or torque_thresholds is not None:
            start_time = T.time()
            current_time = T.time()
            while current_time-start_time < time:
                wrench_msg = rospy.wait_for_message('/wrench', WrenchStamped)
                if force_thresholds is not None:
                    if abs(wrench_msg.wrench.force.x) > force_thresholds[0] or \
                       abs(wrench_msg.wrench.force.y) > force_thresholds[1] or \
                       abs(wrench_msg.wrench.force.z) > force_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopl(20)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                if torque_thresholds is not None:
                    if abs(wrench_msg.wrench.torque.x) > torque_thresholds[0] or \
                       abs(wrench_msg.wrench.torque.y) > torque_thresholds[1] or \
                       abs(wrench_msg.wrench.torque.z) > torque_thresholds[2]:

                        stopj_string = String()
                        stopj_string.data = 'stopl(20)'
                        self.ur_driver_pub.publish(stopj_string)
                        self.ur_driver_pub.publish(stopj_string)

                        return False
                current_time = T.time()

        else:
            if time is not None:
                T.sleep(time)
            else:
                T.sleep(1)

        return True


    #Not working
    def rotate_ee_movec(self, axis, angle):

      ee_pos = self.get_ee_pose()

      axis_angle = [ee_pos[3], ee_pos[4], ee_pos[5]]

      final_axis_angle = utils.rotate_around_axis(axis_angle, axis, angle)

      mid_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], 0, 0, 0]

      new_ee_pos = [ee_pos[0], ee_pos[1], ee_pos[2], final_axis_angle[0], final_axis_angle[1], final_axis_angle[2]]

      movec_string = String()

      movec_string.data =  'movec(p[' + ','.join(str(element) for element in mid_ee_pos) + '],'+ \
                                 'p[' + ','.join(str(element) for element in new_ee_pos) + '],a=1.2,v=0.25,r=0.05,mode=0)'
      
      self.ur_driver_pub.publish(movec_string)
      self.ur_driver_pub.publish(movec_string)

      T.sleep(1)

      return True

    def rotate_ee_degrees(self, axis, angle, time=None,
                          force_thresholds=None,
                          torque_thresholds=None):

      return self.rotate_ee(axis, angle / 180 * np.pi, time, force_thresholds, torque_thresholds)

    def get_ee_pose(self):
        rospy.sleep(1)
        trans = self.tf_buffer.lookup_transform('base', 'tool0_controller', rospy.Time(0))

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        axis_angle = -utils.quat_to_axis_angle([trans.transform.rotation.x, trans.transform.rotation.y,
                                                trans.transform.rotation.z, trans.transform.rotation.w])

        return [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                axis_angle[0], axis_angle[1], axis_angle[2]]

    def get_joints(self):
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, 1)
            return [joint_state_msg.position[0] * 180 / math.pi, joint_state_msg.position[1] * 180 / math.pi, joint_state_msg.position[2] * 180 / math.pi,
                    joint_state_msg.position[3] * 180 / math.pi, joint_state_msg.position[4] * 180 / math.pi, joint_state_msg.position[5] * 180 / math.pi]
        except:
            return None