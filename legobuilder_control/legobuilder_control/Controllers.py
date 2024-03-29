#!/usr/bin/env python

import math
from rclpy.node import Client, Publisher

from legobuilder_interfaces.srv import BrickpickCommand

import numpy as np
import time as T
from std_msgs.msg import String

class BrickPickController:
    def __init__(self, brickpick_adapter_cli : Client):
        self.brickpick_adapter_cli = brickpick_adapter_cli

        self.u_max_long = 65535
        self.u_max_short = 65535
        self.u_max_plunger = 1023

    def set_long_ctrl(self, u : float):
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_long)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_long_ctrl'
        command_request.u = u
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future

    def set_short_ctrl(self, u : float):
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_short)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_short_ctrl'
        command_request.u = u
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future
    
    def set_plunger_ctrl(self, u : float):
        # TODO - Add set_plunger_ctrl to brickpick embedded
        raise NotImplementedError
        u = min(max(u, -1.0), 1.0)
        u = math.floor(u * self.u_max_plunger)
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_plunger_ctrl'
        command_request.u = u
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future

    def stop(self):
        command_request = BrickpickCommand.Request()
        command_request.command = '/stop'
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future
    
    def set_long_target_mm(self, target_mm : float):
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_long_target_mm'
        command_request.target_mm = target_mm
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future

    def set_short_target_mm(self, target_mm : float):
        command_request = BrickpickCommand.Request()
        command_request.command = '/set_short_target_mm'
        command_request.target_mm = target_mm
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future

    # Blocking services
    def plunger_up(self):
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_plunger'
        command_request.plunger_target = 0
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future

    def plunger_down(self):
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_plunger'
        command_request.plunger_target = 1
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future
    
    def long_goto_brick(self, brick : int):
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_long_brick'
        command_request.target_brick = brick
        response_future = self.brickpick_adapter_cli.call_async(command_request) # type: ignore
        return response_future
    
    def short_goto_brick(self, brick : int):
        command_request = BrickpickCommand.Request()
        command_request.command = '/goto_short_brick'
        command_request.target_brick = brick
        response_future = self.brickpick_adapter_cli.call_async(command_request) # type: ignore
        return response_future
    
    def reset(self):
        # TODO - Add a reset command to brickpick embedded
        raise NotImplementedError
        command_request = BrickpickCommand.Request()
        command_request.command = '/reset'
        response_future = self.brickpick_adapter_cli.call_async(command_request)  # type: ignore
        return response_future
    

class URController:
    def __init__(self, ur_driver_pub):
        self.current_arm_position = [0.0, 0.0, 0.0]

        self.ur_driver_pub = ur_driver_pub # type: Publisher
        T.sleep(1)


    def set_tcp(self, tcp_pose):
        set_tcp_string = String()

        set_tcp_string.data =  'set_tcp(p[' + ','.join(str(element) for element in tcp_pose) + '])'

        self.ur_driver_pub.publish(set_tcp_string)
        self.ur_driver_pub.publish(set_tcp_string)

        T.sleep(1)

        return True

    def freedrive(self, axes=[1,1,1,1,1,1], time=10):
        freedrive_mode_string = String()
        
        freedrive_mode_string.data = 'def freedrive_control():\n    zero_ftsensor()\n'+\
                                      '    freedrive_mode()\n'+\
                                      '    sleep('+str(time)+')\n    end_freedrive_mode()\nend'

        self.ur_driver_pub.publish(freedrive_mode_string)
        self.ur_driver_pub.publish(freedrive_mode_string)

        T.sleep(time)

        return True

    def stop(self):
        stopj_string = String()
        stopj_string.data = 'stopj(2)'
        self.ur_driver_pub.publish(stopj_string)
        self.ur_driver_pub.publish(stopj_string)

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

    def move_joints_in_degrees(self, desired_arm_angles, time=None):
        return self.move_joints_in_radians([(angle * np.pi/180.0) for angle in desired_arm_angles], time)

    def move_joints_in_radians(self, desired_arm_angles, time=None):
        move_joints_string = String()

        if time is not None:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '],t='+str(time)+')'
        else:
            move_joints_string.data =  'movej([' + ','.join(str(angle) for angle in desired_arm_angles) + '])'

        self.ur_driver_pub.publish(move_joints_string)
        self.ur_driver_pub.publish(move_joints_string)
        return True

    def move_ee(self, ee_pose, time=None):
        move_ee_string = String()

        if time is not None:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '],t='+str(time)+')'
        else:
            move_ee_string.data =  'movel(p[' + ','.join(str(element) for element in ee_pose) + '])'

        self.ur_driver_pub.publish(move_ee_string)
        self.ur_driver_pub.publish(move_ee_string)
        return True