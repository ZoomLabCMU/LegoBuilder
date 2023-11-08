import numpy as np
import serial
from collections import deque

from std_msgs.msg import String
import requests

from legobuilder_interfaces.srv import BrickpickCommand


IP_ADDRESS = "192.168.2.108"

class BrickPickAdapter(object):
    def __init__(self):
        # TODO - Kinematics (load from config)
        self.TCP = np.array([8.0, -16.0, 238.6]) #Coordinate transform in mm to corner of fixed brick


    ### ROS node callback functions ###
    def push_cmd(self, request : BrickpickCommand.Request) -> String:
        if request.command == "/set_long_ctrl":
            params = {'u': request.u}
            response = requests.get(f"http://{IP_ADDRESS}/set_long_ctrl", params=params)
        elif request.command == "/set_short_ctrl":
            params = {'u': request.u}
            response = requests.get(f"http://{IP_ADDRESS}/set_short_ctrl", params=params)
        elif request.command == "/set_long_target_brick":
            params = {'target_brick': request.target_brick}
            response = requests.get(f"http://{IP_ADDRESS}/set_long_target_brick", params=params)
        elif request.command == "/set_long_short_brick":
            params = {'target_brick': request.target_brick}
            response = requests.get(f"http://{IP_ADDRESS}/set_short_target_brick", params=params)
        elif request.command == "/set_long_target_mm":
            params = {'target_mm': request.target_mm}
            response = requests.get(f"http://{IP_ADDRESS}/set_long_target_mm", params=params)
        elif request.command == "/set_short_target_mm":
            params = {'target_mm': request.target_mm}
            response = requests.get(f"http://{IP_ADDRESS}/set_short_target_mm", params=params)
        elif request.command == "/stop":
            response = requests.get(f"http://{IP_ADDRESS}/stop")
        elif request.command == "/goto_plunger":
            params = {'plunger_target': request.plunger_target}
            response = requests.get(f"http://{IP_ADDRESS}/goto_plunger", params=params)
        elif request.command == "/goto_long_brick":
            params = {'plunger_target': request.target_brick}
            response = requests.get(f"http://{IP_ADDRESS}/goto_long_brick", params=params)
        elif request.command == "/goto_short_brick":
            params = {'plunger_target': request.target_brick}
            response = requests.get(f"http://{IP_ADDRESS}/goto_short_brick", params=params)
        else:
            return "No response received..." #type: ignore
        return str(response.content)  #type: ignore

