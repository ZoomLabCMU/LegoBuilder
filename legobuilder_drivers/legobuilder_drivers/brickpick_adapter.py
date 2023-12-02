import numpy as np

import requests

from legobuilder_interfaces.srv import BrickpickCommand


class BrickPickAdapter(object):
    def __init__(self, ip_address : str):
        # TODO - Kinematics (load from config)
        self.ip_address = ip_address
        self.TCP = np.array([8.0, -16.0, 238.6]) #Coordinate transform in mm to corner of fixed brick


    ### ROS node callback functions ###
    def push_cmd(self, request : BrickpickCommand.Request) -> str:
        if request.command == "/set_long_ctrl":
            params = {'u': request.u}
            response = requests.get(f"http://{self.ip_address}/set_long_ctrl", params=params)
        elif request.command == "/set_short_ctrl":
            params = {'u': request.u}
            response = requests.get(f"http://{self.ip_address}/set_short_ctrl", params=params)
        elif request.command == "/set_long_target_brick":
            params = {'target_brick': request.target_brick}
            response = requests.get(f"http://{self.ip_address}/set_long_target_brick", params=params)
        elif request.command == "/set_long_short_brick":
            params = {'target_brick': request.target_brick}
            response = requests.get(f"http://{self.ip_address}/set_short_target_brick", params=params)
        elif request.command == "/set_long_target_mm":
            params = {'target_mm': request.target_mm}
            response = requests.get(f"http://{self.ip_address}/set_long_target_mm", params=params)
        elif request.command == "/set_short_target_mm":
            params = {'target_mm': request.target_mm}
            response = requests.get(f"http://{self.ip_address}/set_short_target_mm", params=params)
        elif request.command == "/stop":
            response = requests.get(f"http://{self.ip_address}/stop")
        elif request.command == "/goto_plunger":
            params = {'plunger_target': request.plunger_target}
            response = requests.get(f"http://{self.ip_address}/goto_plunger", params=params)
        elif request.command == "/goto_long_brick":
            params = {'plunger_target': request.target_brick}
            response = requests.get(f"http://{self.ip_address}/goto_long_brick", params=params)
        elif request.command == "/goto_short_brick":
            params = {'plunger_target': request.target_brick}
            response = requests.get(f"http://{self.ip_address}/goto_short_brick", params=params)
        else:
            return "No response received..."
        return str(response.content)

