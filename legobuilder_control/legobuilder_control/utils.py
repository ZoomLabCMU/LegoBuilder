import math
import numpy as np
import subprocess

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped

# Function from https://stackoverflow.com/questions/21030391/how-to-normalize-an-array-in-numpy
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

# Function from https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
def multiply_quaternions(quaternion1, quaternion2):
    return np.array([[quaternion1[3] * quaternion2[0] + quaternion1[0] * quaternion2[3] + quaternion1[1] * quaternion2[2] - quaternion1[2] * quaternion2[1]],
                     [quaternion1[3] * quaternion2[1] - quaternion1[0] * quaternion2[2] + quaternion1[1] * quaternion2[3] + quaternion1[2] * quaternion2[0]],
                     [quaternion1[3] * quaternion2[2] + quaternion1[0] * quaternion2[1] - quaternion1[1] * quaternion2[0] + quaternion1[2] * quaternion2[3]],
                     [quaternion1[3] * quaternion2[3] - quaternion1[0] * quaternion2[0] - quaternion1[1] * quaternion2[1] - quaternion1[2] * quaternion2[2]]], dtype=np.float64)

# Function from https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
def quat_to_axis_angle(quaternion):
	quat = np.array(quaternion)

	if(quat[3] > 1):
		quat = normalize(quat)

	angle = 2 * math.acos(quat[3])
	s = math.sqrt(1 - quat[3] * quat[3])
	if (s < 0.001):
		x = quat[0]
		y = quat[1]
		z = quat[2]
	else:
		x = quat[0] / s
		y = quat[1] / s
		z = quat[2] / s

	axis = np.array([x, y, z])
	normalized_axis = normalize(axis)
	axis_angle = angle * normalized_axis

	return axis_angle

# Function from https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
def axis_angle_to_quaternion(axis_angle):
	axis_angle = np.array(axis_angle)
	angle = np.linalg.norm(axis_angle)
	axis = normalize(axis_angle)

	s = math.sin(angle / 2)

	x = axis[0] * s
	y = axis[1] * s
	z = axis[2] * s
	w = math.cos(angle / 2)

	return [x,y,z,w]

def rotate_around_axis(axis_angle, axis, angle):
	quaternion1 = axis_angle_to_quaternion(-np.array(axis_angle))
	axis_angle2 = angle * normalize(axis)
	quaternion2 = axis_angle_to_quaternion(-np.array(axis_angle2))
	final_quaternion = multiply_quaternions(quaternion1,quaternion2)
	final_axis_angle = -quat_to_axis_angle(final_quaternion)
	return final_axis_angle


def save_rgb(dir_path, filename):
    cmd = "python scripts/save_images.py " + dir_path + filename
    rgb_p = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
    return rgb_p

def save_proprioceptive(dir_path, filename):
    cmd = "rosbag record /tf /joint_states /wrench -O " + dir_path + filename
    bag_p = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
    return bag_p


def np_to_posestamped(position, orientation, frame_id):
	res = PoseStamped()
	res.header.stamp = rclpy.time.Time().to_msg()
	res.header.frame_id = frame_id
	res.pose.position.x = position[0]
	res.pose.position.y = position[1]
	res.pose.position.z = position[2]
	res.pose.orientation.x = orientation[0]
	res.pose.orientation.y = orientation[1]
	res.pose.orientation.z = orientation[2]
	res.pose.orientation.w = orientation[3]

	return res

def posestamped_to_np(p : PoseStamped):
	return [p.pose.position.x, p.pose.position.y, p.pose.position.z,
		 	p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]

def posestamped_to_TCP(p : PoseStamped):
	# return [x, y, z, Rx, Ry, Rz] for ur script
	TCP_pose = [
		p.pose.position.x,
		p.pose.position.y,
		p.pose.position.z
	]

	TCP_pose.extend(quat_to_axis_angle([
		p.pose.orientation.x,
		p.pose.orientation.y,
		p.pose.orientation.z,
		p.pose.orientation.w
	]))
	return TCP_pose