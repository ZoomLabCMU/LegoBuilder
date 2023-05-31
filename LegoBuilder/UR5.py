import rospy
from easy_ur.srv import *
import math
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool, Float64MultiArray
from geometry_msgs.msg import Point32
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
import numpy as np
import time

from std_srvs.srv import Empty

"""
Coming Towards You -> +x
Pointing to Shelf -> +y
"""
class UR5:
    def __init__(self):
        rospy.init_node("example_servo")
        self.cur_pos = PoseStamped()
        self.servo_msg  = PoseStamped()
        self.servo_pub = rospy.Publisher('target_servo',PoseStamped, queue_size = 1)
        self.servo_msg.header.frame_id = "base_link"
        self.servo_seq = 0

        self.joint_publisher = rospy.Publisher('target_joints', Float64MultiArray, queue_size=1)

        rospy.wait_for_service('/ur_stop')
        rospy.wait_for_service("/ur_pose")

        self.stop_service = rospy.ServiceProxy('/ur_stop', Trigger)
        self.trigger =  TriggerRequest()

        self.home_pos = [0.2938,-0.3491, 0.3]
        self.home_orn = [self.cur_pos.pose.orientation.x,self.cur_pos.pose.orientation.y,self.cur_pos.pose.orientation.z,self.cur_pos.pose.orientation.w]
        print(self.home_pos)
        self.set_ur_speed(0.6)
        self.set_ur_acceleration(0.35)
        self.data = None
        self.tgt_jts = None

    def stop_ur(self):
        self.stop_service(self.trigger)

    def set_ur_speed(self, speed):
        rospy.wait_for_service('/ur_speed')
        try:
            service = rospy.ServiceProxy('/ur_speed', SetSpeed)
            service(speed)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def set_ur_acceleration(self, acceleration):
        rospy.wait_for_service('/ur_acceleration')
        try:
            service = rospy.ServiceProxy('/ur_acceleration', SetAcceleration)
            service(acceleration)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def servo_pos(self, pos, orn):
        self.servo_msg.pose.position.x,self.servo_msg.pose.position.y,self.servo_msg.pose.position.z = pos
        self.servo_msg.pose.orientation.x,self.servo_msg.pose.orientation.y,self.servo_msg.pose.orientation.z,self.servo_msg.pose.orientation.w =orn
        self.servo_msg.header.seq = self.servo_seq
        self.servo_seq+=1
        self.servo_msg.header.stamp = rospy.get_rostime()
        self.servo_pub.publish(self.servo_msg)

    def callback_pos(self, msg):
        self.cur_pos.pose = msg.pose#.x

    def print_current_pose(self):
        start_pos = [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        start_orn = [self.cur_pos.pose.orientation.x,self.cur_pos.pose.orientation.y,self.cur_pos.pose.orientation.z,self.cur_pos.pose.orientation.w]
        print(start_pos, start_orn)

    def get_pose(self):
        start_pos = [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        start_orn = [self.cur_pos.pose.orientation.x,self.cur_pos.pose.orientation.y,self.cur_pos.pose.orientation.z,self.cur_pos.pose.orientation.w]
        return start_pos, start_orn

    def set_pose(self, pose, orn):
        rospy.wait_for_service("/ur_pose")
        try:
            service = rospy.ServiceProxy("/ur_pose", SetPose)
            pose_cmd = Pose()
            pose_cmd.position.x = pose[0]
            pose_cmd.position.y = pose[1]
            pose_cmd.position.z = pose[2]
            pose_cmd.orientation.x = orn[0]
            pose_cmd.orientation.y = orn[1]
            pose_cmd.orientation.z = orn[2]
            pose_cmd.orientation.w = orn[3]
            service(pose_cmd)
        except:
            print("Set Pose failed")

    def generate_spiral(self, POS):
        maxradius = 0.01 #1.5 cm radius
        gap = 0.002 #5 mm
        numpoints = int(maxradius/gap)
        x_origin  = POS[0]
        y_origin = POS[1]
        z_origin  = POS[2]

        #maxradius = 0.03
        #gap = 0.005
        numsteps = 20#int(maxradius/gap)
        numpoints = 1000
        angle = np.linspace(0,numsteps*np.pi,numpoints)
        radius = np.linspace(0,maxradius, numpoints)
        #TODO-  change axis assignment based on the orientation of surface
        y = y_origin + radius * np.cos(angle)
        z =  z_origin + radius * np.sin(angle)
        #clip z axis
        z[z<0.014] = 0.014
        z[z>0.032] = 0.032
        x = np.ones(numpoints)*x_origin
        return x,y,z

    def joint_callback(self, msg):
        self.data = list(msg.data)
    def tgt_jt_callback(self, msg):
        self.tgt_jts = list(msg.data)

    def set_ur_wrist_joint_val(self, jt_cmd):
        print(self.data)
        temp = self.data
        new_jts = Float64MultiArray()
        temp[-1] = jt_cmd
        new_jts.data = temp
        temp = self.tgt_jts
        rospy.sleep(0.5)
        self.joint_publisher.publish(new_jts)
        rospy.sleep(0.5)
        if self.tgt_jts == temp:
            raise ValueError("Restart ROSCORE, Fuck ROS!")

    def go_home(self):
        # print(self.cur_pos)        
        self.set_ur_speed(0.6)
        self.set_ur_acceleration(0.35)
        start_pos = [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        start_orn = quaternion_from_euler(math.radians(180), math.radians(0), math.radians(90))
        self.rotate_wrist()
        POS1 = start_pos
        POS2 = self.home_pos
        print(POS1, POS2)
        
        dist = np.linalg.norm(np.array(POS1)-np.array(POS2))
        num_points = int(dist/0.00035)
        lin_path = np.linspace(POS1,POS2,num_points)
        step_count=0
        t_delay=0.02
        while step_count<num_points:
            self.servo_pos(lin_path[step_count],start_orn)
            rospy.sleep(t_delay)
            step_count=step_count+1
        rospy.sleep(t_delay)
        rospy.sleep(4)
        return "kaka"

    def rotate_wrist(self, theta=90, rad=False):
        goal_pos = [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        if not rad: 
            theta = np.radians(theta)
        goal_orn = quaternion_from_euler(-np.pi, 0, theta)
        rospy.sleep(0.05)
        self.set_pose(goal_pos,goal_orn)
        rospy.sleep(0.05)

    def go_to_pose(self, pos, nordbo, mandm, orn=None, spd_mult=0.5, acc=0.3, f_thresh = None, tgt_lbl=-1, example_or_normal = "normal"):
        self.set_ur_speed(0.6/spd_mult)
        self.set_ur_acceleration(acc/spd_mult)
        if orn is None:
            orn = [self.cur_pos.pose.orientation.x,self.cur_pos.pose.orientation.y,self.cur_pos.pose.orientation.z,self.cur_pos.pose.orientation.w]
        
        start_pos = [self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        dist = np.linalg.norm(np.array(start_pos)-np.array(pos))
        # Linear Interpolation with each step == 0.55mm -> this comment is sus
        # If dist = 0.02, and spd_mmult = 5, num_pts = 285 ==> dist/num_pts = 20/285 -> 0.07mm
        num_points = int(dist/0.00020*spd_mult)
        # if example_or_normal == "example":
        #     num_points = 20
        print("# interpolation points: ", num_points)
        print("Each step with distance: ", dist/num_points)
        lin_path = np.linspace(start_pos, pos, num_points)

        step_count=0
        # This delay determines how fast your robot moves 
        t_delay=0.01
        # if example_or_normal == "example":
        #     t_delay = 2.0
        data = []
        while step_count<num_points:
            self.servo_pos(lin_path[step_count],orn)
            dat1 = mandm.get_curr_reading()
            dat2 = nordbo.get_curr_reading()
            if dat1 is not None:
                self.data_arr = np.hstack([time.time(),dat2, dat1.flatten(), tgt_lbl])
                data.append(self.data_arr)
                if (f_thresh is not None) and (self.data_arr[1] > f_thresh): # fx exceeds threshold
                    print(self.data_arr)
                    print("HAKUNA")
                    rospy.sleep(t_delay)
                    break
            rospy.sleep(t_delay)
            step_count=step_count+1
        rospy.sleep(t_delay)
        return data

    def run_demo(self):
        #rospy.sleep(1)
        #quaternion_from_euler(math.radians(180), math.radians(0), math.radians(-90))
        start_pos = [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        start_orn = [self.cur_pos.pose.orientation.x,self.cur_pos.pose.orientation.y,self.cur_pos.pose.orientation.z,self.cur_pos.pose.orientation.w]
        self.set_ur_speed(0.5)
        self.set_ur_acceleration(0.35)
        #move up 5 cm
        self.set_pose([start_pos[0], start_pos[1],start_pos[2]+0.05],start_orn)

        #servo down 10 cm
        POS1 =  [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z]
        POS2 =   [self.cur_pos.pose.position.x, self.cur_pos.pose.position.y,self.cur_pos.pose.position.z-0.1]

        dist = np.linalg.norm(np.array(POS1)-np.array(POS2))
        num_points = int(dist/0.00055)
        lin_path = np.linspace(POS1,POS2,num_points)

        step_count=0
        t_delay=0.05
        print("Num_points = ", num_points)
        while step_count<num_points:

            self.servo_pos(lin_path[step_count],start_orn)
            #if (ft_z< -15):
            #    print("FT ", ft_z)
            #   rospy.sleep(t_delay)
            #   break
            if(step_count==150):
                rospy.sleep(t_delay)# use delay before breaking
                break
            rospy.sleep(t_delay)
            step_count=step_count+1

        self.set_pose([start_pos[0], start_pos[1],start_pos[2]],start_orn)

# if __name__ == '__main__':
    # ur5 = UR5()
    # rospy.Subscriber('/ur_pose', PoseStamped, ur5.callback_pos)
    # rospy.Subscriber('/ur_joints', Float64MultiArray, ur5.joint_callback)
    # rospy.Subscriber('/target_joints', Float64MultiArray, ur5.tgt_jt_callback)
    # rospy.sleep(0.5)

    # ur5.set_ur_speed(0.5)
    # ur5.set_ur_acceleration(0.35)
    # # ur5.run_demo()
    # # ur5.rotate_wrist(1.57079633, True)
    # # ur5.go_home()
    # ur5.set_ur_wrist_joint_val(0)