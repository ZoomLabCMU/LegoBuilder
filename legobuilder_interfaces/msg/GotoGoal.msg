# Move command
string policy
float64[] ee_pose
sensor_msgs/JointState joint_states
# Time limiting
float64 timeout
bool use_timeout
# Force-Torque limiting
geometry_msgs/Wrench wrench_thresh
bool use_ft