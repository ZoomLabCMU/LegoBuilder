# Requriements
## OS
Ubuntu 22.04 (recommended) \
Ubuntu 20.04 (untested)

## Packages
### ROS 2 Humble Hawksbill
This project uses the ROS 2 Humble Hawksbill build as its middlware which will be supported until 2027. \
Follow the official installation instructions online, and build from source if using Ubuntu 20.04. \
[Installation Instructions](https://docs.ros.org/en/humble/Installation.html)

### ur_robot_driver
This is the driver for Universal Robots interfacing with ROS 2. \
[UR Driver Repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
```
sudo apt-get install ros-humble-ur
```

### realsense2_camera
This is the driver for the Intel Realsense D405 stereo camera. \
[Intel Realsense Repo](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development)

Start by installing the Intel Realsense SDK.
```
sudo apt install ros-humble-librealsense2*
```
Now install the Intel Realsense ROS 2 wrapper.
```
sudo apt install ros-humble-realsense2-*
```
