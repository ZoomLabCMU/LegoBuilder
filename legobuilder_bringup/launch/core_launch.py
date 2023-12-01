from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #config = ...

    ### Drivers ###
    # BrickPick Driver
    brickpick_adapter_node = Node(
        package='legobuilder_drivers',
        executable='brickpick_adapter',
        name='brickpick_adapter'
    )
    # UR5 Driver
    # TODO - Write me
    '''
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.??.???
    '''

    ### Sensors ###
    # RealSense D405
    # TODO - Write me
    '''
    From realsense2_camera/launch/rs_launch.py
    brickpcick_camera_node = Node(
        package='realsense2_camera',
        namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
        name=LaunchConfiguration('camera_name' + param_name_suffix),
        executable='realsense2_camera_node',
        parameters=[params, params_from_file],
        output=LaunchConfiguration('output' + param_name_suffix),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
        emulate_tty=True,
    )
    '''
    # Proprioceptive Sensor
    # TODO - Write me

    ld.add_action(brickpick_adapter_node)

    return ld