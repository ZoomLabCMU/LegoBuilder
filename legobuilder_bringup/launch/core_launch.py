from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sim_arg = DeclareLaunchArgument(
        'sim', 
        default_value='false', 
        description="Set to 'true' to enable simulation"
    )

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
    ur5e_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_robot_driver'), '/launch/', 'ur_control.launch.py']),
        launch_arguments={'ur_type': 'ur5e',
                          'robot_ip': 'yyy.yyy.yy.yy',
                          'use_mock_hardware': LaunchConfiguration('sim'),
                          'launch_rviz': LaunchConfiguration('sim')}.items()
    )

    ### Sensors ###
    # Workspace Camera
    workspace_camera_node = Node(
        package='legobuilder_drivers',
        executable='workspace_camera',
        name='workspace_camera'
    )
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
    ld.add_action(sim_arg)
    ld.add_action(brickpick_adapter_node)
    ld.add_action(workspace_camera_node)
    ld.add_action(ur5e_ld)

    return ld