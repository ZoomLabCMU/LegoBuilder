from launch import LaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
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
    workspace_camera_arg = DeclareLaunchArgument(
        'enable_ws_cam',
        default_value='true',
        description="Set to 'true' to activate workspace camera"
    )
    ee_camera_arg = DeclareLaunchArgument(
        'enable_ee_cam',
        default_value='true',
        description="Set to 'true' to enable end effector camera"
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
    '''
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.12
    '''
    ur5e_ld_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_robot_driver'), '/launch/', 'ur_control.launch.py']),
        launch_arguments={'ur_type': 'ur5e',
                          'robot_ip': '192.168.56.101',
                          'launch_rviz': 'true'}.items(),
        condition=LaunchConfigurationEquals('sim', 'true')
    )

    ur5e_ld_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_robot_driver'), '/launch/', 'ur_control.launch.py']),
        launch_arguments={'ur_type': 'ur5e',
                          'robot_ip': '192.168.1.12',
                          'launch_rviz': 'false'}.items(),
        condition=LaunchConfigurationNotEquals('sim', 'true')
    )

    ### Sensors ###
    # Workspace Camera
    workspace_camera_node = Node(
        package='legobuilder_drivers',
        executable='workspace_camera',
        name='workspace_camera',
        condition=IfCondition(LaunchConfiguration('enable_ws_cam'))
    )

    # RealSense D405
    realsense_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('realsense2_camera'), '/launch/', 'rs_launch.py']),
        launch_arguments={},
        condition=IfCondition(LaunchConfiguration('enable_ee_cam'))
    )
    # Proprioceptive Sensor
    # TODO - Write me

    ### Controller ###
    lb_control_node = Node(
        package='legobuilder_control',
        executable='legobuilder_controller',
        name='legobuilder_controller'
    )

    ld.add_action(sim_arg)
    ld.add_action(workspace_camera_arg)
    ld.add_action(ee_camera_arg)
    
    ld.add_action(brickpick_adapter_node)
    ld.add_action(workspace_camera_node)
    ld.add_action(lb_control_node)

    ld.add_action(realsense_ld)
    ld.add_action(ur5e_ld_sim)
    ld.add_action(ur5e_ld_real)

    return ld