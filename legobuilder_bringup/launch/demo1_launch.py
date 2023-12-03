from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #config = ...

    demo1_control_node = Node(
        package='legobuilder_control',
        executable='demo1',
        name='demo1_controller'
    )

    ld.add_action(demo1_control_node)
    return ld