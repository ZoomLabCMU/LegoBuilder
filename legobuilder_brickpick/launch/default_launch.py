from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #config = ...

    brickpick_adapter_node = Node(
        package='legobuilder_brickpick',
        executable='brickpick_adapter',
        name='brickpick_adapter'
    )

    brickpick_teleop_node = Node(
        package='legobuilder_brickpick',
        executable='teleop',
        name='brickpick_teleop'
    )

    ld.add_action(brickpick_adapter_node)
    ld.add_action(brickpick_teleop_node)

    return ld