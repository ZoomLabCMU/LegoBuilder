from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #config = ...

    # Nodes for teleop control
    brickpick_teleop_node = Node(
        package='legobuilder_control',
        executable='bp_teleop',
        name='brickpick_teleop'
    )

    ld.add_action(brickpick_teleop_node)

    return ld