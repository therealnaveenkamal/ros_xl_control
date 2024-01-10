import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model_node'
        ),
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output = 'screen'
        )
    ])
