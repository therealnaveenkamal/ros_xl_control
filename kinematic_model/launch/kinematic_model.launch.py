import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model_node',
            output = 'screen'
        ),
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            name='wheel_velocities_publisher_node',
            output = 'screen'
        )
    ])
