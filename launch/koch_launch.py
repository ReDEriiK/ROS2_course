from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
            output='screen',

        ),
        Node(
            package='ros2_course',
            #namespace='ros2_course',
            executable='koch',
            name='koch',
            output='screen',
        ),
    ])
