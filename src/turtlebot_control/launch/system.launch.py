from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os


def generate_launch_description():

    # TurtleBot model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    gazebo_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'turtlebot3_gazebo',
            'turtlebot3_world.launch.py',
            'gui:=false'
        ],
        output='screen'
    )

    control_node = Node(
        package='turtlebot_control',
        executable='control_node',
        name='click_control_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        control_node
    ])


