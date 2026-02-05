from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Paths
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # Set TurtleBot3 model
    set_tb3_model = ExecuteProcess(
        cmd=['bash', '-c', 'export TURTLEBOT3_MODEL=burger'],
        shell=True
    )

    # Launch Gazebo with ROS plugins
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn TurtleBot3 Burger
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file',
            os.path.join(
                turtlebot3_gazebo_share,
                'models',
                'turtlebot3_burger',
                'model.sdf'
            ),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Start your GUI controller
    gui_controller = Node(
        package='click_controller',
        executable='gui_controller',
        output='screen'
    )

    return LaunchDescription([
        set_tb3_model,
        gazebo,
        spawn_robot,
        gui_controller
    ])
