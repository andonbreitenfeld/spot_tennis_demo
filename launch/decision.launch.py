from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    nav_manager = Node(
        package='spot_tennis_demo',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
    )

    return LaunchDescription([

        nav_manager,
    ])
