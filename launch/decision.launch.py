from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ball_selector = Node(
        package='spot_tennis_demo',
        executable='ball_selector',
        name='ball_selector',
        output='screen',
    )

    nav_manager = Node(
        package='spot_tennis_demo',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
    )

    return LaunchDescription([
        ball_selector,
        nav_manager,
    ])
