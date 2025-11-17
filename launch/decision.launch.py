from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    spot_apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_apriltag'),
                'launch',
                'spot_apriltag.launch.py'
            ])
        )
    )
    
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
    
    bin_detector = Node(
        package='spot_tennis_demo',
        executable='bin_detector',
        name='bin_detector',
        output='screen',
    )

    return LaunchDescription([
        spot_apriltag,
        ball_selector,
        nav_manager,
        bin_detector,
    ])