from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    bt_executor = Node(
        package='spot_tennis_demo',
        executable='bt_executor',
        name='bt_executor',
        parameters=[{
            'bt_xml_file': PathJoinSubstitution([
                FindPackageShare('spot_tennis_demo'),
                'config',
                'bt_tree.xml',
            ]),
        }],
        output='screen',
    )

    nav_manager = Node(
        package='spot_tennis_demo',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
    )

    return LaunchDescription([
        bt_executor,
        nav_manager,
    ])
