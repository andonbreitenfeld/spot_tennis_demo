from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    spot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_bringup'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        launch_arguments={
            'hostname': '192.168.50.3',
            'controller_configuration': 'Dualsense5',
            'kinematic_model': 'body_assist',
            'dock_id': '520',
            'publish_images': 'True',
            'image_config': PathJoinSubstitution([
                FindPackageShare('spot_tennis_demo'),
                'config',
                'hand_camera_only.yaml',
            ]),
        }.items(),
    )

    ball_selector = Node(
        package='spot_tennis_demo',
        executable='ball_selector',
        name='ball_selector',
        output='screen',
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_navigation'),
                'launch',
                'amcl.launch.py',
            ])
        ),
        launch_arguments={
            'map': PathJoinSubstitution([
                FindPackageShare('spot_tennis_demo'),
                'maps',
                'ahg_ars_elm.yaml',
            ]),
        }.items(),
    )

    return LaunchDescription([
        spot_bringup,
        ball_selector,
        amcl,
    ])