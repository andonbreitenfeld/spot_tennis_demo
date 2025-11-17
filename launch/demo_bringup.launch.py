from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
                'hand_and_front_cams.yaml',
            ]),
        }.items(),
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
        amcl,
    ])