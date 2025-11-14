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
            'image_config': '/home/abreitenfeld/project_ws/src/spot_ros/spot_driver/config/publish_all_images.yaml',
        }.items(),
    )

    yolo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py',
            ])
        ),
        launch_arguments={
            'model': PathJoinSubstitution([
                FindPackageShare('spot_tennis_demo'),
                'models',
                'tennis-seg-high.pt',
            ]),
            'input_image_topic': '/spot_image_server/rgb/hand_rgb/image',
            'input_depth_topic': '/spot_image_server/depth/hand_rgb/image',
            'input_depth_info_topic': '/spot_image_server/depth/hand_rgb/camera_info',
            'use_3d': 'True',
            'device': 'cpu',
            'threshold': '0.30',
        }.items(),
    )

    ball_selector = Node(
        package='spot_tennis_demo',
        executable='ball_selector',
        name='ball_selector',
        output='screen',
    )

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

    return LaunchDescription([
        spot_bringup,
        yolo_bringup,
        ball_selector,
        bt_executor,
    ])
