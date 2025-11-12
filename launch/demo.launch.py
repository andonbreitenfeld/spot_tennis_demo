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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_navigation'),
                'launch',
                'bringup_launch.py',
            ])
        )
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
                'yolo_tennis.pt',
            ]),

            'input_image_topic': '/spot_image_server/rgb/frontleft/image',
            'input_depth_topic': '/spot_image_server/depth/frontleft/image',
            'input_depth_info_topic': '/spot_image_server/depth/frontleft/camera_info',
            'use_3d': 'True',
        }.items(),
    )

    # YOLO detections to TF frames
    object_pose_3d = Node(
        package='spot_tennis_demo',
        executable='object_pose_3d',
        name='object_pose_3d',
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
        amcl,
        nav2,
        yolo_bringup,
        object_pose_3d,
        bt_executor,
    ])
