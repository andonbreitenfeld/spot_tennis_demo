from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    apriltag_config = PathJoinSubstitution([
        FindPackageShare('spot_tennis_demo'),
        'config',
        'apriltag_settings.yaml'
    ])
    
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[
            {'use_sim_time': False},
            apriltag_config
        ],
        remappings=[
            ('image_rect', '/spot_image_server/rgb/hand_rgb/image'),
            ('camera_info', '/spot_image_server/rgb/hand_rgb/camera_info'),
        ],
        emulate_tty=True,
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
        apriltag_node,
        ball_selector,
        nav_manager,
        bin_detector,
    ])