import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lane_simulation_pkg')
    gazebo_share = get_package_share_directory('gazebo_ros')
    spawn_y = LaunchConfiguration('spawn_y')

    world_file = os.path.join(package_share, 'worlds', 'lane_world.world')
    robot_file = os.path.join(package_share, 'urdf', 'lane_robot.urdf')

    with open(robot_file, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lane_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', spawn_y, '-z', '0.25',
        ],
        output='screen',
    )

    lane_detector = Node(
        package='lane_detection_pkg',
        executable='video_reader',
        name='camera_lane_detector',
        parameters=[{
            'image_topic': '/lane_robot/front_camera/image_raw',
            'center_calibration_px': 8.0,
        }],
        output='screen',
    )

    direction_subscriber = Node(
        package='lane_detection_pkg',
        executable='direction_subscriber',
        name='direction_subscriber',
        parameters=[{
            'straight_tolerance': 30.0,
            'turn_threshold': 45.0,
        }],
        output='screen',
    )

    lane_controller = Node(
        package='lane_detection_pkg',
        executable='lane_controller',
        name='lane_controller',
        parameters=[{
            'forward_speed': 0.35,
            'steering_gain': 0.008,
            'max_angular_speed': 0.8,
            'detection_timeout': 0.5,
        }],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial lateral position of the robot in metres',
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        lane_detector,
        direction_subscriber,
        lane_controller,
    ])
