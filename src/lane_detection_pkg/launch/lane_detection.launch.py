from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_detection_pkg',
            executable='video_reader',
            name='video_reader',
            output='screen',
            parameters=[{
                'center_calibration_px': -200.0,
            }],
        ),
        Node(
            package='lane_detection_pkg',
            executable='direction_subscriber',
            name='direction_subscriber',
            output='screen',
            parameters=[{
                'straight_tolerance': 30.0,
                'turn_threshold': 45.0,
            }],
        ),
    ])
