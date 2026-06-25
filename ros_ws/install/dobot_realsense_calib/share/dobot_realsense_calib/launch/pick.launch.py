"""실행 노드 런치 예시: ros2 launch dobot_realsense_calib pick.launch.py"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    home = os.path.expanduser('~')
    return LaunchDescription([
        Node(
            package='dobot_realsense_calib',
            executable='pick_node',
            name='pick_node',
            output='screen',
            parameters=[{
                'calib_path': os.path.join(home, 'calibration.yaml'),
                'image_topic': '/camera/camera1/color/image_raw',
                'pose_topic': '/dobot_pose',
                'target': 'ball',
                'auto': False,
                'safe_z': 60.0,
                'lift_z': 60.0,
                'place_xy': [200.0, -120.0],
            }],
        ),
    ])
