"""
pc_launch.py — 노트북에서 실행
  - dobot_controller_node  : Dobot Pick & Place 제어
  - inspection_node        : YOLO 검사 + 이중 카메라 표시
  - socket_bridge_node     : /conveyor/cmd → 라즈베리파이 TCP 변환
                             (RPi는 ROS 없이 기존 rpi_server.py 실행)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '37')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ssafy/pene_pjt/best.pt',
        description='YOLOv8 모델 파일 경로 (.pt)',
    )

    dobot_node = Node(
        package='golf_ball_inspector',
        executable='dobot_controller',
        name='dobot_controller',
        output='screen',
    )

    inspection_node = Node(
        package='golf_ball_inspector',
        executable='inspection',
        name='inspection_node',
        output='screen',
        parameters=[{'model_path': LaunchConfiguration('model_path')}],
    )

    bridge_node = Node(
        package='golf_ball_inspector',
        executable='socket_bridge',
        name='socket_bridge',
        output='screen',
        parameters=[{
            'rpi_host': '192.168.110.143',
            'rpi_port': 9999,
        }],
    )

    return LaunchDescription([
        domain_id,
        model_path_arg,
        dobot_node,
        inspection_node,
        bridge_node,
    ])
