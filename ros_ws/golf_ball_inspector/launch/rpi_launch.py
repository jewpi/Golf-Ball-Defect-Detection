"""
rpi_launch.py — 라즈베리파이에서 실행
  - conveyor_node : 스텝모터(컨베이어) + 서보모터 제어

사전 조건:
  - 노트북과 라즈베리파이가 동일 네트워크에 있어야 함
  - 양쪽 모두 동일한 ROS_DOMAIN_ID 설정
    예) export ROS_DOMAIN_ID=42
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '37')

    conveyor_node = Node(
        package='golf_ball_inspector',
        executable='conveyor',
        name='conveyor_node',
        output='screen',
    )

    return LaunchDescription([domain_id, conveyor_node])
