from setuptools import setup
import os
from glob import glob

package_name = 'dobot_realsense_calib'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Eye-to-Hand 평면 Affine 캘리브레이션 (Dobot Magician + RealSense)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 캘리브레이션 데이터 수집 (대화형: 로봇 jog → 스페이스로 캡처)
            'calib_collect = dobot_realsense_calib.calib_collect:main',
            # 수집한 좌표쌍으로 Affine 행렬 계산 + yaml 저장
            'calib_compute = dobot_realsense_calib.calib_compute:main',
            # 캘리브레이션 검증 (이미지 클릭 → 예상 로봇좌표 출력)
            'calib_verify = dobot_realsense_calib.calib_verify:main',
            # 실행: 골프공 검출 → 로봇좌표 변환 → 집기
            'pick_node = dobot_realsense_calib.pick_node:main',
        ],
    },
)
