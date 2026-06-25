from setuptools import setup
import os
from glob import glob

package_name = 'golf_ball_inspector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='dlwlsdud0510@gmail.com',
    description='Golf ball defect detection and sorting system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dobot_controller = golf_ball_inspector.dobot_controller_node:main',
            'inspection       = golf_ball_inspector.inspection_node:main',
            'conveyor         = golf_ball_inspector.conveyor_node:main',
            'socket_bridge    = golf_ball_inspector.socket_bridge_node:main',
        ],
    },
)
