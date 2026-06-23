import os
from setuptools import setup

package_name = 'paddle_speech_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hl',
    maintainer_email='3352885695@qq.com',
    description='Paddle Speech ROS Action Server',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tts_action_server = paddle_speech_ros.tts_action_server:main',
        ],
    },
)
