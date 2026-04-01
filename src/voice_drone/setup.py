from setuptools import setup
from glob import glob
import os

package_name = 'voice_drone'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='safe bench ros2 voice drone stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'console_node = voice_drone.console_node:main',
            'intent_node = voice_drone.intent_node:main',
            'oak_node = voice_drone.oak_node:main',
            'detect_node = voice_drone.detect_node:main',
            'mission_node = voice_drone.mission_node:main',
            'telemetry_node = voice_drone.telemetry_node:main',
            'safe_bridge_node = voice_drone.safe_bridge_node:main',
            'fake_drone_node = voice_drone.fake_drone_node:main',
            'status_node = voice_drone.status_node:main',
        ],
    },
)
