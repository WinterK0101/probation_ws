from setuptools import setup
import os
from glob import glob

package_name = 'auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loong',
    maintainer_email='loongkiat30@gmail.com',
    description='AUV control package for autonomous gate navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_controller = auv_control.mode_controller:main',
            'depth_controller = auv_control.depth_controller:main',
            'gate_controller = auv_control.gate_controller:main'
        ],
    },
)