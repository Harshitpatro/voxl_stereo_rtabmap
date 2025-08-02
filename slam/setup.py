from setuptools import setup
import os
from glob import glob

package_name = 'voxl_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harshit',
    maintainer_email='your_email@example.com',
    description='VOXL sensor publishers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voxl_camera_info_publisher = voxl_sensors.voxl_camera_info_publisher:main',
            'voxl_stereo_tf_publisher = voxl_sensors.voxl_stereo_tf_publisher:main',
        ],
    },
)