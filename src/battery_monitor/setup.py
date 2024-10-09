from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'battery_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='guido@dogsally.com',
    description='ROS2 package for battery monitoring using INA219 sensor',
     license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor_node = battery_monitor.battery_monitor_node:main',
        ],
    },
)
