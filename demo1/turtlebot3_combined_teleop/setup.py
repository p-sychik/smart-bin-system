import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot3_combined_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fola',
    maintainer_email='fola@email.com',
    description='Combined keyboard teleop for TurtleBot3 movement, servos, and action recording',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'combined_teleop_node = turtlebot3_combined_teleop.combined_teleop_node:main',
        ],
    },
)
