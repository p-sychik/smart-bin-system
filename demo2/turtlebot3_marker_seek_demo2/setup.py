import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'turtlebot3_marker_seek_demo2'

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
    description='Marker seek demo using ArUco detection and autonomous approach for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_seek_node = turtlebot3_marker_seek_demo2.marker_seek_node:main',
        ],
    },
)
