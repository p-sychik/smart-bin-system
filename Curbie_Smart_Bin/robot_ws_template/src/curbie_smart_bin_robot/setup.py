import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'curbie_smart_bin_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex',
    maintainer_email='s2552017@ed.ac.uk',
    description='Robot-side bringup, dual cameras, and sonar publishing for the Curbie Smart Bin app',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_publisher = curbie_smart_bin_robot.sonar_publisher:main',
        ],
    },
)
