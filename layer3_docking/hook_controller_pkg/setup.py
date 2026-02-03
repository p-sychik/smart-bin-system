from setuptools import find_packages, setup

package_name = 'hook_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Amarov',
    maintainer_email='github@mail.amarov.dev',
    description='Controls the hooking system of the bin',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'hook_controller = hook_controller_pkg.hook_controller:main'
        ],
    },
)
