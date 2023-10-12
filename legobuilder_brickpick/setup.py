import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'legobuilder_brickpick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Chang',
    maintainer_email='ctchang001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = legobuilder_brickpick.hello_world_node:main',
            'teleop = legobuilder_brickpick.brickpick_teleop_node:main',
            'brickpick_adapter = legobuilder_brickpick.brickpick_adapter_node:main',
        ],
    },
)