from setuptools import find_packages, setup

package_name = 'legobuilder_control'

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
    maintainer='chris',
    maintainer_email='ctchang001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bp_teleop = legobuilder_control.brickpick_teleop_node:main',
            'legobuilder_controller = legobuilder_control.legobuilder_control_node:main',
            'demo1 = legobuilder_control.legobuilder_planner_node:demo1'
        ],
    },
)
