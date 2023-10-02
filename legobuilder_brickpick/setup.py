import os
from glob import glob
from setuptools import setup

# fetch values from package.xml
package_name = "lego_builder_brickpick"

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=[package_name],
    package_dir={'': 'src'},
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # Also important
    install_requires=['setuptools'],
    zip_safe=True,
    # Personal info
    maintainer='Chris Chang',
    maintainer_email='ctchang@andrew.cmu.edu',
    description='BrickPick end effector package',
    license='Apache License 2.0',
    # Add python scripts/entry points here
    # entry_points={
    #     'console_scripts': [
    #             'my_script = my_package.my_script:main',
    #     ],
    # },
)