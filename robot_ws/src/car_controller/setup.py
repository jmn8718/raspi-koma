import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'car_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures your launch files are actually installed to the share folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='motoko',
    maintainer_email='jmn8718@gmail.com',
    description='ROS 2 controller for Raspberry Pi RC Car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = car_controller.motors:main',
            'sonar = car_controller.sonar_node:main', 
            # Points to car_controller/sonar_node.py
        ],
    },
)