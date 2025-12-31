import os
from glob import glob
from setuptools import setup

package_name = 'car_description'

# Function to collect all files in a directory recursively
def get_web_files():
    data_files = []
    for root, dirs, files in os.walk('web'):
        install_path = os.path.join('share', package_name, root)
        # Get full paths of files
        file_paths = [os.path.join(root, f) for f in files]
        if file_paths:
            data_files.append((install_path, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register the package with ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch files (e.g., rviz_launch.py)
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
            
        # Install URDF files (e.g., car.urdf)
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.urdf'))),
            
        # Install RViz config files (optional but recommended)
        (os.path.join('share', package_name, 'rviz'), 
            glob(os.path.join('rviz', '*.rviz'))),
        
        # Install config files (e.g., joystick.yaml)
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
            
    ] + get_web_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Miguel Navarro',
    maintainer_email='jmn8718@gmail.com',
    description='Visualization and URDF description for raspikoma robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you add Python nodes to this workspace, list them here
        ],
    },
)