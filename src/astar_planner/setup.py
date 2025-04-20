from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'astar_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),  # Include all .py files in launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple A* planner in ROS 2',
    license='MIT',  # Update the license here
    entry_points={
        'console_scripts': [
            'planner_node = astar_planner.planner_node:main',
        ],
    },
)



