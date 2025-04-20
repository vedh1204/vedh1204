from setuptools import setup
import os

package_name = 'vedh_gazebo'

# Ensure the resource file exists and contains the package name
resource_file_path = os.path.join('resource', package_name)
os.makedirs(os.path.dirname(resource_file_path), exist_ok=True)
with open(resource_file_path, 'w') as f:
    f.write(package_name)

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', [resource_file_path]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/custom_world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/custom_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedh',
    maintainer_email='your@email.com',
    description='A* path planner and Gazebo simulation with TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_path_planner = astar_path_planner:main',
        ],
    },
)


