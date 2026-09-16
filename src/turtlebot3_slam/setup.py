from setuptools import setup

package_name = 'turtlebot3_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/slam.launch.py']),
        ('share/' + package_name + '/config',
            ['config/slam_toolbox.yaml', 'config/tb3_slam.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description=(
        'SLAM mapping module for TurtleBot3 (burger): launch files for '
        'slam_toolbox / cartographer, burger-tuned parameters, an RViz view '
        'and a map_saver node.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_saver = turtlebot3_slam.map_saver_node:main',
        ],
    },
)
