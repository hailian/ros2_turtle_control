from setuptools import setup

package_name = 'turtlebot3_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/navigation.launch.py',
             'launch/patrol.launch.py']),
        ('share/' + package_name + '/config',
            ['config/nav2_params.yaml', 'config/tb3_navigation.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description=(
        'Path planning and navigation module for TurtleBot3 (burger): '
        'full nav2 stack (NavFn global planner + DWB local controller + '
        'behavior trees) on top of a previously built map, with navigation '
        'status monitoring and waypoint patrol missions.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav_monitor = turtlebot3_navigation.nav_monitor:main',
            'waypoint_patrol = '
            'turtlebot3_navigation.waypoint_patrol:main',
        ],
    },
)
