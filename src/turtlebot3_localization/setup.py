from setuptools import setup

package_name = 'turtlebot3_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/localization.launch.py',
             'launch/relocalization.launch.py']),
        ('share/' + package_name + '/config',
            ['config/amcl.yaml', 'config/tb3_localization.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description=(
        'Localization module for TurtleBot3 (burger): loads a previously '
        'built map, runs AMCL particle-filter localization against live '
        'laser scans, and reports pose estimates with confidence analysis.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_monitor = turtlebot3_localization.pose_monitor:main',
            'auto_relocalization = '
            'turtlebot3_localization.auto_relocalization:main',
        ],
    },
)
