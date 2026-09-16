#!/usr/bin/env python3
"""SLAM 建图模块入口:一键启动建图后端 + map_saver 节点 + RViz。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py   # 另开终端
    ros2 launch turtlebot3_slam slam.launch.py
    ros2 run turtlebot3_teleop teleop_keyboard                 # 遥控建图
    ros2 service call /save_map std_srvs/srv/Trigger "{}"      # 保存地图

可选参数:
    slam_method:=slam_toolbox|cartographer   建图后端,默认 slam_toolbox
    use_rviz:=true|false                     是否启动 RViz,默认 true
    map_saver:=true|false                    是否启动地图保存节点,默认 true
    save_dir:=~/turtlebot3_maps              地图保存目录
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slam = get_package_share_directory('turtlebot3_slam')
    pkg_cartographer = get_package_share_directory('turtlebot3_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_method = LaunchConfiguration('slam_method')
    use_rviz = LaunchConfiguration('use_rviz')
    run_map_saver = LaunchConfiguration('map_saver')
    save_dir = LaunchConfiguration('save_dir')

    use_slam_toolbox = PythonExpression(
        ["'", slam_method, "' == 'slam_toolbox'"])
    use_cartographer = PythonExpression(
        ["'", slam_method, "' == 'cartographer'"])

    # ---- 建图后端 1: slam_toolbox (默认) ------------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'slam_toolbox.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_slam_toolbox),
    )

    # ---- 建图后端 2: cartographer (官方 turtlebot3 配置) --------------
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            os.path.join(pkg_cartographer, 'config'),
            '-configuration_basename', 'turtlebot3_lds_2d.lua',
        ],
        condition=IfCondition(use_cartographer),
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        condition=IfCondition(use_cartographer),
    )

    # ---- 地图保存节点: /save_map 服务 ---------------------------------
    map_saver_node = Node(
        package='turtlebot3_slam',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_dir': save_dir},
            {'map_name': LaunchConfiguration('map_name')},
        ],
        condition=IfCondition(run_map_saver),
    )

    # ---- RViz 可视化 ---------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_slam, 'config', 'tb3_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用 Gazebo 仿真时钟(仿真必为 true,实机改为 false)'),
        DeclareLaunchArgument(
            'slam_method', default_value='slam_toolbox',
            description='建图后端: slam_toolbox 或 cartographer'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 可视化'),
        DeclareLaunchArgument(
            'map_saver', default_value='true',
            description='是否启动 map_saver 节点(/save_map 服务)'),
        DeclareLaunchArgument(
            'save_dir', default_value='~/turtlebot3_maps',
            description='地图保存目录(~ 开头会展开为用户主目录)'),
        DeclareLaunchArgument(
            'map_name', default_value='',
            description='地图文件名,留空则自动使用 map_时间戳 命名'),
        slam_toolbox_node,
        cartographer_node,
        cartographer_grid_node,
        map_saver_node,
        rviz_node,
    ])
