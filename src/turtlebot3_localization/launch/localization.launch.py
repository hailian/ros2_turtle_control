#!/usr/bin/env python3
"""定位模块入口:导入已建好的地图,启动 AMCL 扫描定位 + 位姿监测 + RViz。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py        # 终端 1
    ros2 launch turtlebot3_localization localization.launch.py      # 终端 2
    ros2 run turtlebot3_teleop teleop_keyboard                      # 终端 3

地图来源(优先级从高到低):
    1. map:=/path/to/map.yaml 显式指定
    2. map 留空时,自动选取 map_dir(默认 ~/turtlebot3_maps)下最新的地图

初始位姿(粒子滤波需要先验):
    1. launch 参数 initial_x / initial_y / initial_yaw(默认对应
       turtlebot3_world 出生点 -2.0, -0.5, 0.0)
    2. 或启动后在 RViz 用 "2D Pose Estimate" 工具手动给定 /initialpose
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_map_and_launch(context):
    """在 launch 解析阶段决定地图路径(支持自动选择最新地图)。"""
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    run_monitor = LaunchConfiguration('pose_monitor')
    set_initial_pose = LaunchConfiguration('set_initial_pose')

    map_yaml = LaunchConfiguration('map').perform(context).strip()
    if not map_yaml:
        map_dir = os.path.expanduser(
            LaunchConfiguration('map_dir').perform(context).strip())
        candidates = sorted(
            glob.glob(os.path.join(map_dir, '*.yaml')),
            key=os.path.getmtime)
        if not candidates:
            raise RuntimeError(
                f'未在 {map_dir} 找到地图(.yaml)。请先用 turtlebot3_slam '
                '模块建图保存,或用 map:=/path/to/map.yaml 指定地图文件。')
        map_yaml = candidates[-1]
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f'地图文件不存在: {map_yaml}')

    # AMCL 初始位姿(仿真中机器人出生于 turtlebot3_world 的 -2.0, -0.5)
    initial_pose = {
        'set_initial_pose': ParameterValue(set_initial_pose, value_type=bool),
        'initial_pose.x': ParameterValue(
            LaunchConfiguration('initial_x'), value_type=float),
        'initial_pose.y': ParameterValue(
            LaunchConfiguration('initial_y'), value_type=float),
        'initial_pose.yaw': ParameterValue(
            LaunchConfiguration('initial_yaw'), value_type=float),
    }

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('turtlebot3_localization'),
                         'config', 'amcl.yaml'),
            {'use_sim_time': use_sim_time},
            initial_pose,
        ],
    )

    # map_server 与 amcl 均为生命周期节点,由 lifecycle_manager 统一激活。
    # 延迟 2 秒启动,等待各节点服务就绪,规避仿真时钟下的配置响应竞态。
    lifecycle_manager_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        )])

    pose_monitor_node = Node(
        package='turtlebot3_localization',
        executable='pose_monitor',
        name='pose_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'report_period': LaunchConfiguration('report_period')},
        ],
        condition=IfCondition(run_monitor),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('turtlebot3_localization'),
            'config', 'tb3_localization.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return [
        LogInfo(msg=f'定位使用地图: {map_yaml}'),
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        pose_monitor_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='地图 yaml 路径;留空则自动选 map_dir 下最新地图'),
        DeclareLaunchArgument(
            'map_dir', default_value='~/turtlebot3_maps',
            description='自动选图时查找的目录'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用 Gazebo 仿真时钟(实机改为 false)'),
        DeclareLaunchArgument(
            'set_initial_pose', default_value='true',
            description='是否使用 launch 参数作为 AMCL 初始位姿'),
        DeclareLaunchArgument(
            'initial_x', default_value='-2.0',
            description='初始位姿 x(默认为 turtlebot3_world 出生点)'),
        DeclareLaunchArgument(
            'initial_y', default_value='-0.5',
            description='初始位姿 y(默认为 turtlebot3_world 出生点)'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='初始位姿朝向 yaw(弧度)'),
        DeclareLaunchArgument(
            'pose_monitor', default_value='true',
            description='是否启动位姿监测节点'),
        DeclareLaunchArgument(
            'report_period', default_value='2.0',
            description='位姿监测报告周期(秒)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 可视化'),
        OpaqueFunction(function=_resolve_map_and_launch),
    ])
