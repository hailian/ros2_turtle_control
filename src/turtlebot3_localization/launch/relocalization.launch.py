#!/usr/bin/env python3
"""自动重定位入口:导入地图后不设初始位姿,机器人自行环绕运动,
由粒子滤波收敛计算出自己在地图中的位置(全局重定位)。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py        # 终端 1
    ros2 launch turtlebot3_localization relocalization.launch.py   # 终端 2

启动后自动执行:
    1. /global_localization 全图撒布粒子
    2. 原地旋转 360° 全向观测
    3. 未收敛则自动绕行小方形回路(激光安全保护)
    4. 收敛后停车并在 /relocalization_status 报告位置

与 localization.launch.py 的区别:无需(也不应)提供 initial_x/y/yaw;
需要再次重定位(如机器人被搬动)时调用:
    ros2 service call /relocalize std_srvs/srv/Empty
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

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

    set_initial_pose = ParameterValue(
        LaunchConfiguration('set_initial_pose'), value_type=bool)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml}],
    )

    # 关键:默认不设初始位姿,由全局撒布粒子 + 环绕运动求解
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('turtlebot3_localization'),
                         'config', 'amcl.yaml'),
            {'use_sim_time': use_sim_time},
            {'set_initial_pose': set_initial_pose},
        ],
    )

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

    auto_relocalization_node = Node(
        package='turtlebot3_localization',
        executable='auto_relocalization',
        name='auto_relocalization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'loop_leg': ParameterValue(
                LaunchConfiguration('loop_leg'), value_type=float),
            'timeout': ParameterValue(
                LaunchConfiguration('timeout'), value_type=float),
        }],
    )

    pose_monitor_node = Node(
        package='turtlebot3_localization',
        executable='pose_monitor',
        name='pose_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('pose_monitor')),
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
        LogInfo(msg=f'重定位使用地图: {map_yaml}'),
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        auto_relocalization_node,
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
            'set_initial_pose', default_value='false',
            description='全局重定位应为 false(粒子全图撒布,无需初始位姿)'),
        DeclareLaunchArgument(
            'loop_leg', default_value='2.2',
            description='探索直行腿长 m(遇障碍自动中止并转向)'),
        DeclareLaunchArgument(
            'timeout', default_value='240.0',
            description='重定位总超时 s'),
        DeclareLaunchArgument(
            'pose_monitor', default_value='true',
            description='是否同时启动位姿监测节点'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 可视化'),
        OpaqueFunction(function=_resolve_map_and_launch),
    ])
