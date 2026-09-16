#!/usr/bin/env python3
"""路径规划模块入口:加载地图与 AMCL 定位,启动 nav2 全栈路径规划。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py        # 终端 1
    ros2 launch turtlebot3_navigation navigation.launch.py          # 终端 2
    RViz 中用 "2D Goal Pose" 工具点选目标点,机器人自主规划路径并巡航
    (也可: ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped ...)

地图来源(优先级从高到低):
    1. map:=/path/to/map.yaml 显式指定
    2. map 留空时,自动选取 map_dir(默认 ~/turtlebot3_maps)下最新的地图

启动的节点:
    map_server / amcl          地图服务与定位(同 turtlebot3_localization)
    planner_server             NavFn 全局规划(代价地图上 A* 搜索)
    controller_server          DWB 局部控制(速度采样 + 轨迹打分跟踪路径)
    behavior_server            恢复行为(旋转/后退/等待/清代价地图)
    bt_navigator               行为树调度:周期重规划 + 失败恢复
                               (原生订阅 /goal_pose,直接响应目标点)
    waypoint_follower          多航点跟随(备用)
    nav_monitor                导航监测:目标/路径/进度/速度 中文报告,
                               经动作状态话题覆盖所有来源的导航任务
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
    run_monitor = LaunchConfiguration('nav_monitor')
    set_initial_pose = LaunchConfiguration('set_initial_pose')

    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if not os.path.isfile(params_file):
        raise RuntimeError(f'nav2 参数文件不存在: {params_file}')

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

    common = [
        params_file,
        {'use_sim_time': use_sim_time},
    ]

    # AMCL 初始位姿(仿真中机器人出生于 turtlebot3_world 的 -2.0, -0.5)
    amcl_extra = {
        'set_initial_pose': ParameterValue(set_initial_pose, value_type=bool),
        'initial_pose.x': ParameterValue(
            LaunchConfiguration('initial_x'), value_type=float),
        'initial_pose.y': ParameterValue(
            LaunchConfiguration('initial_y'), value_type=float),
        'initial_pose.yaw': ParameterValue(
            LaunchConfiguration('initial_yaw'), value_type=float),
    }

    map_server_node = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=common + [{'yaml_filename': map_yaml}],
    )
    amcl_node = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=common + [amcl_extra],
    )
    planner_node = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=common,
    )
    controller_node = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=common,
    )
    behavior_node = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=common,
    )
    bt_navigator_node = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=common,
    )
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=common,
    )

    # 以上均为生命周期节点,由 lifecycle_manager 按上述顺序统一激活。
    # 延迟 3 秒启动,等待各节点服务就绪,规避仿真时钟下的配置响应竞态。
    lifecycle_manager_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'planner_server',
                               'controller_server', 'behavior_server',
                               'bt_navigator', 'waypoint_follower'],
            }],
        )])

    nav_monitor_node = Node(
        package='turtlebot3_navigation', executable='nav_monitor',
        name='nav_monitor', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'report_period': LaunchConfiguration('report_period')},
        ],
        condition=IfCondition(run_monitor),
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('turtlebot3_navigation'),
            'config', 'tb3_navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return [
        LogInfo(msg=f'导航使用地图: {map_yaml}'),
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        nav_monitor_node,
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
            'params_file', default_value=os.path.join(
                get_package_share_directory('turtlebot3_navigation'),
                'config', 'nav2_params.yaml'),
            description='nav2 参数文件路径'),
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
            description='初始朝向 yaw(弧度)'),
        DeclareLaunchArgument(
            'nav_monitor', default_value='true',
            description='是否启动导航监测节点'),
        DeclareLaunchArgument(
            'report_period', default_value='2.0',
            description='导航监测报告周期(秒)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 可视化'),
        OpaqueFunction(function=_resolve_map_and_launch),
    ])
