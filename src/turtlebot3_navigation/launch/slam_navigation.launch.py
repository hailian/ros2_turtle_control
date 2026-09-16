#!/usr/bin/env python3
"""边扫图边导航入口:slam_toolbox 在线建图 + nav2 全栈路径规划,无需预建地图。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py        # 终端 1
    ros2 launch turtlebot3_navigation slam_navigation.launch.py     # 终端 2
    RViz 中 "2D Goal Pose" 点选已探索区域内的目标点即可导航

与 navigation.launch.py 的区别:
    - 不启动 map_server / amcl:slam_toolbox 在线增量建图,自己发布
      /map(静态层代价地图随之生长)与 map -> odom TF(定位来源)
    - 全局规划允许穿越未知区域(allow_unknown),机器人边走边扫,
      行为树 1 Hz 重规划,新扫到的障碍会即时改变路径
    - 目标点最好选在已探索(可见灰白)区域内;指向完全未知区域的
      目标也能规划,但路径笔直穿过未知区,只能靠局部避障兜底

建图成果随手保存(可选,map_saver:=true 时):
    ros2 service call /save_map std_srvs/srv/Trigger "{}"

参数与 navigation.launch.py 同名同义(params_file 默认同一份
nav2_params.yaml,其中 map_server/amcl 段在本模式下不启动、不生效);
新增 map_saver / save_dir / map_name 三个建图相关参数。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_stack(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    run_monitor = LaunchConfiguration('nav_monitor')

    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if not os.path.isfile(params_file):
        raise RuntimeError(f'nav2 参数文件不存在: {params_file}')

    common = [params_file, {'use_sim_time': use_sim_time}]

    # ---- 在线建图:slam_toolbox(参数复用 turtlebot3_slam 模块调优) ----
    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[
            os.path.join(get_package_share_directory('turtlebot3_slam'),
                         'config', 'slam_toolbox.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- nav2 全栈(无 map_server / amcl,地图与定位均来自 slam) -------
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

    # slam_toolbox 非生命周期节点,不在管理清单内;lifecycle_manager
    # 只按序激活 nav2 各节点,同样延迟 3 秒规避仿真时钟竞态。
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
                'node_names': ['planner_server', 'controller_server',
                               'behavior_server', 'bt_navigator',
                               'waypoint_follower'],
            }],
        )])

    # ---- 导航监测(SLAM 模式下位姿取自 map->odom TF) --------------------
    nav_monitor_node = Node(
        package='turtlebot3_navigation', executable='nav_monitor',
        name='nav_monitor', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'report_period': LaunchConfiguration('report_period')},
            {'relay_goal': LaunchConfiguration('relay_goal')},
        ],
        condition=IfCondition(run_monitor),
    )

    map_saver_node = Node(
        package='turtlebot3_slam', executable='map_saver',
        name='map_saver', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_dir': LaunchConfiguration('save_dir')},
            {'map_name': LaunchConfiguration('map_name')},
        ],
        condition=IfCondition(LaunchConfiguration('map_saver')),
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
        LogInfo(msg='边扫图边导航:slam_toolbox 在线建图 + nav2 路径规划'),
        slam_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_node,
        nav_monitor_node,
        map_saver_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(
                get_package_share_directory('turtlebot3_navigation'),
                'config', 'nav2_params.yaml'),
            description='nav2 参数文件路径(map_server/amcl 段不生效)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用 Gazebo 仿真时钟(实机改为 false)'),
        DeclareLaunchArgument(
            'map_saver', default_value='true',
            description='是否启动地图保存节点(/save_map 服务)'),
        DeclareLaunchArgument(
            'save_dir', default_value='~/turtlebot3_maps',
            description='地图保存目录'),
        DeclareLaunchArgument(
            'map_name', default_value='map',
            description='地图名(实际保存自动加时间戳后缀)'),
        DeclareLaunchArgument(
            'nav_monitor', default_value='true',
            description='是否启动导航监测节点'),
        DeclareLaunchArgument(
            'relay_goal', default_value='true',
            description='nav_monitor 是否把 /goal_pose 转发为导航动作'),
        DeclareLaunchArgument(
            'report_period', default_value='2.0',
            description='导航监测报告周期(秒)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz2 可视化'),
        OpaqueFunction(function=_launch_stack),
    ])
