#!/usr/bin/env python3
"""自动巡逻入口:启动完整导航栈后,waypoint_patrol 节点自主巡航一串航点。

用法(仿真):
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py      # 终端 1
    ros2 launch turtlebot3_navigation patrol.launch.py            # 终端 2

默认航点为 turtlebot3_world 内圈 4 个空旷点,绕障碍一圈;可覆盖:
    waypoints:='-1.5,0.5; 1.0,1.7,1.57; 1.9,-1.0'   # "x,y[,yaw]" 分号分隔
    loop_count:=2            # 巡逻圈数(0 = 无限)
    wait_at_waypoint:=3.0    # 每点停留秒数

地图/初始位姿等参数与 navigation.launch.py 相同(自动透传);
RViz 默认不启动(巡逻无人值守),需要时 use_rviz:=true。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_patrol(context):
    use_sim_time = LaunchConfiguration('use_sim_time')

    # waypoints launch 参数为 "x,y[,yaw];x,y[,yaw];..." 透传给节点
    waypoint_list = [w.strip()
                     for w in LaunchConfiguration('waypoints')
                     .perform(context).split(';') if w.strip()]

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation'),
            'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'map_dir': LaunchConfiguration('map_dir'),
            'use_sim_time': use_sim_time,
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            # nav_monitor 通过动作状态话题监测所有来源的导航,无需特殊配置
            'nav_monitor': 'true',
        }.items(),
    )

    patrol_params = {
        'use_sim_time': use_sim_time,
        'loop_count': LaunchConfiguration('loop_count'),
        'wait_at_waypoint': LaunchConfiguration('wait_at_waypoint'),
        'on_failure': LaunchConfiguration('on_failure'),
        'waypoints': waypoint_list,
    }

    patrol_node = Node(
        package='turtlebot3_navigation', executable='waypoint_patrol',
        name='waypoint_patrol', output='screen',
        parameters=[patrol_params],
    )

    return [navigation_launch, patrol_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints',
            default_value='-1.5,0.5; 1.0,1.7; 1.9,-1.0; -1.0,-1.9',
            description='航点列表:"x,y[,yaw];x,y[,yaw];..."(yaw 缺省朝向下一点)'),
        DeclareLaunchArgument(
            'loop_count', default_value='1',
            description='巡逻圈数,0 表示无限循环'),
        DeclareLaunchArgument(
            'wait_at_waypoint', default_value='2.0',
            description='每个航点停留时长(秒)'),
        DeclareLaunchArgument(
            'on_failure', default_value='continue',
            description='单点失败策略:continue 跳过 / abort 终止'),
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
            description='初始朝向 yaw(弧度)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='是否启动 RViz2 可视化(巡逻默认无人值守)'),
        OpaqueFunction(function=_launch_patrol),
    ])
