# AGENTS.md

面向 AI 编码助手的项目指引。改动本仓库前请先读完。

## 项目性质

- **ROS 2 Humble** 工作区(非 catkin/ROS 1),Ubuntu 22.04。
- 三个 `ament_python` 功能包,用 `colcon` 构建,Python 节点基于 `rclpy`。
- 目标硬件是 **TurtleBot3 Burger**(差速驱动 + LDS-01 激光雷达)。
- 节点运行日志与代码注释以**中文**为主,新增日志/注释请保持一致风格。

## 目录布局

```
src/
├── turtlebot3_slam/          # 建图:slam.launch.py + map_saver_node.py
├── turtlebot3_localization/  # 定位:localization.launch.py / relocalization.launch.py
│                             #   pose_monitor.py / auto_relocalization.py
└── turtlebot3_navigation/    # 导航:navigation / patrol / slam_navigation launch
                              #   nav_monitor.py / waypoint_patrol.py
```
每个包内有自己的 `README.md`(用法、launch 参数表、实测数据、常见问题),是细节的权威来源,改对应包前先读它。

## 构建与运行

```bash
cd ~/codespace/ros2_turtle_control
colcon build --packages-select <pkg>        # 只编改动包,必要时加 --symlink-install
source install/setup.bash                    # 每次新 shell 都要 source
```

- 运行 launch 前务必 `export TURTLEBOT3_MODEL=burger`。
- 实机把相关 launch 的 `use_sim_time:=false`。
- 改动 Python 节点后通常只需重 build(或 `--symlink-install` 免重编);改 `config/*.yaml`/`*.rviz` 无需重编。

## 关键约定与易错点

1. **仿真时钟**:所有 launch 默认 `use_sim_time:=true`。实机必须改 `false`,否则 TF/动作全部卡住。
2. **DWB 容差不等式**:`config/nav2_params.yaml` 中 DWB 的 `xy_goal_tolerance`(0.05)**必须 `<` goal_checker 容差(0.08)。颠倒会卡死在终点附近反复恢复。调整时保持此不等关系。
3. **不要用 `nav2_simple_commander` 的 `waitUntilNav2Active()`**:Humble 版会阻塞等待一条 `/amcl_pose` 消息,静止机器人永远等不到。导航包已改为轮询 lifecycle 状态,新增阻塞等待逻辑时勿回退到此 API。
4. **AMCL 静止时不输出**:AMCL 仅在移动 >0.2 m / 旋转 >0.2 rad 后更新 `/amcl_pose`,这是正常的,不是 bug。
5. **lifecycle_manager 竞态**:仿真时钟下 `change_state` 响应可能丢失,launch 已用延迟启动规避;若新增 nav2 lifecycle 节点,保留该延迟。
6. **DDS 发现竞态**:命令行 `ros2 topic pub /goal_pose` 单发(`--once`/`-w 1`)可能整条丢失,用 `-t 2` 发两次 + `-w 1` 等匹配。
7. **地图选择**:`map` 留空时自动选 `~/turtlebot3_maps/` 下最新地图;初始位姿默认 `(-2.0,-0.5,0.0)`,不在建图出生点时必须覆盖或 RViz 2D Pose Estimate。

## 新增 / 修改节点

- Python 节点放 `<pkg>/<pkg>/` 下,记得更新该目录的 `__init__.py` 与 `setup.py` 的 `entry_points.console_scripts`(否则 `ros2 run` 找不到)。
- launch 文件放 `<pkg>/launch/`,参数用 `LaunchConfiguration` + `DeclareLaunchArgument` 声明,默认值与子包 README 参数表保持一致。
- nav2 参数集中放 `config/`,不要在 launch 里散落硬编码。
- 监测类节点(如 `nav_monitor`/`pose_monitor`)通过订阅 nav2 动作 feedback / TF / 话题实现,不耦合具体调用方,新增监测逻辑沿用此模式。

## 代码风格与许可

- **语言/注释**:Python 节点与运行日志以**中文**为主;文档字符串(docstring)用中文简述原理与参数,关键调优参数在注释里写明取值依据(参考 `nav2_params.yaml` 的逐段注释风格)。
- **格式**:Python 遵循 PEP 8;缩进 4 空格,不使用 tab;文件头保留 `#!/usr/bin/env python3` shebang。
- **launch 参数**:新增可调项一律走 `DeclareLaunchArgument` 暴露,默认值代入子包 README 的「launch 参数」表,保持文档与代码一致。
- **配置与代码分离**:nav2 / AMCL 等第三方参数只在 `config/*.yaml` 调整,不写死进 launch 或节点;改参后无需重编(`colcon build` 只针对代码)。
- **依赖声明**:新增 `<depend>`/`<exec_depend>` 必须同步写进对应 `package.xml`,并在 README「环境要求」/AGENTS「构建与运行」补一句安装说明;可用 `rosdep install --from-paths src --ignore-src -y` 校验。
- **许可**:三个包均为 **Apache-2.0**(见各 `package.xml`)。新增文件沿用该许可证;当前 `<maintainer>` 是占位值 `user@todo.todo`,提交前请替换为真实维护者。

## 验证改动

ROS 2 没有现成单元测试框架跑通整栈。验证方式:

```bash
colcon build --packages-select <pkg> && source install/setup.bash
ros2 launch <pkg> <x>.launch.py     # 观察启动无报错、节点均 active
ros2 node list / ros2 topic list    # 确认节点与话题按预期出现
```

导航/定位类改动用 Gazebo(`turtlebot3_gazebo turtlebot3_world.launch.py`)实测;只改参数时重点核对 launch 日志与 RViz 表现,并对照对应 README「常见问题」自查。
