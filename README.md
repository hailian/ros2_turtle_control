# ros2_turtle_control

基于 **ROS 2 Humble** 的 **TurtleBot3 Burger** 控制工作区,封装了从建图、定位到自主导航的完整能力。三个功能包层层依赖,构成一条清晰的机器人落地链路:

**建图 (SLAM) → 定位 (Localization) → 导航 (Navigation)**

## 功能包概览

| 包 | 职责 | 入口 launch |
| --- | --- | --- |
| [`turtlebot3_slam`](src/turtlebot3_slam/README.md) | SLAM 建图,保存可复用地图 | `slam.launch.py` |
| [`turtlebot3_localization`](src/turtlebot3_localization/README.md) | AMCL 定位 / 全局自动重定位 | `localization.launch.py`、`relocalization.launch.py` |
| [`turtlebot3_navigation`](src/turtlebot3_navigation/README.md) | nav2 全栈路径规划与运动控制 / 航点巡逻 / 边扫图边导航 | `navigation.launch.py`、`patrol.launch.py`、`slam_navigation.launch.py` |

各包均自带详细 README(用法、launch 参数、实测数据、常见问题),本文档只做总览。

## 环境要求

- **ROS 2 Humble**(推荐 Ubuntu 22.04)
- **TurtleBot3 Burger**(LDS-01 激光雷达)
- 仿真依赖:`turtlebot3_gazebo`、`turtlebot3_teleop`
- 导航依赖:`nav2_*` 系列、`slam_toolbox`(建图与边扫图边导航后端)
- 三个包均为 `ament_python` 构建类型,由 `colcon` 编译

首次拉取仓库后安装缺失的系统/ROS 依赖:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
```

## 编译

```bash
cd ~/codespace/ros2_turtle_control
colcon build --packages-select turtlebot3_slam turtlebot3_localization turtlebot3_navigation
source install/setup.bash
```

只编译改动到的包即可,例如:`colcon build --packages-select turtlebot3_navigation`。

## 典型工作流

### 1. 建图(首次进场)

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py       # 终端 1:仿真
ros2 launch turtlebot3_slam slam.launch.py                     # 终端 2:建图
ros2 run turtlebot3_teleop teleop_keyboard                     # 终端 3:遥控绕行
ros2 service call /save_map std_srvs/srv/Trigger "{}"          # 保存地图
```

地图默认保存到 `~/turtlebot3_maps/`(`map_时间戳.pgm` + `.yaml`)。

### 2. 定位(常规定位 / 重定位)

```bash
ros2 launch turtlebot3_localization localization.launch.py
# 或机器人被搬动后全局自动求解位置:
ros2 launch turtlebot3_localization relocalization.launch.py
```

### 3. 导航(交互 / 巡逻 / 边扫图边导航)

```bash
ros2 launch turtlebot3_navigation navigation.launch.py         # RViz 点选目标
ros2 launch turtlebot3_navigation patrol.launch.py             # 航点自动巡逻
ros2 launch turtlebot3_navigation slam_navigation.launch.py    # 免预建地图,边扫边走
```

## 通用约定

- **地图目录**:默认 `~/turtlebot3_maps/`;`map:=` 显式指定,留空时自动选该目录下**最新**地图。
- **初始位姿**:默认取 turtlebot3_world 出生点 `(-2.0, -0.5, 0.0)`;机器人不在该点时用 `initial_x/y/yaw:=` 覆盖,或在 RViz 用 **2D Pose Estimate** 重设,错误的先验会让 NavFn 规划不出路径。
- **仿真时钟**:所有 launch 默认 `use_sim_time:=true`;切到实机务必改 `false`。
- **日志语言**:节点运行报告与代码注释以中文为主。

## 常见问题速查

- 命令行发目标没反应:加 `-t 2` 发两次规避 DDS 发现竞态(`ros2 topic pub -w 1 -t 2 /goal_pose ...`)。
- 导航卡在终点附近反复恢复:DWB 的 `xy_goal_tolerance`(0.05)必须小于 goal_checker 容差(0.08)。
- `lifecycle_manager` 偶发卡在 Configuring:仿真时钟竞态,launch 已做延迟规避;仍出现则重启 launch。
- 更多见各功能包 README 的「常见问题」一节。

## 目录结构

```
ros2_turtle_control/
├── src/
│   ├── turtlebot3_slam/          # SLAM 建图
│   ├── turtlebot3_localization/  # AMCL 定位 / 自动重定位
│   └── turtlebot3_navigation/    # nav2 导航 / 巡逻 / 边扫边导
├── build/  install/  log/        # colcon 产物(已 gitignore)
└── .gitignore
```

## 许可

三个功能包均基于 **Apache-2.0** 许可证发布(见各 `package.xml` 的 `<license>`)。

> 注:各包 `package.xml` 的 `<maintainer>` 目前为占位值 `user@todo.todo`,
> 正式发布前请替换为实际维护者邮箱与名称。
