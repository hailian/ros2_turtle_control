# turtlebot3_slam — TurtleBot3 SLAM 建图模块

针对 **TurtleBot3 Burger**(LDS-01 激光雷达)封装的一键 SLAM 建图模块,包含:

| 内容 | 说明 |
| --- | --- |
| `launch/slam.launch.py` | 建图入口:建图后端 + 地图保存节点 + RViz |
| `config/slam_toolbox.yaml` | slam_toolbox 参数,按 burger 的 LDS-01(360° / 0.12~3.5 m / 5 Hz)调优 |
| `config/tb3_slam.rviz` | RViz 可视化配置(地图、激光、TF、机器人模型) |
| `turtlebot3_slam/map_saver_node.py` | `map_saver` 节点:订阅 `/map`,通过 `/save_map` 服务保存地图 |

支持两种建图后端(参数 `slam_method` 切换):

- **`slam_toolbox`**(默认)— 支持回环检测与地图序列化,推荐
- **`cartographer`** — 使用官方 `turtlebot3_cartographer` 配置

## 环境依赖

- ROS 2 Humble
- `turtlebot3_gazebo`、`turtlebot3_teleop`(仿真与遥控)
- `slam_toolbox` 或 `cartographer_ros`(二选一即可,默认方案只需 slam_toolbox)
- `nav2_map_server`(可选,只在用官方命令行保存地图时需要)

## 快速开始(Gazebo 仿真)

```bash
cd ~/codespace/ros2_turtle_control
colcon build --packages-select turtlebot3_slam
source install/setup.bash
```

**终端 1 — 启动仿真:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**终端 2 — 启动 SLAM 建图模块:**

```bash
source install/setup.bash
ros2 launch turtlebot3_slam slam.launch.py
```

**终端 3 — 键盘遥控建图:**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

驾驶机器人缓慢绕行环境,建议先沿外圈走一遍再走内部,回到起点形成闭环,
地图质量会明显提升。RViz 中观察 `/map` 直到地图完整。

**终端 2(或新终端)— 保存地图:**

```bash
ros2 service call /save_map std_srvs/srv/Trigger "{}"
```

地图默认保存到 `~/turtlebot3_maps/`,生成 `map_时间戳.pgm`(栅格图)
和 `map_时间戳.yaml`(地图元数据),可直接用 nav2 的 `map_server` 加载。

## launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `slam_method` | `slam_toolbox` | 建图后端:`slam_toolbox` / `cartographer` |
| `use_sim_time` | `true` | 仿真时钟;实机需改为 `false` |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `map_saver` | `true` | 是否启动地图保存节点 |
| `save_dir` | `~/turtlebot3_maps` | 地图保存目录 |
| `map_name` | 空(自动时间戳) | 地图文件名前缀 |

示例:

```bash
# 使用 cartographer 后端,保存到指定目录并指定文件名
ros2 launch turtlebot3_slam slam.launch.py \
    slam_method:=cartographer \
    save_dir:=~/maps map_name:=tb3_world
```

## map_saver 节点

- 订阅 `/map`(`nav_msgs/OccupancyGrid`),始终缓存最新一帧地图
- 服务 `/save_map`(`std_srvs/Trigger`)触发保存
- 输出与 nav2 兼容:PGM 中 254=空闲、0=占用、205=未知;YAML 含
  `resolution / origin / occupied_thresh / free_thresh`
- 可单独运行:

```bash
ros2 run turtlebot3_slam map_saver --ros-args \
    -p save_dir:=~/turtlebot3_maps -p map_name:=my_map
```

也可以不依赖本节点,直接用官方工具保存:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_maps/map
```

## 常见问题

- **RViz 两张地图来回切换,日志反复打印两条 `Trying to create a map of size ...`**:
  该日志是 RViz 每次收到新尺寸地图时的正常输出;两条不同尺寸交替出现说明
  `/map` 上有**两个发布者**(两个建图节点同时在跑,例如本模块与官方
  `turtlebot3_cartographer` 的 launch 同时启动,或本模块被启动了两次)。
  排查与处理:

  ```bash
  ros2 topic info /map -v        # Publisher count 应为 1
  ros2 node list | grep -iE "slam|carto"   # 只应有一个建图后端
  ```

  把多余的建图节点停掉(整个终端 Ctrl+C,或按 PID kill),只保留一个即可。
- **`[ERROR] [gzclient]: process has died`**(无显示器环境):
  gzclient 图形界面无法启动,但 gzserver 仿真正常,不影响建图。
- **RViz 看不到地图**:确认 launch 使用 `use_sim_time:=true`,
  且 `/scan`、`/odom`、TF `map → odom → base_footprint` 正常
  (`ros2 run tf2_ros tf2_echo map odom`)。
- **建图漂移**:遥控时降低线速度/角速度(slam_toolbox 参数中
  `minimum_travel_distance/heading` 已按 burger 收紧至 0.3 m/rad)。

## 目录结构

```
turtlebot3_slam/
├── launch/slam.launch.py        # 建图入口 launch
├── config/
│   ├── slam_toolbox.yaml        # slam_toolbox burger 调优参数
│   └── tb3_slam.rviz            # RViz 配置
├── turtlebot3_slam/
│   └── map_saver_node.py        # 地图保存节点
├── package.xml
└── setup.py
```
