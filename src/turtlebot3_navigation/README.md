# turtlebot3_navigation — TurtleBot3 路径规划模块

在**已建好的地图**(`turtlebot3_slam`)与**稳定定位**(`turtlebot3_localization`
同款 AMCL 配置)之上,启动 nav2 全栈路径规划与运动控制:

- **全局规划** NavFn:静态地图 + 实时激光障碍合成代价地图上 A* 搜索无碰路径,
  行为树 1 Hz 周期重规划应对环境变化
- **局部控制** DWB:速度空间动态窗口采样,前向仿真轨迹打分(贴路径/朝目标/
  避障/防振荡),输出 `/cmd_vel` 差速指令
- **恢复行为**:导航失败时依次尝试清代价地图 → 原地旋转 → 后退 → 等待
- **两个入口**:
  - `navigation.launch.py` 交互导航:RViz 点选目标点,机器人自主规划巡航
  - `patrol.launch.py` 自动巡逻:按航点列表自主巡航,循环巡逻并汇报

组成节点:`map_server`(地图)、`amcl`(定位)、`planner_server`(全局规划)、
`controller_server`(局部控制)、`behavior_server`(恢复行为)、`bt_navigator`
(行为树调度)、`waypoint_follower`(多航点跟随,备用)、`nav_monitor`(导航监测)。

实测(Gazebo turtlebot3_world,burger):
- 单点导航:出生点 → 对角目标 (1.0, 1.7),路径 4.7 m,**22 s** 到达,
  终点直线误差 **0.36 m**(goal_checker 判定成功);返程 8 s,误差 0.14 m
- 自动巡逻:内圈 4 航点绕障一圈,**4/4 全部成功**,平均每段 16 s,
  总用时 73 s,终点误差 0.08 m

## 快速开始(仿真)

前提:已用 `turtlebot3_slam` 建好地图(默认目录 `~/turtlebot3_maps`),
机器人位于建图时的出生点(默认 -2.0, -0.5)。

```bash
cd ~/codespace/ros2_turtle_control
colcon build --packages-select turtlebot3_navigation
source install/setup.bash
```

**终端 1 — 仿真:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**终端 2 — 导航:**

```bash
ros2 launch turtlebot3_navigation navigation.launch.py
```

**发送目标点**(三选一):

1. RViz 工具栏 **2D Goal Pose** 在地图上点选(navn_monitor 转发给导航栈)
2. 命令行:
   ```bash
   ros2 topic pub --once -w 1 /goal_pose geometry_msgs/msg/PoseStamped \
     '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.7}}}'
   ```
3. Nav2 面板(RViz 右侧 Navigation 2 面板)直接下发

终端 2 每 2 秒输出导航报告,例如:

```
[导航报告] 目标: (+1.00, +1.70) | 状态: 导航中 | 沿路径剩余 2.47/2.64 m (6%)
| 距目标直线 2.38 m | 速度 0.20 m/s | 预计 13 s | 已用 11 s | 累计: 成功 0/失败 0/取消 0
```

RViz 中查看:绿色加粗线为全局路径 `/plan`,彩色热力图为全局/局部代价地图
(障碍膨胀层),红色点云为激光,机器人模型实时显示跟踪效果。

## 自动巡逻(航点巡航)

```bash
ros2 launch turtlebot3_navigation patrol.launch.py
```

默认按 turtlebot3_world 内圈 4 个空旷航点绕障巡航一圈,每点停留 2 s。
自动巡逻无人值守,RViz 默认不启动(`use_rviz:=true` 打开)。

```
[巡逻] 任务开始:4 个航点 (-1.50,+0.50) → (+1.00,+1.70) → (+1.90,-1.00) → (-1.00,-1.90),1 圈,每点停留 2 s;...
[巡逻] 到达航点 1/4,本段用时 8 s,停留 2 s
...
[巡逻] 任务完成:共 4 段,成功 4,失败 0(成功率 100%),平均每段 16 s,总用时 73 s
```

自定义任务:

```bash
# 航点 "x,y[,yaw]" 分号分隔;yaw 缺省自动朝向下一航点(首尾相接延续)
ros2 launch turtlebot3_navigation patrol.launch.py \
  waypoints:='-1.5,0.5; 1.0,1.7,1.57; 1.9,-1.0' \
  loop_count:=2 wait_at_waypoint:=3.0 on_failure:=continue
```

launch 参数(patrol 专属):

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `waypoints` | `-1.5,0.5; 1.0,1.7; 1.9,-1.0; -1.0,-1.9` | 航点列表 `x,y[,yaw];...` |
| `loop_count` | 1 | 巡逻圈数,0 = 无限循环 |
| `wait_at_waypoint` | 2.0 | 每点停留时长(秒) |
| `on_failure` | continue | 单点失败策略:continue 跳过 / abort 终止 |

进度话题:`ros2 topic echo /patrol_status`。

## 地图与初始位姿

与 `turtlebot3_localization` 完全一致:

- **地图**:`map:=` 显式指定;留空自动选 `map_dir`(默认 `~/turtlebot3_maps`)
  下最新地图。
- **初始位姿**:默认取 turtlebot3_world 出生点 (-2.0, -0.5, 0.0)。机器人
  不在出生点时务必 `initial_x/y/yaw:=` 覆盖,或在 RViz 用 **2D Pose Estimate**
  重设——错误先验会让激光把障碍标错位置,NavFn 规划不出路径。
- 冷启动拿不准位置:先跑 `turtlebot3_localization` 的
  `relocalization.launch.py` 全局重定位,再启动导航。

## launch 参数(navigation.launch.py)

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map` | 空(自动选最新) | 地图 yaml 路径 |
| `map_dir` | `~/turtlebot3_maps` | 自动选图目录 |
| `params_file` | 本包 config | nav2 参数文件 |
| `use_sim_time` | `true` | 仿真时钟;实机改 `false` |
| `set_initial_pose` | `true` | 用 launch 参数作为 AMCL 初始位姿 |
| `initial_x / initial_y / initial_yaw` | -2.0 / -0.5 / 0.0 | 初始位姿 |
| `nav_monitor` | `true` | 是否启动导航监测节点 |
| `relay_goal` | `true` | 是否把 `/goal_pose` 转发为导航动作(与其他任务节点并存时置 `false`) |
| `report_period` | 2.0 | 报告周期(秒) |
| `use_rviz` | `true` | 是否启动 RViz2 |

## nav_monitor 输出说明

- **状态**:等待目标 / 导航中 / 已到达 / 失败 / 已取消;由
  `navigate_to_pose` 动作结果判定,`到达` 报告附带终点误差与用时。
- **沿路径剩余 / 进度**:取 `/plan` 最新全局路径,先定位机器人所对应的
  最近路径点,再累加其后路径段长——比直线距离更贴近真实行程。
- **预计到达**:路径剩余 ÷ 当前速度(速度 < 0.05 m/s 时不估计)。
- **恢复行为次数**:来自动作反馈,数值持续增长说明机器人反复卡困,
  常见原因是目标点贴近障碍膨胀区或定位漂移。
- 巡逻等外部任务直接调用动作,`nav_monitor` 以"跟踪外部任务"模式
  报告路径与进度,不干预控制。

## 常见问题

- **`ros2 topic pub --once` 发目标没反应**:`--once` 存在订阅匹配竞态,
  加 `-w 1`(等匹配到订阅者再发),本文示例已带。
- **差 0.2 m 左右卡死、反复触发恢复行为**:DWB 的
  `xy_goal_tolerance`(RotateToGoal 进入纯旋转模式的判定)必须**小于**
  goal_checker 的容差,否则两个判死区不重叠,DWB 只旋转不前进而
  goal_checker 永不满足。本包取 0.10 / 0.20,已实测绕障一圈 4/4 成功;
  调整时务必保持这个不等关系。
- **NavFn "failed to create plan"**:优先怀疑初始位姿错误(见上节);
  其次目标点落在障碍/未知区(可加大规划 `tolerance`);`ros2 topic echo
  /global_costmap/costmap` 看实时代价地图是否有大片误标障碍。
- **AMCL 静止时无输出**:AMCL 只在移动超 0.2 m / 旋转超 0.2 rad 后更新,
  静止时 `/amcl_pose` 静默属正常现象。
- **`waypoint_patrol` 卡在"等待导航栈激活"**:本包不使用
  `nav2_simple_commander` 的 `waitUntilNav2Active()`——Humble 版会阻塞
  等待一条 `/amcl_pose` 消息,静止机器人永远等不到;已改为轮询
  amcl/bt_navigator 生命周期状态,120 s 未就绪会明确报错退出。
- **lifecycle_manager 偶发卡在 Configuring**:仿真时钟竞态,launch 已
  延迟 3 秒启动管理器规避;仍出现则重启 launch。

## 目录结构

```
turtlebot3_navigation/
├── launch/
│   ├── navigation.launch.py        # 全栈导航入口(RViz/命令行发目标)
│   └── patrol.launch.py            # 自动巡逻入口(航点巡航)
├── config/
│   ├── nav2_params.yaml            # nav2 全栈参数(AMCL/代价地图/规划/控制)
│   └── tb3_navigation.rviz         # RViz 导航视图(路径/代价地图/Nav2 面板)
├── turtlebot3_navigation/
│   ├── nav_monitor.py              # 导航监测节点(+ /goal_pose 转发)
│   └── waypoint_patrol.py          # 航点巡逻任务节点
├── package.xml
└── setup.py
```
