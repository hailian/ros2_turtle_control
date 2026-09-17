# turtlebot3_localization — TurtleBot3 定位模块

在**已建好的地图**(由 `turtlebot3_slam` 模块建图保存)上,通过激光扫描与
地图的持续匹配,实时分析判断机器人自身位置。提供两种入口:

- **`localization.launch.py`** 已知初始位姿的常规定位(AMCL 跟踪)
- **`relocalization.launch.py`** 全局自动重定位:不设初始位姿、不用 RViz
  2D Pose Estimate,机器人自行环绕运动求解出自己的位置

组成:

- **nav2 map_server** 导入地图并发布 `/map`
- **nav2 AMCL** 蒙特卡洛定位:里程计运动预测 + 激光似然场匹配 + 粒子滤波
  重采样,持续输出位姿估计 `/amcl_pose` 与粒子云 `/particle_cloud`,
  并发布 `map → odom` TF 修正量
- **pose_monitor** 位姿监测节点:汇总扫描环境特征与位姿置信度,
  周期输出中文分析报告(话题 `/localization_status`)
- **auto_relocalization** 自动重定位节点:全局撒布粒子 → 原地环视 360° →
  朝开阔方向探索(带激光安全保护/脱困/看门狗),协方差收敛后自动停车
  并报告"我在哪"(话题 `/relocalization_status`)

实测(Gazebo turtlebot3_world,burger):
- 常规定位:缓慢遥控绕行 4 圈,估计位置与真值误差约 **4 cm**,无发散
- 自动重定位(出生点不同于建图起点、零初始信息):自动探索 4.7 m、
  **105 s** 收敛,报告位置与真值误差 **< 10 cm**

## 快速开始(仿真)

前提:已用 `turtlebot3_slam` 模块建好地图(默认目录 `~/turtlebot3_maps`)。

```bash
cd ~/codespace/ros2_turtle_control
colcon build --packages-select turtlebot3_localization
source install/setup.bash
```

**终端 1 — 仿真:**

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**终端 2 — 定位模块:**

```bash
ros2 launch turtlebot3_localization localization.launch.py
```

**终端 3 — 遥控移动(静止时 AMCL 不更新,需移动触发):**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

RViz 中查看:地图上的机器人模型、红色激光点云(与地图墙体重合说明定位准)、
蓝色箭头粒子云。终端 2 每 2 秒输出定位分析报告,例如:

```
[定位报告] 位姿: x=+0.43 y=+0.61 yaw=+2.70 rad (+155°) | 置信度: 一般
(σxy=0.181 m, σyaw=0.184 rad) | 环境特征: 359/360 有效波束, 最近障碍 0.50 m
(相对朝向 +180°), 平均测距 1.90 m | 累计更新 150 次
```

## 自动重定位(机器人环绕一周自己算出位置)

不设初始位姿、不用 RViz 2D Pose Estimate:

```bash
ros2 launch turtlebot3_localization relocalization.launch.py
```

启动后自动执行:

1. 调用 AMCL `reinitialize_global_localization` 服务,粒子均匀撒满全图自由空间
2. 原地旋转 360°,360° 激光全向观测,快速剔除错误假设
3. 协方差未收敛则继续探索:朝激光最开阔的可行方向直行(默认 2.2 m,
   前方 0.3 m 内有障碍自动中止转向;被围困时先后退脱困;状态卡死 45 s
   看门狗强制转向)
4. σxy/σyaw 连续多帧低于阈值 **且累计行程 ≥ 2.5 m**(防止在对称环境中
   "紧密地收敛到错误位置")后停车,输出最终位置报告

```
[重定位成功] 机器人位置: x=+0.33 m, y=-1.75 m, 朝向 -0.65 rad (-37°) |
σxy=0.329 m, σyaw=0.225 rad | 累计行程 4.7 m, 绕行 0 圈, 用时 105 s
```

机器人被搬动("绑架")后重新求解:

```bash
ros2 service call /relocalize std_srvs/srv/Empty
```

launch 参数(在通用参数之外):

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `loop_leg` | 2.2 | 探索直行腿长 m |
| `timeout` | 240.0 | 重定位总超时 s |

节点内参数(可用 `--ros-args -p` 覆盖):`sigma_xy_thresh`(0.3)、
`sigma_yaw_thresh`(0.25)、`min_traveled`(2.5)、`safety_dist`(0.3)、
`rotate_speed`(0.4)、`drive_speed`(0.1)、`autostart`(true,启动即
自动求解;turtlebot3_navigation 的绑架恢复以 `autostart:=false`
复用本节点,按需经 `/relocalize` 触发)等。

**提示**:高度对称的环境(如 turtlebot3_world 的 3×3 圆柱阵列)存在镜像/
旋转歧义,自动重定位需要足够的探索行程才能打破——若在超时内未收敛,
可加大 `timeout`、遥控协助走一段,或最后手段用 RViz 2D Pose Estimate。

## 地图与初始位姿

- **地图选择**:`map:=/path/to/map.yaml` 显式指定;留空时自动选取
  `map_dir`(默认 `~/turtlebot3_maps`)下**最新**的地图。
- **初始位姿**:粒子滤波需要先验。默认 `set_initial_pose:=true` 且
  `initial_x/y/yaw` 取 (-2.0, -0.5, 0.0)——turtlebot3_world 的出生点
  (slam_toolbox 建图时 map 系与出生点对齐,实测偏移 <1 cm)。
  换了出生点或拿不准时,在 RViz 用工具栏 **2D Pose Estimate** 在地图上
  点按拖出机器人大致位置与朝向,即可重设 `/initialpose`。
- 机器人被搬动("绑架")后同样用 2D Pose Estimate 重定位。

## launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map` | 空(自动选最新) | 地图 yaml 路径 |
| `map_dir` | `~/turtlebot3_maps` | 自动选图目录 |
| `use_sim_time` | `true` | 仿真时钟;实机改 `false` |
| `set_initial_pose` | `true` | 用 launch 参数作为初始位姿 |
| `initial_x / initial_y / initial_yaw` | -2.0 / -0.5 / 0.0 | 初始位姿 |
| `pose_monitor` | `true` | 是否启动位姿监测节点 |
| `report_period` | 2.0 | 报告周期(秒) |
| `use_rviz` | `true` | 是否启动 RViz2 |

## pose_monitor 输出说明

- **置信度**:由 AMCL 协方差计算(σxy 为水平位置标准差,σyaw 为朝向
  标准差),分四档:优秀(<0.05 m)、良好(<0.15 m)、一般(<0.30 m)、偏低。
  σ 是保守上界——实测 σ≈0.18 时真实误差可低至 4 cm。
- **环境特征**:当前帧扫描的有效波束数、最近障碍距离及其相对机器人
  朝向的方位、平均测距——用于人工核对机器人相对环境的姿态是否合理。
- 订阅话题:`ros2 topic echo /localization_status`

## 常见问题

- **一直显示"尚未收到 AMCL 位姿"**:AMCL 只在移动超过 0.2 m / 旋转超过
  0.2 rad 后才更新;先用遥控缓慢移动。确认 `ros2 lifecycle get /amcl`
  为 `active`。
- **置信度偏低/位置不对**:多为初始位姿偏差未完全收敛——保持慢速
  (≤0.15 m/s)多走一段;或用 RViz 2D Pose Estimate 重设;避免贴墙、
  贴柱行驶(遮挡激光)。
- **自动重定位超时**:见上文"高度对称环境"提示;nav2 Humble 的 AMCL
  全局重定位服务名为 `reinitialize_global_localization`(非 ROS 1 的
  `global_localization`),调用错名字会挂起无响应。
- **lifecycle_manager 偶发卡在 Configuring**:仿真时钟下配置响应丢失的
  竞态,本 launch 已通过延迟 2 秒启动管理器规避;若仍出现,重启 launch
  即可。
- **两张地图来回切换**:与本包无关,通常是 `/map` 上有两个发布者
  (重复启动了 map_server 或建图节点),`ros2 topic info /map -v` 排查。

## 目录结构

```
turtlebot3_localization/
├── launch/
│   ├── localization.launch.py      # 常规定位入口(初始位姿已知)
│   └── relocalization.launch.py    # 自动重定位入口(全局求解)
├── config/
│   ├── amcl.yaml                   # AMCL burger 调优参数
│   └── tb3_localization.rviz       # RViz 定位视图
├── turtlebot3_localization/
│   ├── pose_monitor.py             # 位姿监测节点
│   └── auto_relocalization.py      # 自动重定位节点
├── package.xml
└── setup.py
```
