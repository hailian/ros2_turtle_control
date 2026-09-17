#!/usr/bin/env python3
"""绑架恢复节点(协调器):检测绑架后自动重定位并续跑被中断的目标。

绑架场景:机器人被物理搬移(或 AMCL 在导航中发散),粒子滤波的位姿
估计与真实位置脱节,代价地图被错误扫描污染,导航会撞向错误位置。

检测(满足其一,静默期外):
    1. map -> odom TF 跳变:绑架后 AMCL 粒子被扫描拉扯,TF 会出现
       平移/旋转突跳(正常定位修正为缓变小量);
    2. AMCL 协方差持续爆炸:连续多帧 σxy 超阈值(定位发散)。

恢复流程(状态机,服务回调 + 周期检查驱动):
    取消全部导航目标(cancel-all) → 调用 /relocalize 触发
    turtlebot3_localization 的 auto_relocalization 节点执行全局重定位
    (撒粒子 → 原地环视 → 带激光安全的探索运动,对称环境也能收敛)
    → 监听 /relocalization_status 直至 [重定位成功]/[重定位超时] →
    自动续跑被中断的目标(auto_resume_goal)→ 静默期后回到监测。

订阅:
    /tf, /amcl_pose, /goal_pose,
    navigate_to_pose/_action/status, /relocalization_status
服务:
    navigate_to_pose/_action/cancel_goal, /relocalize
发布:
    /kidnap_status (中文过程报告)

说明:
    - 仅适用于常规导航模式(依赖 AMCL);SLAM 模式无 AMCL,不适用;
    - 取消目标用 rclcpp 动作约定名 <action>/_action/cancel_goal,
      空 goal_id 即取消全部目标(rclpy 命名习惯的 _action/cancel
      在 rclcpp 服务端上不存在);
    - 需与 auto_relocalization 节点(autostart:=false)同栈运行,
      navigation.launch.py 已自动编排;
    - 巡逻任务中被取消的航点按 patrol 节点的取消策略处理(终止任务)。
"""

import math

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import tf2_ros


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# 状态机阶段
MONITORING, CANCELLING, WAITING, RELOCATING, COOLDOWN = range(5)


class KidnapRecovery(Node):

    def __init__(self):
        super().__init__('kidnap_recovery')
        self.declare_parameter('tf_jump_dist', 0.4)
        self.declare_parameter('tf_jump_rot', 1.0)
        self.declare_parameter('sigma_xy_thresh', 0.6)
        self.declare_parameter('sigma_consecutive', 3)
        self.declare_parameter('auto_resume_goal', True)
        self.declare_parameter('warmup_time', 8.0)
        self.declare_parameter('reloc_timeout', 260.0)
        self.declare_parameter('check_period', 0.5)

        p = lambda name: self.get_parameter(name).value
        self._jump_dist = float(p('tf_jump_dist'))
        self._jump_rot = float(p('tf_jump_rot'))
        self._sigma_thresh = float(p('sigma_xy_thresh'))
        self._sigma_consecutive = int(p('sigma_consecutive'))
        self._auto_resume = bool(p('auto_resume_goal'))
        self._warmup = float(p('warmup_time'))
        self._reloc_timeout = float(p('reloc_timeout'))

        self._state = MONITORING
        self._state_since = self.get_clock().now()
        self._prev_tf = None            # 上一帧 map->odom (x, y, yaw)
        self._sigma_xy = None
        self._sigma_high_count = 0      # 连续 σxy 超阈计数
        self._last_goal = None          # 最近 /goal_pose(用于续跑)
        self._goal_active = False       # 当前是否有导航在执行(实时)
        self._goal_active_snapshot = False  # 检测到绑架时刻的快照

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self.create_subscription(
            PoseStamped, 'goal_pose', self._on_goal_pose, 10)
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status',
            self._on_action_status, 10)
        self.create_subscription(
            String, 'relocalization_status', self._on_reloc_status, 10)

        self._cancel_cli = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self._relocalize_cli = self.create_client(Empty, 'relocalize')
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._status_pub = self.create_publisher(String, 'kidnap_status', 10)
        self.create_timer(float(p('check_period')), self._check)

        self.get_logger().info(
            'kidnap_recovery 就绪:监测 map->odom 跳变与 AMCL 协方差,'
            '检测到绑架后取消目标、触发自动重定位并续跑')

    # ---- 订阅回调 -------------------------------------------------------

    def _on_pose(self, msg):
        cov = msg.pose.covariance   # 6x6:x y z roll pitch yaw
        self._sigma_xy = math.hypot(
            math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[7], 0.0)))
        if self._state != MONITORING:
            return
        if self._sigma_xy > self._sigma_thresh:
            self._sigma_high_count += 1
            if self._sigma_high_count >= self._sigma_consecutive:
                self._begin_recovery(
                    f'AMCL 协方差连续 {self._sigma_high_count} 帧超阈'
                    f'(σxy={self._sigma_xy:.2f} m),定位发散')
        else:
            self._sigma_high_count = 0

    def _on_goal_pose(self, msg):
        self._last_goal = msg

    def _on_action_status(self, msg):
        if msg.status_list:
            code = msg.status_list[-1].status
            self._goal_active = code in (1, 2)   # ACCEPTED / EXECUTING

    def _on_reloc_status(self, msg):
        if self._state != RELOCATING:
            return
        if '[重定位成功]' in msg.data:
            self._finish_recovery(msg.data)
        elif '[重定位超时]' in msg.data:
            self._finish_recovery(msg.data)

    # ---- 状态机 ----------------------------------------------------------

    def _elapsed_in_state(self):
        return (self.get_clock().now() - self._state_since).nanoseconds / 1e9

    def _switch_state(self, state):
        self._state = state
        self._state_since = self.get_clock().now()

    def _check(self):
        if self._state == MONITORING:
            self._monitor_tf()
        elif self._state == WAITING:
            # 取消已下发,等控制器停稳再触发重定位
            if self._elapsed_in_state() >= 1.5:
                self._switch_state(RELOCATING)
                if self._relocalize_cli.wait_for_service(timeout_sec=1.0):
                    self._relocalize_cli.call_async(Empty.Request())
                    self._publish('[绑架恢复] 已触发自动重定位'
                                  '(撒粒子 → 环视 → 探索收敛),期间机器人'
                                  '自主运动,请勿遥控干预')
                else:
                    self._publish('[绑架恢复] /relocalize 服务不可用:'
                                  '请确认 auto_relocalization 节点已启动'
                                  '(autostart:=false 由 launch 编排)')
                    self._switch_state(COOLDOWN)
        elif self._state == RELOCATING:
            if self._elapsed_in_state() > self._reloc_timeout:
                self._publish('[绑架恢复] 等待重定位结果超时'
                              f'({self._reloc_timeout:.0f} s),回到监测')
                self._switch_state(COOLDOWN)
        elif self._state == COOLDOWN:
            if self._elapsed_in_state() >= self._warmup:
                self._publish('[绑架恢复] 静默期结束,恢复监测')
                self._switch_state(MONITORING)

    def _monitor_tf(self):
        if self._elapsed_in_state() < self._warmup:
            return   # 启动/恢复静默期:AMCL 初始修正会有自然跳变
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_from_quaternion(t.transform.rotation)
        if self._prev_tf is not None:
            dist = math.hypot(x - self._prev_tf[0], y - self._prev_tf[1])
            dyaw = abs(math.atan2(
                math.sin(yaw - self._prev_tf[2]),
                math.cos(yaw - self._prev_tf[2])))
            if dist > self._jump_dist or dyaw > self._jump_rot:
                self._begin_recovery(
                    f'map->odom TF 跳变 {dist:.2f} m / {math.degrees(dyaw):.0f}°')
                return
        self._prev_tf = (x, y, yaw)

    def _begin_recovery(self, reason):
        self._publish(f'[绑架恢复] 检测到绑架:{reason}。'
                      '取消导航目标并启动自动重定位')
        # 快照"检测时是否有导航在执行":重定位结束时据此决定是否续跑
        self._goal_active_snapshot = self._goal_active
        self._switch_state(CANCELLING)
        if self._cancel_cli.wait_for_service(timeout_sec=1.0):
            future = self._cancel_cli.call_async(
                CancelGoal.Request())   # 空 goal_id = 取消全部目标
            future.add_done_callback(lambda _: self._switch_state(WAITING))
        else:
            self._switch_state(WAITING)   # 无目标可取消,直接进入等待

    def _finish_recovery(self, reloc_result):
        self._prev_tf = None             # 重定位后 TF 基准重建
        self._sigma_high_count = 0
        if '[重定位成功]' in reloc_result:
            self._publish(f'[绑架恢复] {reloc_result}')
            if (self._auto_resume and self._goal_active_snapshot
                    and self._last_goal is not None):
                goal = NavigateToPose.Goal()
                goal.pose = self._last_goal
                if self._nav_client.wait_for_server(timeout_sec=2.0):
                    self._nav_client.send_goal_async(goal)
                    g = self._last_goal.pose.position
                    self._publish(f'[绑架恢复] 续跑被中断的目标 '
                                  f'({g.x:+.2f}, {g.y:+.2f})')
        else:
            self._publish(f'[绑架恢复] {reloc_result};建议 RViz 2D Pose '
                          'Estimate 手动修正后重发目标')
        self._switch_state(COOLDOWN)

    def _publish(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KidnapRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
