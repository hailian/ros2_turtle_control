#!/usr/bin/env python3
"""导航监测节点:跟踪目标点与全局路径,输出导航进度中文报告。

订阅:
    /goal_pose  (PoseStamped)              目标点(RViz 2D Goal Pose 或命令行;
                                            bt_navigator 原生订阅该话题执行)
    /amcl_pose  (PoseWithCovarianceStamped) AMCL 位姿估计(常规导航模式)
    /plan       (nav_msgs/Path)            最新全局规划路径
    /odom       (Odometry)                 实际运动速度
    navigate_to_pose/_action/status        动作状态(到达/失败/取消)
    navigate_to_pose/_action/feedback      动作反馈(恢复行为次数)

位姿来源(自动回退):
    常规导航模式用 /amcl_pose;SLAM 模式(边扫图边导航)没有 AMCL,
    改用 map -> base_link TF(slam_toolbox 持续发布 map -> odom)。

说明:Humble 的 bt_navigator 自己订阅 /goal_pose 下发导航,本节点不
转发目标、不干预控制;通过动作状态/反馈话题监测所有来源的导航任务
(RViz 点选、命令行、waypoint_patrol 巡逻),报告结果与统计。

发布:
    /navigation_status (std_msgs/String)   周期性导航分析报告(中文)

报告内容包括:
    - 目标点坐标与朝向、任务状态(导航中/成功/失败/取消)
    - 沿规划路径的剩余里程与进度百分比
    - 距目标直线距离、当前速度、预计到达时间
    - 恢复行为触发次数、累计成功/失败统计
"""

import math

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros

# 动作状态码(action_msgs/GoalStatus)
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6
STATUS_REJECTED = 7


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMonitor(Node):

    def __init__(self):
        super().__init__('nav_monitor')
        self.declare_parameter('report_period', 2.0)

        self._goal = None          # 最近收到的目标点 (PoseStamped)
        self._goal_time = None     # 目标接收时刻
        self._goal_seen_time = None  # /goal_pose 最近到达时刻(用于区分外部目标)
        self._pose = None          # 最近的 AMCL 位姿
        self._plan = None          # 最近一帧全局路径
        self._speed = 0.0          # 实际速度 (m/s)
        self._state = 'idle'       # idle/navigating/succeeded/failed/canceled
        self._feedback = None      # 动作反馈(恢复行为次数等)
        self._recoveries = 0
        self._stats = {'succeeded': 0, 'failed': 0, 'canceled': 0}
        self._active_goal_id = None  # 正在跟踪的导航目标 (goal_id.uuid)

        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status',
            self._on_action_status, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            'navigate_to_pose/_action/feedback', self._on_action_feedback, 10)
        self._status_pub = self.create_publisher(String, 'navigation_status', 10)

        # SLAM 模式回退位姿来源:map -> base_link TF(slam_toolbox 发布)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            'nav_monitor 就绪:监测 navigate_to_pose 动作(RViz/命令行/巡逻任务)'
            '并周期输出导航报告')
        period = float(self.get_parameter('report_period').value)
        self.create_timer(period, self._report)

    # ---- 订阅回调 -------------------------------------------------------

    def _on_goal(self, msg):
        self._goal = msg
        self._goal_time = self.get_clock().now()
        self._goal_seen_time = self._goal_time
        self._state = 'navigating'
        self._feedback = None
        self._recoveries = 0
        yaw = math.degrees(yaw_from_quaternion(msg.pose.orientation))
        p = msg.pose.position
        self.get_logger().info(
            f'[导航] 收到目标: ({p.x:+.2f}, {p.y:+.2f}, '
            f'朝向 {yaw:+.0f}°),已交由 bt_navigator 规划执行')

    def _on_pose(self, msg):
        self._pose = msg

    def _on_plan(self, msg):
        self._plan = msg

    def _on_odom(self, msg):
        t = msg.twist.twist
        self._speed = math.hypot(t.linear.x, t.linear.y)

    # ---- 动作监测(覆盖所有目标来源) -------------------------------------

    def _on_action_feedback(self, msg):
        self._feedback = msg.feedback
        self._recoveries = msg.feedback.number_of_recoveries

    def _on_action_status(self, msg):
        """跟踪最新导航目标的状态迁移,终态时输出结果报告。

        bt_navigator 是单目标任务,状态数组末尾为最新目标;新目标出现
        即开始跟踪(若 /goal_pose 未见则为外部任务,如 waypoint_patrol)。
        """
        if not msg.status_list:
            return
        status = msg.status_list[-1]
        # goal_id.uuid 在 Humble 是 16 字节数组,转 bytes 才能整体比较
        goal_id = bytes(status.goal_info.goal_id.uuid)

        if goal_id != self._active_goal_id:
            if status.status <= STATUS_EXECUTING:
                # 新目标开始执行;/goal_pose 先到时由 _on_goal 记录时刻,
                # 仅从状态话题看到(外部任务)时在此补记
                self._active_goal_id = goal_id
                self._state = 'navigating'
                self._feedback = None
                self._recoveries = 0
                now = self.get_clock().now()
                self._goal_time = now
                recent_goal_pose = (
                    self._goal_seen_time is not None
                    and (now - self._goal_seen_time).nanoseconds < 2e9)
                if not recent_goal_pose:
                    # 外部任务目标(如 waypoint_patrol):清除旧的手动目标,
                    # 报告与误差改用全局路径终点表示
                    self._goal = None
            else:
                return  # 旧目标的迟到终态通知,忽略

        if status.status == STATUS_SUCCEEDED:
            self._state = 'succeeded'
            self._stats['succeeded'] += 1
            self._publish(f'[导航] 到达目标!{self._terminal_error_text()}'
                          f'用时 {self._elapsed():.0f} s')
        elif status.status == STATUS_CANCELED:
            self._state = 'canceled'
            self._stats['canceled'] += 1
            self._publish(f'[导航] 目标已取消({self._terminal_error_text()})')
        elif status.status in (STATUS_ABORTED, STATUS_REJECTED):
            self._state = 'failed'
            self._stats['failed'] += 1
            self._publish(f'[导航] 导航失败:{self._terminal_error_text()}'
                          '常见原因:目标点在障碍物内、路径被动态堵死且'
                          '恢复行为无效、定位漂移')

    def _terminal_error_text(self):
        """终态报告的误差描述:手动目标用其坐标,外部目标用路径终点。"""
        err = self._distance_to_goal()
        if err == float('inf'):
            pose = self._current_pose()
            if self._plan is not None and pose is not None:
                last = self._plan.poses[-1].pose.position
                err = math.hypot(last.x - pose[0], last.y - pose[1])
        if err == float('inf'):
            return ''
        return f'终点误差 {err:.2f} m,'

    # ---- 报告生成 -------------------------------------------------------

    def _elapsed(self):
        if self._goal_time is None:
            return 0.0
        return (self.get_clock().now() - self._goal_time).nanoseconds / 1e9

    def _pose_from_tf(self):
        """SLAM 模式回退:从 map -> base_link TF 取最新位姿,失败返回 None。"""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        yaw = yaw_from_quaternion(t.transform.rotation)
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _current_pose(self):
        """返回 (x, y, yaw, 来源):优先 AMCL 位姿,SLAM 模式回退 TF。"""
        if self._pose is not None:
            p = self._pose.pose.pose.position
            yaw = yaw_from_quaternion(self._pose.pose.pose.orientation)
            return (p.x, p.y, yaw, 'AMCL')
        from_tf = self._pose_from_tf()
        if from_tf is not None:
            return (*from_tf, 'SLAM TF')
        return None

    def _distance_to_goal(self):
        pose = self._current_pose()
        if self._goal is None or pose is None:
            return float('inf')
        g = self._goal.pose.position
        return math.hypot(g.x - pose[0], g.y - pose[1])

    def _path_progress(self):
        """计算沿路径剩余里程:先找路径上距机器人最近的点,再累加其后段长。"""
        pose = self._current_pose()
        if self._plan is None or len(self._plan.poses) < 2 or pose is None:
            return None
        px, py = pose[0], pose[1]
        pts = [(ps.pose.position.x, ps.pose.position.y)
               for ps in self._plan.poses]
        total = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        nearest = min(range(len(pts)),
                      key=lambda k: (pts[k][0] - px) ** 2 + (pts[k][1] - py) ** 2)
        remaining = sum(math.dist(pts[i], pts[i + 1])
                        for i in range(nearest, len(pts) - 1))
        return total, remaining

    def _report(self):
        if self._goal is None and self._plan is None and self._state == 'idle':
            self._publish('等待目标:RViz 用 "2D Goal Pose" 工具点选,或\n'
                          'ros2 topic pub --once -w 1 /goal_pose '
                          'geometry_msgs/msg/PoseStamped \'{header: '
                          '{frame_id: map}, pose: {position: {x: 1.0, '
                          'y: 1.7}}}\'')
            return
        pose = self._current_pose()
        if pose is None:
            self._publish('尚未获得位姿:常规模式等 /amcl_pose(机器人需移动'
                          '触发更新);SLAM 模式等 map -> odom TF 就绪')
            return
        px, py, _, pose_src = pose

        g = self._goal.pose.position if self._goal else None
        if g is None and self._plan is not None:
            # 目标来自其他任务节点(如 waypoint_patrol),取路径终点展示
            g = self._plan.poses[-1].pose.position

        state_text = {
            'idle': '等待目标', 'navigating': '导航中',
            'succeeded': '已到达', 'failed': '失败', 'canceled': '已取消',
        }.get(self._state, self._state)
        if self._state == 'navigating' and self._goal is None:
            state_text += '(外部任务目标)'

        parts = [f'目标: ({g.x:+.2f}, {g.y:+.2f})', f'状态: {state_text}']

        progress = self._path_progress()
        if progress is not None:
            total, remaining = progress
            pct = max(0.0, min(1.0, 1.0 - remaining / total)) * 100 if total > 0 else 100.0
            parts.append(f'沿路径剩余 {remaining:.2f}/{total:.2f} m ({pct:.0f}%)')

        straight = self._distance_to_goal() if self._goal is not None else \
            math.hypot(g.x - px, g.y - py)
        parts.append(f'距目标直线 {straight:.2f} m')
        parts.append(f'速度 {self._speed:.2f} m/s')

        if self._state == 'navigating' and self._speed > 0.05 and progress:
            eta = progress[1] / self._speed
            parts.append(f'预计 {eta:.0f} s')
        parts.append(f'已用 {self._elapsed():.0f} s')
        if self._recoveries:
            parts.append(f'恢复行为 {self._recoveries} 次')
        parts.append(f'定位: {pose_src}')
        parts.append(f'累计: 成功 {self._stats["succeeded"]}'
                     f'/失败 {self._stats["failed"]}'
                     f'/取消 {self._stats["canceled"]}')
        self._publish('[导航报告] ' + ' | '.join(parts))

    def _publish(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
