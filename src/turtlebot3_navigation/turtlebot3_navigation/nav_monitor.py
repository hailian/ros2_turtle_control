#!/usr/bin/env python3
"""导航监测节点:跟踪目标点与全局路径,输出导航进度中文报告。

订阅:
    /goal_pose  (PoseStamped)              目标点(RViz 2D Goal Pose 或命令行)
    /amcl_pose  (PoseWithCovarianceStamped) 当前位姿估计
    /plan       (nav_msgs/Path)            最新全局规划路径
    /odom       (Odometry)                 实际运动速度

动作:
    navigate_to_pose (nav2_msgs/action)    relay_goal=true 时把 /goal_pose
                                           转发给 bt_navigator 执行

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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavMonitor(Node):

    def __init__(self):
        super().__init__('nav_monitor')
        self.declare_parameter('report_period', 2.0)
        self.declare_parameter('relay_goal', True)

        self._goal = None          # 最近收到的目标点 (PoseStamped)
        self._goal_time = None     # 目标接收时刻
        self._pose = None          # 最近的 AMCL 位姿
        self._plan = None          # 最近一帧全局路径
        self._speed = 0.0          # 实际速度 (m/s)
        self._state = 'idle'       # idle/navigating/succeeded/failed/canceled
        self._relay = bool(self.get_parameter('relay_goal').value)
        self._feedback = None      # 动作反馈(转发目标才有)
        self._recoveries = 0
        self._stats = {'succeeded': 0, 'failed': 0, 'canceled': 0}
        self._goal_handle = None

        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._status_pub = self.create_publisher(String, 'navigation_status', 10)

        if self._relay:
            self._action_client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info(
                'nav_monitor 就绪:订阅 /goal_pose 并转发 navigate_to_pose 动作')
        else:
            self._action_client = None
            self.get_logger().info('nav_monitor 就绪:仅监测,不转发目标(外部任务模式)')

        period = float(self.get_parameter('report_period').value)
        self.create_timer(period, self._report)

    # ---- 订阅回调 -------------------------------------------------------

    def _on_goal(self, msg):
        self._goal = msg
        self._goal_time = self.get_clock().now()
        self._state = 'navigating'
        self._feedback = None
        self._recoveries = 0
        yaw = math.degrees(yaw_from_quaternion(msg.pose.orientation))
        p = msg.pose.position
        self._publish(f'[导航] 收到目标: ({p.x:+.2f}, {p.y:+.2f}, '
                      f'朝向 {yaw:+.0f}°),开始规划路径')
        if not self._relay:
            return
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self._state = 'failed'
            self._publish('[导航] navigate_to_pose 动作不可用:请确认 bt_navigator 已激活')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        # bt_navigator 收到新目标会自动中止上一个,无需显式取消
        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._on_action_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_pose(self, msg):
        self._pose = msg

    def _on_plan(self, msg):
        self._plan = msg

    def _on_odom(self, msg):
        t = msg.twist.twist
        self._speed = math.hypot(t.linear.x, t.linear.y)

    # ---- 动作回调(仅转发模式) -------------------------------------------

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._state = 'failed'
            self._publish('[导航] 目标被 bt_navigator 拒绝,请检查导航栈是否已激活')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_msg):
        self._feedback = feedback_msg.feedback
        self._recoveries = self._feedback.number_of_recoveries

    def _on_action_result(self, future):
        status = future.result().status
        final_err = self._distance_to_goal()
        # rclpy 动作状态码:3 已取消, 4 成功, 5/6 失败
        if status == 4:
            self._state = 'succeeded'
            self._stats['succeeded'] += 1
            self._publish(f'[导航] 到达目标!终点误差 {final_err:.2f} m,'
                          f'用时 {self._elapsed():.0f} s')
        elif status == 3:
            self._state = 'canceled'
            self._stats['canceled'] += 1
            self._publish(f'[导航] 目标已取消(剩余 {final_err:.2f} m)')
        else:
            self._state = 'failed'
            self._stats['failed'] += 1
            self._publish(f'[导航] 导航失败:剩余 {final_err:.2f} m。'
                          '常见原因:目标点在障碍物内、路径被动态堵死且'
                          '恢复行为无效、定位漂移')

    # ---- 报告生成 -------------------------------------------------------

    def _elapsed(self):
        if self._goal_time is None:
            return 0.0
        return (self.get_clock().now() - self._goal_time).nanoseconds / 1e9

    def _distance_to_goal(self):
        if self._goal is None or self._pose is None:
            return float('inf')
        g = self._goal.pose.position
        p = self._pose.pose.pose.position
        return math.hypot(g.x - p.x, g.y - p.y)

    def _path_progress(self):
        """计算沿路径剩余里程:先找路径上距机器人最近的点,再累加其后段长。"""
        if self._plan is None or len(self._plan.poses) < 2 or self._pose is None:
            return None
        p = self._pose.pose.pose.position
        pts = [(ps.pose.position.x, ps.pose.position.y)
               for ps in self._plan.poses]
        total = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        nearest = min(range(len(pts)),
                      key=lambda k: (pts[k][0] - p.x) ** 2 + (pts[k][1] - p.y) ** 2)
        remaining = sum(math.dist(pts[i], pts[i + 1])
                        for i in range(nearest, len(pts) - 1))
        return total, remaining

    def _report(self):
        if self._goal is None and self._plan is None:
            self._publish('等待目标:RViz 用 "2D Goal Pose" 工具点选,或\n'
                          'ros2 topic pub --once /goal_pose '
                          'geometry_msgs/msg/PoseStamped \'{header: '
                          '{frame_id: map}, pose: {position: {x: 1.0, '
                          'y: 1.7}}}\'')
            return
        if self._pose is None:
            self._publish('尚未收到 AMCL 位姿:请确认定位模块已启动且收敛')
            return

        g = self._goal.pose.position if self._goal else None
        if g is None and self._plan is not None:
            # 目标来自其他任务节点(如 waypoint_patrol),取路径终点展示
            last = self._plan.poses[-1].pose.position
            g = last

        state_text = {
            'idle': '等待目标', 'navigating': '导航中',
            'succeeded': '已到达', 'failed': '失败', 'canceled': '已取消',
        }.get(self._state, self._state)
        if not self._relay and self._goal is None and self._plan is not None:
            # 仅监测模式:目标来自其他任务节点(如 waypoint_patrol),
            # 无动作状态可查,按是否在移动区分
            state_text = '外部任务-行进中' if self._speed > 0.05 else '外部任务-静止'

        parts = [f'目标: ({g.x:+.2f}, {g.y:+.2f})', f'状态: {state_text}']

        progress = self._path_progress()
        if progress is not None:
            total, remaining = progress
            pct = max(0.0, min(1.0, 1.0 - remaining / total)) * 100 if total > 0 else 100.0
            parts.append(f'沿路径剩余 {remaining:.2f}/{total:.2f} m ({pct:.0f}%)')

        if self._goal is not None:
            straight = self._distance_to_goal()
        else:
            p = self._pose.pose.pose.position
            straight = math.hypot(g.x - p.x, g.y - p.y)
        parts.append(f'距目标直线 {straight:.2f} m')
        parts.append(f'速度 {self._speed:.2f} m/s')

        if self._state == 'navigating' and self._speed > 0.05 and progress:
            eta = progress[1] / self._speed
            parts.append(f'预计 {eta:.0f} s')
        parts.append(f'已用 {self._elapsed():.0f} s')
        if self._recoveries:
            parts.append(f'恢复行为 {self._recoveries} 次')
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
