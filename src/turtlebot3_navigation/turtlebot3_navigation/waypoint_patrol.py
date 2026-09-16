#!/usr/bin/env python3
"""航点巡逻任务节点:按序自主导航到一串航点,循环巡逻。

原理:基于 nav2_simple_commander 逐点调用 navigate_to_pose 动作,
每个航点由全局规划器规划路径、局部控制器跟踪执行;单点失败默认跳过
继续下一点(on_failure:=abort 可改为终止任务)。

参数(节点名 waypoint_patrol):
    waypoints         航点列表,每项 "x,y" 或 "x,y,yaw"(yaw 单位弧度;
                      缺省自动朝向下一个航点,首尾相接延续)
    loop_count        巡逻圈数;0 表示无限循环(Ctrl-C 结束)
    wait_at_waypoint  每个航点停留时长(秒)
    on_failure        单点失败策略:continue 跳过继续 / abort 终止任务

发布:
    /patrol_status (std_msgs/String)   巡逻进度中文报告
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String

# turtlebot3_world 内圈一圈的 4 个可达航点(已按建图结果核对为空旷区)
DEFAULT_WAYPOINTS = ['-1.5,0.5', '1.0,1.7', '1.9,-1.0', '-1.0,-1.9']


def parse_waypoints(items):
    """把 "x,y[,yaw]" 字符串列表解析为 (x, y, yaw|None) 元组列表。"""
    result = []
    for item in items:
        parts = [float(v) for v in item.split(',')]
        if len(parts) == 2:
            result.append((parts[0], parts[1], None))
        elif len(parts) == 3:
            result.append((parts[0], parts[1], parts[2]))
        else:
            raise ValueError(f'航点格式应为 "x,y" 或 "x,y,yaw": {item}')
    return result


def fill_missing_yaw(waypoints):
    """yaw 缺省时,朝向下一个航点(末点朝回首点,保证巡逻衔接连贯)。"""
    n = len(waypoints)
    return [
        (x, y, math.atan2(waypoints[(i + 1) % n][1] - y,
                          waypoints[(i + 1) % n][0] - x) if yaw is None else yaw)
        for i, (x, y, yaw) in enumerate(waypoints)]


class WaypointPatrol:

    def __init__(self):
        # 复用 BasicNavigator 自身节点收发动作与参数(launch 用节点名
        # waypoint_patrol 注入参数)
        self._nav = BasicNavigator()
        self._nav.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self._nav.declare_parameter('loop_count', 1)
        self._nav.declare_parameter('wait_at_waypoint', 2.0)
        self._nav.declare_parameter('on_failure', 'continue')

        self._waypoints = fill_missing_yaw(
            parse_waypoints(self._nav.get_parameter('waypoints').value))
        self._loop_count = int(self._nav.get_parameter('loop_count').value)
        self._wait = float(self._nav.get_parameter('wait_at_waypoint').value)
        self._on_failure = str(self._nav.get_parameter('on_failure').value)
        self._status_pub = self._nav.create_publisher(
            String, 'patrol_status', 10)

    def _publish(self, text):
        self._nav.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _patrol_one_leg(self, index, x, y, yaw):
        """导航到单个航点,返回 (是否成功, 本段用时秒)。"""
        nav = self._nav
        total = len(self._waypoints)
        self._publish(f'[巡逻] 航点 {index + 1}/{total} ({x:+.2f}, {y:+.2f}, '
                      f'朝向 {math.degrees(yaw):+.0f}°):规划路径出发')
        nav.goToPose(self._make_pose(x, y, yaw))

        start = time.monotonic()
        last_log = start
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            now = time.monotonic()
            if feedback is not None and now - last_log > 5.0:
                self._publish(f'[巡逻] 前往航点 {index + 1}/{total}:'
                              f'剩余 {feedback.distance_remaining:.2f} m,'
                              f'已行 {now - start:.0f} s')
                last_log = now
        leg_time = time.monotonic() - start

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self._publish(f'[巡逻] 到达航点 {index + 1}/{total},'
                          f'本段用时 {leg_time:.0f} s,停留 {self._wait:.0f} s')
            time.sleep(self._wait)
            return True, leg_time
        if result == TaskResult.CANCELED:
            self._publish(f'[巡逻] 航点 {index + 1}/{total} 被取消,终止任务')
            return False, leg_time
        self._publish(f'[巡逻] 航点 {index + 1}/{total} 导航失败'
                      f'({leg_time:.0f} s 后放弃)。'
                      f'策略: {"终止任务" if self._on_failure == "abort" else "跳过继续"}')
        return False, leg_time

    def _wait_nav_stack(self, timeout=120.0):
        """轮询生命周期服务,等待 amcl 与 bt_navigator 全部激活。

        不用 BasicNavigator.waitUntilNav2Active():Humble 版会阻塞等待
        一条 /amcl_pose 消息,而 AMCL 仅在机器人移动后才更新发布,
        静止待命时永远等不到。
        """
        nav = self._nav
        deadline = time.monotonic() + timeout
        clients = {}
        for name in ('amcl', 'bt_navigator'):
            clients[name] = nav.create_client(GetState, f'{name}/get_state')
        pending = dict(clients)
        while pending and time.monotonic() < deadline:
            for name, client in list(pending.items()):
                if not client.wait_for_service(timeout_sec=0.5):
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(nav, future, timeout_sec=2.0)
                if (future.result() is not None
                        and future.result().current_state.label == 'active'):
                    pending.pop(name)
            if pending:
                self._publish('[巡逻] 等待导航栈激活(剩: '
                              + ', '.join(pending) + ')...')
                time.sleep(2.0)
        return not pending

    def run(self):
        loops_text = '无限' if self._loop_count == 0 else str(self._loop_count)
        plan_text = ' → '.join(
            f'({x:+.2f},{y:+.2f})' for x, y, _ in self._waypoints)
        self._publish(f'[巡逻] 任务开始:{len(self._waypoints)} 个航点 {plan_text},'
                      f'{loops_text} 圈,每点停留 {self._wait:.0f} s;'
                      f'等待导航栈激活(地图/AMCL/规划器/控制器)...')

        if not self._wait_nav_stack():
            self._publish('[巡逻] 导航栈 120 s 内未就绪,任务终止'
                          '(请确认 navigation.launch.py 已启动)')
            return
        self._publish('[巡逻] 导航栈已激活,开始巡航')

        succeeded = failed = 0
        total_leg_time = 0.0
        start = time.monotonic()
        lap = 0
        try:
            while self._loop_count == 0 or lap < self._loop_count:
                lap += 1
                if self._loop_count != 1:
                    self._publish(f'[巡逻] ===== 第 {lap} 圈 =====')
                for i, (x, y, yaw) in enumerate(self._waypoints):
                    ok, leg_time = self._patrol_one_leg(i, x, y, yaw)
                    total_leg_time += leg_time
                    if ok:
                        succeeded += 1
                    else:
                        failed += 1
                        if self._on_failure == 'abort':
                            return self._summary(
                                succeeded, failed, total_leg_time,
                                time.monotonic() - start, aborted=True)
        except KeyboardInterrupt:
            self._nav.cancelTask()
            self._publish('[巡逻] 收到中断,取消当前目标并汇总')

        return self._summary(
            succeeded, failed, total_leg_time, time.monotonic() - start)

    def _summary(self, succeeded, failed, leg_time, wall_time, aborted=False):
        legs = succeeded + failed
        rate = succeeded / legs * 100 if legs else 0.0
        avg = leg_time / legs if legs else 0.0
        text = (f'[巡逻] 任务{"中止" if aborted else "完成"}:'
                f'共 {legs} 段,成功 {succeeded},失败 {failed}'
                f'(成功率 {rate:.0f}%),平均每段 {avg:.0f} s,'
                f'总用时 {wall_time:.0f} s')
        self._publish(text)


def main(args=None):
    rclpy.init(args=args)
    patrol = WaypointPatrol()
    try:
        patrol.run()
    except KeyboardInterrupt:
        pass
    finally:
        patrol._nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
