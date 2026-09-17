#!/usr/bin/env python3
"""自动重定位节点:不依赖手动 2D Pose Estimate,机器人自行环绕运动,
由粒子滤波收敛计算出自己在地图中的位置。

流程(状态机):
    1. 调用 AMCL 的 /global_localization 服务:粒子均匀撒满全图自由空间
    2. 原地旋转 360°:360° 激光全向观测,快速剔除错误假设、确定朝向
    3. 若未收敛,自动绕行一个小方形回路(带激光安全保护),增加观测多样性
    4. 协方差收敛(σxy/σyaw 连续多帧低于阈值)后停车,报告最终位置

发布:
    /relocalization_status (std_msgs/String) 状态与最终定位报告
    /cmd_vel               阶段性运动指令(收敛/超时后自动停车)

服务:
    /relocalize (std_srvs/Empty) 手动触发一次重定位(如机器人被搬动后)
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

# 状态机阶段
WAIT_SERVICES = '等待AMCL就绪'
GLOBAL_INIT = '全局粒子撒布'
ROTATE = '原地环视360度'
DRIVE_LEG = '直线行进'
DRIVE_TURN = '转向开阔方向'
REVERSE = '后退脱困'
CONVERGED = '定位成功'
FAILED = '超时未收敛'
IDLE = '空闲'


def normalize_angle(a):
    """归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoRelocalization(Node):

    def __init__(self):
        super().__init__('auto_relocalization')
        # 可调参数
        self.declare_parameter('rotate_speed', 0.4)       # rad/s
        self.declare_parameter('drive_speed', 0.1)       # m/s
        self.declare_parameter('loop_leg', 2.2)          # 探索直行腿长 m
        self.declare_parameter('sigma_xy_thresh', 0.3)   # 收敛阈值 m
        self.declare_parameter('sigma_yaw_thresh', 0.25) # 收敛阈值 rad
        self.declare_parameter('converge_count', 5)      # 连续满足阈值帧数
        self.declare_parameter('timeout', 240.0)         # 总超时 s
        self.declare_parameter('safety_dist', 0.3)       # 前向安全距离 m
        self.declare_parameter('min_traveled', 2.5)      # 接受收敛前最少行程 m
        self.declare_parameter('respread_interval', 1e9) # 重撒冷却 s(默认关闭)
        self.declare_parameter('autostart', True)       # 启动即自动求解

        self._p = {name: self.get_parameter(name).value
                   for name in ['rotate_speed', 'drive_speed', 'loop_leg',
                                'sigma_xy_thresh', 'sigma_yaw_thresh',
                                'converge_count', 'timeout', 'safety_dist',
                                'min_traveled', 'respread_interval']}

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'relocalization_status', 10)
        scan_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, scan_qos)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.relocalize_srv = self.create_service(
            Empty, 'relocalize', self._on_relocalize)

        # nav2(humble)AMCL 的全局重定位服务名
        self._global_srv = self.create_client(
            Empty, 'reinitialize_global_localization')
        self._nomotion_srv = self.create_client(
            Empty, 'request_nomotion_update')

        # 运行状态
        self._state = IDLE
        self._start_time = None
        self._rotated = 0.0        # 本阶段已旋转角
        self._leg_time = 0.0       # 当前直行腿剩余时长
        self._leg_index = 0        # 方形回路第几条边(0-3)
        self._consec = 0           # 连续收敛计数
        self._loops = 0            # 已绕行圈数
        self._pose_msg = None
        self._forward_min = float('inf')
        self._open_angle = 0.0      # 最开阔方向(机体系弧度)
        self._odom_yaw = 0.0
        self._last_odom_xy = None   # 里程计累计行程跟踪
        self._traveled = 0.0
        self._turn_target = None    # DRIVE_TURN 的目标朝向(map/odom 系)
        self._reverse_time = 0.0    # 后退脱困剩余时长 s
        self._abort_streak = 0      # 连续中止计数(未完成一条直行腿)
        self._last_respread = None  # 上次重撒粒子时刻(冷却控制)
        self._state_ts = None       # 当前状态的进入时刻(看门狗)

        # 50 Hz 控制节拍用壁钟:节点随导航栈冷启动时(仿真刚起、/clock
        # 未就绪)仿真时钟定时器会停摆,实测表现为卡在环视不发指令;
        # 壁钟定时器不受影响,且控制环本就不应依赖仿真时钟推进
        self.create_timer(
            0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if bool(self.get_parameter('autostart').value):
            self.get_logger().info(
                'auto_relocalization 就绪:启动即自动重定位;'
                '也可调用 "ros2 service call /relocalize std_srvs/srv/Empty" 触发')
            self._start()
        else:
            # 供其他模块(如 turtlebot3_navigation 的绑架恢复)按需触发
            self.get_logger().info(
                'auto_relocalization 就绪:autostart 已关闭,'
                '等待 /relocalize 服务触发')

    # ---------------- 回调 ----------------
    def _on_relocalize(self, request, response):
        self._start()
        return response

    def _on_odom(self, msg):
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._last_odom_xy is not None:
            self._traveled += math.hypot(xy[0] - self._last_odom_xy[0],
                                         xy[1] - self._last_odom_xy[1])
        self._last_odom_xy = xy
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    def _on_amcl_pose(self, msg):
        self._pose_msg = msg
        cov = msg.pose.covariance
        sigma_xy = math.hypot(math.sqrt(max(cov[0], 0.0)),
                              math.sqrt(max(cov[7], 0.0)))
        sigma_yaw = math.sqrt(max(cov[35], 0.0))
        if (sigma_xy < self._p['sigma_xy_thresh']
                and sigma_yaw < self._p['sigma_yaw_thresh']):
            self._consec += 1
        else:
            self._consec = 0

    def _on_scan(self, msg):
        """记录前向最近障碍(直行安全)与最开阔的可通行方向(探索引导)。"""
        forward_min = []
        traversable = []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = msg.angle_min + i * msg.angle_increment
            if abs(math.degrees(ang)) <= 30.0:
                forward_min.append(r)
            # 只在留有安全余量的方向里选开阔方向,避免指向障碍阴影
            if r > self._p['safety_dist'] * 1.5:
                traversable.append((r, ang))
        self._forward_min = min(forward_min) if forward_min else float('inf')
        if traversable:
            # 前方受阻时只在侧向(±60° 以外)选,防止选中掠过障碍的
            # 近前方波束导致原地反复中止
            blocked = self._forward_min < self._p['safety_dist'] * 1.5
            if blocked:
                side = [(r, a) for r, a in traversable
                        if abs(math.degrees(a)) > 60.0]
                traversable = side or traversable
            self._open_angle = max(traversable)[1]

    # ---------------- 状态机 ----------------
    def _start(self):
        self._set_state(WAIT_SERVICES)
        # 超时/看门狗计时用单调壁钟:冷启动时仿真时钟尚未就绪(读数为 0),
        # 仿真时钟一到 elapsed 会瞬间变成数千秒导致误判超时
        self._start_time = time.monotonic()
        self._rotated = 0.0
        self._leg_index = 0
        self._consec = 0
        self._loops = 0
        self._traveled = 0.0
        self._publish_status('开始自动重定位:撒布全局粒子后环绕运动求解位置')

    def _publish_status(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _enter_turn(self):
        """进入回转:目标朝向 = 当前朝向 + 激光最开阔方向。

        朝长走廊/开阔区域走,获得无遮挡的全景观测,打破对称环境的
        感知混淆;被障碍挡住时该方向也自然指向可通行处。
        保证目标至少偏离当前朝向 45°,确保每次中止都有实质转向。
        """
        offset = self._open_angle
        if abs(offset) < math.pi / 4:
            offset = math.copysign(math.pi / 2,
                                   offset if offset != 0.0 else 1.0)
        self._turn_target = normalize_angle(self._odom_yaw + offset)
        self._rotated = 0.0
        self._set_state(DRIVE_TURN)

    def _evaluate(self):
        """运动过程中随时检查收敛与超时。返回 True 表示状态已迁移。

        除协方差收敛外,还要求累计行程达到 min_traveled:对称环境中
        小范围运动即可让粒子"紧密地收敛到错误位置"(感知混淆),
        走过足够距离、经过非对称结构后才能确认。
        """
        traveled_ok = self._traveled >= self._p['min_traveled']
        if (self._consec >= self._p['converge_count']
                and traveled_ok and self._pose_msg):
            self._set_state(CONVERGED)
            return True
        elapsed = time.monotonic() - self._start_time
        if elapsed > self._p['timeout']:
            self._set_state(FAILED)
            return True
        return False

    def _set_state(self, new_state):
        self._state = new_state
        self._state_ts = time.monotonic()

    def _tick(self):
        if self._state == IDLE:
            # 死者开关:空闲时持续发布零速,防止 gazebo 残留旧指令
            self.cmd_pub.publish(Twist())
            return

        # 看门狗:运动状态超过 45 s 未迁移视为卡死,强制转向脱困
        if self._state_ts is not None and self._state not in (
                WAIT_SERVICES, IDLE, CONVERGED, FAILED):
            stuck = time.monotonic() - self._state_ts
            if stuck > 45.0:
                self._publish_status(f'状态 {self._state} 超过 45 s,看门狗强制转向')
                self._stop_robot()
                self._enter_turn()
                return

        cmd = Twist()
        if self._state == WAIT_SERVICES:
            if self._global_srv.service_is_ready():
                self._global_srv.call_async(Empty.Request())
                if self._nomotion_srv.service_is_ready():
                    self._nomotion_srv.call_async(Empty.Request())
                self._set_state(ROTATE)
                self._rotated = 0.0
                self._publish_status(
                    '全局粒子已撒布,开始原地环视 360°(全向激光观测)')
            else:
                self.cmd_pub.publish(Twist())  # 死者开关
            return

        if self._evaluate():
            return

        if self._state == ROTATE:
            step = self._p['rotate_speed'] * 0.05
            cmd.angular.z = self._p['rotate_speed']
            self._rotated += step
            if self._rotated >= 2.0 * math.pi:
                self._stop_robot()
                if self._nomotion_srv.service_is_ready():
                    self._nomotion_srv.call_async(Empty.Request())
                self._publish_status(
                    '环视完成,粒子尚未完全收敛:转向开阔方向继续探索'
                    f'(直行腿长 {self._p["loop_leg"]} m)')
                self._leg_index = 0
                self._enter_turn()
            else:
                self.cmd_pub.publish(cmd)
            return

        if self._state == DRIVE_LEG:
            if self._forward_min < self._p['safety_dist']:
                self._stop_robot()
                self._abort_streak += 1
                if (self._forward_min < self._p['safety_dist'] * 0.7
                        or self._abort_streak >= 2):
                    # 贴得太近或连续受阻:先后退脱困再转向,
                    # 防止在障碍边缘反复打转
                    self._publish_status(
                        f'前方 {self._forward_min:.2f} m 有障碍,后退脱困')
                    self._reverse_time = 2.0
                    self._set_state(REVERSE)
                else:
                    self._publish_status(
                        f'前方 {self._forward_min:.2f} m 有障碍,'
                        '转向开阔方向')
                    self._enter_turn()
                return
            self._leg_time -= 0.05
            if self._leg_time <= 0.0:
                self._stop_robot()
                self._abort_streak = 0
                self._enter_turn()
            else:
                cmd.linear.x = self._p['drive_speed']
                self.cmd_pub.publish(cmd)
            return

        if self._state == REVERSE:
            cmd.linear.x = -self._p['drive_speed']
            self._reverse_time -= 0.05
            if self._reverse_time <= 0.0:
                self._stop_robot()
                self._enter_turn()
            else:
                self.cmd_pub.publish(cmd)
            return

        if self._state == DRIVE_TURN:
            # 朝目标(开阔)方向旋转,超圈保护 2π
            diff = normalize_angle(self._turn_target - self._odom_yaw)
            self._rotated += self._p['rotate_speed'] * 0.05
            if abs(diff) < 0.12 or self._rotated >= 2.0 * math.pi:
                self._stop_robot()
                self._leg_index += 1
                if self._leg_index >= 4:
                    self._leg_index = 0
                    self._loops += 1
                    # 长期无进展才重撒粒子(默认关闭:AMCL 的自适应粒子
                    # 注入已能处理坏假设,周期性清零反而打断正常收敛)
                    now = time.monotonic()
                    cooldown = now - self._last_respread \
                        if self._last_respread is not None else 1e9
                    if cooldown > self._p['respread_interval']:
                        if self._global_srv.service_is_ready():
                            self._global_srv.call_async(Empty.Request())
                        if self._nomotion_srv.service_is_ready():
                            self._nomotion_srv.call_async(Empty.Request())
                        self._consec = 0
                        self._last_respread = now
                        self._publish_status(
                            f'完成第 {self._loops} 段探索仍未收敛:'
                            '已重新撒布全局粒子,继续求解')
                self._set_state(DRIVE_LEG)
                self._leg_time = self._p['loop_leg'] / self._p['drive_speed']
            else:
                cmd.angular.z = (self._p['rotate_speed'] if diff > 0
                                 else -self._p['rotate_speed'])
                self.cmd_pub.publish(cmd)
            return

        if self._state == CONVERGED:
            self._stop_robot()
            p = self._pose_msg.pose.pose.position
            yaw = yaw_from_quaternion(self._pose_msg.pose.pose.orientation)
            cov = self._pose_msg.pose.covariance
            sigma_xy = math.hypot(math.sqrt(max(cov[0], 0.0)),
                                  math.sqrt(max(cov[7], 0.0)))
            sigma_yaw = math.sqrt(max(cov[35], 0.0))
            elapsed = time.monotonic() - self._start_time
            self._publish_status(
                f'[重定位成功] 机器人位置: x={p.x:+.2f} m, y={p.y:+.2f} m, '
                f'朝向 {yaw:+.2f} rad ({math.degrees(yaw):+.0f}°) | '
                f'σxy={sigma_xy:.3f} m, σyaw={sigma_yaw:.3f} rad | '
                f'累计行程 {self._traveled:.1f} m, 绕行 {self._loops} 圈, '
                f'用时 {elapsed:.0f} s')
            self._set_state(IDLE)
            return

        if self._state == FAILED:
            self._stop_robot()
            if self._pose_msg:
                p = self._pose_msg.pose.pose.position
                self._publish_status(
                    f'[重定位超时] 未能收敛,当前最优估计 (x={p.x:+.2f}, '
                    f'y={p.y:+.2f}),建议继续遥控慢速行驶或检查地图质量')
            else:
                self._publish_status('[重定位超时] 期间未收到任何位姿更新')
            self._set_state(IDLE)
            return


def main(args=None):
    rclpy.init(args=args)
    node = AutoRelocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException as err:  # 任何异常退出前必须停车,防指令残留
        node.cmd_pub.publish(Twist())
        raise err
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
