#!/usr/bin/env python3
"""位姿监测节点:汇总激光扫描特征与 AMCL 位姿估计,输出定位分析报告。

订阅:
    /amcl_pose  (PoseWithCovarianceStamped)  AMCL 位姿 + 协方差
    /scan       (LaserScan)                  实时激光扫描

发布:
    /localization_status (std_msgs/String)   周期性定位分析报告(中文)

报告内容包括:
    - 当前位姿估计 (x, y, yaw)
    - 由协方差计算的定位置信度评价
    - 周围环境扫描特征:有效波束数、最近障碍距离与方位、平均距离
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def evaluate_confidence(sigma_xy, sigma_yaw):
    """按位置/朝向标准差给出定性评价与建议。"""
    if sigma_xy < 0.05 and sigma_yaw < 0.05:
        return '优秀'
    if sigma_xy < 0.15 and sigma_yaw < 0.10:
        return '良好'
    if sigma_xy < 0.30 and sigma_yaw < 0.20:
        return '一般'
    return '偏低(建议遥控慢速移动或用 2D Pose Estimate 重设初始位姿)'


class PoseMonitor(Node):

    def __init__(self):
        super().__init__('pose_monitor')
        self.declare_parameter('report_period', 2.0)

        scan_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self._pose = None      # 最近的 AMCL 位姿
        self._scan = None      # 最近的一帧扫描
        self._pose_count = 0   # 累计收到的位姿更新次数
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, scan_qos)
        self._status_pub = self.create_publisher(String, 'localization_status', 10)

        period = self.get_parameter('report_period').value
        self.create_timer(period, self._report)
        self.get_logger().info(
            'pose_monitor 就绪:等待 /amcl_pose 与 /scan,周期输出定位分析报告')

    def _on_pose(self, msg):
        self._pose = msg
        self._pose_count += 1

    def _on_scan(self, msg):
        self._scan = msg

    def _scan_features(self, robot_yaw):
        """提取当前扫描的环境特征:有效波束、最近障碍距离/方位、平均距离。"""
        scan = self._scan
        if scan is None:
            return '无扫描数据'
        valid = [(r, scan.angle_min + i * scan.angle_increment)
                 for i, r in enumerate(scan.ranges)
                 if scan.range_min < r < scan.range_max and math.isfinite(r)]
        if not valid:
            return f'{len(scan.ranges)} 波束中无有效回波'
        ranges = [r for r, _ in valid]
        min_range, min_angle = min(valid, key=lambda item: item[0])
        # 激光坐标系的障碍方位转到机器人朝向的相对角(-180°~180°,0 为正前方)
        rel_deg = math.degrees(math.atan2(
            math.sin(min_angle - robot_yaw),
            math.cos(min_angle - robot_yaw)))
        return (f'{len(valid)}/{len(scan.ranges)} 有效波束, '
                f'最近障碍 {min_range:.2f} m (相对朝向 {rel_deg:+.0f}°), '
                f'平均测距 {sum(ranges) / len(ranges):.2f} m')

    def _report(self):
        if self._pose is None:
            self._publish('尚未收到 AMCL 位姿:请确认地图已加载、'
                          '初始位姿已设置(set_initial_pose 或 RViz 2D Pose Estimate)')
            return

        p = self._pose.pose.pose.position
        yaw = yaw_from_quaternion(self._pose.pose.pose.orientation)
        cov = self._pose.pose.covariance   # 6x6 行优先:x y z roll pitch yaw
        sigma_x = math.sqrt(max(cov[0], 0.0))
        sigma_y = math.sqrt(max(cov[7], 0.0))
        sigma_yaw = math.sqrt(max(cov[35], 0.0))
        sigma_xy = math.hypot(sigma_x, sigma_y)

        text = (f'[定位报告] 位姿: x={p.x:+.2f} y={p.y:+.2f} '
                f'yaw={yaw:+.2f} rad ({math.degrees(yaw):+.0f}°) | '
                f'置信度: {evaluate_confidence(sigma_xy, sigma_yaw)} '
                f'(σxy={sigma_xy:.3f} m, σyaw={sigma_yaw:.3f} rad) | '
                f'环境特征: {self._scan_features(yaw)} | '
                f'累计更新 {self._pose_count} 次')
        self._publish(text)

    def _publish(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
