#!/usr/bin/env python3
"""地图保存节点:订阅 /map,通过 /save_map 服务把当前地图存为 PGM + YAML。

用法:
    # 建图过程中或结束后调用:
    ros2 service call /save_map std_srvs/srv/Trigger "{}"

    # 可在启动时指定保存目录和文件名:
    ros2 run turtlebot3_slam map_saver --ros-args \\
        -p save_dir:=~/turtlebot3_maps -p map_name:=my_map

输出与 nav2 map_saver 一兼容:
    map.pgm  — P5 二进制 PGM,未知=205,空闲=254,占用=0
    map.yaml — nav2 地图服务器可直接加载的元数据
"""

import datetime
import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

# PGM 三态灰度(nav2 惯例)
PGM_UNKNOWN = 205
PGM_FREE = 254
PGM_OCCUPIED = 0


def grid_to_pixels(grid):
    """OccupancyGrid 数据 → PGM 行字节(自顶向下,逐行)。

    grid.data 为行优先、自下而上存储(原点在左下角),而 PGM 从顶部
    逐行扫描,因此行顺序需要倒置。
    """
    width, height = grid.info.width, grid.info.height
    lines = []
    for row in range(height - 1, -1, -1):
        base = row * width
        line = bytearray(width)
        for col in range(width):
            cell = grid.data[base + col]
            if cell < 0:
                line[col] = PGM_UNKNOWN
            elif cell >= 65:
                line[col] = PGM_OCCUPIED
            elif cell <= 25:
                line[col] = PGM_FREE
            else:  # 介于两阈值之间按未知处理
                line[col] = PGM_UNKNOWN
        lines.append(bytes(line))
    return b''.join(lines)


def write_map(grid, pgm_path, yaml_path, map_name):
    """把 OccupancyGrid 写入 pgm_path(.pgm)与 yaml_path(.yaml)。"""
    os.makedirs(os.path.dirname(pgm_path), exist_ok=True)
    pixels = grid_to_pixels(grid)
    header = f'P5\n{grid.info.width} {grid.info.height}\n255\n'.encode()
    with open(pgm_path, 'wb') as f:
        f.write(header)
        f.write(pixels)

    origin = grid.info.origin.position
    yaml_text = (
        f'image: {os.path.basename(pgm_path)}\n'
        f'resolution: {grid.info.resolution}\n'
        f'origin: [{origin.x}, {origin.y}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
        'mode: trinary\n'
    )
    with open(yaml_path, 'w') as f:
        f.write(yaml_text)
    return len(pixels)


class MapSaverNode(Node):

    def __init__(self):
        super().__init__('map_saver')
        self.declare_parameter('save_dir', '~/turtlebot3_maps')
        self.declare_parameter('map_name', '')

        # VOLATILE 订阅端对 VOLATILE / TRANSIENT_LOCAL 发布者均兼容;
        # 建图期间 /map 周期性发布,无需持久化订阅。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._latest_map = None
        self.create_subscription(OccupancyGrid, 'map', self._on_map, qos)
        self.create_service(Trigger, 'save_map', self._on_save_map)
        self.get_logger().info(
            'map_saver 就绪:等待 /map,调用 '
            '"ros2 service call /save_map std_srvs/srv/Trigger {}" 保存地图')

    def _on_map(self, msg):
        self._latest_map = msg

    def _on_save_map(self, request, response):
        if self._latest_map is None:
            response.success = False
            response.message = '尚未收到 /map,请确认建图节点已发布地图'
            return response

        grid = self._latest_map
        save_dir = os.path.expanduser(
            self.get_parameter('save_dir').get_parameter_value().string_value)
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        if not map_name:
            map_name = 'map_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        pgm_path = os.path.join(save_dir, map_name + '.pgm')
        yaml_path = os.path.join(save_dir, map_name + '.yaml')
        try:
            num_cells = write_map(grid, pgm_path, yaml_path, map_name)
        except OSError as err:
            response.success = False
            response.message = f'保存失败: {err}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'地图已保存: {pgm_path} 与 {yaml_path} '
            f'({grid.info.width}x{grid.info.height}, {num_cells} cells)')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
