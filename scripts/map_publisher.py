#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import yaml, numpy as np, cv2, os, time

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_yaml', '')
        map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        if not map_yaml or not os.path.exists(map_yaml):
            self.get_logger().error(f'map_yaml not found: {map_yaml}')
            return
        with open(map_yaml, 'r') as f:
            meta = yaml.safe_load(f)
        map_dir = os.path.dirname(map_yaml)
        img_path = os.path.join(map_dir, meta['image'])
        self.get_logger().info(f'Loading: {img_path}')
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'Failed: {img_path}')
            return

        # 关键修复：cv2 读图是顶左原点，ROS OccupancyGrid 要求底左原点
        # 必须垂直翻转（flip 0）让图像第一行对应地图 y=0（底部）
        img = cv2.flip(img, 0)

        h, w = img.shape
        flat = img.flatten()
        data = np.full(h * w, -1, dtype=np.int8)
        data[flat > 200] = 0    # 白色 -> free
        data[flat < 50] = 100   # 黑色 -> occupied

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(meta['resolution'])
        msg.info.width = w
        msg.info.height = h
        origin = meta['origin']
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        time.sleep(0.5)
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published /map: {w}x{h}, res={meta["resolution"]}, '
            f'origin=({origin[0]:.2f}, {origin[1]:.2f}), Y-flipped'
        )

def main():
    rclpy.init()
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
