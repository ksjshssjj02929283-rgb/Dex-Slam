#!/usr/bin/env python3
"""map_tf_pub.py - 桥接 3D 定位和 Nav2。
订阅 3D 定位位姿 (/lightning/pose/current, PoseStamped) 和底盘里程计 (/feedback/odom, Odometry)，
计算并发布 map -> odom TF，替代 Cartographer/AMCL。
T_map_odom = T_map_robot × inverse(T_odom_robot)  (2D: x, y, yaw)"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_2d(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def extract_2d(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


class MapOdomTFPub(Node):
    def __init__(self):
        super().__init__('map_tf_pub')
        self.br = TransformBroadcaster(self)
        self.loc_pose = None
        self.odom_pose = None
        self.cnt = 0
        self._create_subs()
        self.create_timer(5.0, self._watchdog)
        self.get_logger().info('map_tf_pub ready: map->odom')

    def _create_subs(self):
        self.loc_sub = self.create_subscription(
            PoseStamped, '/lightning/pose/current', self.loc_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/feedback/odom', self.odom_cb, 200)

    def _watchdog(self):
        if self.cnt == 0:
            self.get_logger().warn('No map->odom yet, re-subscribing...')
            self.destroy_subscription(self.loc_sub)
            self.destroy_subscription(self.odom_sub)
            self._create_subs()

    def loc_cb(self, msg):
        # PoseStamped: msg.pose 是 geometry_msgs/Pose
        yaw = quat_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z, msg.pose.orientation.w)
        self.loc_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._publish()

    def odom_cb(self, msg):
        # Odometry: msg.pose.pose 是 geometry_msgs/Pose (PoseWithCovariance 包了一层)
        yaw = quat_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _publish(self):
        if self.loc_pose is None or self.odom_pose is None:
            return
        T_map_robot = make_2d(*self.loc_pose)
        T_odom_robot = make_2d(*self.odom_pose)
        T_map_odom = T_map_robot @ np.linalg.inv(T_odom_robot)
        x, y, yaw = extract_2d(T_map_odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw * 0.5)
        t.transform.rotation.w = math.cos(yaw * 0.5)
        self.br.sendTransform(t)

        self.cnt += 1
        if self.cnt % 200 == 0:
            self.get_logger().info(f'#{self.cnt} map->odom x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}')


def main():
    rclpy.init()
    rclpy.spin(MapOdomTFPub())


if __name__ == '__main__':
    main()
