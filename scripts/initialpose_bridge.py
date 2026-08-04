#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_interfaces.srv import SetPose
from std_srvs.srv import Trigger
import time

class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initialpose_bridge')
        self.set_pose_client = self.create_client(SetPose, '/localization_3d_manager/set_init_pose')
        self.stop_client = self.create_client(Trigger, '/localization_3d_manager/stop_localization')
        self.start_client = self.create_client(Trigger, '/localization_3d_manager/start_localization')
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.callback, 10)
        self.get_logger().info('initialpose_bridge ready - use RViz "2D Pose Estimate"')

    def callback(self, msg):
        self.get_logger().info(f'Received pose: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')
        self.get_logger().info('Stopping localization...')
        self.stop_client.call_async(Trigger.Request())
        time.sleep(1.0)
        self.get_logger().info('Setting init pose...')
        req = SetPose.Request()
        req.pose = msg
        future = self.set_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info('Init pose set OK')
        else:
            self.get_logger().error('Failed to set init pose')
            return
        time.sleep(0.5)
        self.get_logger().info('Starting localization...')
        self.start_client.call_async(Trigger.Request())
        self.get_logger().info('Localization restarted!')

def main():
    rclpy.init()
    node = InitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
