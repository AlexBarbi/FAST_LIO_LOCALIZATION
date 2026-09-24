#!/usr/bin/env python3
# coding=utf8
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from se3_utils import inverse_se3, mat_to_pose, pose_to_mat


class TransformFusion(Node):
    def __init__(self):
        super().__init__('transform_fusion')

        # tf and localization publishing frequency (HZ)
        freq_pub_localization = self.declare_parameter('freq_pub_localization', 50.0).value
        self.map_frame = self.declare_parameter('map_frame', 'global_map').value
        # FAST-LIO world frame, parent of the /Odometry pose
        self.odom_frame = self.declare_parameter('odom_frame', 'camera_init').value

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, '/map_to_odom', self.cb_save_map_to_odom, 1)
        self.pub_localization = self.create_publisher(Odometry, '/localization', 1)
        self.br = TransformBroadcaster(self)

        # 发布定位消息
        self.create_timer(1.0 / freq_pub_localization, self.transform_fusion)
        self.get_logger().info('Transform Fusion Node Inited...')

    def transform_fusion(self):
        cur_odom = self.cur_odom_to_baselink
        if self.cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        # map_frame hangs below odom_frame (the inverse of map -> odom): a TF frame has one parent, and FAST-LIO's frame
        # may already have its own (lio_map in the legged stack)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.map_frame
        pose = mat_to_pose(inverse_se3(T_map_to_odom))
        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z
        tf_msg.transform.rotation = pose.orientation
        self.br.sendTransform(tf_msg)

        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom.pose.pose)
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            localization.pose.pose = mat_to_pose(T_map_to_base_link)
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = self.map_frame
            localization.child_frame_id = cur_odom.child_frame_id or 'body'
            self.pub_localization.publish(localization)

    def cb_save_cur_odom(self, odom_msg):
        self.cur_odom_to_baselink = odom_msg

    def cb_save_map_to_odom(self, odom_msg):
        self.cur_map_to_odom = odom_msg


def main():
    rclpy.init()
    node = TransformFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
