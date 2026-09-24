#!/usr/bin/env python3
# coding=utf8
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from rclpy.utilities import remove_ros_args

from se3_utils import quaternion_from_euler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    parser.add_argument('--frame', default='global_map', help='frame of the pose (default: global_map)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('publish_initial_pose')
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    # 转换为pose
    quat = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    xyz = [args.x, args.y, args.z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                                  orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.header.frame_id = args.frame

    # wait (up to 5 s) for global_localization to discover us instead of a blind sleep
    deadline = time.time() + 5.0
    while pub_pose.get_subscription_count() == 0 and time.time() < deadline:
        time.sleep(0.1)
    if pub_pose.get_subscription_count() == 0:
        node.get_logger().warn('No subscriber on /initialpose, publishing anyway')

    node.get_logger().info('Initial Pose: {} {} {} {} {} {}'.format(
        args.x, args.y, args.z, args.yaw, args.pitch, args.roll))
    pub_pose.publish(initial_pose)
    time.sleep(0.5)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
