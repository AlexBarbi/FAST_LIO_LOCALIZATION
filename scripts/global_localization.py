#!/usr/bin/env python3
# coding=utf8
import threading
import time

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from se3_utils import inverse_se3, mat_to_pose, pose_to_mat


def msg_to_array(pc_msg):
    return point_cloud2.read_points_numpy(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)


class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('global_localization')

        self.map_voxel_size = self.declare_parameter('map_voxel_size', 0.4).value
        self.scan_voxel_size = self.declare_parameter('scan_voxel_size', 0.1).value
        # Global localization frequency (HZ)
        self.freq_localization = self.declare_parameter('freq_localization', 0.5).value
        # The threshold of global localization,
        # only those scan2map-matching with higher fitness than localization_th will be taken
        self.localization_th = self.declare_parameter('localization_th', 0.95).value
        # FOV(rad), modify this according to your LiDAR type; > 3.14 means a spinning LiDAR
        self.fov = self.declare_parameter('fov', 1.6).value
        # The farthest distance(meters) within FOV
        self.fov_far = self.declare_parameter('fov_far', 150.0).value
        self.map_frame = self.declare_parameter('map_frame', 'global_map').value
        # FAST-LIO world frame: /cloud_registered and /Odometry are expressed in it
        self.odom_frame = self.declare_parameter('odom_frame', 'camera_init').value

        self.lock = threading.Lock()
        self.global_map = None
        self.global_map_points = None
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None

        # publisher
        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)

        self.create_subscription(PointCloud2, '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        self.sub_map = self.create_subscription(PointCloud2, '/map', self.cb_global_map, 1)

        # ICP runs in its own group so the scan/odom callbacks keep flowing while it works
        icp_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.cb_initial_pose, 1,
                                 callback_group=icp_group)
        self.create_timer(1.0 / self.freq_localization, self.thread_localization, callback_group=icp_group)

        self.get_logger().info('Localization Node Inited...')
        self.get_logger().warn('Waiting for global map......')

    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
            pc_scan.voxel_down_sample(self.scan_voxel_size * scale),
            pc_map.voxel_down_sample(self.map_voxel_size * scale),
            1.0 * scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return result_icp.transformation, result_icp.fitness

    def publish_point_cloud(self, publisher, header, pc):
        publisher.publish(point_cloud2.create_cloud_xyz32(header, pc[:, :3].astype(np.float32)))

    def crop_global_map_in_FOV(self, pose_estimation, cur_odom):
        # 当前scan原点的位姿
        T_odom_to_base_link = pose_to_mat(cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = inverse_se3(T_map_to_base_link)

        # 把地图转换到lidar系下
        global_map_in_map = self.global_map_points
        global_map_in_base_link = global_map_in_map @ T_base_link_to_map[:3, :3].T + T_base_link_to_map[:3, 3]

        # 将视角内的地图点提取出来
        x = global_map_in_base_link[:, 0]
        in_angle = np.abs(np.arctan2(global_map_in_base_link[:, 1], x)) < self.fov / 2.0
        if self.fov > 3.14:
            # 环状lidar 仅过滤距离
            mask = (np.linalg.norm(global_map_in_base_link, axis=1) < self.fov_far) & in_angle
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            mask = (x > 0) & (x < self.fov_far) & in_angle
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(global_map_in_map[mask])

        # 发布fov内点云
        header = Header(stamp=cur_odom.header.stamp, frame_id=self.map_frame)
        self.publish_point_cloud(self.pub_submap, header, global_map_in_map[mask][::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        # 用icp配准
        self.get_logger().info('Global localization by scan-to-map matching......')

        with self.lock:
            scan = self.cur_scan
            cur_odom = self.cur_odom
        if scan is None or cur_odom is None:
            self.get_logger().warn('First scan or odometry not received!!!!!')
            return False
        scan_tobe_mapped = o3d.geometry.PointCloud()
        scan_tobe_mapped.points = o3d.utility.Vector3dVector(scan)

        tic = time.time()

        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation, cur_odom)

        # 粗配准
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV,
                                                       initial=pose_estimation, scale=5)

        # 精配准
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV,
                                                             initial=transformation, scale=1)
        toc = time.time()
        self.get_logger().info('Time: {}'.format(toc - tic))

        # 当全局定位成功时才更新map2odom
        if fitness > self.localization_th:
            self.T_map_to_odom = transformation

            # 发布map_to_odom
            map_to_odom = Odometry()
            map_to_odom.pose.pose = mat_to_pose(self.T_map_to_odom)
            map_to_odom.header.stamp = cur_odom.header.stamp
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            self.pub_map_to_odom.publish(map_to_odom)
            return True
        else:
            self.get_logger().warn('Not match!!!!')
            self.get_logger().warn('{}'.format(transformation))
            self.get_logger().warn('fitness score:{}'.format(fitness))
            return False

    def cb_global_map(self, pc_msg):
        if self.global_map is not None:
            return
        global_map = o3d.geometry.PointCloud()
        global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg))
        global_map = global_map.voxel_down_sample(self.map_voxel_size)
        self.global_map_points = np.asarray(global_map.points)
        self.global_map = global_map
        self.destroy_subscription(self.sub_map)
        self.get_logger().info('Global map received.')
        self.get_logger().warn('Waiting for initial pose....')

    def cb_save_cur_odom(self, odom_msg):
        with self.lock:
            self.cur_odom = odom_msg

    def cb_save_cur_scan(self, pc_msg):
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        pc_msg.header.frame_id = self.odom_frame
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)

        pc = msg_to_array(pc_msg)
        with self.lock:
            self.cur_scan = pc

    def cb_initial_pose(self, pose_msg):
        if self.global_map is None:
            self.get_logger().warn('Global map not received yet, initial pose ignored')
            return
        initial_pose = pose_to_mat(pose_msg.pose.pose)
        if self.global_localization(initial_pose):
            if not self.initialized:
                self.get_logger().info('Initialize successfully!!!!!!')
            self.initialized = True
        elif not self.initialized:
            self.get_logger().warn('Waiting for initial pose....')

    def thread_localization(self):
        # 每隔一段时间进行全局定位
        # 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        if self.initialized:
            self.global_localization(self.T_map_to_odom)


def main():
    rclpy.init()
    node = GlobalLocalization()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
