#!/usr/bin/env python3
# coding=utf8
import math
import threading
import time
from collections import deque

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from se3_utils import inverse_se3, mat_to_pose, pose_to_mat, quat_to_rot


def msg_to_array(pc_msg):
    return point_cloud2.read_points_numpy(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)


def planar(T):
    """T kept to x, y and yaw."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    P = np.eye(4)
    P[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    P[:2, 3] = T[:2, 3]
    return P


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
        # Initial guess from the legged estimator (see initial_guess_from_odom) instead of waiting for /initialpose
        self.init_from_odom = self.declare_parameter('init_from_odom', True).value
        self.legged_odom_topic = self.declare_parameter('legged_odom_topic', '/odom').value
        # The static TF between them is where the LiDAR sits on the base (FAST-LIO tree of fast_lio_slam_launch.xml)
        self.base_frame = self.declare_parameter('base_frame', 'lio_base').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'unilidar_lio').value

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

        if self.init_from_odom:
            # (stamp in ns, base pose): /odom comes at up to 200 Hz, 2 s is more than FAST-LIO lags behind its stamps
            self.legged_odom = deque(maxlen=400)
            self.create_subscription(Odometry, self.legged_odom_topic, self.cb_save_legged_odom, 50)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.T_base_lidar = None
            self.warned_no_mount = False

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
        if self.init_from_odom:
            self.get_logger().info(f'Initializing from {self.legged_odom_topic} (or /initialpose)....')
        else:
            self.get_logger().warn('Waiting for initial pose....')

    def cb_save_cur_odom(self, odom_msg):
        with self.lock:
            self.cur_odom = odom_msg

    def cb_save_legged_odom(self, odom_msg):
        with self.lock:
            self.legged_odom.append((Time.from_msg(odom_msg.header.stamp).nanoseconds, odom_msg.pose.pose))

    def base_to_lidar(self):
        if self.T_base_lidar is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time())
            except Exception as e:
                # the LiDAR taken at the base only moves the guess by its lever arm (~0.3 m), ICP takes that
                if not self.warned_no_mount:
                    self.get_logger().warn(f'No TF {self.base_frame} -> {self.lidar_frame} ({e}): '
                                           'initial guess assumes the LiDAR at the base')
                    self.warned_no_mount = True
                return np.eye(4)
            t, q = tf.transform.translation, tf.transform.rotation
            self.T_base_lidar = np.eye(4)
            self.T_base_lidar[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
            self.T_base_lidar[:3, 3] = [t.x, t.y, t.z]
        return self.T_base_lidar

    def initial_guess_from_odom(self):
        """T_map_to_odom guessed from the legged estimator, so no initial pose has to be given by hand.

        /odom is the base pose in the estimator's odom frame, whose origin is where the robot stood when the
        controller started (the spawn) and whose yaw is the IMU's. The global map is the camera_init of the
        exploration, i.e. the LiDAR when FAST-LIO started there with the robot standing at the same spawn.
        So /odom kept to x, y and yaw (same flat floor, standing height either way) is the base now in the base
        frame at the start of the exploration, and
            T_map_to_odom = T_base_lidar^-1 * planar(T_odom_base) * T_base_lidar * T_camera_init_lidar^-1
        with T_camera_init_lidar the FAST-LIO pose (/Odometry) at the same stamp. The estimator drifts (legs slip),
        ICP corrects the guess; it only has to be within a few meters.
        """
        with self.lock:
            cur_odom = self.cur_odom
            legged = list(self.legged_odom)
        if cur_odom is None or not legged:
            self.get_logger().warn(f'Initial guess: waiting for /Odometry and {self.legged_odom_topic}',
                                   throttle_duration_sec=10.0)
            return None
        stamp = Time.from_msg(cur_odom.header.stamp).nanoseconds
        t, base_pose = min(legged, key=lambda entry: abs(entry[0] - stamp))
        if abs(t - stamp) > 0.5e9:
            self.get_logger().warn(f'{self.legged_odom_topic} and /Odometry stamps {abs(t - stamp) * 1e-9:.1f} s apart '
                                   '(different clocks?): using the closest', throttle_duration_sec=10.0)
        T_odom_base = planar(pose_to_mat(base_pose))
        T_base_lidar = self.base_to_lidar()
        self.get_logger().info('Initial guess from {}: base at x {:.2f} y {:.2f} yaw {:.1f} deg'.format(
            self.legged_odom_topic, T_odom_base[0, 3], T_odom_base[1, 3],
            math.degrees(math.atan2(T_odom_base[1, 0], T_odom_base[0, 0]))))
        return inverse_se3(T_base_lidar) @ T_odom_base @ T_base_lidar @ inverse_se3(pose_to_mat(cur_odom.pose.pose))

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
        elif self.init_from_odom and self.global_map is not None:
            # retried at every tick until a match passes localization_th; /initialpose still overrides
            guess = self.initial_guess_from_odom()
            if guess is not None and self.global_localization(guess):
                self.get_logger().info(f'Initialize successfully from {self.legged_odom_topic}!!!!!!')
                self.initialized = True


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
