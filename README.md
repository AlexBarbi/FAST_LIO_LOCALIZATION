# FAST-LIO-LOCALIZATION

A simple localization framework that can re-localize in built maps based on [FAST-LIO](https://github.com/hku-mars/FAST_LIO).

This is the **ROS 2** port of [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): the
Python 3 / rclpy localization scripts, on top of a ROS 2 FAST-LIO that runs on its own (the FAST-LIO copy of the
original repository is not part of the port).

## News

- **2021-08-11:** Add **Open3D 0.7** support.
  
- **2021-08-09:** Migrate to **Open3D** for better performance.

## 1. Features
- Realtime 3D global localization in a pre-built point cloud map. 
  By fusing low-frequency global localization (about 0.5~0.2Hz), and high-frequency odometry from FAST-LIO, the entire system is computationally efficient.

<div align="center"><img src="doc/demo.GIF" width=90% /></div>

- Eliminate the accumulative error of the odometry.

<div align="center"><img src="doc/demo_accu.GIF" width=90% /></div>

- The initial localization can be provided either by rough manual estimation from RVIZ, or pose from another sensor/algorithm.

<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="center">
<img src="doc/demo_init.GIF" width=49.6% />
<img src="doc/demo_init_2.GIF" width = 49.6% >
</div>


## 2. Prerequisites
Tested on **ROS 2 Jazzy** (Ubuntu 24.04).

This package is the localization layer only: it runs on top of a ROS 2 FAST-LIO started separately, e.g. the
ROS 2 port of [FAST-LIO](https://github.com/hku-mars/FAST_LIO). That one must publish `/Odometry` and a dense
`/cloud_registered` (`publish.dense_publish_en: true`), both in its world frame (`camera_init`).

- python 3, `rclpy`, `sensor_msgs_py`, `tf2_ros` (all shipped with ROS 2), numpy
- `pcl_ros` (its `pcd_to_pointcloud` node publishes the global map)

```shell
sudo apt install ros-$ROS_DISTRO-pcl-ros
```

- [Open3D](https://www.open3d.org/) (tested with 0.20)

```shell
pip install open3d
```

On Ubuntu 24.04 pip refuses system-wide installs, so put Open3D in a venv that can still see the ROS 2 packages
and pass its interpreter to the launch file (see 4.2):

```shell
python3 -m venv --system-site-packages ~/venv_o3d
~/venv_o3d/bin/pip install open3d
```


## 3. Build
Clone the repository and colcon build:

```
    cd ~/$A_ROS_DIR$/src
    git clone <this repository> FAST_LIO_LOCALIZATION
    cd ..
    colcon build --packages-select fast_lio_localization
    source install/setup.bash
```


## 4. Run Localization
### 4.1 Sample Dataset

Demo rosbag in a large underground garage (Livox AVIA):
[Google Drive](https://drive.google.com/file/d/15ZZAcz84mDxaWviwFPuALpkoeK-KAh-4/view?usp=sharing) | [Baidu Pan (Code: ne8d)](https://pan.baidu.com/s/1ceBiIAUqHa1vY3QjWpxwNA);

Corresponding map: [Google Drive](https://drive.google.com/file/d/1X_mhPlSCNj-1erp_DStCQZfkY7l4w7j8/view?usp=sharing) | [Baidu Pan (Code: kw6f)](https://pan.baidu.com/s/1Yw4vY3kEK8x2g-AsBi6VCw)

The bag is a ROS 1 bag: convert it with [rosbags](https://gitlab.com/ternaris/rosbags) (`rosbags-convert`).
Livox `livox_ros_driver/CustomMsg` messages must be converted to `livox_ros_driver2/msg/CustomMsg`.

The map can be built using LIO-SAM or FAST-LIO-SLAM.

### 4.2 Run

1. Start FAST-LIO with the config of your LiDAR (for the sample bag, the AVIA one of FAST-LIO).

2. Run localization:

```shell
ros2 launch fast_lio_localization localization_launch.xml map:=/path/to/your/map.pcd
```

Arguments: `map` (required), `fov` and `fov_far` (the defaults, 1.6 rad and 150 m, fit a Livox AVIA; a spinning LiDAR
takes `fov:=6.28` and its range), `map_frame`, `odom_frame`, `init_from_odom`, `use_sim_time`, `rviz`, `rviz_cfg`.
If Open3D lives in a venv, add `python:=$HOME/venv_o3d/bin/python`.

Wait for 3~5 seconds until the map cloud shows up in RVIZ;

3. If you are testing with the sample rosbag data:
```shell
ros2 bag play localization_test_scene_1
```

Or if you are running realtime, start your LiDAR driver, e.g. for a MID-360:

```shell
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
Please set the **publish_freq** of the driver to **10Hz**, to ensure there are enough points for global localization in a single scan. 

4. Provide initial pose
```shell
ros2 run fast_lio_localization publish_initial_pose.py 14.5 -7.5 0 -0.25 0 0 
```
The numerical value **14.5 -7.5 0 -0.25 0 0** denotes 6D pose **x y z yaw pitch roll** in the map frame (`global_map`), 
which is a rough initial guess for **localization_test_scene_1.bag**. 

The initial guess can also be provided by the '2D Pose Estimate' Tool in RVIZ.
A new initial pose can be sent at any time to re-localize.

Note that, during the initialization stage, it's better to keep the robot still. Or if you play bags, fistly play the bag for about 0.5s, and then pause the bag until the initialization succeed. 

### 4.3 Parameters, topics and TF

| `global_localization` | default | |
|---|---|---|
| `map_voxel_size` / `scan_voxel_size` | 0.4 / 0.1 | voxel size (m) for the ICP clouds |
| `freq_localization` | 0.5 | global localization frequency (Hz) |
| `localization_th` | 0.95 | minimum ICP fitness to accept a match |
| `fov` | 1.6 | LiDAR FOV (rad); > 3.14 means a spinning LiDAR, the map is then cropped by distance only |
| `fov_far` | 150.0 | map crop distance (m) |
| `map_frame` / `odom_frame` | global_map / camera_init | map frame, FAST-LIO world frame |
| `init_from_odom` | false | guess the initial pose from `legged_odom_topic` (see the script) |

`transform_fusion` takes `freq_pub_localization` (50 Hz), `map_frame` and `odom_frame`.

- Inputs: `/cloud_registered`, `/Odometry` (from FAST-LIO), `/map` (from `pcd_to_pointcloud`), `/initialpose`
- Outputs: `/map_to_odom`, `/localization` (fused odometry in `map`), `/submap` (the map crop the ICP uses)
- TF: `camera_init -> global_map` (transform_fusion). The map frame is published as a child of `camera_init` (the
  inverse of `map -> camera_init`): a TF frame has a single parent, so `camera_init` can keep one of its own
  (e.g. `odom -> lio_map -> camera_init`).


## Related Works
1. [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry (LIO) package
2. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
3. [FAST-LIO-SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): The integration of FAST-LIO with [Scan-Context](https://github.com/irapkaist/scancontext) **loop closure** module.
4. [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization): A simple system that can relocalize a robot on a built map based on LIO-SAM.


## Acknowledgments
Thanks for the authors of [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and [LIO-SAM_based_relocalization](https://github.com/Gaochao-hit/LIO-SAM_based_relocalization).

## TODO
1. Go over the timestamp issue of the published odometry and tf;
2. Using integrated points for global localization;
3. Fuse global localization with the state estimation of FAST-LIO, and smooth the localization trajectory; 
4. Updating...