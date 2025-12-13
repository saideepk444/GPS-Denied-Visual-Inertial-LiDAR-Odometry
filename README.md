# GPS-denied Visual–Inertial–LiDAR Odometry (ROS 2, C++)

ROS 2 workspace that replays EuRoC-style data and produces fused odometry by combining a lightweight visual–inertial frontend, a LiDAR ICP odometry frontend, and a GTSAM factor-graph backend.

## Stack at a glance
- `dataset_player/player`: Replays EuRoC `cam0` images and `imu0` CSV samples as `/camera/image_raw` (mono8) and `/imu` with dataset-relative stamps.
- `multi_modal_odometry/vio_node`: Tracks features with LK optical flow, uses IMU preintegration as a rotation prior, estimates relative pose with `recoverPose`, and publishes `/vio/odom`. Feature counts are packed into covariance for downstream gating.
- `multi_modal_odometry/lidar_odom_node`: Downsamples incoming `/lidar/points`, runs point-to-point ICP against the previous scan, accumulates pose, and publishes `/lidar/odom` (covariance[0] toggles ICP convergence).
- `multi_modal_odometry/fusion_node`: Maintains a GTSAM factor graph with BetweenFactors from VIO/LiDAR plus ImuFactors tying pose/velocity/bias. Gates VIO by feature count and staleness, then publishes optimized `/fused/odom`.

## Layout
- `src/dataset_player`: Dataset playback node (EuRoC layout `mav0/{cam0,imu0}`).
- `src/multi_modal_odometry/src/vio_node.cpp`: VIO frontend using OpenCV + GTSAM preintegration.
- `src/multi_modal_odometry/src/lidar_odom_node.cpp`: Sequential LiDAR ICP odometry.
- `src/multi_modal_odometry/src/fusion_node.cpp`: GTSAM fusion backend.

## Build
Prereqs: ROS 2 (tested with Humble-style APIs), OpenCV, PCL (common/io/filters/registration), GTSAM, Eigen3.

```bash
source /opt/ros/humble/setup.bash        # adjust to your distro
cd /Users/saideepk4/gpsdenied_ws
colcon build --packages-select dataset_player multi_modal_odometry
source install/setup.bash
```

## Running the pipeline
1) Dataset playback (expects EuRoC folder ending in `mav0`):
```bash
ros2 run dataset_player player --ros-args \
  -p dataset_root:=/path/to/EuRoC/MH_01_easy/mav0 \
  -p rate_scale:=1.0 -p loop:=false -p start_time_s:=0.0
```
2) VIO frontend (consumes `/camera/image_raw`, `/imu`):
```bash
ros2 run multi_modal_odometry vio_node
```
3) LiDAR odometry (requires a point cloud source on `/lidar/points`):
```bash
ros2 run multi_modal_odometry lidar_odom_node
```
4) Fusion backend:
```bash
ros2 run multi_modal_odometry fusion_node
```

Check flow with `ros2 topic list` and `ros2 topic hz /camera/image_raw`, `/imu`, `/lidar/odom`, `/fused/odom`. RViz: image display on `/camera/image_raw`, IMU on `/imu`, odometry displays for `/lidar/odom` and `/fused/odom`.

## Node details and parameters
**dataset_player/player**
- Reads `imu0/data.csv` (wx, wy, wz, ax, ay, az) and `cam0/data.csv` (timestamp, filename), builds a time-ordered queue, and sleeps against wall clock so playback timing matches the dataset (scaled by `rate_scale`).
- Publishes stamps relative to the first dataset timestamp (`RCL_ROS_TIME`).
- Parameters: `dataset_root` (required), `rate_scale` (>0), `loop` (bool), `start_time_s` (skip initial seconds).

**multi_modal_odometry/vio_node**
- Subscriptions: `/imu` (preintegrated continuously), `/camera/image_raw` (SensorData QoS).
- Pipeline per frame: detect features with `goodFeaturesToTrack` (first frame), track with LK optical flow, recover relative pose via essential matrix + `recoverPose`, combine IMU rotation prior with vision rotation, scale translation with a small baseline (0.1 m), and propagate velocity from IMU prediction.
- Publishes `/vio/odom` (`frame_id=odom_frame`, `child_frame_id=camera_frame`). Encodes inliers in `pose.covariance[0]` and tracked feature count in `pose.covariance[1]` for fusion gating.
- Key params: `feature_count`, `quality_level`, `min_distance`, `focal_length`, `cx`, `cy`, `min_tracked_for_pose`, `odom_frame`, `camera_frame`, IMU noise/bias parameters.

**multi_modal_odometry/lidar_odom_node**
- Subscriptions: `/lidar/points` (SensorData QoS). Downsamples with a voxel grid, aligns to previous filtered cloud using point-to-point ICP (50 iterations, `max_corr_dist`), and accumulates a global pose.
- Publishes `/lidar/odom` (`frame_id=odom_frame`, `child_frame_id=lidar_frame`). `pose.covariance[0]` is set to 1.0 when ICP converges.
- Key params: `voxel_leaf_size` (m), `max_corr_dist` (m), `odom_frame`, `lidar_frame`.

**multi_modal_odometry/fusion_node**
- Subscriptions: `/vio/odom`, `/lidar/odom`, `/imu`. Maintains Pose3/Vel/Bias nodes keyed by message order, seeded with an identity pose/velocity/bias prior.
- Adds BetweenFactors for each odometry input plus an ImuFactor and bias BetweenFactor per step. Optimizes with Levenberg–Marquardt every time a new factor is added, resets IMU preintegration with the updated bias, and publishes `/fused/odom`.
- Gating: skips VIO when tracked features (`covariance[1]`) drop below `feature_threshold` or when VIO is stale vs the last LiDAR message by more than `vio_timeout_sec`.
- Key params: `odom_frame`, `fused_frame`, `feature_threshold`, `vio_timeout_sec`, IMU noise/bias terms (`accel_noise`, `gyro_noise`, `accel_bias_noise`, `gyro_bias_noise`).

## Notes and assumptions
- EuRoC images are expected as mono8; if your camera data differs, adjust `camera_frame` and encoding accordingly.
- LiDAR odometry simply chains scan-to-scan ICP; provide a stable `/lidar/points` source (KITTI player, Gazebo bridge, etc.).
- Fusion assumes VIO publishes feature count in `pose.covariance[1]` and uses IMU timestamps to preintegrate between factors; keep IMU rate high and monotonic.
