# GPS-denied Visual–Inertial–LiDAR Odometry (C++/ROS 2)

End-to-end ROS 2 pipeline for GPS-denied odometry:
- Dataset player that replays EuRoC-style camera + IMU topics with correct timing.
- LiDAR ICP odometry frontend.
- Factor-graph fusion backend (GTSAM) combining VIO and LiDAR odometry.

## Repository layout
- `src/dataset_player`: Publishes EuRoC `cam0` and `imu0` as ROS topics with dataset timestamps.
- `src/multi_modal_odometry/src/lidar_odom_node.cpp`: LiDAR ICP odometry (cloud->cloud ICP against previous scan).
- `src/multi_modal_odometry/src/fusion_node.cpp`: GTSAM factor graph fusing VIO (/vio/odom) and LiDAR (/lidar/odom) into /fused/odom.

## Quickstart
1) Source ROS 2 (adjust distro/path as needed):
   - `source /opt/ros/humble/setup.bash`
2) Build:
   - `cd /Users/saideepk4/gpsdenied_ws`
   - `colcon build --packages-select dataset_player multi_modal_odometry`
   - `source install/setup.bash`
3) Run dataset player (EuRoC layout expected: `.../mav0/{cam0,imu0}`):
   - `ros2 run dataset_player player --ros-args -p dataset_root:=/path/to/EuRoC/MH_01_easy/mav0 -p rate_scale:=1.0 -p loop:=false`
4) Run VIO (subscribes `/camera/image_raw`, `/imu`, publishes `/vio/odom`):
   - `ros2 run multi_modal_odometry vio_node`
5) Run LiDAR odometry (expects `/lidar/points`):
   - `ros2 run multi_modal_odometry lidar_odom_node`
6) Run fusion backend (consumes `/vio/odom` and `/lidar/odom`):
   - `ros2 run multi_modal_odometry fusion_node`

## Node behavior (what each piece does)
- `dataset_player`:
  - Parses `imu0/data.csv` and `cam0/data.csv`, builds a time-ordered event queue.
  - Sleeps against wall-clock to mimic dataset timing; stamps messages with dataset time (relative to first sample).
  - Publishes `/camera/image_raw` (mono8) and `/imu`.
  - Params: `dataset_root` (required), `rate_scale` (speed up/down), `loop` (repeat), `start_time_s` (skip initial seconds).

- `lidar_odom_node`:
  - Downsamples incoming `/lidar/points` with a voxel grid.
  - Runs point-to-point ICP against the previous filtered cloud; accumulates pose.
  - Publishes `/lidar/odom`; uses pose.covariance[0] as a simple “converged” flag.
  - Params: `voxel_leaf_size` (m), `max_corr_dist` (ICP correspondence distance), `odom_frame`, `lidar_frame`.

- `fusion_node`:
  - Maintains a GTSAM factor graph with Pose3 nodes.
  - Adds BetweenFactors from VIO and LiDAR odometry; seeds with an identity prior plus velocity/bias priors.
  - Adds ImuFactors tying pose/velocity/bias between keyframes; resets preintegration each factor.
  - Skips VIO factors when `features_tracked` (overloaded into pose.covariance[1]) drops below `feature_threshold` or when VIO is stale vs LiDAR (`vio_timeout_sec`).
  - Optimizes after each new factor and publishes `/fused/odom`.
  - Params: `odom_frame`, `fused_frame`, `feature_threshold`, `vio_timeout_sec`, IMU noise params.

## Verifying data flow
- `ros2 topic list` → check `/camera/image_raw`, `/imu`, `/lidar/odom`, `/fused/odom`.
- `ros2 topic hz /camera/image_raw` (≈20 Hz EuRoC), `/imu` (≈200 Hz).
- RViz:
  - Add Image display on `/camera/image_raw`.
  - Add IMU display on `/imu`.
  - Add Odometry displays for `/lidar/odom` and `/fused/odom`.

## Evaluation (ATE / drift / throughput)
- Absolute Trajectory Error (ATE) with evo (requires ground truth CSV/poses):
  - Save fused odom: `ros2 bag record /fused/odom` during playback.
  - Convert to TUM format (write a small script) then run: `evo_ape tum gt.txt fused.txt -r full --save_results results.zip`
- Drift percentage: compute path length vs ATE_rmse (evo outputs both); target <1% drift over sequence.
- Throughput: monitor `ros2 topic hz /camera/image_raw` (~20–30 Hz), `/lidar/points` (10–20 Hz), `/fused/odom` (should follow slower of inputs). Add `RCLCPP_INFO_THROTTLE` if you need inline Hz logs.
- KITTI/Gazebo: feed `/lidar/points` from a KITTI player or Gazebo bridge; record `/fused/odom`, compare vs KITTI ground truth with evo; visualize in RViz the same way.

## Tips and assumptions
- EuRoC is grayscale mono8; stamps are dataset-relative (starting at 0 s).
- LiDAR odom requires a point cloud source on `/lidar/points`; if using a different dataset, remap the topic.
- Fusion expects VIO to populate pose.covariance[1] with “features tracked” to gate low-confidence frames.

## Next steps
- Hook up a VIO frontend publishing `/vio/odom` with covariance[1] = tracked feature count.
- Add bag/dataset launch files to start player + frontends together.
- Evaluate drift against EuRoC ground truth (e.g., evo_ape, evo_rpe).
