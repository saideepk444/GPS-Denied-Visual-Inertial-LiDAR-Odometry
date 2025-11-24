#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>

using gtsam::symbol_shorthand::X; // pose
using gtsam::symbol_shorthand::V; // velocity
using gtsam::symbol_shorthand::B; // bias

// Factor-graph fusion of VIO and LiDAR odometry using GTSAM.
// - Subscribes to /vio/odom and /lidar/odom, /imu
// - Adds BetweenFactors for odometry and ImuFactors tying pose/vel/bias
// - Gates VIO factors by feature count (covariance[1]) to handle vision dropouts
// - Publishes /fused/odom as the optimized pose
class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : rclcpp::Node("fusion_node")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    fused_frame_ = declare_parameter<std::string>("fused_frame", "base_link");
    feature_threshold_ = declare_parameter<int>("feature_threshold", 10);
    vio_timeout_sec_ = declare_parameter<double>("vio_timeout_sec", 1.0);
    accel_noise_ = declare_parameter<double>("accel_noise", 0.05);
    gyro_noise_ = declare_parameter<double>("gyro_noise", 0.01);
    accel_bias_noise_ = declare_parameter<double>("accel_bias_noise", 0.0005);
    gyro_bias_noise_ = declare_parameter<double>("gyro_bias_noise", 0.0005);

    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vio/odom", 50, std::bind(&FusionNode::vio_callback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lidar/odom", 20, std::bind(&FusionNode::lidar_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 200, std::bind(&FusionNode::imu_callback, this, std::placeholders::_1));
    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odom", 20);

    // Tunable noise models for each source; tighten prior, loosen VIO vs LiDAR.
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
    vio_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());
    lidar_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());

    // IMU preintegration setup (gravity along -Z)
    auto pim_params = gtsam::PreintegrationParams::MakeSharedU(9.81);
    pim_params->accelerometerCovariance = gtsam::I_3x3 * accel_noise_ * accel_noise_;
    pim_params->gyroscopeCovariance = gtsam::I_3x3 * gyro_noise_ * gyro_noise_;
    pim_params->integrationCovariance = gtsam::I_3x3 * 1e-6;
    pim_params->biasAccCovariance = gtsam::I_3x3 * accel_bias_noise_ * accel_bias_noise_;
    pim_params->biasOmegaCovariance = gtsam::I_3x3 * gyro_bias_noise_ * gyro_bias_noise_;
    bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << accel_bias_noise_, accel_bias_noise_, accel_bias_noise_,
         gyro_bias_noise_, gyro_bias_noise_, gyro_bias_noise_)
            .finished());
    imu_preintegrator_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(pim_params, bias_);

    // Seed the graph with an identity prior so later factors have a reference.
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3::identity(), prior_noise_));
    graph_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0),
                                                  gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));
    graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), bias_, bias_noise_));
    estimates_.insert(X(0), gtsam::Pose3::identity());
    estimates_.insert(V(0), gtsam::Vector3(0, 0, 0));
    estimates_.insert(B(0), bias_);
    latest_key_ = 0;
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (last_imu_stamp_.nanoseconds() == 0) {
      last_imu_stamp_ = msg->header.stamp;
      return;
    }
    const double dt = (msg->header.stamp - last_imu_stamp_).seconds();
    if (dt <= 0.0) {
      last_imu_stamp_ = msg->header.stamp;
      return;
    }

    const gtsam::Vector3 acc(msg->linear_acceleration.x, msg->linear_acceleration.y,
                             msg->linear_acceleration.z);
    const gtsam::Vector3 gyr(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);
    imu_preintegrator_->integrateMeasurement(acc, gyr, dt);
    last_imu_stamp_ = msg->header.stamp;
  }

  void vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Overload covariance[1] as "features tracked" from the VIO frontend.
    const int tracked = static_cast<int>(msg->pose.covariance[1]);
    if (tracked < feature_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping VIO factor due to low features (%d)", tracked);
      return;
    }
    last_vio_stamp_ = msg->header.stamp;
    add_factor(*msg, vio_noise_);
  }

  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_lidar_stamp_ = msg->header.stamp;
    add_factor(*msg, lidar_noise_);
  }

  // Add a between-factor from the latest estimate to the new odometry pose.
  void add_factor(const nav_msgs::msg::Odometry &odom_msg, const gtsam::SharedNoiseModel &noise)
  {
    if (last_factor_stamp_.nanoseconds() != 0 && odom_msg.header.stamp <= last_factor_stamp_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Out-of-order odometry ignored");
      return;
    }

    if (noise == vio_noise_ &&
        last_lidar_stamp_.nanoseconds() != 0 &&
        (odom_msg.header.stamp - last_lidar_stamp_).seconds() > vio_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "VIO stale vs LiDAR, skipping VIO factor");
      return;
    }

    const gtsam::Pose3 pose = pose_from_msg(odom_msg.pose.pose);

    const gtsam::Pose3 last_pose = estimates_.at<gtsam::Pose3>(X(latest_key_));
    const gtsam::Pose3 between = last_pose.between(pose);

    const size_t new_key = latest_key_ + 1;
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_key_), X(new_key), between, noise));

    // Add IMU factor to tie pose/velocity/bias across this interval.
    graph_.add(gtsam::ImuFactor(X(latest_key_), V(latest_key_),
                                X(new_key), V(new_key),
                                B(latest_key_), *imu_preintegrator_));
    graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(latest_key_), B(new_key), gtsam::imuBias::ConstantBias(), bias_noise_));

    estimates_.insert(X(new_key), pose);
    estimates_.insert(V(new_key), estimates_.at<gtsam::Vector3>(V(latest_key_)));
    estimates_.insert(B(new_key), bias_);

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(20);
    // Solve quickly after each new factor to keep estimates live.
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, estimates_, params);
    estimates_ = optimizer.optimize();
    latest_key_ = new_key;
    last_factor_stamp_ = odom_msg.header.stamp;

    // Update bias and reset preintegrator for the next window.
    bias_ = estimates_.at<gtsam::imuBias::ConstantBias>(B(latest_key_));
    imu_preintegrator_->resetIntegrationAndSetBias(bias_);

    publish_fused(odom_msg.header.stamp, estimates_.at<gtsam::Pose3>(X(latest_key_)));
  }

  gtsam::Pose3 pose_from_msg(const geometry_msgs::msg::Pose &pose_msg)
  {
    Eigen::Quaterniond q(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y,
                         pose_msg.orientation.z);
    Eigen::Vector3d t(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    return gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(t));
  }

  void publish_fused(const rclcpp::Time &stamp, const gtsam::Pose3 &pose)
  {
    nav_msgs::msg::Odometry fused;
    fused.header.stamp = stamp;
    fused.header.frame_id = odom_frame_;
    fused.child_frame_id = fused_frame_;

    fused.pose.pose.position.x = pose.translation().x();
    fused.pose.pose.position.y = pose.translation().y();
    fused.pose.pose.position.z = pose.translation().z();

    const Eigen::Quaterniond q = pose.rotation().toQuaternion();
    fused.pose.pose.orientation.x = q.x();
    fused.pose.pose.orientation.y = q.y();
    fused.pose.pose.orientation.z = q.z();
    fused.pose.pose.orientation.w = q.w();

    fused_pub_->publish(fused);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimates_;
  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel vio_noise_;
  gtsam::SharedNoiseModel lidar_noise_;
  gtsam::SharedNoiseModel bias_noise_;
  size_t latest_key_{0};

  std::string odom_frame_;
  std::string fused_frame_;
  int feature_threshold_{};
  double vio_timeout_sec_{};

  // IMU preintegration state
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
  gtsam::imuBias::ConstantBias bias_{};
  rclcpp::Time last_imu_stamp_{};
  rclcpp::Time last_factor_stamp_{};
  rclcpp::Time last_vio_stamp_{};
  rclcpp::Time last_lidar_stamp_{};
  double accel_noise_{};
  double gyro_noise_{};
  double accel_bias_noise_{};
  double gyro_bias_noise_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
