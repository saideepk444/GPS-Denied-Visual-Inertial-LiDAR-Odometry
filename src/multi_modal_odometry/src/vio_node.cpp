#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegratedImuMeasurements.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/NavState.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <deque>
#include <optional>
#include <vector>

// Lightweight VIO frontend:
// - Buffers IMU and preintegrates between image frames (GTSAM)
// - Tracks features with LK optical flow
// - Combines IMU rotation prior with vision pose (recoverPose)
// - Publishes /vio/odom and stores feature count in covariance[1] for fusion gating
class VIONode : public rclcpp::Node
{
public:
  VIONode() : rclcpp::Node("vio_node")
  {
    feature_count_ = declare_parameter<int>("feature_count", 200);
    quality_level_ = declare_parameter<double>("quality_level", 0.01);
    min_distance_ = declare_parameter<double>("min_distance", 8.0);
    focal_length_ = declare_parameter<double>("focal_length", 460.0);
    cx_ = declare_parameter<double>("cx", 320.0);
    cy_ = declare_parameter<double>("cy", 240.0);
    imu_frame_id_ = declare_parameter<std::string>("imu_frame", "imu");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame", "camera");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame", "odom");
    min_tracked_for_pose_ = declare_parameter<int>("min_tracked_for_pose", 15);
    accel_noise_ = declare_parameter<double>("accel_noise", 0.05);
    gyro_noise_ = declare_parameter<double>("gyro_noise", 0.01);
    accel_bias_noise_ = declare_parameter<double>("accel_bias_noise", 0.0005);
    gyro_bias_noise_ = declare_parameter<double>("gyro_bias_noise", 0.0005);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 200, std::bind(&VIONode::imu_callback, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&VIONode::image_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 20);

    pose_.setIdentity();
    velocity_.setZero();
    bias_ = gtsam::imuBias::ConstantBias(); // zero biases

    // Preintegration params (gravity along -Z)
    auto pim_params = gtsam::PreintegrationParams::MakeSharedU(9.81);
    pim_params->accelerometerCovariance = gtsam::I_3x3 * accel_noise_ * accel_noise_;
    pim_params->gyroscopeCovariance = gtsam::I_3x3 * gyro_noise_ * gyro_noise_;
    pim_params->integrationCovariance = gtsam::I_3x3 * 1e-6;
    pim_params->biasAccCovariance = gtsam::I_3x3 * accel_bias_noise_ * accel_bias_noise_;
    pim_params->biasOmegaCovariance = gtsam::I_3x3 * gyro_bias_noise_ * gyro_bias_noise_;
    preintegrator_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(pim_params, bias_);

    RCLCPP_INFO(get_logger(), "VIO node ready: features=%d, min_tracked=%d",
                feature_count_, min_tracked_for_pose_);
  }

private:
  struct ImuData
  {
    rclcpp::Time stamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
  };

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuData data;
    data.stamp = msg->header.stamp;
    data.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                                msg->angular_velocity.z);
    data.accel = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                 msg->linear_acceleration.z);
    imu_buffer_.push_back(data);

    // Integrate into preintegrator as measurements arrive.
    if (last_imu_stamp_.nanoseconds() != 0) {
      const double dt = (msg->header.stamp - last_imu_stamp_).seconds();
      if (dt > 0.0) {
        preintegrator_->integrateMeasurement(data.accel, data.gyro, dt);
      }
    }
    last_imu_stamp_ = msg->header.stamp;

    // keep last ~1s of IMU for diagnostics (not required for preintegration)
    const rclcpp::Time cutoff = msg->header.stamp - rclcpp::Duration(1, 0);
    while (!imu_buffer_.empty() && imu_buffer_.front().stamp < cutoff) {
      imu_buffer_.pop_front();
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat gray = cv_bridge::toCvCopy(msg, "mono8")->image;
    if (gray.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty image");
      return;
    }

    if (prev_image_.empty()) {
      // First frame: just detect features and start preintegrator window.
      detect_features(gray);
      prev_stamp_ = msg->header.stamp;
      preintegrator_->resetIntegrationAndSetBias(bias_);
      return;
    }

    std::vector<cv::Point2f> tracked_prev, tracked_curr;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_image_, gray, prev_features_, tracked_curr, status, err,
                             cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

    for (size_t i = 0; i < status.size(); ++i) {
      if (status[i]) {
        tracked_prev.push_back(prev_features_[i]);
        tracked_curr.push_back(tracked_curr[i]);
      }
    }

    if (tracked_prev.size() < static_cast<size_t>(min_tracked_for_pose_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Too few features (%zu) to recover pose; redetect", tracked_prev.size());
      detect_features(gray);
      prev_image_ = gray;
      prev_stamp_ = msg->header.stamp;
      return;
    }

    // IMU preintegrated delta between prev and current frame
    gtsam::NavState imu_pred = preintegrator_->predict(
        gtsam::NavState(gtsam::Pose3(pose_), gtsam::Vector3(velocity_.x(), velocity_.y(), velocity_.z())),
        bias_);

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(tracked_prev, tracked_curr, focal_length_, cv::Point2d(cx_, cy_),
                                     cv::RANSAC, 0.999, 1.0, mask);
    cv::Mat R_cv, t_cv;
    const int inliers = cv::recoverPose(E, tracked_prev, tracked_curr, R_cv, t_cv, focal_length_,
                                        cv::Point2d(cx_, cy_), mask);

    Eigen::Matrix3d R_cv_eig;
    cv::cv2eigen(R_cv, R_cv_eig);
    Eigen::Vector3d t_eig;
    cv::cv2eigen(t_cv, t_eig);
    t_eig.normalize();

    // Combine IMU rotation prior with visual rotation (simple composition)
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.linear() = imu_pred.pose().rotation().matrix() * R_cv_eig;
    // Scale translation with a small baseline; could be refined with scale estimation
    delta.translation() = delta.linear() * t_eig * 0.1;

    pose_ = pose_ * delta;
    velocity_ = imu_pred.v();

    publish_odom(msg->header.stamp, inliers, tracked_prev.size());

    // Reset preintegration window at this keyframe
    preintegrator_->resetIntegrationAndSetBias(bias_);
    prev_features_ = tracked_curr;
    if (static_cast<int>(prev_features_.size()) < feature_count_ / 2) {
      detect_features(gray);
    }
    prev_stamp_ = msg->header.stamp;
    prev_image_ = gray;
  }

  void detect_features(const cv::Mat &img)
  {
    cv::goodFeaturesToTrack(img, prev_features_, feature_count_, quality_level_, min_distance_);
  }

  void publish_odom(const rclcpp::Time &stamp, int inliers, size_t tracked)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = camera_frame_id_;

    const Eigen::Quaterniond q(pose_.rotation());
    odom.pose.pose.position.x = pose_.translation().x();
    odom.pose.pose.position.y = pose_.translation().y();
    odom.pose.pose.position.z = pose_.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // store tracking confidence in covariance[0] and features tracked in covariance[1] (used by fusion)
    odom.pose.covariance[0] = static_cast<double>(inliers);
    odom.pose.covariance[1] = static_cast<double>(tracked);

    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::deque<ImuData> imu_buffer_;
  cv::Mat prev_image_;
  std::vector<cv::Point2f> prev_features_;
  rclcpp::Time prev_stamp_{};
  rclcpp::Time last_imu_stamp_{};

  Eigen::Isometry3d pose_;
  Eigen::Vector3d velocity_;
  gtsam::imuBias::ConstantBias bias_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> preintegrator_;

  int feature_count_{};
  double quality_level_{};
  double min_distance_{};
  double focal_length_{};
  double cx_{};
  double cy_{};
  int min_tracked_for_pose_{};
  std::string imu_frame_id_;
  std::string camera_frame_id_;
  std::string odom_frame_id_;
  double accel_noise_{};
  double gyro_noise_{};
  double accel_bias_noise_{};
  double gyro_bias_noise_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VIONode>());
  rclcpp::shutdown();
  return 0;
}
