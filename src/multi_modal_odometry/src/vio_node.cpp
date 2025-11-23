#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <deque>
#include <optional>
#include <vector>

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

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 200, std::bind(&VIONode::imu_callback, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&VIONode::image_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/vio/odom", 20);

    pose_.setIdentity();
    RCLCPP_INFO(get_logger(), "VIO node ready: feature_count=%d", feature_count_);
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
    // keep last ~1s of IMU
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
      detect_features(gray);
      prev_stamp_ = msg->header.stamp;
      prev_image_ = gray;
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

    Eigen::Matrix3d imu_prior = integrate_imu(prev_stamp_, msg->header.stamp);

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

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.linear() = imu_prior * R_cv_eig;
    delta.translation() = delta.linear() * t_eig * 0.1; // small baseline scale

    pose_ = pose_ * delta;

    publish_odom(msg->header.stamp, inliers, tracked_prev.size());

    prev_features_ = tracked_curr;
    if (static_cast<int>(prev_features_.size()) < feature_count_ / 2) {
      detect_features(gray);
    }
    prev_image_ = gray;
    prev_stamp_ = msg->header.stamp;
  }

  Eigen::Matrix3d integrate_imu(const rclcpp::Time &start, const rclcpp::Time &end)
  {
    Eigen::Vector3d delta_theta = Eigen::Vector3d::Zero();
    rclcpp::Time last = start;
    for (const auto &sample : imu_buffer_) {
      if (sample.stamp <= start || sample.stamp > end) {
        continue;
      }
      const double dt = (sample.stamp - last).seconds();
      delta_theta += sample.gyro * dt;
      last = sample.stamp;
    }
    const double angle = delta_theta.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (angle > 1e-6) {
      R = Eigen::AngleAxisd(angle, delta_theta.normalized()).toRotationMatrix();
    }
    return R;
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

    // store tracking confidence in covariance[0]
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

  Eigen::Isometry3d pose_;

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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VIONode>());
  rclcpp::shutdown();
  return 0;
}
