#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>

using gtsam::symbol_shorthand::X;

class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : rclcpp::Node("fusion_node")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    fused_frame_ = declare_parameter<std::string>("fused_frame", "base_link");
    feature_threshold_ = declare_parameter<int>("feature_threshold", 10);

    vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/vio/odom", 50, std::bind(&FusionNode::vio_callback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lidar/odom", 20, std::bind(&FusionNode::lidar_callback, this, std::placeholders::_1));
    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odom", 20);

    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
    vio_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished());
    lidar_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());

    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3::identity(), prior_noise_));
    estimates_.insert(X(0), gtsam::Pose3::identity());
    latest_key_ = 0;
  }

private:
  void vio_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const int tracked = static_cast<int>(msg->pose.covariance[1]);
    if (tracked < feature_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping VIO factor due to low features (%d)", tracked);
      return;
    }
    add_factor(*msg, vio_noise_);
  }

  void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    add_factor(*msg, lidar_noise_);
  }

  void add_factor(const nav_msgs::msg::Odometry &odom_msg, const gtsam::SharedNoiseModel &noise)
  {
    const gtsam::Pose3 pose = pose_from_msg(odom_msg.pose.pose);

    const gtsam::Pose3 last_pose = estimates_.at<gtsam::Pose3>(X(latest_key_));
    const gtsam::Pose3 between = last_pose.between(pose);

    const size_t new_key = latest_key_ + 1;
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(latest_key_), X(new_key), between, noise));
    estimates_.insert(X(new_key), pose);

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(20);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, estimates_, params);
    estimates_ = optimizer.optimize();
    latest_key_ = new_key;

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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimates_;
  gtsam::SharedNoiseModel prior_noise_;
  gtsam::SharedNoiseModel vio_noise_;
  gtsam::SharedNoiseModel lidar_noise_;
  size_t latest_key_{0};

  std::string odom_frame_;
  std::string fused_frame_;
  int feature_threshold_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}
