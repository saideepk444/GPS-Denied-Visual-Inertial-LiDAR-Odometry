#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

class LidarOdomNode : public rclcpp::Node
{
public:
  LidarOdomNode() : rclcpp::Node("lidar_odom_node")
  {
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.5);
    max_correspondence_distance_ = declare_parameter<double>("max_corr_dist", 2.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/points", rclcpp::SensorDataQoS(),
        std::bind(&LidarOdomNode::cloud_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar/odom", 10);

    pose_.setIdentity();
    RCLCPP_INFO(get_logger(), "LiDAR ICP node ready (leaf=%.2f m)", voxel_leaf_size_);
  }

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  CloudT::Ptr downsample(const CloudT::Ptr &cloud)
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    CloudT::Ptr filtered(new CloudT);
    vg.filter(*filtered);
    return filtered;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Empty LiDAR cloud received");
      return;
    }

    CloudT::Ptr filtered = downsample(cloud);

    if (!prev_cloud_) {
      prev_cloud_ = filtered;
      last_stamp_ = msg->header.stamp;
      publish_odom(msg->header.stamp, true);
      return;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(50);
    icp.setInputSource(filtered);
    icp.setInputTarget(prev_cloud_);

    CloudT::Ptr aligned(new CloudT);
    icp.align(*aligned);
    if (!icp.hasConverged()) {
      RCLCPP_WARN(get_logger(), "ICP failed to converge, skipping frame");
      return;
    }

    const Eigen::Matrix4f T = icp.getFinalTransformation();
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.linear() = T.block<3, 3>(0, 0).cast<double>();
    delta.translation() = T.block<3, 1>(0, 3).cast<double>();
    pose_ = pose_ * delta;

    prev_cloud_ = filtered;
    last_stamp_ = msg->header.stamp;

    publish_odom(msg->header.stamp, icp.hasConverged());
  }

  void publish_odom(const rclcpp::Time &stamp, bool converged)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = lidar_frame_;

    const Eigen::Quaterniond q(pose_.rotation());
    odom.pose.pose.position.x = pose_.translation().x();
    odom.pose.pose.position.y = pose_.translation().y();
    odom.pose.pose.position.z = pose_.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance[0] = converged ? 1.0 : 0.0;

    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  CloudT::Ptr prev_cloud_{nullptr};
  Eigen::Isometry3d pose_;
  rclcpp::Time last_stamp_{};

  double voxel_leaf_size_{};
  double max_correspondence_distance_{};
  std::string odom_frame_;
  std::string lidar_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomNode>());
  rclcpp::shutdown();
  return 0;
}
