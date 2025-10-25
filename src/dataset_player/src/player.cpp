#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

class DatasetPlayer : public rclcpp::Node
{
public:
  DatasetPlayer() : rclcpp::Node("dataset_player")
  {
    // Publishers: image at ~30 Hz, IMU at ~30 Hz (for now)
    img_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 50);

    // Timer: 33 ms period ≈ 30 Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(33),
                               std::bind(&DatasetPlayer::publish_once, this));
  }

private:
  void publish_once()
  {
    // Image in mono8 grayscale 480 by 640
    sensor_msgs::msg::Image img;
    img.header.stamp = now();       // stamping messages immediately
    img.header.frame_id = "camera"; // produces from camera sensor
    img.height = 480;
    img.width = 640;
    img.encoding = "mono8";                       // 8-bit grayscale
    img.step = img.width;                         // bytes per row for mono8
    img.data.assign(img.height * img.width, 128); // fill gray pixels
    img_pub_->publish(img);

    // acceleromater + gyroscope
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = "imu";
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = 0.01;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 0.0;
    imu_pub_->publish(imu);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Publishing dummy image + IMU");
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatasetPlayer>());
  rclcpp::shutdown();
  return 0;
}
