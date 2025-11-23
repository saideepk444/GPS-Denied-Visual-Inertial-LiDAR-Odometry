#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class DatasetPlayer : public rclcpp::Node
{
public:
  DatasetPlayer() : rclcpp::Node("dataset_player")
  {
    dataset_root_ = declare_parameter<std::string>("dataset_root", "");
    rate_scale_ = declare_parameter<double>("rate_scale", 1.0);
    loop_ = declare_parameter<bool>("loop", false);
    start_time_s_ = declare_parameter<double>("start_time_s", 0.0);

    if (rate_scale_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "rate_scale must be > 0, resetting to 1.0");
      rate_scale_ = 1.0;
    }
    if (start_time_s_ < 0.0) {
      RCLCPP_WARN(get_logger(), "start_time_s must be >= 0, resetting to 0");
      start_time_s_ = 0.0;
    }

    img_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 50);

    if (!load_dataset()) {
      RCLCPP_ERROR(get_logger(), "Failed to load dataset; node will idle.");
      return;
    }

    running_ = true;
    playback_thread_ = std::thread(&DatasetPlayer::playback_loop, this);
  }

  ~DatasetPlayer() override
  {
    running_ = false;
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
  }

private:
  struct ImuSample
  {
    uint64_t t_ns;
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
  };

  struct CamSample
  {
    uint64_t t_ns;
    std::string path;
  };

  enum class EventType
  {
    IMU,
    CAM
  };

  struct Event
  {
    EventType type;
    uint64_t t_ns;
    size_t idx;
  };

  bool load_dataset()
  {
    if (dataset_root_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'dataset_root' is empty.");
      return false;
    }

    const std::string imu_csv = dataset_root_ + "/imu0/data.csv";
    const std::string cam_csv = dataset_root_ + "/cam0/data.csv";
    const std::string cam_data_dir = dataset_root_ + "/cam0/data";

    if (!load_imu_csv(imu_csv)) {
      return false;
    }
    if (!load_cam_csv(cam_csv, cam_data_dir)) {
      return false;
    }

    build_events();
    if (events_.empty()) {
      RCLCPP_ERROR(get_logger(), "No events to play after parsing dataset.");
      return false;
    }

    t0_ns_ = events_.front().t_ns;
    const double duration_s = (events_.back().t_ns - t0_ns_) * 1e-9;
    RCLCPP_INFO(get_logger(),
                "Loaded dataset: %zu IMU samples, %zu images, duration %.2f s",
                imu_samples_.size(), cam_samples_.size(), duration_s);
    return true;
  }

  bool load_imu_csv(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open IMU csv: %s", path.c_str());
      return false;
    }

    std::string line;
    size_t line_no = 0;
    while (std::getline(file, line)) {
      ++line_no;
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::stringstream ss(line);
      std::string token;
      std::vector<std::string> tokens;
      while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
      }

      if (tokens.size() != 7) {
        RCLCPP_WARN(get_logger(), "IMU line %zu malformed (expected 7 fields)", line_no);
        continue;
      }

      try {
        ImuSample sample{};
        sample.t_ns = std::stoull(tokens[0]);
        sample.wx = std::stod(tokens[1]);
        sample.wy = std::stod(tokens[2]);
        sample.wz = std::stod(tokens[3]);
        sample.ax = std::stod(tokens[4]);
        sample.ay = std::stod(tokens[5]);
        sample.az = std::stod(tokens[6]);
        imu_samples_.push_back(sample);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "IMU line %zu parse error: %s", line_no, e.what());
        continue;
      }
    }

    if (imu_samples_.empty()) {
      RCLCPP_ERROR(get_logger(), "No IMU samples parsed from %s", path.c_str());
      return false;
    }
    return true;
  }

  bool load_cam_csv(const std::string &csv_path, const std::string &data_dir)
  {
    std::ifstream file(csv_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open camera csv: %s", csv_path.c_str());
      return false;
    }

    std::string line;
    size_t line_no = 0;
    while (std::getline(file, line)) {
      ++line_no;
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::stringstream ss(line);
      std::string ts_str;
      std::string filename;
      if (!std::getline(ss, ts_str, ',')) {
        RCLCPP_WARN(get_logger(), "Camera line %zu missing timestamp", line_no);
        continue;
      }
      if (!std::getline(ss, filename)) {
        RCLCPP_WARN(get_logger(), "Camera line %zu missing filename", line_no);
        continue;
      }

      try {
        CamSample sample{};
        sample.t_ns = std::stoull(ts_str);
        sample.path = data_dir + "/" + filename;
        cam_samples_.push_back(std::move(sample));
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Camera line %zu parse error: %s", line_no, e.what());
        continue;
      }
    }

    if (cam_samples_.empty()) {
      RCLCPP_ERROR(get_logger(), "No camera samples parsed from %s", csv_path.c_str());
      return false;
    }
    return true;
  }

  void build_events()
  {
    events_.clear();
    events_.reserve(imu_samples_.size() + cam_samples_.size());

    for (size_t i = 0; i < imu_samples_.size(); ++i) {
      events_.push_back({EventType::IMU, imu_samples_[i].t_ns, i});
    }
    for (size_t i = 0; i < cam_samples_.size(); ++i) {
      events_.push_back({EventType::CAM, cam_samples_[i].t_ns, i});
    }

    std::sort(events_.begin(), events_.end(),
              [](const Event &a, const Event &b) { return a.t_ns < b.t_ns; });
  }

  void playback_loop()
  {
    RCLCPP_INFO(get_logger(), "Starting playback: rate_scale=%.2f loop=%s start_time=%.2fs",
                rate_scale_, loop_ ? "true" : "false", start_time_s_);

    do {
      const auto wall_start = std::chrono::steady_clock::now();
      for (const auto &event : events_) {
        if (!running_ || !rclcpp::ok()) {
          return;
        }

        const double t_rel = static_cast<double>(event.t_ns - t0_ns_) * 1e-9;
        if (t_rel < start_time_s_) {
          continue; // skip the initial portion if requested
        }
        const double target = (t_rel - start_time_s_) / rate_scale_;
        wait_until(wall_start, target);

        if (!running_ || !rclcpp::ok()) {
          return;
        }
        publish_event(event);
      }

      if (loop_) {
        RCLCPP_INFO(get_logger(), "Looping sequence from start");
      }
    } while (loop_ && running_ && rclcpp::ok());

    RCLCPP_INFO(get_logger(), "Playback finished.");
  }

  void wait_until(const std::chrono::steady_clock::time_point &start, double target_seconds)
  {
    while (running_ && rclcpp::ok()) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed + 0.001 >= target_seconds) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void publish_event(const Event &event)
  {
    if (event.type == EventType::IMU) {
      publish_imu(imu_samples_[event.idx]);
    } else {
      publish_image(cam_samples_[event.idx]);
    }
  }

  void publish_imu(const ImuSample &sample)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sample.t_ns - t0_ns_), RCL_ROS_TIME);
    msg.header.frame_id = "imu";
    msg.angular_velocity.x = sample.wx;
    msg.angular_velocity.y = sample.wy;
    msg.angular_velocity.z = sample.wz;
    msg.linear_acceleration.x = sample.ax;
    msg.linear_acceleration.y = sample.ay;
    msg.linear_acceleration.z = sample.az;
    imu_pub_->publish(msg);
  }

  void publish_image(const CamSample &sample)
  {
    const cv::Mat img = cv::imread(sample.path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to read image: %s", sample.path.c_str());
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(static_cast<int64_t>(sample.t_ns - t0_ns_), RCL_ROS_TIME);
    header.frame_id = "camera";
    auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img).toImageMsg();
    img_pub_->publish(*msg);
  }

  std::string dataset_root_;
  double rate_scale_{1.0};
  bool loop_{false};
  double start_time_s_{0.0};
  uint64_t t0_ns_{0};
  bool running_{false};

  std::vector<ImuSample> imu_samples_;
  std::vector<CamSample> cam_samples_;
  std::vector<Event> events_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::thread playback_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DatasetPlayer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
