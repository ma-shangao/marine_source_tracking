// Copyright 2025 author
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometryRecorder : public rclcpp::Node
{
public:
  OdometryRecorder()
  : Node("odometry_recorder")
  {
    RCLCPP_INFO(get_logger(), "Initializing Odometry Recorder node");

    // Declare parameters
    declare_parameter("output_file", "odometry_data.csv");
    declare_parameter("append_timestamp", true);
    declare_parameter("recording_rate_hz", 10.0);  // Recording frequency in Hz

    // Get parameters
    std::string base_filename = get_parameter("output_file").as_string();
    bool append_timestamp = get_parameter("append_timestamp").as_bool();
    double recording_rate = get_parameter("recording_rate_hz").as_double();

    // Create filename with timestamp if requested
    if (append_timestamp) {
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      ss << "_" << std::setfill('0') << std::setw(3) << ms.count();

      // Insert timestamp before file extension
      size_t dot_pos = base_filename.find_last_of('.');
      if (dot_pos != std::string::npos) {
        filename_ = base_filename.substr(0, dot_pos) + "_" + ss.str() +
          base_filename.substr(dot_pos);
      } else {
        filename_ = base_filename + "_" + ss.str() + ".csv";
      }
    } else {
      filename_ = base_filename;
    }

    // Create output directory if it doesn't exist
    std::filesystem::path file_path(filename_);
    if (file_path.has_parent_path()) {
      std::filesystem::create_directories(file_path.parent_path());
    }

    // Open CSV file and write header
    csv_file_.open(filename_, std::ios::out);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filename_.c_str());
      return;
    }

    // Write CSV header
    csv_file_ <<
      "ros_time_sec,ros_time_nanosec,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
    csv_file_.flush();

    RCLCPP_INFO(get_logger(), "Recording odometry data to: %s at %.1f Hz", filename_.c_str(),
      recording_rate);

    // Create subscription to odometry topic (store latest message)
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/blueboat/odometry",
      rclcpp::SensorDataQoS(),
      std::bind(&OdometryRecorder::odometryCallback, this, std::placeholders::_1)
    );

    // Create timer for periodic recording
    auto timer_period = std::chrono::duration<double>(1.0 / recording_rate);
    recording_timer_ = create_wall_timer(
      timer_period,
      std::bind(&OdometryRecorder::recordingTimerCallback, this)
    );

    // Statistics
    message_count_ = 0;
    start_time_ = this->now();
  }

  ~OdometryRecorder()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();

      auto end_time = this->now();
      auto duration = end_time - start_time_;

      RCLCPP_INFO(get_logger(), "Odometry recording finished.");
      RCLCPP_INFO(get_logger(), "Total messages recorded: %zu", message_count_);
      RCLCPP_INFO(get_logger(), "Recording duration: %.2f seconds", duration.seconds());
      RCLCPP_INFO(get_logger(), "Average rate: %.2f Hz",
                  message_count_ / duration.seconds());
      RCLCPP_INFO(get_logger(), "Data saved to: %s", filename_.c_str());
    }
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store the latest odometry message
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    latest_odometry_ = msg;
  }

  void recordingTimerCallback()
  {
    std::lock_guard<std::mutex> lock(odometry_mutex_);

    if (!csv_file_.is_open() || !latest_odometry_) {
      return;
    }

    auto msg = latest_odometry_;

    // Get ROS time
    rclcpp::Time ros_time = msg->header.stamp;

    // Extract position
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // Extract orientation (quaternion)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // Write to CSV file with high precision
    csv_file_ << std::fixed << std::setprecision(9)
              << ros_time.seconds() << ","
              << ros_time.nanoseconds() << ","
              << std::setprecision(6)
              << x << ","
              << y << ","
              << z << ","
              << qx << ","
              << qy << ","
              << qz << ","
              << qw << "\n";

    csv_file_.flush(); // Ensure data is written immediately

    message_count_++;

    // Log periodic updates
    if (message_count_ % 100 == 0) {
      auto current_time = this->now();
      auto duration = current_time - start_time_;
      double rate = message_count_ / duration.seconds();

      RCLCPP_INFO(get_logger(), "Recorded %zu messages (%.1f Hz) - Latest: (%.3f, %.3f)",
                  message_count_, rate, x, y);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr recording_timer_;
  std::ofstream csv_file_;
  std::string filename_;
  size_t message_count_;
  rclcpp::Time start_time_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  std::mutex odometry_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryRecorder>();

  RCLCPP_INFO(node->get_logger(), "Starting odometry recording. Press Ctrl+C to stop.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
