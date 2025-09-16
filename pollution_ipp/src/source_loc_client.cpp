// Copyright 2025 author
#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pollution_interfaces/action/source_localization.hpp"

using SourceLocalization = pollution_interfaces::action::SourceLocalization;
using GoalHandleSourceLoc = rclcpp_action::ClientGoalHandle<SourceLocalization>;

class SourceLocClient : public rclcpp::Node
{
public:
  SourceLocClient()
  : Node("source_loc_client")
  {
    declare_parameter<double>("ci_threshold", 10.0);
    declare_parameter<std::string>("output_file", "source_loc_estimates.csv");
    declare_parameter<bool>("append_timestamp", true);
    declare_parameter<double>("recording_rate_hz", 1.0);  // Recording frequency in Hz

    client_ = rclcpp_action::create_client<SourceLocalization>(
      this,
      "source_localization"
    );

    // Setup CSV file
    setup_csv_file();

    // Send a test goal after a short delay
    timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SourceLocClient::send_goal, this)
    );
  }

  ~SourceLocClient()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  rclcpp_action::Client<SourceLocalization>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;  // Add timer for execution time

  // CSV recording members
  std::ofstream csv_file_;
  std::string csv_filename_;
  rclcpp::Time last_recording_time_;
  double recording_rate_hz_;
  bool goal_active_;

  void setup_csv_file()
  {
    // Get parameters
    std::string base_filename = get_parameter("output_file").as_string();
    bool append_timestamp = get_parameter("append_timestamp").as_bool();
    recording_rate_hz_ = get_parameter("recording_rate_hz").as_double();

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
        csv_filename_ = base_filename.substr(0, dot_pos) + "_" + ss.str() +
          base_filename.substr(dot_pos);
      } else {
        csv_filename_ = base_filename + "_" + ss.str() + ".csv";
      }
    } else {
      csv_filename_ = base_filename;
    }

    // Create directory if it doesn't exist
    std::filesystem::path file_path(csv_filename_);
    if (file_path.has_parent_path()) {
      std::filesystem::create_directories(file_path.parent_path());
    }

    // Open CSV file and write header
    csv_file_.open(csv_filename_, std::ios::out);
    if (!csv_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Recording estimates to: %s", csv_filename_.c_str());

    // Write CSV header
    csv_file_ << "ros_time_sec,ros_time_nanosec,elapsed_time_sec,confidence,"
              << "best_estimate_x,best_estimate_y,"
              << "x_lower_bound,x_upper_bound,y_lower_bound,y_upper_bound\n";
    csv_file_.flush();

    goal_active_ = false;
  }

  void record_estimate_data(const std::shared_ptr<const SourceLocalization::Feedback> feedback)
  {
    if (!csv_file_.is_open() || !goal_active_) {
      return;
    }

    auto current_time = this->now();

    // Check if enough time has passed since last recording
    if (last_recording_time_.seconds() > 0) {
      auto time_diff = current_time - last_recording_time_;
      double recording_interval_sec = 1.0 / recording_rate_hz_;  // Convert Hz to seconds
      if (time_diff.seconds() < recording_interval_sec) {
        return;  // Not enough time passed
      }
    }

    // Record current time
    last_recording_time_ = current_time;

    // Calculate elapsed time since goal start
    auto elapsed = current_time - start_time_;

    // Write data to CSV with high precision
    csv_file_ << std::fixed << std::setprecision(9)
              << current_time.seconds() << ","
              << current_time.nanoseconds() % 1000000000 << ","
              << std::setprecision(6)
              << elapsed.seconds() << ","
              << std::setprecision(6)
              << feedback->current_confidence << ","
              << std::setprecision(3)
              << feedback->current_best_estimate.x << ","
              << feedback->current_best_estimate.y << ","
              << feedback->x_lower_bound << ","
              << feedback->x_upper_bound << ","
              << feedback->y_lower_bound << ","
              << feedback->y_upper_bound << "\n";
    csv_file_.flush();
  }

  void send_goal()
  {
    timer_->cancel(); // Only send one goal

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = SourceLocalization::Goal();
    goal_msg.confidence_threshold = get_parameter("ci_threshold").as_double();

    // Record start time when sending goal
    start_time_ = this->now();
    last_recording_time_ = rclcpp::Time(0);  // Reset recording timer
    goal_active_ = true;  // Mark goal as active
    RCLCPP_INFO(get_logger(), "Sending goal at time: %.6f", start_time_.seconds());

    auto send_goal_options = rclcpp_action::Client<SourceLocalization>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SourceLocClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&SourceLocClient::feedback_callback, this, std::placeholders::_1,
      std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&SourceLocClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleSourceLoc::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleSourceLoc::SharedPtr,
    const std::shared_ptr<const SourceLocalization::Feedback> feedback)
  {
    // Calculate elapsed time in feedback
    auto current_time = this->now();
    auto elapsed = current_time - start_time_;

    // Record estimate data to CSV at specified intervals
    record_estimate_data(feedback);

    RCLCPP_INFO(get_logger(),
      "Received feedback [%.2fs]: Confidence %.3f, Best estimate: (%.2f, %.2f), "
      "X bounds: [%.2f, %.2f], Y bounds: [%.2f, %.2f]",
      elapsed.seconds(),
      feedback->current_confidence,
      feedback->current_best_estimate.x,
      feedback->current_best_estimate.y,
      feedback->x_lower_bound,
      feedback->x_upper_bound,
      feedback->y_lower_bound,
      feedback->y_upper_bound
    );
  }

  void result_callback(const GoalHandleSourceLoc::WrappedResult & result)
  {
    // Mark goal as inactive
    goal_active_ = false;

    // Calculate total execution time
    auto end_time = this->now();
    auto total_duration = end_time - start_time_;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }

    RCLCPP_INFO(get_logger(),
      "Result: Success=%s, Message='%s', Confidence=%.3f, Location=(%.2f, %.2f, %.2f)",
      result.result->success ? "true" : "false",
      result.result->message.c_str(),
      result.result->confidence_level,
      result.result->estimated_source_location.x,
      result.result->estimated_source_location.y,
      result.result->estimated_source_location.z
    );

    // Log the total execution time
    RCLCPP_INFO(get_logger(),
      "Total execution time: %.6f seconds (%.2f minutes)",
      total_duration.seconds(),
      total_duration.seconds() / 60.0
    );

    // Close CSV file
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(get_logger(), "CSV recording completed: %s", csv_filename_.c_str());
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SourceLocClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
