// Copyright 2025 author
#include <iostream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rapidcsv.h"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pollution_interfaces/msg/probe.hpp"

#define assertm(exp, msg) assert((void(msg), exp))

class pollution_sim_scv : public rclcpp::Node
{
public:
  pollution_sim_scv()
  : Node("pollution_sim_scv")
  {
            // Declare parameters

    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));
            // timer_period is the time step duration in seconds
    this->declare_parameter("timer_period", 1.0);
    this->declare_parameter("time_step_num", 60);
    this->declare_parameter("csv_dir_path", "./sim");
    this->declare_parameter("gt_frame_id", "ground_truth");
    this->declare_parameter("sim_sonde_reading", true);

            // Prepare the gt grid map
    gt_data_time.setFrameId(this->get_parameter("gt_frame_id").as_string());
    gt_data_time.setGeometry(
                grid_map::Length(500, 250),
                5.0
    );
    gt_data_time.setPosition(
                grid_map::Position(0.0, 0.0)
    );

    RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Created grid map" << std::endl <<
                "map length: " << gt_data_time.getLength().x() << std::endl <<
                "map width: " << gt_data_time.getLength().y() << std::endl <<
                "map resolution: " << gt_data_time.getResolution() << std::endl
    );

            // To debug the grid map rviz
    gt_data_time.add("elevation", 0.0);

            // Prepare the data from the CSV file
    for (int i = 1; i < this->get_parameter("time_step_num").as_int(); i++) {
      std::filesystem::path csv_file_path = this->get_parameter("csv_dir_path").as_string();
      csv_file_path /= "output_" + std::to_string(i) + ".csv";

      rapidcsv::Document doc(csv_file_path, rapidcsv::LabelParams(0, -1));
      RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    "Loaded csv file: " << csv_file_path
      );
      grid_map::Matrix data_matrix(gt_data_time.getSize()(0), gt_data_time.getSize()(1));

        int csv_row_num = doc.GetRowCount();
        assertm(csv_row_num == gt_data_time.getSize()(0) * gt_data_time.getSize()(1),
        "The csv file row number is not equal to the grid map size");

      RCLCPP_INFO(
                    this->get_logger(),
                    "Start dumping data"
      );

      for (grid_map::GridMapIterator iterator(gt_data_time); !iterator.isPastEnd(); ++iterator) {
        const int lin_idx = iterator.getLinearIndex();
        // data_matrix(lin_idx) = doc.GetCell<double>("T", lin_idx);
        data_matrix(lin_idx) = doc.GetCell<double>("T", csv_row_num - lin_idx - 1);
      }
      gt_data_time.add("time_" + std::to_string(i), data_matrix);
    }

    start_time_step = get_clock()->now().seconds();
    RCLCPP_INFO_STREAM(
                get_logger(),
                "Simulation start ROS time: " << start_time_step
    );

            // Best effort QoS for sensor data
    rclcpp::QoS qos_best_effort(10);
    qos_best_effort.best_effort();

            // Init the publisher
    RCLCPP_DEBUG(
                this->get_logger(),
                "Creating publishers"
    );
    gt_data_time_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("gt_grid_map", 10);
    probe_pub = this->create_publisher<pollution_interfaces::msg::Probe>("pollution_probe",
      qos_best_effort);

    timer = rclcpp::create_timer(
                this,
                this->get_clock(),
                std::chrono::milliseconds(
                    int64_t(this->get_parameter("timer_period").as_double() *
                          1000.0)
      ),
                std::bind(&pollution_sim_scv::timer_callback, this)
    );
    if (this->get_parameter("sim_sonde_reading").as_bool()) {
      odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/blueboat/odometry",
                    rclcpp::SensorDataQoS(),
                    std::bind(&pollution_sim_scv::odom_callback, this, std::placeholders::_1)
      );
    }
  }

private:
  grid_map::GridMap gt_data_time;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gt_data_time_pub;
  rclcpp::Publisher<pollution_interfaces::msg::Probe>::SharedPtr probe_pub;
  pollution_interfaces::msg::Probe probe_msg;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr timer;
        /* Set the simulation start tiem, ignore the initialisation, as the node constructor doesn't work well with the /clock */
  int start_time_step;
  void timer_callback()
  {
    grid_map_msgs::msg::GridMap::UniquePtr msg;
            // Convert time_0 layer to msg
    msg = grid_map::GridMapRosConverter::toMessage(gt_data_time);
            // Publish the grid map
    RCLCPP_DEBUG(
                this->get_logger(),
                "Publishing grid map"
    );
    gt_data_time_pub->publish(std::move(msg));
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (int(this->now().seconds()) < start_time_step) {
      start_time_step = this->now().seconds();
      RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    "Start time step: " << start_time_step
      );
    }

            // Get the current 2D coordinates
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

            // Convert the coordinates to grid map index
    grid_map::Index index;
    gt_data_time.getIndex(grid_map::Position(x, y), index);
            // Get the current time step
    int time_step = msg->header.stamp.sec - start_time_step;
    RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Current time step: " << time_step
    );
    if (time_step > 0) {
                // Get the value of the grid map at the index, TODO: Consider adding noise
      double value;
      value = gt_data_time.at("time_" + std::to_string(time_step), index);
      // Clip the negative values to zero
      if (value < 0.0) {
        value = 0.0;
      }
      RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    "Current value: " << value <<
                    " at Position: " << x << ", " << y <<
                    std::endl
      );
      probe_msg.header = msg->header;
      probe_msg.position = msg->pose.pose.position;
      probe_msg.concentration = value;
      probe_pub->publish(probe_msg);
      RCLCPP_DEBUG_STREAM(
                    this->get_logger(),
                    "Published probe message"
      );
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pollution_sim_scv>());
  rclcpp::shutdown();
  return 0;
}
