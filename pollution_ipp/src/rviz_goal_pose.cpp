// Copyright 2025 author
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "ardupilot_msgs/msg/global_position.hpp"

class RvizGoalPose : public rclcpp::Node
{
public:
  RvizGoalPose()
  : Node("rviz_goal_pose")
  {
    declare_parameter("origin_lat", 51.566151);
    declare_parameter("origin_lon", -4.034345);
    local_cart = GeographicLib::LocalCartesian(
        get_parameter("origin_lat").as_double(),
        get_parameter("origin_lon").as_double(),
        0.0
    );
    goal_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        rclcpp::QoS(10),
        std::bind(&RvizGoalPose::goalPoseCallback, this, std::placeholders::_1)
    );
    goal_wp_pub = create_publisher<ardupilot_msgs::msg::GlobalPosition>(
        "ap/cmd_gps_pose",
        rclcpp::QoS(10)
    );
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr goal_wp_pub;
  GeographicLib::LocalCartesian local_cart;
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double wp_lat;
    double wp_lon;
    double wp_alt;
    local_cart.Reverse(
        msg->pose.position.x,
        msg->pose.position.y,
        0.0,
        wp_lat,
        wp_lon,
        wp_alt
    );

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double yaw = tf2::getYaw(q);

    ardupilot_msgs::msg::GlobalPosition wp_msg;
    wp_msg.header.frame_id = "map";
    wp_msg.latitude = wp_lat;
    wp_msg.longitude = wp_lon;
    wp_msg.altitude = wp_alt;
    wp_msg.coordinate_frame = 5;
      // TODO: Yaw seems to be ignored by ArduPilot
    wp_msg.set__yaw(yaw);

    goal_wp_pub->publish(wp_msg);
    RCLCPP_INFO_STREAM(get_logger(), "localcart: " <<
        msg->pose.position.x << ", " << msg->pose.position.y <<
        " ardupilot waypoint published: " <<
        wp_lat << ", " << wp_lon << ", " << wp_alt);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizGoalPose>());
  rclcpp::shutdown();
  return 0;
}
