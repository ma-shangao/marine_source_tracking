#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF() : Node("odom_to_tf")
  {
    std::string odom_topic;
    frame_id_ = this->declare_parameter("frame_id", std::string(""));
    child_frame_id_ = this->declare_parameter("child_frame_id", std::string(""));
    odom_topic = this->declare_parameter("odom_topic", std::string("/odom/perfect"));
    RCLCPP_INFO(this->get_logger(), "odom_topic set to %s", odom_topic.c_str());
    inverse_tf_ = this->declare_parameter("inverse_tf", false);
    use_original_timestamp_ = this->declare_parameter("use_original_timestamp", false);

    if (frame_id_ != "")
    {
      RCLCPP_INFO(this->get_logger(), "frame_id set to %s", frame_id_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "frame_id was not set. The frame_id of "
                                      "the odom message will be used.");
    }
    if (child_frame_id_ != "")
    {
      RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id_.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "child_frame_id was not set. The child_frame_id of the odom "
                                      "message will be used.");
    }
    // Detect the odom_topic topic type
    std::map<std::string, std::vector<std::string> > topic_types = this->get_topic_names_and_types();
    RCLCPP_INFO(this->get_logger(), "Obtained topic types");
    // Check if the odom_topic is in the topic_types map with a timeout of 5 seconds
    if (topic_types.find(odom_topic) == topic_types.end()){
      std::this_thread::sleep_for(std::chrono::seconds(5));
      topic_types = get_topic_names_and_types();
    }
    if (topic_types.find(odom_topic) == topic_types.end())
    {
      RCLCPP_ERROR(this->get_logger(), "The odom_topic %s was not found in the topic types map"
      " Check if the topic name starts with a / and if the topic is being published",
      odom_topic.c_str());
      rclcpp::shutdown();
      return;
    }

    if (topic_types[odom_topic][0] == "nav_msgs/msg/Odometry")
    {
      RCLCPP_INFO(this->get_logger(), "Detected topic type nav_msgs/msg/Odometry");
      sub_odom_ = 
        this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 
        rclcpp::SensorDataQoS(),
        std::bind(&OdomToTF::odomCallback, this, _1));
    }
    else if (topic_types[odom_topic][0] == "geometry_msgs/msg/PoseStamped")
    {
      RCLCPP_INFO(this->get_logger(), "Detected topic type geometry_msgs/msg/PoseStamped");
      sub_pose_ = 
        this->create_subscription<geometry_msgs::msg::PoseStamped>(odom_topic, 
        rclcpp::SensorDataQoS(),
        std::bind(&OdomToTF::poseStampedCallback, this, _1));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Detected topic type %s", topic_types[odom_topic][0].c_str());
      RCLCPP_ERROR(this->get_logger(), "The odom_topic must be of type nav_msgs/msg/Odometry or "
                                       "geometry_msgs/msg/PoseStamped");
      rclcpp::shutdown();
      return;
    }



    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  std::string frame_id_, child_frame_id_;
  bool inverse_tf_, use_original_timestamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    geometry_msgs::msg::TransformStamped tfs_;
    if (not use_original_timestamp_)
    {
      tfs_.header.stamp = msg->header.stamp;
    }
    else
    {
      tfs_.header.stamp = this->now();
    }
    if (not inverse_tf_)
    {
      tfs_.header.frame_id = frame_id_ != "" ? frame_id_ : msg->header.frame_id;
      tfs_.child_frame_id = child_frame_id_ != "" ? child_frame_id_ : msg->child_frame_id;
      tfs_.transform.translation.x = msg->pose.pose.position.x;
      tfs_.transform.translation.y = msg->pose.pose.position.y;
      tfs_.transform.translation.z = msg->pose.pose.position.z;

      tfs_.transform.rotation = msg->pose.pose.orientation;
    }
    else
    {
      tfs_.header.frame_id = frame_id_ != "" ? frame_id_ : msg->child_frame_id;
      tfs_.child_frame_id = child_frame_id_ != "" ? child_frame_id_ : msg->header.frame_id;
      tf2::Vector3 trans;
      tf2::Quaternion rot_q;
      tf2::fromMsg(msg->pose.pose.position, trans);
      tf2::fromMsg(msg->pose.pose.orientation, rot_q);
      tf2::Transform tf2_tf = tf2::Transform(rot_q, trans);
      tfs_.transform = tf2::toMsg(tf2_tf.inverse());
    }
    tfb_->sendTransform(tfs_);
  }
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    geometry_msgs::msg::TransformStamped tfs_;
    if (not use_original_timestamp_)
    {
      tfs_.header.stamp = msg->header.stamp;
    }
    else
    {
      tfs_.header.stamp = this->now();
    }
    if (frame_id_ == "" and inverse_tf_ == false)
    {
      RCLCPP_ERROR(this->get_logger(), "frame_id was not set. The frame_id is mandatory when using"
                                        "geometry_msgs::msg::Pose");
      rclcpp::shutdown();
      return;
    }
    if (child_frame_id_ == "" and inverse_tf_ == true)
    {
      RCLCPP_ERROR(this->get_logger(), "child_frame_id was not set. The child_frame_id is mandatory when using"
                                        "geometry_msgs::msg::Pose and inverse_tf_ is set to true");
      rclcpp::shutdown();
      return;
    }

    if (not inverse_tf_)
    {
      tfs_.header.frame_id = frame_id_;
      tfs_.child_frame_id = child_frame_id_ != "" ? child_frame_id_ : msg->header.frame_id;
      tfs_.transform.translation.x = msg->pose.position.x;
      tfs_.transform.translation.y = msg->pose.position.y;
      tfs_.transform.translation.z = msg->pose.position.z;

      tfs_.transform.rotation = msg->pose.orientation;
    }
    else
    {
      tfs_.header.frame_id = frame_id_ != "" ? frame_id_ : msg->header.frame_id;
      tfs_.child_frame_id = child_frame_id_;
      tf2::Vector3 trans;
      tf2::Quaternion rot_q;
      tf2::fromMsg(msg->pose.position, trans);
      tf2::fromMsg(msg->pose.orientation, rot_q);
      tf2::Transform tf2_tf = tf2::Transform(rot_q, trans);
      tfs_.transform = tf2::toMsg(tf2_tf.inverse());
    }
    tfb_->sendTransform(tfs_);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTF>());
  rclcpp::shutdown();
  return 0;
}
