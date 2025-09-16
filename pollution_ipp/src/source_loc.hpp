// Copyright 2025 author
#ifndef SOURCE_LOC_HPP
#define SOURCE_LOC_HPP

#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.h"
#include "GeographicLib/LocalCartesian.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "pollution_interfaces/msg/probe.hpp"
#include "pollution_interfaces/action/source_localization.hpp"
#include "ardupilot_msgs/msg/global_position.hpp"

#include "igl/cumsum.h"

using SourceLocalization = pollution_interfaces::action::SourceLocalization;
using GoalHandleSourceLoc = rclcpp_action::ServerGoalHandle<SourceLocalization>;

class SourceLoc : public rclcpp::Node
{
public:
  SourceLoc();
  ~SourceLoc() override;
  /// @brief Get the probability map
  const grid_map::GridMap & getProbMap() const {return prob_map;}
  /// @brief Get the last hit position
  const grid_map::Position & getLastHitPosition() const {return last_hit_position;}

private:
  // Action server
  rclcpp_action::Server<SourceLocalization>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleSourceLoc> current_goal_handle_;
  double target_confidence_threshold_;
  bool localization_active_;

  // Threading
  std::thread worker_thread_;
  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  // std::queue<pollution_interfaces::msg::Probe::SharedPtr> probe_queue_;
  pollution_interfaces::msg::Probe::SharedPtr last_probe_msg_ = nullptr;
  bool worker_running_;

  /// @brief Confidence interval gamma for source localization
  double ci_gamma;

  // @brief Probability map for the source
  grid_map::GridMap prob_map;
  rclcpp::Subscription<pollution_interfaces::msg::Probe>::SharedPtr probe_sub;
  // @brief Publisher for the source probability map
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr prob_map_pub;
  // @brief Publisher for selected waypoint
  rclcpp::Publisher<ardupilot_msgs::msg::GlobalPosition>::SharedPtr selected_waypoint_pub;
  // @brief Publisher for rviz
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rviz_waypoint_pub;
  // @brief Surface flow direction in radians
  double reliable_surface_flow_direction; // adjust according to openfoam U field
  // @brief Position of the last probe hit
  grid_map::Position last_hit_position;
  // @brief GeographicLib LocalCartesian object for coordinate transformations
  GeographicLib::LocalCartesian local_cart;
  // @brief Callback function for the probe subscriber
  void probeCallback(const pollution_interfaces::msg::Probe::SharedPtr msg);

  /* @brief Select and send the candidate waypoints.
  *  @param prob_index The index of the probe data in the probability map.
  */
  void selectPubCandidateWaypoints(const grid_map::Index & prob_index);

  /* @brief Calculate the information gain for a candidate waypoint.
  *  @param candidate_index The index of the candidate waypoint in the probability grid map.
  *  @return The information gain for the candidate waypoint.
  */
  double calculateInfoGain(
    const grid_map::Index & candidate_index,
    const grid_map::GridMap & grid_map
  );

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SourceLocalization::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSourceLoc> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleSourceLoc> goal_handle);

  // Worker thread function
  void workerThreadFunction();
  void processProbeData(const pollution_interfaces::msg::Probe::SharedPtr msg);

  // Helper methods for action server
  void publish_feedback();
  void check_completion();

    /* @brief Update the probability map based on the probe data.
  *  @param prob_map The probability map to be updated.
  *  @param is_hit Whether the measurement is a hit or not.
  *  @param probe_position The position of the probe in the grid map.
  *  @param last_hit_position The position of the last hit in the grid map.
    */
  void updateProbMap(
    grid_map::GridMap & prob_map,
    const bool is_hit,
    const grid_map::Position & probe_position,
    const grid_map::Position & last_hit_position
  );

  /// @brief Calculate the expectation of the source location based on the probability map.
  /// @return The expected source location as a grid_map::Position.
  grid_map::Position sourceLocExpectation();

  /// @brief Calculate the sum of the variances of the source location.
  /// @return The sum of the variances of the source location.
  double sourceLocVars();

  /// @brief Calculate the Equal-Tailed Credible Interval of the source location.
  /// @param gamma The confidence level for the credible interval (default is 0.95).
  /// @return
  std::vector<double> sourceLocETI(double gamma = 0.95);
  /// @brief Calculate the Smallest Credible Interval of the source location.
  /// @param gamma The confidence level for the credible interval (default is 0.95).
  /// @return
  std::vector<double> sourceLocSCI(double gamma = 0.95);

  /* @brief Calculate the probability of a Gaussian distribution.
  *  @param distance The distance from the center of the Gaussian.
  *  @param sigma The standard deviation of the Gaussian.
  *  @return The probability of the Gaussian distribution.
  */
  double evaluate1DGaussian(const double distance, const double sigma);

  /* @brief Calculate the KL divergence between two probability distributions.
  *  @param p The first probability distribution.
  *  @param q The second probability distribution.
  * @return The KL divergence between the two probability distributions.
  */
  double kLDivergence(
    const Eigen::ArrayXXd & p,
    const Eigen::ArrayXXd & q
  );

  /// @brief Calculate the KL divergence assuming mutually independent mutltivariate Bernoulli
  /// @param p 
  /// @param q 
  /// @return 
  double kLDivergenceBaseline(
    const Eigen::ArrayXXd & p,
    const Eigen::ArrayXXd & q
  );
};

#endif // SOURCE_LOC_HPP
