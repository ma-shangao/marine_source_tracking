// Copyright 2025 author
#include "source_loc.hpp"

SourceLoc::SourceLoc()
: Node("source_loc"), localization_active_(false), target_confidence_threshold_(0.8)
{
  RCLCPP_INFO(get_logger(), "Initializing SourceLoc node");

  declare_parameter("odom_frame_id", "odom");
  declare_parameter("prob_map_res", 5.0);
  declare_parameter("hit_threshold", 0.1);
  declare_parameter("origin_lat", 51.566151);
  declare_parameter("origin_lon", -4.034345);

  declare_parameter("viz_waypoint", true);
  declare_parameter("move_base_mode", "ardupilot");

  declare_parameter("ci_gamma", 0.99);

  declare_parameter("surface_flow_direction", 0.785398); // 45 degrees in radians

  declare_parameter("start_position_x", 125.0);
  declare_parameter("start_position_y", 125.0);

  // Initialize the action server
  action_server_ = rclcpp_action::create_server<SourceLocalization>(
    this,
    "source_localization",
    std::bind(&SourceLoc::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SourceLoc::handle_cancel, this, std::placeholders::_1),
    std::bind(&SourceLoc::handle_accepted, this, std::placeholders::_1)
  );

    // Initialize the probability grid map
  prob_map.setFrameId(this->get_parameter("odom_frame_id").as_string());
  prob_map.setGeometry(
      grid_map::Length(500, 250),
      this->get_parameter("prob_map_res").as_double()
  );
  prob_map.setPosition(
      grid_map::Position(0.0, 0.0)
  );
    // prob_map.add("probability", 0.0);
    // add elevation layer to avoid rviz error
  prob_map.add("elevation", 0.0);
  prob_map.add("aux_weight", 0.0);
  prob_map.add("source_prob", 1.0);
  prob_map.add("info_gain");

    // Prepare the latlon <-> local Cartesian conversion
  local_cart = GeographicLib::LocalCartesian(
      this->get_parameter("origin_lat").as_double(),
      this->get_parameter("origin_lon").as_double(),
      0.0
  );

  reliable_surface_flow_direction = get_parameter("surface_flow_direction").as_double();

    // Initialize the subscriber
  probe_sub = this->create_subscription<pollution_interfaces::msg::Probe>(
      "pollution_probe",
      rclcpp::SensorDataQoS(),
      std::bind(&SourceLoc::probeCallback, this, std::placeholders::_1)
  );
    // Initialize the publisher
  prob_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "prob_map",
      10
  );

  if (get_parameter("viz_waypoint").as_bool()) {
    rviz_waypoint_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "rviz/waypoint",
      10
    );
  } else {
    rviz_waypoint_pub = nullptr;
  }
  if (get_parameter("move_base_mode").as_string() == "ardupilot") {
    selected_waypoint_pub = this->create_publisher<ardupilot_msgs::msg::GlobalPosition>(
        "ap/cmd_gps_pose",
        10
    );
  } else {
    selected_waypoint_pub = nullptr;
  }

  ci_gamma = get_parameter("ci_gamma").as_double();

  last_hit_position.x() = this->get_parameter("start_position_x").as_double();
  last_hit_position.y() = this->get_parameter("start_position_y").as_double();
}

SourceLoc::~SourceLoc()
{
  worker_running_ = false;
  data_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void SourceLoc::probeCallback(const pollution_interfaces::msg::Probe::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received probe data");

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_probe_msg_ = msg;
  }
  data_cv_.notify_one();

//   bool is_hit = false;
//   if (msg->concentration >= this->get_parameter("hit_threshold").as_double()) {
//     is_hit = true;
//     RCLCPP_INFO_STREAM(this->get_logger(),
//       "Probe hit detected at position: " << msg->position.x << ", " << msg->position.y);
//     last_hit_position.x() = msg->position.x;
//     last_hit_position.y() = msg->position.y;
//   }
//   double probe_x = msg->position.x;
//   double probe_y = msg->position.y;

//   grid_map::Index prob_index;
//   prob_map.getIndex(grid_map::Position(probe_x, probe_y), prob_index);

//   RCLCPP_INFO_STREAM(get_logger(), "Probe at: " << probe_x << ", " << probe_y
//                                                 << " hit: " << is_hit <<
//     " recorded last hit position: " << last_hit_position.x() << ", " << last_hit_position.y());
//   updateProbMap(
//     prob_map,
//     is_hit,
//     grid_map::Position(probe_x, probe_y),
//     last_hit_position
//   );

//   selectPubCandidateWaypoints(prob_index);

//   prob_map_pub->publish(grid_map::GridMapRosConverter::toMessage(prob_map));
//   RCLCPP_INFO(this->get_logger(), "Published probability map");

//   // Handle action server feedback and completion checking
//   if (localization_active_) {
//     publish_feedback();
//     check_completion();
//   }
}

void SourceLoc::selectPubCandidateWaypoints(const grid_map::Index & prob_index)
{
  // Configure the loop limits of the candidate waypoints
  int expansion_size = 5;
  grid_map::Index start_index;
  start_index(0) = std::max(prob_index(0) - expansion_size, 0);
  start_index(1) = std::max(prob_index(1) - expansion_size, 0);
  grid_map::Size submap_size;
  submap_size(0) = std::min(prob_map.getSize()(0) - start_index(0), 2 * expansion_size + 1);
  submap_size(1) = std::min(prob_map.getSize()(1) - start_index(1), 2 * expansion_size + 1);

  grid_map::Index best_index;
  double best_info_gain = -1.0;

  prob_map.clear("info_gain");

  // Collect all indices in the submap
  std::vector<grid_map::Index> indices;
  for (grid_map::SubmapIterator iter(prob_map, start_index, submap_size); !iter.isPastEnd();
    ++iter)
  {
    indices.push_back(*iter);
  }
  #pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < indices.size(); ++i) {
    const grid_map::Index & idx = indices[i];
    // Calculate the information gain
    double info_gain = calculateInfoGain(idx, prob_map);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Info gain at index " << idx << ": " << info_gain);
    // Check if the information gain is better than the best found so far
    if (info_gain > best_info_gain) {
      best_info_gain = info_gain;
      best_index = idx;
      RCLCPP_DEBUG_STREAM(this->get_logger(),
        "New best index: " << best_index << " with info gain: " << best_info_gain);
    }
    prob_map.at("info_gain", idx) = info_gain;
  }

  // Convert the best index to latlon

  grid_map::Position wp_position;
  assert(
    prob_map.getPosition(
      best_index,
      wp_position
    )
  );
  double wp_lat;
  double wp_lon;
  double wp_alt;
  local_cart.Reverse(
    wp_position.x(),
    wp_position.y(),
    0.0,
    wp_lat,
    wp_lon,
    wp_alt
  );

  if (get_parameter("viz_waypoint").as_bool()) {
    geometry_msgs::msg::PointStamped rviz_wp_msg;
    rviz_wp_msg.header.frame_id = prob_map.getFrameId();
    rviz_wp_msg.point.x = wp_position.x();
    rviz_wp_msg.point.y = wp_position.y();
    rviz_wp_msg.point.z = 0.0;
    rviz_waypoint_pub->publish(rviz_wp_msg);
  }

  if (get_parameter("move_base_mode").as_string() == "ardupilot") {
    ardupilot_msgs::msg::GlobalPosition wp_msg;
    wp_msg.header.frame_id = "map";
    wp_msg.latitude = wp_lat;
    wp_msg.longitude = wp_lon;
    wp_msg.altitude = wp_alt;
    wp_msg.coordinate_frame = 5;

    selected_waypoint_pub->publish(wp_msg);
    RCLCPP_INFO_STREAM(get_logger(), "localcart: "  <<
      wp_position.x() << ", " << wp_position.y()
                                                    << " ardupilot waypoint published: "
                                                    << wp_lat << ", " << wp_lon << ", " << wp_alt);
    RCLCPP_DEBUG_STREAM(get_logger(), "Best waypoint index: " << best_index);
  }

}

double SourceLoc::calculateInfoGain(
  const grid_map::Index & candidate_index,
  const grid_map::GridMap & grid_map
)
{
  // Calculate the information gain for the candidate waypoint
  // double info_gain = 0.0;
  grid_map::Position candidate_position;
  assert(
    grid_map.getPosition(candidate_index, candidate_position) == true
  );

  // Make copies of the grid map for hit and miss scenarios
  grid_map::GridMap grid_map_hit = grid_map;
  grid_map::GridMap grid_map_miss = grid_map;

  // Simulate a hit
  updateProbMap(
    grid_map_hit,
    true,
    candidate_position,
    last_hit_position
  );

  // Calculate the information gain via KL divergence
  double info_gain_hit = kLDivergence(
    grid_map_hit.get("source_prob").array().cast<double>(),
    grid_map.get("source_prob").array().cast<double>()
  );
  // double info_gain_hit = kLDivergenceBaseline(
  //   grid_map_hit.get("source_prob").array().cast<double>(),
  //   grid_map.get("source_prob").array().cast<double>()
  // );

  // Simulate a miss
  updateProbMap(
    grid_map_miss,
    false,
    candidate_position,
    last_hit_position
  );
  // Calculate the information gain via KL divergence
  double info_gain_miss = kLDivergence(
    grid_map_miss.get("source_prob").array().cast<double>(),
    grid_map.get("source_prob").array().cast<double>()
  );
  // double info_gain_miss = kLDivergenceBaseline(
  //   grid_map_miss.get("source_prob").array().cast<double>(),
  //   grid_map.get("source_prob").array().cast<double>()
  // );

  double info_gain = grid_map.at("source_prob", candidate_index) *
    info_gain_hit +
    (1.0 - grid_map.at("source_prob", candidate_index)) * info_gain_miss;

  return info_gain;
}

void SourceLoc::updateProbMap(
  grid_map::GridMap & prob_map,
  const bool is_hit,
  const grid_map::Position & probe_position,
  const grid_map::Position & last_hit_position
)
{
  double ideal_direction;
  if (is_hit) {
    ideal_direction = reliable_surface_flow_direction + M_PI;
  } else {
    ideal_direction = std::atan2(
      last_hit_position.y() - probe_position.y(),
      last_hit_position.x() - probe_position.x());
  }
  grid_map::Index prob_index;
  prob_map.getIndex(probe_position, prob_index);
  grid_map::Index neighbour_start_index;

  neighbour_start_index(0) = std::max(prob_index(0) - 1, 0);
  neighbour_start_index(1) = std::max(prob_index(1) - 1, 0);

  grid_map::Size neighbour_size;
  neighbour_size(0) = std::min(prob_map.getSize()(0) - neighbour_start_index(0), 3);
  neighbour_size(1) = std::min(prob_map.getSize()(1) - neighbour_start_index(1), 3);

  std::unordered_set<size_t> open_set;
  std::unordered_set<size_t> closed_set;
  std::unordered_set<size_t> active_set;

  prob_map["aux_weight"].setZero();

  for (grid_map::SubmapIterator iter(prob_map, neighbour_start_index, neighbour_size);
    !iter.isPastEnd(); ++iter)
  {
    double aux_weight;
    if ((*iter == prob_index).all()) {
      aux_weight = evaluate1DGaussian(
        (is_hit ? 0.0 : M_PI),
        (is_hit ? 1.0 : 2.0)
      );
      closed_set.insert(grid_map::getLinearIndexFromIndex(*iter, prob_map.getSize()));
    } else {
      grid_map::Position grid_position;
      prob_map.getPosition(*iter, grid_position);

      double angle_grid_to_robot = std::atan2(
        grid_position.y() - probe_position.y(),
        grid_position.x() - probe_position.x()
      );
      double angle_diff = std::atan2(
        std::sin(ideal_direction - angle_grid_to_robot),
        std::cos(ideal_direction - angle_grid_to_robot)
      );
      aux_weight = evaluate1DGaussian(
        angle_diff,
        (is_hit ? 1.0 : 2.0)
      );
      open_set.insert(grid_map::getLinearIndexFromIndex(*iter, prob_map.getSize()));
    }
    prob_map.at("aux_weight", *iter) = aux_weight;
  }

  while(!open_set.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "main loop");
    active_set = open_set;
    closed_set.insert(active_set.begin(), active_set.end());
    open_set.clear();

    for (auto it = active_set.begin(); it != active_set.end(); ++it) {
      closed_set.insert(*it);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Loop: " << *it);
      grid_map::Index current_index = grid_map::getIndexFromLinearIndex(*it, prob_map.getSize());
      neighbour_start_index(0) = std::max(current_index(0) - 1, 0);
      neighbour_start_index(1) = std::max(current_index(1) - 1, 0);
      neighbour_size(0) = std::min(prob_map.getSize()(0) - neighbour_start_index(0), 3);
      neighbour_size(1) = std::min(prob_map.getSize()(1) - neighbour_start_index(1), 3);
      for (grid_map::SubmapIterator iter(prob_map, neighbour_start_index, neighbour_size);
            !iter.isPastEnd(); ++iter)
      {
        size_t linear_index = grid_map::getLinearIndexFromIndex(*iter, prob_map.getSize());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "LINEAR INDEX: " << linear_index);
        if (closed_set.find(linear_index) == closed_set.end() &&
          open_set.find(linear_index) == open_set.end())
        {
          // RCLCPP_INFO(this->get_logger(), "INTO IF");
          prob_map.at("aux_weight", *iter) = prob_map.at("aux_weight", current_index);
          open_set.insert(linear_index);
        } else if (closed_set.find(linear_index) == closed_set.end()) {
          prob_map.at("aux_weight", *iter) = std::max(prob_map.at("aux_weight", current_index),
            prob_map.at("aux_weight", *iter));
          RCLCPP_DEBUG_STREAM(this->get_logger(), "elif update: " << linear_index);
        }
      }
    }
  }

  // Normalize the auxiliary weight layer

  double sum_aux_weight = prob_map.get("aux_weight").sum();
  assert(sum_aux_weight > 0.0);
  prob_map.get("aux_weight") = (prob_map.get("aux_weight").array() /
    sum_aux_weight).matrix();

  // Update the source probability layer based on the auxiliary weight layer
  for (grid_map::GridMapIterator iter(prob_map); !iter.isPastEnd(); ++iter) {
    RCLCPP_DEBUG_STREAM(this->get_logger(),
      "Source probability at index before update: " << *iter << " = " <<
      prob_map.at("source_prob", *iter));
    prob_map.at("source_prob", *iter) *= prob_map.at("aux_weight", *iter);
    RCLCPP_DEBUG_STREAM(this->get_logger(),
      "Source probability at index after update: " << *iter << " = " << prob_map.at("source_prob",
      *iter));
  }

  // Normalize the source probability layer

  double sum_source_prob = prob_map.get("source_prob").sum();
  assert(sum_source_prob > 0.0);
  prob_map.get("source_prob") = (prob_map.get("source_prob").array() /
    sum_source_prob).matrix();
}

grid_map::Position SourceLoc::sourceLocExpectation()
{
  std::vector<grid_map::Index> indices;
  for (grid_map::GridMapIterator iter(prob_map); !iter.isPastEnd(); ++iter) {
    indices.push_back(*iter);
  }
  double x = 0.0;
  double y = 0.0;
  #pragma omp parallel for reduction(+:x, y)
  for (size_t i = 0; i < indices.size(); ++i) {
    const grid_map::Index & idx = indices[i];
    double prob = prob_map.at("source_prob", idx);
    grid_map::Position pos;
    assert(prob_map.getPosition(idx, pos));
    x += pos.x() * prob;
    y += pos.y() * prob;
  }
  grid_map::Position expectation(x, y);
  return expectation;
}

double SourceLoc::sourceLocVars()
{
  grid_map::Position expectation = sourceLocExpectation();
  double var_x = 0.0;
  double var_y = 0.0;
  std::vector<grid_map::Index> indices;
  for (grid_map::GridMapIterator iter(prob_map); !iter.isPastEnd(); ++iter) {
    indices.push_back(*iter);
  }
    #pragma omp parallel for reduction(+:var_x, var_y)
  for (size_t i = 0; i < indices.size(); ++i) {
    const grid_map::Index & idx = indices[i];
    double prob = prob_map.at("source_prob", idx);
    grid_map::Position pos;
    assert(prob_map.getPosition(idx, pos));
    var_x += prob * std::pow(pos.x() - expectation.x(), 2);
    var_y += prob * std::pow(pos.y() - expectation.y(), 2);
  }
  return var_x + var_y;
}

std::vector<double> SourceLoc::sourceLocETI(double gamma)
{
  // Get the PMF in x and y directions

  Eigen::VectorXf pmf_x(prob_map.getSize()(0));
    Eigen::VectorXf pmf_y(prob_map.getSize()(1));
      pmf_x = prob_map.get("source_prob").rowwise().sum();
      pmf_y = prob_map.get("source_prob").colwise().sum();

  // Normalize the PMF
      pmf_x /= pmf_x.sum();
      pmf_y /= pmf_y.sum();

  // Calculate the cumulative distribution functions (CDF)

      Eigen::VectorXf cdf_x;
      Eigen::VectorXf cdf_y;
      igl::cumsum(pmf_x, 1, cdf_x);
  igl::cumsum(pmf_y, 1, cdf_y);
  // Calculate tail probabilities for equal-tailed interval
  double alpha = (1.0 - gamma) / 2.0;  // e.g., 0.025 for 95% CI
  double lower_threshold = alpha;       // e.g., 0.025
  double upper_threshold = 1.0 - alpha; // e.g., 0.975

  // Find indices where CDF crosses the thresholds
  auto findThresholdIndex = [](const Eigen::VectorXf & cdf, double threshold) -> int {
      for (int i = 0; i < cdf.size(); ++i) {
        if (cdf(i) >= threshold) {
          return i;
        }
      }
      return cdf.size() - 1;
    };

  int x_lower_idx = findThresholdIndex(cdf_x, lower_threshold);
  int x_upper_idx = findThresholdIndex(cdf_x, upper_threshold);
  int y_lower_idx = findThresholdIndex(cdf_y, lower_threshold);
  int y_upper_idx = findThresholdIndex(cdf_y, upper_threshold);

  // Convert indices to actual coordinates
  grid_map::Index x_lower_index(x_lower_idx, prob_map.getSize()(1) / 2);
    grid_map::Index x_upper_index(x_upper_idx, prob_map.getSize()(1) / 2);
      grid_map::Index y_lower_index(prob_map.getSize()(0) / 2, y_lower_idx);
        grid_map::Index y_upper_index(prob_map.getSize()(0) / 2, y_upper_idx);

          grid_map::Position x_lower_pos, x_upper_pos, y_lower_pos, y_upper_pos;

          prob_map.getPosition(x_lower_index, x_upper_pos);
  prob_map.getPosition(x_upper_index, x_lower_pos);
  prob_map.getPosition(y_lower_index, y_upper_pos);
  prob_map.getPosition(y_upper_index, y_lower_pos);

  // Return [x_lower, x_upper, y_lower, y_upper]
  return {x_lower_pos.x(), x_upper_pos.x(), y_lower_pos.y(), y_upper_pos.y()};
}

std::vector<double> SourceLoc::sourceLocSCI(double gamma)
{
  // Get the PMF in x and y directions
  Eigen::VectorXf pmf_x(prob_map.getSize()(0));
    Eigen::VectorXf pmf_y(prob_map.getSize()(1));

      pmf_x = prob_map.get("source_prob").rowwise().sum();
      pmf_y = prob_map.get("source_prob").colwise().sum();

  // Normalize the PMF
      pmf_x /= pmf_x.sum();
      pmf_y /= pmf_y.sum();

  // Calculate CDF
      Eigen::VectorXf cdf_x, cdf_y;
      igl::cumsum(pmf_x, 1, cdf_x);
  igl::cumsum(pmf_y, 1, cdf_y);

  // Find shortest interval for X
  auto findShortestInterval = [](const Eigen::VectorXf & cdf, double gamma) -> std::pair<int, int> {
      int n = cdf.size();
      int min_width = std::numeric_limits<int>::max();
      int best_start = 0, best_end = n - 1;

    // Try all possible intervals with probability mass >= gamma
      for (int start = 0; start < n; ++start) {
        for (int end = start; end < n; ++end) {
        // Calculate probability mass in interval [start, end]
          double prob_mass = cdf(end) - (start > 0 ? cdf(start - 1) : 0.0);

          if (prob_mass >= gamma) {
            int width = end - start;
            if (width < min_width) {
              min_width = width;
              best_start = start;
              best_end = end;
            }
            break; // Found shortest interval starting at 'start'
          }
        }
      }
      return {best_start, best_end};
    };

  auto [x_start, x_end] = findShortestInterval(cdf_x, gamma);
  auto [y_start, y_end] = findShortestInterval(cdf_y, gamma);

  // Convert indices to coordinates
  grid_map::Index x_start_index(x_start, prob_map.getSize()(1) / 2);
  grid_map::Index x_end_index(x_end, prob_map.getSize()(1) / 2);
  grid_map::Index y_start_index(prob_map.getSize()(0) / 2, y_start);
  grid_map::Index y_end_index(prob_map.getSize()(0) / 2, y_end);

  grid_map::Position x_start_pos, x_end_pos, y_start_pos, y_end_pos;
  prob_map.getPosition(x_start_index, x_end_pos);
  prob_map.getPosition(x_end_index, x_start_pos);
  prob_map.getPosition(y_start_index, y_end_pos);
  prob_map.getPosition(y_end_index, y_start_pos);

  return {x_start_pos.x(), x_end_pos.x(), y_start_pos.y(), y_end_pos.y()};
}

double SourceLoc::evaluate1DGaussian(const double distance, const double sigma)
{
  return exp(-0.5 * (std::pow(distance, 2) / std::pow(sigma, 2))) / (sigma * std::sqrt(2 * M_PI));
}

double SourceLoc::kLDivergence(
  const Eigen::ArrayXXd & p,
  const Eigen::ArrayXXd & q
)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "p size: " << p.size() << " q size: " << q.size());
  // Ensure that the two distributions have the same size
  assert(p.size() == q.size());

  // Calculate the KL divergence for categorical distributions
  Eigen::ArrayXXd kld = p * (p / q).log();

  RCLCPP_DEBUG(get_logger(), "calculated kld element wise");

  // Switch nan values to 0
  kld = kld.unaryExpr([](double x) {return std::isnan(x) ? 0.0 : x;});
  return kld.sum();
}

double SourceLoc::kLDivergenceBaseline(const Eigen::ArrayXXd &p, const Eigen::ArrayXXd &q)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "p size: " << p.size() << " q size: " << q.size());
  // Ensure that the two distributions have the same size
  assert(p.size() == q.size());
  // Calculate the KL divergence for bernoulli distributions (mutually independant)
  Eigen::ArrayXXd kld = p * (p / q).log() + (1-p) * ((1-p)/(1-q)).log();

  RCLCPP_DEBUG(get_logger(), "calculated kld element wise");

  // Switch nan values to 0
  kld = kld.unaryExpr([](double x) {return std::isnan(x) ? 0.0 : x;});
  return kld.sum();
}

// Action Server Implementation
rclcpp_action::GoalResponse SourceLoc::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SourceLocalization::Goal> goal)
{
  (void)uuid; // Suppress unused parameter warning

  RCLCPP_INFO(get_logger(), "Received source localization goal");
  RCLCPP_INFO(get_logger(), "Confidence threshold: %.2f", goal->confidence_threshold);

  // Validate goal parameters
  if (goal->confidence_threshold < 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid confidence threshold: %.2f", goal->confidence_threshold);
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check if another localization is already active
  if (localization_active_) {
    RCLCPP_WARN(get_logger(), "Source localization already in progress");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SourceLoc::handle_cancel(
  const std::shared_ptr<GoalHandleSourceLoc> goal_handle)
{
  (void)goal_handle; // Suppress unused parameter warning
  RCLCPP_INFO(get_logger(), "Received request to cancel source localization");

  localization_active_ = false;
  current_goal_handle_.reset();

  // Stop the worker thread properly
  worker_running_ = false;
  data_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void SourceLoc::handle_accepted(const std::shared_ptr<GoalHandleSourceLoc> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Source localization goal accepted, starting localization");

  current_goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();
  target_confidence_threshold_ = goal->confidence_threshold;
  localization_active_ = true;
  worker_running_ = true;

  // Initial feedback
  publish_feedback();
  worker_thread_ = std::thread(&SourceLoc::workerThreadFunction, this);
}

// Worker thread function:
void SourceLoc::workerThreadFunction()
{
  RCLCPP_INFO(get_logger(), "Worker thread started for source localization");
  while (worker_running_) {
    pollution_interfaces::msg::Probe::SharedPtr msg;

    // Wait for data
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      data_cv_.wait(lock, [this] {return last_probe_msg_ || !worker_running_;});

      if (!worker_running_) {break;}

      msg = last_probe_msg_;
    }

    // Process the probe data (heavy computation)
    processProbeData(msg);
  }
  RCLCPP_INFO(get_logger(), "Worker thread exiting");
  localization_active_ = false;
  worker_running_ = false;
}

void SourceLoc::processProbeData(const pollution_interfaces::msg::Probe::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Executing source localization goal");

  bool is_hit = false;
  if (msg->concentration >= this->get_parameter("hit_threshold").as_double()) {
    is_hit = true;
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Probe hit detected at position: " << msg->position.x << ", " << msg->position.y);
    last_hit_position.x() = msg->position.x;
    last_hit_position.y() = msg->position.y;
  }
  double probe_x = msg->position.x;
  double probe_y = msg->position.y;

  grid_map::Index prob_index;
  prob_map.getIndex(grid_map::Position(probe_x, probe_y), prob_index);

  RCLCPP_INFO_STREAM(get_logger(), "Probe at: " << probe_x << ", " << probe_y
                                                << " hit: " << is_hit <<
    " recorded last hit position: " << last_hit_position.x() << ", " << last_hit_position.y());

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    updateProbMap(
      prob_map,
      is_hit,
      grid_map::Position(probe_x, probe_y),
      last_hit_position
    );

    selectPubCandidateWaypoints(prob_index);
  }

  prob_map_pub->publish(grid_map::GridMapRosConverter::toMessage(prob_map));
  RCLCPP_INFO(this->get_logger(), "Published probability map");

  // Handle action server feedback and completion checking
  if (localization_active_) {
    publish_feedback();
    check_completion();
  }
}

void SourceLoc::publish_feedback()
{
  if (!current_goal_handle_ || !localization_active_) {
    return;
  }

  auto feedback = std::make_shared<SourceLocalization::Feedback>();

  // Get current best estimate (expectation of probability distribution)
  grid_map::Position best_estimate = sourceLocExpectation();
  feedback->current_best_estimate.x = best_estimate.x();
  feedback->current_best_estimate.y = best_estimate.y();
  feedback->current_best_estimate.z = 0.0;

  std::vector<double> ci;
  ci = sourceLocSCI(ci_gamma);
  RCLCPP_INFO_STREAM(get_logger(), "Source localization SCI: " << ci[0] << ", " << ci[1] << ", "
                                                               << ci[2] << ", " << ci[3]);
  // ci = sourceLocETI();
  // RCLCPP_INFO_STREAM(get_logger(), "Source localization ETI: " << ci[0] << ", " << ci[1] << ", "
  //   << ci[2] << ", " << ci[3]);

  double interval_x = ci[1] - ci[0];
  double interval_y = ci[3] - ci[2];
  feedback->current_confidence = std::max(interval_x, interval_y);
  feedback->x_lower_bound = ci[0];
  feedback->x_upper_bound = ci[1];
  feedback->y_lower_bound = ci[2];
  feedback->y_upper_bound = ci[3];

  current_goal_handle_->publish_feedback(feedback);
}

void SourceLoc::check_completion()
{
  if (!current_goal_handle_ || !localization_active_) {
    return;
  }

  // Check if goal was cancelled
  if (current_goal_handle_->is_canceling()) {
    auto result = std::make_shared<SourceLocalization::Result>();
    result->success = false;
    result->message = "Goal was cancelled";
    grid_map::Position best_estimate = sourceLocExpectation();
    result->estimated_source_location.x = best_estimate.x();
    result->estimated_source_location.y = best_estimate.y();
    result->estimated_source_location.z = 0.0;
    double variance = sourceLocVars();
    result->confidence_level = std::exp(-variance / 1000.0);

    current_goal_handle_->canceled(result);
    localization_active_ = false;
    current_goal_handle_.reset();

    // Stop the worker thread properly
    worker_running_ = false;
    data_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    RCLCPP_INFO(get_logger(), "Source localization goal canceled");
    return;
  }

  // Check if confidence threshold has been reached
  // double variance = sourceLocVars();
  // double current_confidence = variance;

  std::vector<double> ci;
  ci = sourceLocSCI(ci_gamma);
  double interval_x = ci[1] - ci[0];
  double interval_y = ci[3] - ci[2];
  double current_confidence = std::max(interval_x, interval_y);

  if (current_confidence <= target_confidence_threshold_) {
    auto result = std::make_shared<SourceLocalization::Result>();
    result->success = true;
    result->message = "Source localization completed successfully";
    result->confidence_level = current_confidence;
    grid_map::Position best_estimate = sourceLocExpectation();
    result->estimated_source_location.x = best_estimate.x();
    result->estimated_source_location.y = best_estimate.y();
    result->estimated_source_location.z = 0.0;

    current_goal_handle_->succeed(result);
    localization_active_ = false;
    current_goal_handle_.reset();

    // Stop the worker thread properly
    worker_running_ = false;
    data_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }

    RCLCPP_INFO(get_logger(), "Source localization goal succeeded with confidence %.3f",
                current_confidence);
  }
}
