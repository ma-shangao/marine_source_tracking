// Copyright 2025 author
#include "source_loc.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SourceLoc>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
