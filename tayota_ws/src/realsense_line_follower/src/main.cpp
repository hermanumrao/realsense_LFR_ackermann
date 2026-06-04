#include "realsense_line_follower/line_follower.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LineFollower>());

  rclcpp::shutdown();

  return 0;
}
