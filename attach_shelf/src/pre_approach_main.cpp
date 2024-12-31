#include "attach_shelf/pre_approach.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto preApproach = std::make_shared<PreApproachNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(preApproach);
  executor.spin(); // The spinning stops when rclcpp::shutdown() is called.

  RCLCPP_INFO(preApproach->get_logger(), "Node has been shut down cleanly.");
  return 0;
}