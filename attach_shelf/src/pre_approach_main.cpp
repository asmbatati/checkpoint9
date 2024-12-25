#include "attach_shelf/pre_approach.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto preApproach = std::make_shared<PreApproachNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(preApproach);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
