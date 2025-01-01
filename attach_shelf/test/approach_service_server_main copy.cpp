#include "attach_shelf/approach_service_server.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<ApproachServiceServer>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}