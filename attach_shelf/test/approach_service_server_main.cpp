#include "attach_shelf/approach_service_server.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ApproachServiceServer> approach_service =
      std::make_shared<ApproachServiceServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_service);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
