#ifndef PRE_APPROACH_HPP
#define PRE_APPROACH_HPP

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

enum class MotionState {
  Premotion,
  MovingForward,
  Rotating,
  FinalApproach,
  Finished
};

class PreApproach : public rclcpp::Node {
public:
  PreApproach();

private:
  static constexpr char kNodeName[]{"pre_approach_v2_node"};
  static constexpr char kScanTopicName[]{"/scan"};
  static constexpr char kVelCmdTopicName[]{"/diffbot_base_controller/cmd_vel_unstamped"};
  static constexpr char kOdomTopicName[]{"odom"};

  static constexpr char kObstacleParamName[]{"obstacle"};
  static constexpr char kDegreesParamName[]{"degrees"};

  static constexpr auto kTimerPeriod{100ms};

  static constexpr double kForwardVel{0.2}; // [m/s]
  static constexpr double kAngularVel{0.5}; // [rad/s]

  static constexpr double kPi{3.1416};
  static constexpr double kDegreesToRad{kPi / 180};

  constexpr static char kServiceName[]{"approach_shelf"};
  static constexpr auto kServiceTimeout{1s};
  static constexpr auto kFutureTimeout{30s};

  void scan_cb(const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg);
  void odom_cb(const std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  bool get_params();
  void declare_params();
  void timer_cb();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_{};

  double dist_to_obstacle_current_{}; // [m]
  double dist_to_obstacle_final_{};   // [m]
  double rotation_final_{};           // [rad]
  double rotation_last_{};            // [rad]
  double rotation_total_{};           // [rad]
  double rotation_current_{0.0};      // [rad]
  MotionState motion_state_{MotionState::Premotion};
  bool received_odom_{false};
  bool received_scan_{false};
  bool final_approach_{};
};

#endif // PRE_APPROACH_HPP
