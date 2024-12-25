#ifndef APPROACH_SERVICE_SERVER_HPP
#define APPROACH_SERVICE_SERVER_HPP

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "attach_shelf/srv/go_to_loading.hpp"

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void service_callback(
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
      const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

  bool finding_shelf_legs();
  void finding_center_position();
  void adding_fixed_cartframe();
  void move_to_cart_center();
  void rotating_center_cart();
  void moving_under_cart();

  // CallbackGroup variable
  rclcpp::CallbackGroup::SharedPtr navi_callback_group_;

  // ROS2 publish/subscriber callback variable
  rclcpp::SubscriptionOptions navigation_options;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS2 Service variable
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr server_;

  // tf2 listener variables
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // frame names
  std::string odom_frame_, laser_frame_, cart_frame_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  int left_leg_index_, right_leg_index_;

  float cart_x_;
  float cart_y_;
  float cart_yaw_;
  tf2::Quaternion cart_quat_;

  bool start_service_;
  bool move_extra_distance_;
  bool found_center_position_;
  bool robot_rotating_done_;
  bool find_two_legs_;

  // robot and cart pose
  geometry_msgs::msg::Pose cart_pose_, robot_pose_;
};

#endif // APPROACH_SERVICE_SERVER_HPP
