#include "attach_shelf/pre_approach_v2.h"

PreApproachNode::PreApproachNode() : Node("pre_approach_node_v2") {
  // Declare parameters
  this->declare_parameter<float>("obstacle", 0.0);
  this->declare_parameter<int>("degrees", 0);
  this->declare_parameter<bool>("final_approach", false);

  navi_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  scan_options.callback_group = navi_callback_group_;
  odom_options.callback_group = navi_callback_group_;

  // Create subscribers and publishers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PreApproachNode::scan_callback, this, std::placeholders::_1),
      scan_options);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diffbot_base_controller/odom", 10,
      std::bind(&PreApproachNode::odometry_callback, this,
                std::placeholders::_1),
      odom_options);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Initialize state
  state_ = State::STEP_1;
  yaw_ = 0.0;
  rotation_degrees_ = 0;
  obstacle_distance_ = 0.0;
  final_client_state_ = false;

  // Create timer for publishing velocity commands
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproachNode::timer_callback, this),
      navi_callback_group_);

  srv_client_ =
      this->create_client<attach_shelf::srv::GoToLoading>("approch_shelf");

  RCLCPP_INFO(this->get_logger(), "End of Constructor");
}

double PreApproachNode::get_yaw_from_quaternion(double x, double y, double z, double w) {
  double yaw;
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

void PreApproachNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  yaw_ = get_yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  RCLCPP_INFO(this->get_logger(), "[odometry_callback] yaw : %f ", yaw_);
}

void PreApproachNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  rclcpp::sleep_for(std::chrono::milliseconds(10));

  if (state_ == State::STEP_1) {
    RCLCPP_INFO(this->get_logger(), "scan_callback ,state::Moving_Forward");
    float min_distance = msg->ranges[540];

    RCLCPP_INFO(this->get_logger(), "min_distance : %f ", min_distance);
    if (min_distance <= obstacle_distance_) {
      state_ = State::STEP_2;
    }
  }
}

void PreApproachNode::timer_callback() {
  geometry_msgs::msg::Twist cmd_vel_msg;
  float rotation_radian = static_cast<float>(rotation_degrees_ * (M_PI / 180));

  // Get parameter values
  this->get_parameter("obstacle", obstacle_distance_);
  this->get_parameter("degrees", rotation_degrees_);
  this->get_parameter("final_approach", final_approach_);

  switch (state_) {
  case State::STEP_1:
    RCLCPP_INFO(this->get_logger(), "Starting STEP_1 ");
    cmd_vel_msg.linear.x = 0.5;
    cmd_vel_msg.angular.z = 0.0;
    break;

  case State::STEP_2:
    RCLCPP_INFO(this->get_logger(), "Starting STEP_2 ");
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = rotation_radian * 0.2;
    if (std::fabs(rotation_radian - yaw_) <= 0.03)
      state_ = State::STEP_3;
    break;

  case State::STEP_3:
    RCLCPP_INFO(this->get_logger(), "Starting STEP_3 ");
    if (final_approach_) {
      auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
      request->attach_to_shelf = final_approach_;
      auto result = srv_client_->async_send_request(request);
      final_client_state_ = result.get()->complete;
    } else {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
    }
    break;
  }
  cmd_vel_pub_->publish(cmd_vel_msg);
}
