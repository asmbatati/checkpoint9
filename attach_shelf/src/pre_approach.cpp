#include "attach_shelf/pre_approach.h"

PreApproachNode::PreApproachNode() : Node("pre_approach_node") {
  // Declare parameters
  this->declare_parameter<float>("obstacle", 0.0);
  this->declare_parameter<int>("degrees", 0);

  navi_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = navi_callback_group_;
  sub_cb_group_ = timer_cb_group_;
  navigation_options.callback_group = sub_cb_group_;

  // Create subscribers and publishers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PreApproachNode::scan_callback, this, std::placeholders::_1),
      navigation_options);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diffbot_base_controller/odom", 10,
      std::bind(&PreApproachNode::odometry_callback, this,
                std::placeholders::_1),
      navigation_options);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Initialize state
  state_ = State::MOVING_FORWARD;
  yaw_ = 0.0;
  rotation_degrees_ = 0;
  obstacle_distance_ = 0.0;

  // Create timer for publishing velocity commands
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproachNode::timer_callback, this), timer_cb_group_);

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
  yaw_ = get_yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void PreApproachNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (state_ == State::MOVING_FORWARD) {
    RCLCPP_INFO(this->get_logger(), "scan_callback, state::Moving_Forward");
    float min_distance = msg->ranges[540]; // Center laser scan value
    if (min_distance <= obstacle_distance_) {
      state_ = State::ROTATING;
    }
  }
}

void PreApproachNode::timer_callback() {
  geometry_msgs::msg::Twist cmd_vel_msg;
  float rotation_radian = static_cast<float>(rotation_degrees_ * (M_PI / 180));

  // Get parameter values
  this->get_parameter("obstacle", obstacle_distance_);
  this->get_parameter("degrees", rotation_degrees_);

  switch (state_) {
  case State::MOVING_FORWARD:
    cmd_vel_msg.linear.x = 0.5; // Move forward
    break;

  case State::ROTATING:
    RCLCPP_INFO(this->get_logger(),
                "rotation_degree : %d, rotation_radian : %f, yaw:%f",
                rotation_degrees_, rotation_radian, yaw_);
    cmd_vel_msg.angular.z = rotation_radian * 0.2; // Rotate
    if (std::fabs(rotation_radian - yaw_) <= 0.03)
      state_ = State::STOPPED;
    break;

  case State::STOPPED:
    RCLCPP_INFO(this->get_logger(), "Stopped");
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_msg.linear.x = 0.0;
    break;
  }

  cmd_vel_pub_->publish(cmd_vel_msg);
}
