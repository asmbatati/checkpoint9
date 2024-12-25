#include "attach_shelf/approach_service_server.h"

ApproachServiceServer::ApproachServiceServer()
    : Node("approach_service_server_node") {

  navi_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  navigation_options.callback_group = navi_callback_group_;

  // Create subscribers and publishers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ApproachServiceServer::scan_callback, this,
                std::placeholders::_1),
      navigation_options);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  // Create service
  server_ = this->create_service<attach_shelf::srv::GoToLoading>(
      "approach_shelf",
      std::bind(&ApproachServiceServer::service_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, navi_callback_group_);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  odom_frame_ = "odom";
  laser_frame_ = "robot_front_laser_base_link";
  cart_frame_ = "cart_frame";

  left_leg_index_ = 0;
  right_leg_index_ = 0;

  cart_x_ = 0.0;
  cart_y_ = 0.0;
  cart_yaw_ = 0.0;

  move_extra_distance_ = false;
  found_center_position_ = false;
  robot_rotating_done_ = false;
  find_two_legs_ = false;
  start_service_ = false;
}

void ApproachServiceServer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_scan_ = msg;

  if (finding_shelf_legs()) {
    adding_fixed_cartframe();

    if (!found_center_position_) {
      finding_center_position();
    }
    if (found_center_position_ && !move_extra_distance_) {
      move_to_cart_center();
    }
    if (move_extra_distance_ && !robot_rotating_done_) {
      rotating_center_cart();
    }
    if (robot_rotating_done_) {
      moving_under_cart();
    }
  }
}

void ApproachServiceServer::service_callback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {

  if (request->attach_to_shelf) {
    if (find_two_legs_) {
      response->complete = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to find 2 legs");
      response->complete = false;
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Failed to get request value from client");
    response->complete = false;
  }
}

bool ApproachServiceServer::finding_shelf_legs() {
  // Logic for finding the legs of the shelf based on laser intensity values
  std::vector<size_t> intensity_indices;

  for (size_t i = 0; i < last_scan_->intensities.size(); i++) {
    if (last_scan_->intensities[i] >= 7000.0) {
      intensity_indices.push_back(i);
    }
  }

  if (intensity_indices.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to detect any shelf legs.");
    return false;
  }

  std::vector<std::vector<size_t>> grouped_indices;
  std::vector<size_t> current_group;
  current_group.push_back(intensity_indices[0]);

  for (size_t i = 1; i < intensity_indices.size(); i++) {
    if (intensity_indices[i] - intensity_indices[i - 1] < 5) {
      current_group.push_back(intensity_indices[i]);
    } else {
      grouped_indices.push_back(current_group);
      current_group.clear();
      current_group.push_back(intensity_indices[i]);
    }
  }
  if (!current_group.empty()) {
    grouped_indices.push_back(current_group);
  }

  if (grouped_indices.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "Detected fewer than two shelf legs.");
    return false;
  }

  left_leg_index_ = grouped_indices[0][0];
  right_leg_index_ = grouped_indices[1][0];
  find_two_legs_ = true;

  RCLCPP_INFO(this->get_logger(), "Successfully detected two shelf legs.");
  return true;
}

void ApproachServiceServer::finding_center_position() {
  // Calculate the center position between the detected legs
  double left_distance = last_scan_->ranges[left_leg_index_];
  double right_distance = last_scan_->ranges[right_leg_index_];

  double avg_angle = last_scan_->angle_min +
                     (left_leg_index_ + right_leg_index_) / 2.0 *
                         last_scan_->angle_increment;

  cart_x_ = (left_distance + right_distance) / 2.0 * cos(avg_angle);
  cart_y_ = (left_distance + right_distance) / 2.0 * sin(avg_angle);
  cart_quat_.setRPY(0.0, 0.0, avg_angle);

  found_center_position_ = true;
  RCLCPP_INFO(this->get_logger(),
              "Center position: x=%.2f, y=%.2f, angle=%.2f degrees",
              cart_x_, cart_y_, avg_angle * 180 / M_PI);
}

void ApproachServiceServer::adding_fixed_cartframe() {
  // Broadcast a transform for the cart's center frame
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = laser_frame_;
  transform.child_frame_id = cart_frame_;
  transform.transform.translation.x = cart_x_;
  transform.transform.translation.y = cart_y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = cart_quat_.x();
  transform.transform.rotation.y = cart_quat_.y();
  transform.transform.rotation.z = cart_quat_.z();
  transform.transform.rotation.w = cart_quat_.w();

  tf_broadcaster_->sendTransform(transform);
  RCLCPP_INFO(this->get_logger(), "Broadcasted cart frame transform.");
}

void ApproachServiceServer::move_to_cart_center() {
  // Command the robot to move towards the cart center
  geometry_msgs::msg::Twist cmd_vel_msg;
  double distance_to_cart = sqrt(pow(cart_x_, 2) + pow(cart_y_, 2));

  if (distance_to_cart > 0.4) {
    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = atan2(cart_y_, cart_x_) * 0.1;
    cmd_vel_pub_->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "Moving towards cart center...");
  } else {
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);
    move_extra_distance_ = true;
    RCLCPP_INFO(this->get_logger(), "Reached cart center.");
  }
}

void ApproachServiceServer::rotating_center_cart() {
  // Rotate the robot to align with the cart center
  geometry_msgs::msg::Twist cmd_vel_msg;
  if (fabs(cart_yaw_) > 0.05) {
    cmd_vel_msg.angular.z = (cart_yaw_ > 0 ? -0.1 : 0.1);
    cmd_vel_pub_->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "Rotating to align with cart...");
  } else {
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);
    robot_rotating_done_ = true;
    RCLCPP_INFO(this->get_logger(), "Aligned with cart.");
  }
}

void ApproachServiceServer::moving_under_cart() {
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0.1;

  for (int i = 0; i < 30; i++) {
    cmd_vel_pub_->publish(cmd_vel_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Use std::this_thread::sleep_for
  }

  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd_vel_msg);
  RCLCPP_INFO(this->get_logger(), "Moved under cart and stopped.");
}
