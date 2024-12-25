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
  // Logic for finding the legs of the shelf
  // ...
  return false; // Placeholder
}

void ApproachServiceServer::finding_center_position() {
  // Logic for finding the center position
  // ...
}

void ApproachServiceServer::adding_fixed_cartframe() {
  // Logic for adding the fixed cart frame
  // ...
}

void ApproachServiceServer::move_to_cart_center() {
  // Logic for moving to the cart center
  // ...
}

void ApproachServiceServer::rotating_center_cart() {
  // Logic for rotating to the cart center
  // ...
}

void ApproachServiceServer::moving_under_cart() {
  // Logic for moving under the cart
  // ...
}
