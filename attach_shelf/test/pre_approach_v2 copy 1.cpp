#include "attach_shelf/pre_approach_v2.h"

PreApproachNode::PreApproachNode() : Node("pre_approach_v2_node") {

    // Declare and get parameters
    obstacle_distance_ = this->declare_parameter<float>("obstacle", 0.0);
    rotation_degrees_ = this->declare_parameter<int>("degrees", 0);
    final_approach_ = this->declare_parameter<bool>("final_approach", false);

    navi_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = navi_callback_group_;
    sub_cb_group_ = timer_cb_group_;
    navigation_options.callback_group = sub_cb_group_;

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
    state_ = State::MOVING_FORWARD;
    yaw_ = 0.0;
    rotation_degrees_ = 0;
    obstacle_distance_ = 0.0;
    final_client_state_ = false;

    // Create timer for publishing velocity commands
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PreApproachNode::timer_callback, this), timer_cb_group_);

    srv_client_ = this->create_client<attach_shelf::srv::GoToLoading>("approach_shelf");

    // Wait for the service to become available
    while (!srv_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;  // Return instead of `return 0` since this is not `main`
        }
        RCLCPP_WARN(this->get_logger(), "Service 'approach_shelf' not available. Retrying...");
    }
    RCLCPP_INFO(this->get_logger(), "Service 'approach_shelf' is now available.");

    RCLCPP_INFO(this->get_logger(), "Node initialized and ready.");
}

double PreApproachNode::get_yaw_from_quaternion(double x, double y, double z, double w) {
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void PreApproachNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  yaw_ = get_yaw_from_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void PreApproachNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (state_ == State::MOVING_FORWARD) {
    // RCLCPP_INFO(this->get_logger(), "scan_callback, state::Moving_Forward");
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
    this->get_parameter("final_approach", final_approach_);

    switch (state_) {
        case State::MOVING_FORWARD:
            // RCLCPP_INFO(this->get_logger(), "[MOVING_FORWARD] Entering MOVING_FORWARD state.");
            cmd_vel_msg.linear.x = 0.5; // Move forward
            cmd_vel_msg.angular.z = 0.0;
            break;

        case State::ROTATING:
            // RCLCPP_INFO(this->get_logger(),"rotation_degree : %d, rotation_radian : %f, yaw:%f",rotation_degrees_, rotation_radian, yaw_);
            cmd_vel_msg.angular.z = rotation_radian * 0.2; // Rotate
            if (std::fabs(rotation_radian - yaw_) <= 0.03) {
                state_ = final_approach_ ? State::APPROACH : State::STOPPED;
            }
            break;

      case State::APPROACH: {
            RCLCPP_INFO_ONCE(this->get_logger(), "[APPROACH] Performing final approach.");
            auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
            request->attach_to_shelf = final_approach_;

            auto result_future = srv_client_->async_send_request(request);
            auto future_status = result_future.wait_for(std::chrono::seconds(30));

            if (future_status == std::future_status::ready) {
                auto response = result_future.get();
                if (response->complete) {
                    RCLCPP_INFO(this->get_logger(), "[APPROACH] Service call successful.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "[APPROACH] Service call failed.");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[APPROACH] Service call timed out.");
            }

            RCLCPP_INFO(this->get_logger(), "[APPROACH] Transitioning to STOPPED state.");
            state_ = State::STOPPED;
            break;
        }

        case State::STOPPED:
            RCLCPP_INFO_ONCE(this->get_logger(), "STOPPING");
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_pub_->publish(cmd_vel_msg);

            // Stop the timer and log shutdown
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Timer stopped. Node will shut down.");
            rclcpp::shutdown();
            break;
    }

    cmd_vel_pub_->publish(cmd_vel_msg);
}

void PreApproachNode::call_approach_service() {
    auto client = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = this->get_parameter("final_approach").as_bool();

    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Service /approach_shelf unavailable!");
        return;
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->complete) {
        RCLCPP_INFO(this->get_logger(), "Final approach successful!");
        } else {
        RCLCPP_WARN(this->get_logger(), "Final approach failed!");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed!");
    }
}
