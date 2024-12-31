#include "attach_shelf/approach_service_server.h"

ApproachServiceServer::ApproachServiceServer() : Node("approach_service_server_node") {

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
    // server_ = this->create_service<attach_shelf::srv::GoToLoading>(
    //     "approach_shelf",
    //     std::bind(&ApproachServiceServer::service_callback, this,
    //               std::placeholders::_1, std::placeholders::_2),
    //     rmw_qos_profile_services_default, navi_callback_group_);

    RCLCPP_INFO(this->get_logger(), "Initializing service 'approach_shelf'");
    server_ = this->create_service<attach_shelf::srv::GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachServiceServer::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    if (server_) {
        RCLCPP_INFO(this->get_logger(), "Service 'approach_shelf' initialized successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize service 'approach_shelf'.");
    }

    // Create tf_broadcast timer callback for publishing velocity commands
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

bool ApproachServiceServer::finding_shelf_legs() {
    if (!this->last_scan_) {
        RCLCPP_ERROR(this->get_logger(), "No laser scan data available.");
        return false;
    }

    // Verify intensities size
    if (this->last_scan_->intensities.empty()) {
        RCLCPP_WARN(this->get_logger(), "No high-intensity laser points detected.");
        return false;
    }

    std::vector<size_t> first_laser_intensity_index;
    for (size_t i = 0; i < this->last_scan_->intensities.size(); i++) {
        if (this->last_scan_->intensities[i] >= 7000.0) {
            first_laser_intensity_index.push_back(i);
        }
    }

    if (first_laser_intensity_index.empty()) {
        RCLCPP_WARN(this->get_logger(), "No high-intensity points found.");
        return false;
    }

    if (first_laser_intensity_index.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough high-intensity points for clustering.");
        return false;
    }

    // Logic for clustering high-intensity indices
    std::vector<std::vector<size_t>> object_counter;
    std::vector<size_t> second_laser_intensity_index;

    second_laser_intensity_index.push_back(first_laser_intensity_index[0]);
    for (size_t i = 1; i < first_laser_intensity_index.size(); i++) {
        if (first_laser_intensity_index[i] - first_laser_intensity_index[i - 1] < 5) {
            second_laser_intensity_index.push_back(first_laser_intensity_index[i]);
        } else {
            object_counter.push_back(second_laser_intensity_index);
            second_laser_intensity_index.clear();
            second_laser_intensity_index.push_back(first_laser_intensity_index[i]);
        }
    }
    if (!second_laser_intensity_index.empty()) {
        object_counter.push_back(second_laser_intensity_index);
    }

    if (object_counter.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find enough clusters.");
        return false;
    }

    this->left_leg_index_ = object_counter[0][0];
    this->right_leg_index_ = object_counter[1][0];
    this->find_two_legs_ = true;

    RCLCPP_INFO(this->get_logger(), "Left leg index: %u, Right leg index: %u", this->left_leg_index_, this->right_leg_index_);

    // Check bounds before accessing ranges
    if (left_leg_index_ >= static_cast<int>(last_scan_->ranges.size()) || 
        right_leg_index_ >= static_cast<int>(last_scan_->ranges.size())) {
        RCLCPP_ERROR(this->get_logger(), "Leg indices are out of bounds.");
        return false;
    }

    return true;
}


void ApproachServiceServer::finding_center_position() {

    double left_shelf_distance_from_robot = last_scan_->ranges[left_leg_index_];
    double right_shelf_distance_from_robot =
        last_scan_->ranges[right_leg_index_];
    float cart_magnitude = 0.0;

    // The rays are from 0 to 1080 clockwise [225 to -45 degrees]
    cart_magnitude =
        (left_shelf_distance_from_robot + right_shelf_distance_from_robot) /
        2.0;
    // Calculate the average angle between the two legs
    cart_yaw_ = last_scan_->angle_min + (left_leg_index_ + right_leg_index_) /
                                            2.0 * last_scan_->angle_increment;
    // Calculate the cartesian coordinates
    cart_x_ = cart_magnitude * cos(cart_yaw_);
    cart_y_ = cart_magnitude * sin(cart_yaw_);
    cart_quat_.setRPY(0.0, 0.0, cart_yaw_);

    // adding_fixed_cartframe();
    found_center_position_ = true;
}

void ApproachServiceServer::adding_fixed_cartframe() {
    if (!find_two_legs_) {
        RCLCPP_WARN(this->get_logger(), "Cannot add fixed cart frame. Shelf legs not found.");
        return;
    }

    if (left_leg_index_ >= static_cast<int>(last_scan_->ranges.size()) || 
        right_leg_index_ >= static_cast<int>(last_scan_->ranges.size())) {
        RCLCPP_ERROR(this->get_logger(), "Leg indices are out of bounds.");
        return;
    }

    geometry_msgs::msg::TransformStamped cart_transform;

    cart_transform.header.stamp = this->get_clock()->now();
    cart_transform.header.frame_id = laser_frame_;
    cart_transform.child_frame_id = cart_frame_;

    cart_transform.transform.translation.x = cart_x_;
    cart_transform.transform.translation.y = cart_y_;
    cart_transform.transform.translation.z = 0.0;

    cart_transform.transform.rotation.x = cart_quat_.x();
    cart_transform.transform.rotation.y = cart_quat_.y();
    cart_transform.transform.rotation.z = cart_quat_.z();
    cart_transform.transform.rotation.w = cart_quat_.w();

    tf_broadcaster_->sendTransform(cart_transform);

    RCLCPP_INFO(this->get_logger(), "Broadcasted cart frame at x=%.2f, y=%.2f, yaw=%.2f",
                cart_x_, cart_y_, cart_yaw_);
}


void ApproachServiceServer::move_to_cart_center() {

    geometry_msgs::msg::Twist cmd_vel_msg;
    double distance_to_cart = sqrt(pow(cart_x_, 2) + pow(cart_y_, 2));
    if (distance_to_cart > 0.4) {
        RCLCPP_INFO(this->get_logger(), "Moving to cart_frame");
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = 0.1;
        cmd_vel_pub_->publish(cmd_vel_msg);
        found_center_position_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Stop to moving cart");
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
        move_extra_distance_ = true;
    }
}

void ApproachServiceServer::rotating_center_cart() {

    geometry_msgs::msg::Twist cmd_vel_msg;
    if (robot_pose_.orientation.z >= 0.05) {
        RCLCPP_INFO(this->get_logger(), "Rotating left to cart_frame");
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = -0.1;
        cmd_vel_pub_->publish(cmd_vel_msg);
    } else if (robot_pose_.orientation.z <= -0.05) {
        RCLCPP_INFO(this->get_logger(), "Rotating right to moving cart");
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.1;
        cmd_vel_pub_->publish(cmd_vel_msg);
    } else if (((robot_pose_.orientation.z > 0.0) &&
                (robot_pose_.orientation.z <= 0.05)) ||
                ((robot_pose_.orientation.z >= -0.05) &&
                (robot_pose_.orientation.z <= 0.0))) {
        RCLCPP_INFO(this->get_logger(), "Stoping to moving cart");
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
        robot_rotating_done_ = true;
    }
}

void ApproachServiceServer::moving_under_cart() {

    geometry_msgs::msg::Twist cmd_vel_msg;
    int count = 3;

    while (count != 0) {
        cmd_vel_msg.linear.x = 0.1;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
        count--;
    }
}

void ApproachServiceServer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;

    finding_shelf_legs();
    adding_fixed_cartframe();
    
    if (!start_service_) {
        RCLCPP_DEBUG(this->get_logger(), "No active service request. Skipping movement logic.");
        return;
    }

    // Log first few ranges and intensities for debugging
    for (size_t i = 0; i < std::min<size_t>(last_scan_->ranges.size(), 10); ++i) {
        RCLCPP_INFO(this->get_logger(), "Range[%zu]: %f Intensity: %f",
                    i, last_scan_->ranges[i], last_scan_->intensities[i]);
    }

    // Check if we can find shelf legs
    if (finding_shelf_legs()) {
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

    RCLCPP_INFO(this->get_logger(), "[SERVICE] Received request: attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    if (request->attach_to_shelf) {
        if (find_two_legs_) {
            start_service_ = true;
            response->complete = true;
            RCLCPP_INFO(this->get_logger(), "[SERVICE] Operation successful. Shelf legs detected.");
        } else {
            response->complete = false;
            RCLCPP_WARN(this->get_logger(), "[SERVICE] Operation failed. Unable to detect shelf legs.");
        }
    } else {
        start_service_ = false;
        response->complete = false;
        RCLCPP_ERROR(this->get_logger(), "[SERVICE] Invalid request: attach_to_shelf=false.");
    }
}

void ApproachServiceServer::broadcast_timer_callback() {
    if (!find_two_legs_) {
        RCLCPP_WARN(this->get_logger(), "Cannot broadcast transform: Shelf legs not found.");
        return;
    }

    if (left_leg_index_ >= static_cast<int>(last_scan_->ranges.size()) || 
        right_leg_index_ >= static_cast<int>(last_scan_->ranges.size())) {
        RCLCPP_ERROR(this->get_logger(), "Leg indices are out of bounds.");
        return;
    }

    double left_distance = last_scan_->ranges[left_leg_index_];
    double right_distance = last_scan_->ranges[right_leg_index_];

    double center_distance = (left_distance + right_distance) / 2.0;
    double left_angle = last_scan_->angle_min + left_leg_index_ * last_scan_->angle_increment;
    double right_angle = last_scan_->angle_min + right_leg_index_ * last_scan_->angle_increment;
    double center_angle = (left_angle + right_angle) / 2.0;

    double center_x = center_distance * cos(center_angle);
    double center_y = center_distance * sin(center_angle);

    tf2::Quaternion shelf_quat;
    shelf_quat.setRPY(0.0, 0.0, center_angle);

    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = cart_frame_;
    transform_msg.transform.translation.x = center_x;
    transform_msg.transform.translation.y = center_y;
    transform_msg.transform.rotation.x = shelf_quat.x();
    transform_msg.transform.rotation.y = shelf_quat.y();
    transform_msg.transform.rotation.z = shelf_quat.z();
    transform_msg.transform.rotation.w = shelf_quat.w();

    tf_broadcaster_->sendTransform(transform_msg);

    RCLCPP_INFO(this->get_logger(), "Broadcasted transform: x=%.2f, y=%.2f, yaw=%.2f",
                center_x, center_y, center_angle);
}


