#ifndef APPROACH_SERVICE_SERVER_HPP
#define APPROACH_SERVICE_SERVER_HPP

#include <chrono>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logging.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// to pusblish the transform
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
// to listen to the transform time stamp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// custom service message (GoToLoading.srv)
#include "attach_shelf/srv/go_to_loading.hpp"

#define PI 3.141592653589793

using GoToLanding = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node {
public:
    ApproachServiceServer();

private:
    // ROS2 Service callback
    void service_callback(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

    // Callbacks
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // void broadcast_timer_callback();

    // Helper methods for shelf handling
    double get_yaw_from_quaternion(double x, double y, double z, double w);
    bool finding_shelf_legs();
    void finding_center_position();
    void adding_fixed_cartframe();
    bool move_to_cart_center();
    void rotating_center_cart();
    bool moving_under_cart();
    void lift_shelf();


    // ROS2 Members
    rclcpp::CallbackGroup::SharedPtr navi_callback_group_;
    rclcpp::SubscriptionOptions navigation_options;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr server_;

    // tf2 Members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_;
    geometry_msgs::msg::TransformStamped transformStamped_;

    // Frame and scan data
    std::string odom_frame_, laser_frame_, cart_frame_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    int left_leg_index_, right_leg_index_;
    float cart_x_, cart_y_, cart_yaw_;
    tf2::Quaternion cart_quat_;

    // State variables
    bool start_service_;
    bool move_extra_distance_;
    bool reached_center_position_;
    bool robot_rotating_done_;
    bool find_two_legs_;

    // Robot and cart poses
    geometry_msgs::msg::Pose cart_pose_, robot_pose_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    double yaw_;
};


#endif // APPROACH_SERVICE_SERVER_H