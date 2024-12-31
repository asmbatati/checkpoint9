#ifndef PRE_APPROACH_HPP
#define PRE_APPROACH_HPP

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "attach_shelf/srv/go_to_loading.hpp"


class PreApproachNode : public rclcpp::Node {
public:
    PreApproachNode();
    
private:
    enum class State { MOVING_FORWARD, ROTATING, STOPPED, APPROACH };

    double get_yaw_from_quaternion(double x, double y, double z, double w);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();
    void call_approach_service();

    // Members
    rclcpp::CallbackGroup::SharedPtr navi_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

    rclcpp::SubscriptionOptions navigation_options;
    rclcpp::SubscriptionOptions scan_options;
    rclcpp::SubscriptionOptions odom_options;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr srv_client_;

    double yaw_;
    float obstacle_distance_;
    int rotation_degrees_;
    State state_;
    rclcpp::Time rotation_start_time_;
    bool final_approach_;
    bool final_client_state_;
};

#endif // PRE_APPROACH_HPP
