/**
 * @file walker.cpp
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/walker.hpp"

Movement::Movement() : Node("movement"), state(StateType::STOP) {
    // ... constructor implementation
        auto p_topic_name = "cmd_vel";
        auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Publisher to publish TWIST type of message to the topic
        velocity_publisher_ = this->create_publisher<TWIST>(p_topic_name, 10);

        // Subscriber to subscribe to /scan topic
        auto s_topic_name = "/scan";
        auto sub_callback = std::bind(&Movement::subscribeCallback, this, _1);
        laser_subscriber_ = this->create_subscription
        <sensor_msgs::msg::LaserScan>(
        s_topic_name, default_qos, sub_callback);
        auto timer_callback = std::bind(&Movement::timerCallback, this);
        timer_ = this->create_wall_timer(10ms, timer_callback);
}
void Movement::subscribeCallback(const LASER &msg) { current_scan = msg; }

void Movement::timerCallback() {
    if (current_scan.header.stamp.sec == 0) {
        return;
    }

    // Define a TWIST type of publish message
    auto pub = TWIST();

    if (state == StateType::STOP) {
        // If STOP due to obstacle, then turn else move straight
        if (obstacleDetected()) {
            state = StateType::ROTATE;
            pub.angular.z = -0.25;
            velocity_publisher_->publish(pub);
            RCLCPP_INFO_STREAM(this->get_logger(),
            "STOP state");} else {
            state = StateType::MOTION;
            pub.linear.x = 0.1;
            velocity_publisher_->publish(pub);
            RCLCPP_INFO_STREAM(this->get_logger(),
            "STOP state");}
    } else if (state == StateType::MOTION) {
        if (obstacleDetected()) {  // Check transition
            state = StateType::STOP;
            pub.linear.x = 0;
            velocity_publisher_->publish(pub);
            RCLCPP_INFO_STREAM(this->get_logger(),
            "MOTION state");
        }
    } else if (state == StateType::ROTATE) {
        // If it is in ROTATE state, rotate till no obstacle is found
        if (!obstacleDetected()) {
            state = StateType::MOTION;
            pub.linear.x = 0.1;
            velocity_publisher_->publish(pub);
            RCLCPP_INFO_STREAM(this->get_logger(),
            "ROTATE state");
        }
    }
}
bool Movement::obstacleDetected() {
    for (std::size_t i = 0;
            i < sizeof(current_scan.ranges) /
            sizeof(current_scan.ranges[0]); i++) {
        if (current_scan.ranges[i] > current_scan.range_min
            && current_scan.ranges[i] < current_scan.range_max) {
            RCLCPP_INFO(this->get_logger(),
                        "Distance: %f is valid", current_scan.ranges[i]);
            if (current_scan.ranges[i] < 1.0) {
                RCLCPP_INFO(this->get_logger(),
                            "Obstacle detected, rotating");
                return true;
            }
        }
        return false;
    }
    return false;
}
