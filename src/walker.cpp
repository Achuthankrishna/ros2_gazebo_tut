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

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
//FROM TURTLEBOT3_BRINGUP
using std::placeholders::_1;
//FROM TURTLEBOT3_SPAWN
using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

/**
 * @brief  States of the robot
*/
typedef enum {
  MOTION = 0,
  STOP,
  ROTATE,
} StateType;

/**
 * @brief Movement Class
 *
 */
class Movement : public rclcpp::Node {
public:
    // Adding movement constructor
    Movement() : Node("movement"), state(STOP) {

        auto p_topic_name = "cmd_vel";
        auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // Publisher to publish TWIST type of message to the topic
        velocity_publisher_ = this->create_publisher<TWIST>(p_topic_name, 10);

        // Subscriber to subscribe to /scan topic
        auto s_topic_name = "/scan";
        auto sub_callback = std::bind(&Movement::subscribeCallback, this, _1);
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            s_topic_name, default_qos, sub_callback);
        //Buffer_Time
        auto timer_callback = std::bind(&Movement::timerCallback, this);
        //Timer CB
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    void subscribeCallback(const LASER &msg) { current_scan = msg; }

    void timerCallback() {
        if (current_scan.header.stamp.sec == 0) {
            return;
        }

        // Define a TWIST type of publish message
        auto pub = TWIST();

        if (state == STOP) {
            // If STOP due to obstacle, then turn else move straight
            if (obstacleDetected()) {
                state = ROTATE;
                pub.angular.z = -0.15;
                velocity_publisher_->publish(pub);
                RCLCPP_INFO_STREAM(this->get_logger(), "STOP state");
            } else {
                state = MOTION;
                pub.linear.x = 0.1;
                velocity_publisher_->publish(pub);
                RCLCPP_INFO_STREAM(this->get_logger(), "STOP state");
            }
        } else if (state == MOTION) {
            if (obstacleDetected()) {  // Check transition
                state = STOP;
                pub.linear.x = 0;
                velocity_publisher_->publish(pub);
                RCLCPP_INFO_STREAM(this->get_logger(), "MOTION state");
            }
        } else if (state == ROTATE) {
            // If it is in ROTATE state, rotate till no obstacle is found
            if (!obstacleDetected()) {
                state = MOTION;
                pub.linear.x = 0.1;
                velocity_publisher_->publish(pub);
                RCLCPP_INFO_STREAM(this->get_logger(), "ROTATE state");
            }
        }
    }
