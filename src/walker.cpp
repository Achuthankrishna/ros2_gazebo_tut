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
        // Callback function for subscriber
        auto sub_callback = std::bind(&Movement::subscribeCallback, this, _1);
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            s_topic_name, default_qos, sub_callback);

        // Buffer timer for processing
        auto timer_callback = std::bind(&Movement::timerCallback, this);
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

