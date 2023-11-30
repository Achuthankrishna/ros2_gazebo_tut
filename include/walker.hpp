/**
 * @file walker.hpp
 * @author Vyshnav Achuthan (vyachu07@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

/**
 * @brief Enum representing different states of the Movement.
 */
enum class StateType {
  MOTION = 0,
  STOP,
  ROTATE,
};

/**
 * @brief Movement class represents a simple robot that can navigate avoiding obstacles.
 */
class Movement : public rclcpp::Node {
 public:
    Movement();

 private:
  /**
   * @brief Callback function for the laser scan subscriber.
   * @param msg The laser scan message received from the sensor.
   */
  void subscribeCallback(const LASER& msg);
  /**
   * @brief Timer Callback
   * 
   */
  void timerCallback();
  /**
   * @brief Check for obstacle detection
   * 
   * @return true 
   * @return false 
   */
  bool obstacleDetected();

    // Initialization of publisher, subscriber, and variables
    rclcpp::Subscription<LASER>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<TWIST>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LASER current_scan;
    StateType state;
};

