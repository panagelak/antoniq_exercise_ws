// Copyright 2026 Panagiotis Angelakis
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// C++ Libs
#include <cmath>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

// ros
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::placeholders;

class ObstacleMonitor {
public:
  explicit ObstacleMonitor(rclcpp::Node::SharedPtr node) : node_(node)
  {
    node_->declare_parameter<double>("obstacle_distance", 0.7);
    node_->declare_parameter<double>("front_half_angle", 0.35);
    node_->declare_parameter<std::string>("scan_topic", "scan");

    obstacle_distance_ = node_->get_parameter("obstacle_distance").as_double();
    front_half_angle_ = node_->get_parameter("front_half_angle").as_double();
    std::string scan_topic = node_->get_parameter("scan_topic").as_string();

    // Each callback gets its own group so the blocking start_obstacle_monitor call (waiting on
    // the condition variable below) doesn't stall the scan subscription or the stop service.
    scan_callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    start_callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(), std::bind(&ObstacleMonitor::scanCB, this, _1),
        scan_options);

    start_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "start_obstacle_monitor", std::bind(&ObstacleMonitor::startCB, this, _1, _2),
        rclcpp::ServicesQoS().get_rmw_qos_profile(), start_callback_group_);
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "stop_obstacle_monitor", std::bind(&ObstacleMonitor::stopCB, this, _1, _2),
        rclcpp::ServicesQoS().get_rmw_qos_profile(), stop_callback_group_);

    RCLCPP_INFO(node_->get_logger(), "Starting (idle until start_obstacle_monitor is called)");
  }

  // Blocks until either an obstacle shows up within obstacle_distance_ in front of the robot
  // (success=true) or stop_obstacle_monitor is called first (success=false).
  bool startCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    obstacle_detected_ = false;
    stop_requested_ = false;
    activated_ = true;
    RCLCPP_INFO(node_->get_logger(), "Obstacle monitor activated");
    cv_.wait(lock, [this]() { return obstacle_detected_ || stop_requested_ || !rclcpp::ok(); });
    activated_ = false;

    res->success = obstacle_detected_;
    res->message = obstacle_detected_ ? "Obstacle detected within range"
                                      : "Stopped without detecting an obstacle";
    RCLCPP_INFO(node_->get_logger(), "Obstacle monitor deactivated: %s", res->message.c_str());
    return true;
  }

  bool stopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    cv_.notify_all();
    res->success = true;
    res->message = "Stop requested";
    return true;
  }

  void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!activated_) {
      return;
    }

    double min_range_in_front = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      // Normalize to (-pi, pi] so the front cone (angle 0) is correctly checked even when the
      // scan's own angle range wraps all the way around (e.g. [0, 2*pi) for a 360-degree LiDAR).
      double wrapped_angle = std::remainder(angle, 2.0 * M_PI);
      if (std::fabs(wrapped_angle) > front_half_angle_) {
        continue;
      }
      float range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }
      min_range_in_front = std::min(min_range_in_front, static_cast<double>(range));
    }

    if (min_range_in_front <= obstacle_distance_) {
      obstacle_detected_ = true;
      cv_.notify_all();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  double obstacle_distance_;
  double front_half_angle_;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool activated_{false};
  bool obstacle_detected_{false};
  bool stop_requested_{false};

  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr start_callback_group_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("obstacle_monitor_node");

  auto monitor = std::make_shared<ObstacleMonitor>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
