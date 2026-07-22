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

#include <sys/ioctl.h>  // For ioctl, TIOCGWINSZ
#include <unistd.h>     // For STDOUT_FILENO

#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

// C++ Libs
#include <chrono>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

// ros
#include "rclcpp/rclcpp.hpp"
#include <antoniq_interfaces/srv/rotate_heading.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class RotateHeading {
public:
  explicit RotateHeading(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
    service_rotate_ = node_->create_service<antoniq_interfaces::srv::RotateHeading>(
        "rotate_heading_server", std::bind(&RotateHeading::rotateHeadingCB, this, _1, _2));
    pub_cmd_vel_ =
        node_->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_rotate_heading", 1);
    RCLCPP_INFO(node_->get_logger(), "Starting");
    // tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  bool rotateHeadingCB(const std::shared_ptr<antoniq_interfaces::srv::RotateHeading::Request> req,
                       std::shared_ptr<antoniq_interfaces::srv::RotateHeading::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Starting rotate to heading goal towards frame [%s]",
                req->target_frame.c_str());
    geometry_msgs::msg::PoseStamped start_pose = getMapPose();
    RCLCPP_INFO(node_->get_logger(), "Got start pose");
    geometry_msgs::msg::PoseStamped target_pose;
    if (!getFramePose(req->target_frame, target_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not look up frame [%s] in map",
                   req->target_frame.c_str());
      res->success = false;
      return true;
    }
    double yaw_target =
        getYawFromPoses(start_pose, target_pose.pose.position.x, target_pose.pose.position.y);
    double current_yaw = get_current_yaw(start_pose);
    RCLCPP_INFO(node_->get_logger(), "Got starting yaw : %f and target : %f", current_yaw,
                yaw_target);
    geometry_msgs::msg::TwistStamped command_msg;
    command_msg.header.frame_id = "base_footprint";
    double delta = yaw_target - current_yaw;
    while (std::fabs(delta) > req->threshold && rclcpp::ok()) {
      geometry_msgs::msg::PoseStamped current_pose = getMapPose();
      current_yaw = get_current_yaw(current_pose);
      delta = yaw_target - current_yaw;
      // Ensure the delta is within the range of -pi to +pi
      delta = std::remainder(delta, 2 * M_PI);
      command_msg.twist.angular.z = req->kp * delta;
      if (std::fabs(command_msg.twist.angular.z) < req->min_angular_vel) {
        if (command_msg.twist.angular.z > 0) {
          command_msg.twist.angular.z = req->min_angular_vel;
        } else {
          command_msg.twist.angular.z = -req->min_angular_vel;
        }
      }
      command_msg.header.stamp = node_->get_clock()->now();
      pub_cmd_vel_->publish(command_msg);
      std::this_thread::sleep_for(20ms);
      RCLCPP_INFO(node_->get_logger(), "Target -> [%f] Current -> [%f] delta -> [%f]", yaw_target,
                  current_yaw, delta);
    }
    for (size_t i = 0; i < 10; i++) {
      command_msg.twist.angular.z = 0.0;
      command_msg.header.stamp = node_->get_clock()->now();
      pub_cmd_vel_->publish(command_msg);
      std::this_thread::sleep_for(20ms);
    }
    res->success = true;
    return true;
  }
  double get_current_yaw(geometry_msgs::msg::PoseStamped msg)
  {
    tf2::Quaternion quat_current;
    tf2::fromMsg(msg.pose.orientation, quat_current);
    tf2::Matrix3x3 matrix_current(quat_current);
    double roll_curr, pitch_curr, yaw_curr;
    matrix_current.getRPY(roll_curr, pitch_curr, yaw_curr);
    return yaw_curr;
  }
  double getYawFromPoses(const geometry_msgs::msg::PoseStamped &start_pose, double target_x,
                         double target_y)
  {
    // Calculate differences in x and y
    double delta_x = target_x - start_pose.pose.position.x;
    double delta_y = target_y - start_pose.pose.position.y;

    // Compute the yaw using atan2
    double bearing_yaw = std::atan2(delta_y, delta_x);
    return bearing_yaw;
  }
  geometry_msgs::msg::PoseStamped getMapPose()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    geometry_msgs::msg::PoseStamped pose;
    while (rclcpp::ok()) {
      try {
        transformStamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;
        return pose;
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform: %s", ex.what());
      }
    }
    return pose;
  }
  // Unlike getMapPose() (base_footprint, always expected to be present), the requested
  // target_frame may not exist, so this retries with a bound instead of looping forever.
  bool getFramePose(const std::string &frame_id, geometry_msgs::msg::PoseStamped &pose_out)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    constexpr int max_attempts = 50;
    for (int attempt = 0; attempt < max_attempts && rclcpp::ok(); ++attempt) {
      try {
        transformStamped = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);
        pose_out.pose.position.x = transformStamped.transform.translation.x;
        pose_out.pose.position.y = transformStamped.transform.translation.y;
        pose_out.pose.position.z = transformStamped.transform.translation.z;
        pose_out.pose.orientation = transformStamped.transform.rotation;
        return true;
      } catch (tf2::TransformException &ex) {
        std::this_thread::sleep_for(100ms);
      }
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<antoniq_interfaces::srv::RotateHeading>::SharedPtr service_rotate_;
  // rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("rotate_heading_server", node_options);

  auto rotate_wrapper = std::make_shared<RotateHeading>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
