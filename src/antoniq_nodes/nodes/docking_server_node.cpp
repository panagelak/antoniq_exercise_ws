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
#include <tf2_ros/transform_broadcaster.h>
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

#include "rclcpp/rclcpp.hpp"

// tf
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// geometry
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// std msgs
#include <antoniq_interfaces/srv/docking.hpp>

// control toolbox
#include <control_toolbox/pid.hpp>

//
using namespace std::chrono_literals;
using namespace std::placeholders;

class DockingNode {
public:
  explicit DockingNode(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
    service_docking_ = node_->create_service<antoniq_interfaces::srv::Docking>(
        "docking_service", std::bind(&DockingNode::DockingCB, this, _1, _2));
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_docking", 1);

    // tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_->get_node_base_interface(), node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  bool DockingCB(const std::shared_ptr<antoniq_interfaces::srv::Docking::Request> req,
                 std::shared_ptr<antoniq_interfaces::srv::Docking::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), "Received Docking Request");
    // setup pid controllers
    pid_x_ = control_toolbox::Pid(req->x_gains[0], req->x_gains[1], req->x_gains[2],
                                  req->windup_limit, -req->windup_limit, true);
    pid_y_ = control_toolbox::Pid(req->y_gains[0], req->y_gains[1], req->y_gains[2],
                                  req->windup_limit, -req->windup_limit, true);
    pid_yaw_ =
        control_toolbox::Pid(req->angular_gains[0], req->angular_gains[1], req->angular_gains[2],
                             req->windup_limit, -req->windup_limit, true);
    // twist command
    geometry_msgs::msg::Twist twist_command;
    rclcpp::Time last_update_time;
    bool first = true;
    while (true) {
      // if we do not have pose return
      if (!getCurrentPose()) {
        std::this_thread::sleep_for(20ms);
        continue;
      }
      if (first) {
        last_update_time = node_->get_clock()->now();
        std::this_thread::sleep_for(20ms);
        first = false;
        continue;
      }
      if (!rclcpp::ok()) {
        break;
      }
      double now = node_->get_clock()->now().seconds();
      uint64_t period = (now - last_update_time.seconds()) * 1000000000.0;
      RCLCPP_INFO(node_->get_logger(), "Computing");
      double map_x = pid_x_.computeCommand(req->target_x - current_x_, period);
      double map_y = pid_x_.computeCommand(req->target_y - current_y_, period);
      // yaw
      double delta_yaw = req->target_yaw - current_yaw_;
      delta_yaw = std::remainder(delta_yaw, 2 * M_PI);
      twist_command.angular.z = pid_yaw_.computeCommand(delta_yaw, period);
      RCLCPP_INFO(node_->get_logger(), "Command : x-> %f y-> %f yaw-> %f", twist_command.linear.x,
                  twist_command.linear.y, twist_command.angular.z);
      //=== tranform twist from map frame to robot frame
      twist_command.linear.x = std::cos(current_yaw_) * map_x + std::sin(current_yaw_) * map_y;
      twist_command.linear.y = -std::sin(current_yaw_) * map_x + std::cos(current_yaw_) * map_y;
      //=== apply limits
      if (twist_command.linear.x > req->pos_vel_limit) {
        twist_command.linear.x = req->pos_vel_limit;
      } else if (twist_command.linear.x < -req->pos_vel_limit) {
        twist_command.linear.x = -req->pos_vel_limit;
      }
      if (twist_command.linear.y > req->pos_vel_limit) {
        twist_command.linear.y = req->pos_vel_limit;
      } else if (twist_command.linear.y < -req->pos_vel_limit) {
        twist_command.linear.y = -req->pos_vel_limit;
      }
      if (twist_command.angular.z > req->ang_vel_limit) {
        twist_command.angular.z = req->ang_vel_limit;
      } else if (twist_command.angular.z < -req->ang_vel_limit) {
        twist_command.angular.z = -req->ang_vel_limit;
      }

      //=== publish
      pub_cmd_vel_->publish(twist_command);
      //=== check contition
      double dist_x = std::fabs(req->target_x - current_x_);
      double dist_y = std::fabs(req->target_y - current_y_);
      double dist_yaw = std::fabs(req->target_yaw - current_yaw_);
      RCLCPP_INFO(node_->get_logger(), "Current Dist : x-> %f y-> %f yaw-> %f", current_x_,
                  current_y_, current_yaw_);
      RCLCPP_INFO(node_->get_logger(), "Target Dist : x-> %f y-> %f yaw-> %f", dist_x, dist_y,
                  dist_yaw);
      if (dist_x < req->pos_threshold && dist_y < req->pos_threshold &&
          dist_yaw < req->ang_threshold) {
        // publish zero
        for (size_t i = 0; i < 10; i++) {
          twist_command.linear.x = 0.0;
          twist_command.linear.y = 0.0;
          twist_command.angular.z = 0.0;
          pub_cmd_vel_->publish(twist_command);
          std::this_thread::sleep_for(2ms);
        }
        RCLCPP_INFO(node_->get_logger(), "Docking succeded");
        break;
      }
      //
      last_update_time = node_->get_clock()->now();
      std::this_thread::sleep_for(20ms);
    }

    res->success = true;
    return true;
  }
  bool getCurrentPose()
  {
    try {
      geometry_msgs::msg::TransformStamped currentTransform =
          tf_buffer_->lookupTransform("map", "summit_base_footprint", tf2::TimePointZero);
      tf2::Quaternion q(
          currentTransform.transform.rotation.x, currentTransform.transform.rotation.y,
          currentTransform.transform.rotation.z, currentTransform.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      current_x_ = currentTransform.transform.translation.x;
      current_y_ = currentTransform.transform.translation.y;
      current_yaw_ = yaw;
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "Could Not get current pose of robot");
      return false;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Service<antoniq_interfaces::srv::Docking>::SharedPtr service_docking_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  //
  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;
  //
  geometry_msgs::msg::TransformStamped currentTransform_;
  double current_x_, current_y_, current_yaw_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("docking_server_node", node_options);
  auto rotate_wrapper = std::make_shared<DockingNode>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
