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

#ifndef ANTONIQ_NODES__NAVIGATE_TO_POSE_CLIENT_HPP_
#define ANTONIQ_NODES__NAVIGATE_TO_POSE_CLIENT_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace antoniq_nodes
{

/**
 * @class antoniq_nodes::NavigateToPoseClient
 * @brief A simple wrapper on the nav2 NavigateToPose action for block-style calling,
 * following the same dedicated callback group pattern as robs4crop_util::ServiceClient
 * so it can be driven from a node whose default executor is already spinning elsewhere.
 */
using std::chrono_literals::operator""s;
class NavigateToPoseClient {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavigateToPoseClient(const rclcpp::Node::SharedPtr &provided_node,
                                const std::string &action_name = "navigate_to_pose")
      : action_name_(action_name), node_(provided_node)
  {
    callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_, callback_group_);
  }

  /**
   * @brief Block until the action server is available, retrying indefinitely (subject to
   * rclcpp::ok()) rather than giving up after a fixed timeout -- on startup the server (e.g.
   * nav2's bt_navigator) may simply not be up yet, since nav2's own lifecycle bring-up can take
   * a while, so waiting is the correct behavior here, not failing.
   * @return bool true once the server is available; false only if rclcpp shuts down while waiting
   */
  bool waitForServer()
  {
    while (!client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s action client: interrupted while waiting for action server",
                     action_name_.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "%s action client: waiting for action server to appear...",
                  action_name_.c_str());
    }
    return true;
  }

  /**
   * @brief Send a NavigateToPose goal for the given target pose and behavior tree, blocking
   * until the action succeeds, is aborted/canceled, or the call itself times out.
   * @param pose Target pose, in the frame the navigate_to_pose server expects (typically "map")
   * @param behavior_tree_xml Full filesystem path to the behavior tree XML to run for this goal
   * @return bool true if the goal was accepted and completed successfully
   */
  bool sendGoalAndWait(const geometry_msgs::msg::PoseStamped &pose,
                       const std::string &behavior_tree_xml)
  {
    if (!waitForServer()) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: action server not available",
                   action_name_.c_str());
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;
    goal.behavior_tree = behavior_tree_xml;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
    auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);
    if (callback_group_executor_.spin_until_future_complete(goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: send_goal call failed",
                   action_name_.c_str());
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: goal was rejected by server",
                   action_name_.c_str());
      return false;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (callback_group_executor_.spin_until_future_complete(result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: get_result call failed",
                   action_name_.c_str());
      return false;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return true;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "%s action client: goal was aborted",
                     action_name_.c_str());
        return false;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "%s action client: goal was canceled",
                     action_name_.c_str());
        return false;
      default:
        RCLCPP_ERROR(node_->get_logger(), "%s action client: unknown result code",
                     action_name_.c_str());
        return false;
    }
  }

  /**
   * @brief Send a goal and block only until it's accepted or rejected (not until it finishes),
   * so the caller can race it against something else (e.g. an obstacle monitor) instead of
   * blocking on the full result up front. Returns nullptr if the server is unavailable, the
   * send itself fails, or the goal is rejected.
   * @param pose Target pose, in the frame the navigate_to_pose server expects (typically "map")
   * @param behavior_tree_xml Full filesystem path to the behavior tree XML to run for this goal
   */
  GoalHandle::SharedPtr sendGoal(const geometry_msgs::msg::PoseStamped &pose,
                                 const std::string &behavior_tree_xml)
  {
    if (!waitForServer()) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: action server not available",
                   action_name_.c_str());
      return nullptr;
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;
    goal.behavior_tree = behavior_tree_xml;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
    auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);
    if (callback_group_executor_.spin_until_future_complete(goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: send_goal call failed",
                   action_name_.c_str());
      return nullptr;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: goal was rejected by server",
                   action_name_.c_str());
      return nullptr;
    }
    return goal_handle;
  }

  /**
   * @brief Block until an already-sent goal reaches a terminal state. Safe to call from a
   * background thread (e.g. via std::async) while the goal is raced against something else on
   * the calling thread, since it drives this client's own dedicated executor.
   * @param goal_handle Handle returned by sendGoal()
   * @return rclcpp_action::ResultCode the goal finished with (or UNKNOWN if the get_result call
   * itself failed)
   */
  rclcpp_action::ResultCode getResult(const GoalHandle::SharedPtr &goal_handle)
  {
    auto result_future = client_->async_get_result(goal_handle);
    if (callback_group_executor_.spin_until_future_complete(result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "%s action client: get_result call failed",
                   action_name_.c_str());
      return rclcpp_action::ResultCode::UNKNOWN;
    }
    return result_future.get().code;
  }

  /**
   * @brief Fire-and-forget cancel of an in-flight goal. Deliberately does not spin/wait for the
   * cancel acknowledgement, so it's safe to call from a different thread than the one blocked in
   * getResult() for the same goal -- that call's own spin will pick up both the cancel ack and
   * the resulting CANCELED terminal state as it keeps running.
   * @param goal_handle Handle returned by sendGoal()
   */
  void cancelGoal(const GoalHandle::SharedPtr &goal_handle)
  {
    client_->async_cancel_goal(goal_handle);
  }

  /**
   * @brief Gets the action name
   * @return string Action name
   */
  std::string getActionName()
  {
    return action_name_;
  }

protected:
  std::string action_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

}  // namespace antoniq_nodes

#endif  // ANTONIQ_NODES__NAVIGATE_TO_POSE_CLIENT_HPP_
