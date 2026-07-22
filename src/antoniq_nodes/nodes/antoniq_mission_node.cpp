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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// C++ Libs
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <antoniq_interfaces/msg/mission_status.hpp>
#include <antoniq_interfaces/srv/rotate_heading.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "antoniq_nodes/mission_logic.hpp"
#include "antoniq_nodes/navigate_to_pose_client.hpp"
#include "antoniq_nodes/service_client.hpp"

using namespace std::chrono_literals;

class AntoniqMission {
public:
  explicit AntoniqMission(rclcpp::Node::SharedPtr node) : node_(node)
  {
    node_->declare_parameter<int>("row_count", 3);
    node_->declare_parameter<double>("rotate_min_angular_vel", 0.1);
    node_->declare_parameter<double>("rotate_threshold", 0.05);
    node_->declare_parameter<double>("rotate_kp", 0.7);
    node_->declare_parameter<double>("waypoint_reach_tolerance", 0.15);
    node_->declare_parameter<int>("max_recovery_attempts", 5);

    const std::string behavior_tree_pkg_share =
        ament_index_cpp::get_package_share_directory("antoniq_navigation");
    straight_line_behavior_tree_xml_ =
        behavior_tree_pkg_share + "/behavior_trees/navigateToPoseStraightLine.xml";
    reverse_straight_line_behavior_tree_xml_ =
        behavior_tree_pkg_share + "/behavior_trees/navigateToPoseBackStraightLine.xml";
    default_navigate_to_pose_behavior_tree_xml_ =
        behavior_tree_pkg_share + "/behavior_trees/navigateToPose.xml";

    rotate_heading_client_ =
        std::make_shared<robs4crop_util::ServiceClient<antoniq_interfaces::srv::RotateHeading>>(
            "rotate_heading_server", node_);
    start_obstacle_monitor_client_ =
        std::make_shared<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>>(
            "start_obstacle_monitor", node_);
    stop_obstacle_monitor_client_ =
        std::make_shared<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>>(
            "stop_obstacle_monitor", node_);
    flip_waypoints_client_ =
        std::make_shared<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>>(
            "flip_waypoint_orientations", node_);
    navigate_to_pose_client_ =
        std::make_shared<antoniq_nodes::NavigateToPoseClient>(node_, "navigate_to_pose");
    status_pub_ =
        node_->create_publisher<antoniq_interfaces::msg::MissionStatus>("mission_status", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Drives the boustrophedon_waypoint_1..row_count*2 frames published by workstation_tf_manager
  // in order. Waypoint 1 sits a little behind the robot's spawn pose (see
  // workstation_tf_manager.py), so the mission starts straight at waypoint 2. Legs landing on
  // an even waypoint (2, 4, 6, ...) run the length of a row -- entering it -- so they use the
  // straight-line planner (navigateToPoseStraightLine.xml) with obstacle_monitor_node watching
  // the road ahead; legs landing on an odd waypoint (3, 5, 7, ...) turn through the headland
  // into the next row, so they use the regular costmap/recovery-aware navigateToPose.xml
  // instead, unmonitored. navigateToPose.xml is our own copy rather than nav2's stock default
  // (empty behavior_tree) because the stock tree's FollowPath node defaults to a goal checker
  // literally named "current_goal_checker", which nav2_params.yaml doesn't register (it defines
  // general_goal_checker/reverse_goal_checker) -- that combination aborts every FollowPath call
  // immediately. No explicit pre-rotate: rotate_heading_client_ is kept wired up but unused for
  // now.
  bool runMission()
  {
    int row_count = node_->get_parameter("row_count").as_int();

    if (!antoniq_nodes::isValidRowCount(row_count)) {
      RCLCPP_ERROR(node_->get_logger(), "row_count must be >= 1 (got %d)", row_count);
      return false;
    }
    int waypoint_count = antoniq_nodes::waypointCount(row_count);

    if (!waitForRequiredServers()) {
      return false;
    }
    if (!waitForWaypointFrame(antoniq_nodes::waypointFrameName(2))) {
      return false;
    }

    int max_recovery_attempts = node_->get_parameter("max_recovery_attempts").as_int();
    int total_obstacle_recoveries = 0;

    int waypoint_index = 2;
    while (waypoint_index <= waypoint_count) {
      std::string target_frame = antoniq_nodes::waypointFrameName(waypoint_index);
      int row = antoniq_nodes::rowForWaypoint(waypoint_index);
      publishStatus(row, row_count, waypoint_index, waypoint_count, target_frame);

      bool within_row_leg = antoniq_nodes::isWithinRowLeg(waypoint_index);
      if (!within_row_leg) {
        if (!navigateToWaypointWithRecovery(target_frame,
                                            default_navigate_to_pose_behavior_tree_xml_)) {
          RCLCPP_ERROR(node_->get_logger(), "Mission aborted: could not reach %s",
                       target_frame.c_str());
          return false;
        }
        ++waypoint_index;
        continue;
      }

      RowLegResult result = navigateRowWithObstacleMonitor(waypoint_index, target_frame);
      if (result == RowLegResult::FAILED) {
        RCLCPP_ERROR(node_->get_logger(), "Mission aborted: could not reach %s",
                     target_frame.c_str());
        return false;
      }
      if (result == RowLegResult::OBSTACLE_RECOVERED) {
        if (++total_obstacle_recoveries > max_recovery_attempts) {
          RCLCPP_ERROR(node_->get_logger(), "Mission aborted: too many obstacle recoveries (%d)",
                       max_recovery_attempts);
          return false;
        }
        // handleObstacleRecovery flipped every waypoint's orientation and swapped every row's
        // start/end frame names *before* reversing, so "waypoint_index" (the target we just
        // failed to reach) now names the row-start frame the robot backed off to. Resuming one
        // waypoint ahead treats that as the turn-arrival it physically is and continues the
        // mission's normal even=straight-line/odd=turn alternation, mirrored, from there.
        ++waypoint_index;
        RCLCPP_INFO(node_->get_logger(), "Resuming mission at waypoint %d after obstacle recovery",
                    waypoint_index);
        continue;
      }

      // SUCCEEDED
      ++waypoint_index;
    }

    RCLCPP_INFO(node_->get_logger(), "Mission completed successfully");
    return true;
  }

private:
  enum class RowLegResult { SUCCEEDED, OBSTACLE_RECOVERED, FAILED };

  // Blocks until every service/action server this mission depends on is actually up, instead of
  // letting the first real call fail (or, for the action client, time out after a fixed window)
  // just because e.g. nav2's lifecycle bring-up hasn't finished yet when this node starts.
  bool waitForRequiredServers()
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for required service/action servers to come up...");
    if (!waitForServiceClient(*rotate_heading_client_, "rotate_heading_server") ||
        !waitForServiceClient(*start_obstacle_monitor_client_, "start_obstacle_monitor") ||
        !waitForServiceClient(*stop_obstacle_monitor_client_, "stop_obstacle_monitor") ||
        !waitForServiceClient(*flip_waypoints_client_, "flip_waypoint_orientations") ||
        !navigate_to_pose_client_->waitForServer()) {
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "All required servers are available");
    return true;
  }

  template <typename ServiceT>
  bool waitForServiceClient(robs4crop_util::ServiceClient<ServiceT> &client,
                            const std::string &name)
  {
    while (!client.wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service [%s]",
                     name.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for service [%s] to become available...",
                  name.c_str());
    }
    return true;
  }

  // Blocks until workstation_tf_manager has actually started broadcasting the boustrophedon
  // waypoint frames, so the first navigate_to_pose goal isn't built from a TF lookup that's only
  // going to fail (or, worse, block for getFramePose()'s own 5s retry window) because that node
  // hasn't come up yet. frame_id is checked with canTransform() rather than looping
  // lookupTransform()+catch, since a missing frame there is an expected, non-exceptional
  // startup state, not an error.
  bool waitForWaypointFrame(const std::string &frame_id)
  {
    RCLCPP_INFO(node_->get_logger(),
                "Waiting for waypoint TF frames (workstation_tf_manager) to become active...");
    while (rclcpp::ok()) {
      if (tf_buffer_->canTransform("map", frame_id, tf2::TimePointZero)) {
        RCLCPP_INFO(node_->get_logger(), "Waypoint frames are active");
        return true;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for frame [%s] to become available...",
                  frame_id.c_str());
      std::this_thread::sleep_for(500ms);
    }
    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for frame [%s]", frame_id.c_str());
    return false;
  }

  // Enters a row with obstacle_monitor_node watching ahead of the robot for the whole leg. The
  // monitor is started (and its request given a moment to actually reach the server and arm
  // itself) before the straight-line planner goal is ever sent, so there's no window where the
  // robot is moving down the row unmonitored. Races the navigate_to_pose result against the
  // (blocking, server-side) obstacle-monitor start call: whichever finishes first decides the
  // outcome. If the monitor reports an obstacle, the in-flight goal is cancelled (stopping the
  // robot) and handleObstacleRecovery() takes over; otherwise the monitor is told to stop once
  // navigation concludes on its own.
  RowLegResult navigateRowWithObstacleMonitor(int waypoint_index, const std::string &target_frame)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getFramePose(target_frame, goal_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not look up frame [%s] in map",
                   target_frame.c_str());
      return RowLegResult::FAILED;
    }

    RCLCPP_INFO(node_->get_logger(), "Arming obstacle monitor before entering row toward %s",
                target_frame.c_str());
    auto obstacle_future =
        std::async(std::launch::async, [this]() { return callStartObstacleMonitor(); });
    // let the (already in-flight) start request actually arm the monitor
    std::this_thread::sleep_for(200ms);

    auto goal_handle =
        navigate_to_pose_client_->sendGoal(goal_pose, straight_line_behavior_tree_xml_);
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Could not send navigate_to_pose goal for %s",
                   target_frame.c_str());
      callStopObstacleMonitor();
      obstacle_future.get();
      return RowLegResult::FAILED;
    }

    RCLCPP_INFO(node_->get_logger(), "Entering row toward %s with obstacle monitoring active",
                target_frame.c_str());
    auto nav_future = std::async(std::launch::async, [this, goal_handle]() {
      return navigate_to_pose_client_->getResult(goal_handle);
    });

    bool obstacle_future_done = false;
    bool obstacle_detected = false;
    rclcpp_action::ResultCode nav_result = rclcpp_action::ResultCode::UNKNOWN;

    while (rclcpp::ok()) {
      if (!obstacle_future_done && obstacle_future.wait_for(20ms) == std::future_status::ready) {
        obstacle_detected = obstacle_future.get();
        obstacle_future_done = true;
        if (obstacle_detected) {
          RCLCPP_WARN(node_->get_logger(),
                      "Obstacle detected ahead, cancelling navigate_to_pose goal");
          navigate_to_pose_client_->cancelGoal(goal_handle);
        }
      }
      if (nav_future.wait_for(20ms) == std::future_status::ready) {
        nav_result = nav_future.get();
        break;
      }
    }

    if (!obstacle_future_done) {
      // navigate_to_pose concluded on its own first; tell the monitor to stand down and
      // collect its (now unblocked) result before moving on.
      callStopObstacleMonitor();
      obstacle_detected = obstacle_future.get();
    }

    if (obstacle_detected) {
      return handleObstacleRecovery(waypoint_index) ? RowLegResult::OBSTACLE_RECOVERED
                                                    : RowLegResult::FAILED;
    }

    if (nav_result != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node_->get_logger(), "navigate_to_pose failed for %s", target_frame.c_str());
      return RowLegResult::FAILED;
    }
    return RowLegResult::SUCCEEDED;
  }

  // Obstacle response: flip immediately, before the robot moves at all --
  // flip_waypoint_orientations rotates every waypoint's orientation by pi and swaps every row's
  // start/end frame names (see workstation_tf_manager.py). That makes "waypoint_index" (the
  // target we just failed to reach) refer to the row-start frame the robot is about to back off
  // to, so reversing there under its new name is really "backing off into waypoint_index" --
  // e.g. failing to reach waypoint_4 straight-line, flipping, and reversing to (the now-renamed)
  // waypoint_4. From there the mission's normal even=straight-line/odd=turn alternation just
  // continues, mirrored, with no further special-casing.
  bool handleObstacleRecovery(int waypoint_index)
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "Obstacle recovery: flipping orientations and swapping start/end names before backing off");

    auto flip_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr flip_response;
    if (!flip_waypoints_client_->invoke(flip_request, flip_response) || !flip_response->success) {
      RCLCPP_ERROR(node_->get_logger(), "flip_waypoint_orientations call failed");
      return false;
    }

    // workstation_tf_manager broadcasts on a 0.1s timer rather than synchronously with the flip
    // (see workstation_tf_manager.py), so the service call above returning doesn't guarantee the
    // updated /tf has actually reached this node's TF buffer yet. getFramePose() below would
    // otherwise happily return the still-cached *old* transform (a stale lookup isn't an
    // exception, so its retry loop wouldn't catch it) -- wait long enough for the new broadcast
    // to land first.
    std::this_thread::sleep_for(500ms);

    std::string arrival_frame = antoniq_nodes::waypointFrameName(waypoint_index);

    // The reverse goal orientation is set to the robot's own current heading, not the waypoint
    // frame's stored orientation: reverse_goal_checker's yaw tolerance is wide open, so this
    // doesn't affect whether the goal is accepted, but the MPPI FollowBackPath controller's angle
    // critics actively steer toward whatever heading the goal carries mid-path. Matching the goal
    // heading to the current heading means the straight-line path never needs to turn, so the
    // robot only ever translates backward to get there.
    geometry_msgs::msg::PoseStamped robot_pose, arrival_pose;
    if (!getFramePose("base_footprint", robot_pose) || !getFramePose(arrival_frame, arrival_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not look up current pose or %s in map",
                   arrival_frame.c_str());
      return false;
    }
    geometry_msgs::msg::PoseStamped reverse_goal;
    reverse_goal.header.frame_id = "map";
    reverse_goal.header.stamp = node_->get_clock()->now();
    reverse_goal.pose.position = arrival_pose.pose.position;
    reverse_goal.pose.orientation = robot_pose.pose.orientation;

    RCLCPP_INFO(node_->get_logger(), "Reversing to %s (post-flip) while holding current heading",
                arrival_frame.c_str());
    return navigateToPoseWithRecovery(reverse_goal, arrival_frame,
                                      reverse_straight_line_behavior_tree_xml_);
  }

  bool callStartObstacleMonitor()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr response;
    if (!start_obstacle_monitor_client_->invoke(request, response)) {
      RCLCPP_ERROR(node_->get_logger(), "start_obstacle_monitor call failed");
      return false;
    }
    return response->success;
  }

  bool callStopObstacleMonitor()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr response;
    if (!stop_obstacle_monitor_client_->invoke(request, response)) {
      RCLCPP_ERROR(node_->get_logger(), "stop_obstacle_monitor call failed");
      return false;
    }
    return response->success;
  }

  void publishStatus(int row, int row_count, int waypoint_index, int waypoint_count,
                     const std::string &target_frame)
  {
    antoniq_interfaces::msg::MissionStatus status;
    status.row = row;
    status.row_count = row_count;
    status.waypoint_index = waypoint_index;
    status.waypoint_count = waypoint_count;
    status.waypoint_frame = target_frame;
    status.status_description =
        antoniq_nodes::describeMissionStatus(row, row_count, waypoint_index, waypoint_count);
    status_pub_->publish(status);
  }

  bool rotateTowardsGoal(const std::string &target_frame)
  {
    auto request = std::make_shared<antoniq_interfaces::srv::RotateHeading::Request>();
    request->target_frame = target_frame;
    request->min_angular_vel = node_->get_parameter("rotate_min_angular_vel").as_double();
    request->threshold = node_->get_parameter("rotate_threshold").as_double();
    request->kp = node_->get_parameter("rotate_kp").as_double();

    antoniq_interfaces::srv::RotateHeading::Response::SharedPtr response;
    if (!rotate_heading_client_->invoke(request, response)) {
      RCLCPP_ERROR(node_->get_logger(), "rotate_heading_server call failed");
      return false;
    }
    return response->success;
  }

  bool navigateToPose(const geometry_msgs::msg::PoseStamped &goal_pose,
                      const std::string &behavior_tree)
  {
    RCLCPP_INFO(node_->get_logger(), "Sending navigate_to_pose goal (%.2f, %.2f) via %s",
                goal_pose.pose.position.x, goal_pose.pose.position.y,
                behavior_tree.empty() ? "default behavior tree" : behavior_tree.c_str());
    return navigate_to_pose_client_->sendGoalAndWait(goal_pose, behavior_tree);
  }

  bool navigateToWaypoint(const std::string &target_frame, const std::string &behavior_tree)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getFramePose(target_frame, goal_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not look up frame [%s] in map",
                   target_frame.c_str());
      return false;
    }
    return navigateToPose(goal_pose, behavior_tree);
  }

  // navigate_to_pose can report success without the robot actually settling on the goal (goal
  // checker tolerances, a straight-line planner overshoot, ...); this cross-checks against the
  // live TF tree and, if off, recovers by navigating to the goal again, up to a bounded number
  // of attempts. reach_check_frame is the TF frame to measure "did we actually get there"
  // against -- normally the same frame the goal pose came from, but handleObstacleRecovery
  // passes a goal pose with a substituted orientation while still checking position against the
  // real waypoint frame.
  bool navigateToPoseWithRecovery(const geometry_msgs::msg::PoseStamped &goal_pose,
                                  const std::string &reach_check_frame,
                                  const std::string &behavior_tree)
  {
    if (!navigateToPose(goal_pose, behavior_tree)) {
      return false;
    }

    int max_recovery_attempts = node_->get_parameter("max_recovery_attempts").as_int();
    for (int attempt = 1; attempt <= max_recovery_attempts; ++attempt) {
      if (hasReachedWaypoint(reach_check_frame)) {
        return true;
      }
      RCLCPP_WARN(node_->get_logger(), "Did not settle near %s (recovery attempt %d/%d)",
                  reach_check_frame.c_str(), attempt, max_recovery_attempts);
      if (!navigateToPose(goal_pose, behavior_tree)) {
        return false;
      }
    }
    return hasReachedWaypoint(reach_check_frame);
  }

  bool navigateToWaypointWithRecovery(const std::string &target_frame,
                                      const std::string &behavior_tree)
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getFramePose(target_frame, goal_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not look up frame [%s] in map",
                   target_frame.c_str());
      return false;
    }
    return navigateToPoseWithRecovery(goal_pose, target_frame, behavior_tree);
  }

  bool hasReachedWaypoint(const std::string &target_frame)
  {
    geometry_msgs::msg::PoseStamped robot_pose, target_pose;
    if (!getFramePose("base_footprint", robot_pose) || !getFramePose(target_frame, target_pose)) {
      return false;
    }
    double dx = robot_pose.pose.position.x - target_pose.pose.position.x;
    double dy = robot_pose.pose.position.y - target_pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    double tolerance = node_->get_parameter("waypoint_reach_tolerance").as_double();
    return distance <= tolerance;
  }

  // Retries with a bound rather than looping forever, since the frame may not (yet) exist.
  bool getFramePose(const std::string &frame_id, geometry_msgs::msg::PoseStamped &pose_out)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    constexpr int max_attempts = 50;
    for (int attempt = 0; attempt < max_attempts && rclcpp::ok(); ++attempt) {
      try {
        transform_stamped = tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);
        pose_out.header.frame_id = "map";
        pose_out.header.stamp = node_->get_clock()->now();
        pose_out.pose.position.x = transform_stamped.transform.translation.x;
        pose_out.pose.position.y = transform_stamped.transform.translation.y;
        pose_out.pose.position.z = transform_stamped.transform.translation.z;
        pose_out.pose.orientation = transform_stamped.transform.rotation;
        return true;
      } catch (tf2::TransformException &ex) {
        std::this_thread::sleep_for(100ms);
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::string straight_line_behavior_tree_xml_;
  std::string reverse_straight_line_behavior_tree_xml_;
  std::string default_navigate_to_pose_behavior_tree_xml_;
  std::shared_ptr<robs4crop_util::ServiceClient<antoniq_interfaces::srv::RotateHeading>>
      rotate_heading_client_;
  std::shared_ptr<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>>
      start_obstacle_monitor_client_;
  std::shared_ptr<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>>
      stop_obstacle_monitor_client_;
  std::shared_ptr<robs4crop_util::ServiceClient<std_srvs::srv::Trigger>> flip_waypoints_client_;
  std::shared_ptr<antoniq_nodes::NavigateToPoseClient> navigate_to_pose_client_;
  rclcpp::Publisher<antoniq_interfaces::msg::MissionStatus>::SharedPtr status_pub_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Every parameter this node uses is explicitly declared (with a default) in
  // AntoniqMission's constructor, so auto-declaring from overrides is unnecessary and would
  // throw ParameterAlreadyDeclaredException whenever a launch file overrides one of them.
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("antoniq_mission_node");

  auto mission = std::make_shared<AntoniqMission>(node);
  bool success = mission->runMission();

  rclcpp::shutdown();
  return success ? 0 : 1;
}
