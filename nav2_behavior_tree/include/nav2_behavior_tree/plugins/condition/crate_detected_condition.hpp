// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CRATE_DETECTED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CRATE_DETECTED_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_behavior_tree
{
class CrateDetectedCondition : public BT::ConditionNode
{
public:
  CrateDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  CrateDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("crate_measure_length", "Length of the crate to search for (in meters)"),

      // BT::InputPort<double>("crate_measure_width", 0.4,
      // "Width of the crate to search for (in meters)"),

      // BT::InputPort<double>("offset", 2,
      // "Offset of nav_goal to crate (in meters)"),

      // BT::InputPort<std::string>("laser_topic_", 
      // std::string("scan"), "laser scanner topic"),

      // BT::InputPort<std::string>("tolerance_", 
      // std::string("tolerance"), "Tolerance for crate location (in meters)"),

      // BT::InputPort<bool>(
        // "is_crate_found", false, "If the crate is found this will be true).
        // TODO(anyone): implement is_crate_found for faster sequence times
        // when crate is already found earlier
    };
  }

private:
  void laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  std::vector<float> arange(float start, float end, float increment);
  std::vector<geometry_msgs::msg::Vector3> polarToCartesian(sensor_msgs::msg::LaserScan::SharedPtr msg);
  std::vector<geometry_msgs::msg::Vector3> findCorners(std::vector<geometry_msgs::msg::Vector3> cartesian_sensor_data);
  geometry_msgs::msg::PoseStamped findCrate(std::vector<geometry_msgs::msg::Vector3> corner_locations_in_laser_frame);
  void broadcastCrateToTFTree(geometry_msgs::msg::PoseStamped);
  void publishNavGoal(geometry_msgs::msg::PoseStamped);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;  // is this type right??
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string laser_topic_;
  geometry_msgs::msg::PoseStamped nav_goal_;
  geometry_msgs::msg::PoseStamped crate_front_pose_;
  double crate_measure_length_;
  double crate_measure_width_;
  double offset_;
  // double tolerance_;
  bool is_crate_found_;
  bool broadcast_crate_;
  std::mutex mutex_;
};
} // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__CRATE_DETECTED_CONDITION_HPP_