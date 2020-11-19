// Copyright (c) 2018 Intel Corporation
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
#include <chrono>
#include <thread>

#include <memory>
#include <string>
#include "nav2_behavior_tree/plugins/action/docking_action.hpp"

namespace nav2_behavior_tree
{

DockingAction::DockingAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nena_msgs::action::Docking>(xml_tag_name, action_name, conf)
{
  RCLCPP_INFO(node_->get_logger(), "[Docker_action][constructor] Constructing docking action client");
  // getInput("activate_docking", docking_active_);
  sleep_wait_for_resutls_ = 100; //milliseconds
}

void DockingAction::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "[Docker_action][on_tick] I got ticked mate");
  // getInput("activate_docking", docking_active_);
  times_waits_for_results_looped_ = 0; // reset every time the node is executed
  // docking_active_ = true;
  if(docking_active_ == true){
    RCLCPP_INFO(node_->get_logger(), "[Docker_action][on_tick] Activated");
    goal_.start = 1;
    
  } else {
    RCLCPP_INFO(node_->get_logger(), "[Docker_action][on_tick] No action taken");
    // goal_.start = 0;
    on_aborted(); // return failure
  }

  // int start_bit = 9;
  // getInput("activate_docking", goal_.start);
  // goal_.start = start_bit;
  // setOutput("activate_docking", !start_bit); 

  /* TODO(Kevin):
      - if crate found is true: start the docking procedure (call action server)
      -- on update value it will send it to the action server
      - if ticked again, update status to eg running, idle or failed
  */
  // getInput("activate_docking", goal_.start);
}

void DockingAction::on_wait_for_result()
{
  RCLCPP_INFO(node_->get_logger(), "[Docker_action][on_wait_for_result] Maby I should do something now...");
  RCLCPP_INFO(node_->get_logger(), "[Docker_action][on_wait_for_result] Your mom called %i times", times_waits_for_results_looped_);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_wait_for_resutls_));
  times_waits_for_results_looped_++;

  /* TODO(Kevin): 
      - Wait on feedback from action server.
  */ 

  // nav_msgs::msg::Path new_path;
  // getInput("path", new_path);

}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = 
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::DockingAction>(
        name, "docking", config);
    };
  factory.registerBuilder<nav2_behavior_tree::DockingAction>(
    "Docking", builder);
}