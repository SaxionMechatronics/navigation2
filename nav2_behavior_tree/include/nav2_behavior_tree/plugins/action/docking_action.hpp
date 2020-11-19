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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISION_DOCKING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISION_DOCKING_ACTION_HPP_

#include <string>
#include "nena_msgs/action/docking.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class DockingAction : public BtActionNode<nena_msgs::action::Docking>
{
public:
  DockingAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  void on_wait_for_result() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // BT::InputPort<bool>("activate_docking",  "Set high to activate docking"),
        // BT::OutputPort<bool>("activate_docking", "Set high to activate docking"),
        // BT::InputPort<std::string>("controller_id", ""),
      });
  }
private:
  bool docking_active_;
  int times_waits_for_results_looped_;
  int sleep_wait_for_resutls_;
  int dist_;
};
}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISION_DOCKING_ACTION_HPP_
