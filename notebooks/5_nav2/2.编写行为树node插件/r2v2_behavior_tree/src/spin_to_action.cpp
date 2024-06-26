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

#include <string>
#include <memory>

#include "spin_to_action.hpp"

namespace nav2_behavior_tree
{

SpinToAction::SpinToAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<rc2024_interfaces::action::SpinTo>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("spin_direction", dist);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.target_yaw = dist;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  getInput("is_recovery", is_recovery_);
}

void SpinToAction::on_tick()
{
  if (is_recovery_) {
    increment_recovery_count();
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SpinToAction>(name, "spin_to", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinToAction>("SpinTo", builder);
}
