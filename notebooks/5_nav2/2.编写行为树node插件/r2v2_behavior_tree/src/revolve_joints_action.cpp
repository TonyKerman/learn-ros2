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

#include "revolve_joints_action.hpp"

namespace nav2_behavior_tree
{

RevolveJointsAction::RevolveJointsAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<rc2024_interfaces::action::RevolveJoints>(xml_tag_name, action_name, conf)
{

  getInput("is_absolute", goal_.is_absolute);
}

void RevolveJointsAction::on_tick()
{
    getInput("revolve_cmd", goal_.cmd);
    getInput("expected_time", expected_time_);
    goal_.expected_time = rclcpp::Duration::from_seconds(expected_time_);
}
void RevolveJointsAction::on_wait_for_result(
  std::shared_ptr<const rc2024_interfaces::action::RevolveJoints::Feedback>/*feedback*/)
{
  // Grab the new path
  rc2024_interfaces::msg::JointInfoArray new_cmd;
  getInput("revolve_cmd", new_cmd);

  // Check if it is not same with the current one
  if (goal_.cmd != new_cmd) {
    // the action server on the next loop iteration
    goal_.cmd = new_cmd;
    goal_updated_ = true;
  }

    double new_expected_time;
  getInput("expected_time", new_expected_time);
  rclcpp::Duration new_expected_duration = rclcpp::Duration::from_seconds(new_expected_time);
  if (goal_.expected_time != new_expected_duration) {
    goal_.expected_time = new_expected_duration;
    goal_updated_ = true;
  }

}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::RevolveJointsAction>(name, "revolve_joints", config);
    };

  factory.registerBuilder<nav2_behavior_tree::RevolveJointsAction>("RevolveJoints", builder);
}
