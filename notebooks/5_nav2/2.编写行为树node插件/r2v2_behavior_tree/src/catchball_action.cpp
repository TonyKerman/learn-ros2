// Copyright (c) 2018 Samsung Research America
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

#include "catchball_action.hpp"

namespace my_behavior_tree
{

CatchBallAction::CatchBallAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<rc2024_interfaces::action::CatchBall>(xml_tag_name, action_name, conf)
{
    RCLCPP_INFO(node_->get_logger(), "CatchBallAction initialized");
    int time_allowance;
    getInput("time_allowance", time_allowance);
    getInput("ball_id", goal_.color);
    //todo: action 加入 time_allowance;
    //goal_.time_allowance = time_allowance;
}

void CatchBallAction::on_tick()
{
  increment_recovery_count();
  RCLCPP_INFO(node_->get_logger(), "CatchBallAction ticking");
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<my_behavior_tree::CatchBallAction>(name, "catchball", config);
    };

  factory.registerBuilder<my_behavior_tree::CatchBallAction>("CatchBall", builder);
}
