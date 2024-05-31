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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REVOLVE_JOINTS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REVOLVE_JOINTS_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rc2024_interfaces/action/revolve_joints.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Spin
 */
class RevolveJointsAction : public BtActionNode<rc2024_interfaces::action::RevolveJoints>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  RevolveJointsAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
  void on_wait_for_result(
    std::shared_ptr<const rc2024_interfaces::action::RevolveJoints::Feedback> feedback) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<rc2024_interfaces::msg::JointInfoArray>("revolve_cmd","revolve cmd "),
        BT::InputPort<double>("expected_time", 1.0, "time you expect to revolve"),
        BT::InputPort<bool>("is_absolute", false, "is absolute or relative"),
      });
  }

private:
  bool is_absolute_;
  double expected_time_;
};

}  // namespace nav2_behavior_tree

namespace BT
{
    template<>
inline rc2024_interfaces::msg::JointInfoArray convertFromString(const StringView key)
{
    rc2024_interfaces::msg::JointInfoArray joint_info_array;
    auto parts = BT::splitString(key, ';');
    for(unsigned int i = 0; i < parts.size(); i++)
    {
        auto part2s = BT::splitString(parts[i], ',');
        if(part2s.size() != 2)
        {
            throw std::runtime_error("invalid number of fields for joint attribute)");
        }
        else
        {
            rc2024_interfaces::msg::JointInfo joint_info = rc2024_interfaces::msg::JointInfo();
            joint_info.index = BT::convertFromString<int>(part2s[0]);
            joint_info.position = BT::convertFromString<double>(part2s[1]);
            joint_info_array.data.push_back(joint_info);            
        }
    }
    return joint_info_array;
}
}
#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
