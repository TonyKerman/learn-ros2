#ifndef MY_BEHAVIOR_TREE__PLUGINS__ACTION__CATCHBALL_ACTION_HPP_
#define MY_BEHAVIOR_TREE__PLUGINS__ACTION__CATCHBALL_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rc2024_interfaces/action/catch_ball.hpp"

namespace my_behavior_tree
{
    class CatchBallAction : public nav2_behavior_tree::BtActionNode<rc2024_interfaces::action::CatchBall>
    {
        public:
            CatchBallAction(
                const std::string & xml_tag_name,
                const std::string & action_name,
                const BT::NodeConfiguration & conf);
            void on_tick() override;
            static BT::PortsList providedPorts()
            {
                return providedBasicPorts(
                    {
                        BT::InputPort<int>("time_allowance",5, "max time to run the action in seconds"),
                        BT::InputPort<int>("color", 1, "colcor to catch")
                    });
            }
    };
}
#endif  // MY_BEHAVIOR_TREE__PLUGINS__ACTION__PUTBALL_ACTION_HPP_