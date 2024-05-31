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

#ifndef R2V2_BEHAVIORS__REVOLVE_JOINTS_HPP_
#define R2V2_BEHAVIORS__REVOLVE_JOINTS_HPP_

#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_util/node_utils.hpp"
#include "rc2024_interfaces/action/revolve_joints.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace r2v2_behaviors
{
using namespace nav2_behaviors;
using RevolveJoints_T = rc2024_interfaces::action::RevolveJoints;

/**
 * @class nav2_behaviors::Behavior
 * @brief An action server Behavior base class implementing the action server
 * and basic factory.
 */

class RevolveJoints : public nav2_core::Behavior
{
  public:
    using ActionServer = nav2_util::SimpleActionServer<RevolveJoints_T>;
    /**
     * @brief A TimedBehavior constructor
     */
    RevolveJoints();
    ~RevolveJoints();

    Status onRun(
        const std::shared_ptr<const RevolveJoints_T::Goal> command);
    Status onCycleUpdate();

    void onActionCompletion()
    {
    }

    // configure the server on lifecycle setup
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
            collision_checker) override;

    // Cleanup server on lifecycle transition
    void cleanup() override
    {
        action_server_.reset();
        joint_pub_.reset();
        joint_sub_.reset();
    }

    // Activate server on lifecycle transition
    void activate() override
    {
        RCLCPP_INFO(logger_, "Activating1 %s", behavior_name_.c_str());
        joint_pub_->on_activate();
        RCLCPP_INFO(logger_, "Activating2 %s", behavior_name_.c_str());
        action_server_->activate();
        RCLCPP_INFO(logger_, "Activating3 %s", behavior_name_.c_str());

        enabled_ = true;
    }

    // Deactivate server on lifecycle transition
    void deactivate() override
    {
        joint_pub_->on_deactivate();
        action_server_->deactivate();
        enabled_ = false;
    }

  protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    std::string behavior_name_;

    std::shared_ptr<ActionServer> action_server_;

    double cycle_frequency_;
    double enabled_;
    rclcpp::Duration elasped_time_{0, 0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    RevolveJoints_T::Feedback::SharedPtr feedback_;
    std::vector<rc2024_interfaces::msg::JointInfo> cmds_;
    std::vector<rc2024_interfaces::msg::JointInfo> cmds_remaining_;
    bool is_absolute_;
    std::vector<double> current_joint_positions_;
    
    int total_joint_num_;
    rclcpp::Duration expected_time_{0, 0};
    rclcpp::Time end_time_;

    // Clock
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    // Logger
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_behaviors")};

    // Main execution callbacks for the action server implementation calling the
    // Behavior's onRun and cycle functions to execute a specific behavior
    void execute()
    {
        RCLCPP_INFO(logger_, "Running %s", behavior_name_.c_str());

        if (!enabled_)
        {
            RCLCPP_WARN(logger_, "Called while inactive, ignoring request.");
            return;
        }

        if (onRun(action_server_->get_current_goal()) != Status::SUCCEEDED)
        {
            RCLCPP_INFO(logger_, "Initial checks failed for %s",
                        behavior_name_.c_str());
            action_server_->terminate_current();
            return;
        }

        auto start_time = steady_clock_.now();

        // Initialize the ActionT result
        auto result = std::make_shared<typename RevolveJoints_T::Result>();

        rclcpp::WallRate loop_rate(cycle_frequency_);

        while (rclcpp::ok())
        {
            elasped_time_ = steady_clock_.now() - start_time;
            if (action_server_->is_cancel_requested())
            {
                RCLCPP_INFO(logger_, "Canceling %s", behavior_name_.c_str());
                action_server_->terminate_all(result);
                onActionCompletion();
                return;
            }

            // TODO(orduno) #868 Enable preempting a Behavior on-the-fly without
            // stopping
            if (action_server_->is_preempt_requested())
            {
                RCLCPP_ERROR(logger_,
                             "Received a preemption request for %s,"
                             " however feature is currently not implemented. "
                             "Aborting and stopping.",
                             behavior_name_.c_str());
                action_server_->terminate_current(result);
                onActionCompletion();
                return;
            }

            switch (onCycleUpdate())
            {
            case Status::SUCCEEDED:
                RCLCPP_INFO(logger_, "%s completed successfully",
                            behavior_name_.c_str());
                action_server_->succeeded_current(result);
                onActionCompletion();
                return;

            case Status::FAILED:
                RCLCPP_WARN(logger_, "%s failed", behavior_name_.c_str());
                action_server_->terminate_current(result);
                onActionCompletion();
                return;

            case Status::RUNNING:

            default:
                loop_rate.sleep();
                break;
            }
        }
    }
void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace r2v2_behaviors

#endif // NAV2_BEHAVIORS__TIMED_BEHAVIOR_HPP_
