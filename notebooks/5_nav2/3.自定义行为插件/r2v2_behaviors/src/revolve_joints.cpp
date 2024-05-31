// Copyright (c) 2018 Intel Corporation, 2019 Samsung Research America
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "r2v2_behaviors/revolve_joints.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace r2v2_behaviors
{

RevolveJoints::RevolveJoints()
    : action_server_(nullptr), cycle_frequency_(20.0), enabled_(false),
      total_joint_num_(0)
{
}

RevolveJoints::~RevolveJoints() = default;

void RevolveJoints::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>
        collision_checker)
{
    node_ = parent;
    auto node = node_.lock();
    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());
    behavior_name_ = name;
    node->get_parameter("cycle_frequency", cycle_frequency_);
    action_server_ = std::make_shared<ActionServer>(
        node, behavior_name_, std::bind(&RevolveJoints::execute, this));
    nav2_util::declare_parameter_if_not_declared(node, "total_joint_num",
                                                 rclcpp::ParameterValue(0));
    node->get_parameter("total_joint_num", total_joint_num_);
    cmds_.resize(total_joint_num_);
    cmds_remaining_.resize(total_joint_num_);
    std::string joint_feedback_topic_;
    nav2_util::declare_parameter_if_not_declared(
        node, "joint_feedback_topic", rclcpp::ParameterValue("joint_states"));
    node->get_parameter("joint_feedback_topic", joint_feedback_topic_);

    joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        joint_feedback_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&RevolveJoints::jointStateCallback, this,
                  std::placeholders::_1));
    joint_pub_ = node->template create_publisher<sensor_msgs::msg::JointState>(
        "/car/up_cmd", 1);
}

Status RevolveJoints::onRun(
    const std::shared_ptr<const RevolveJoints_T::Goal> command)
{
    RCLCPP_INFO(logger_,
                "Starting RevolveJoints behavior with target joint positions");

    expected_time_ = command->expected_time;

    end_time_ = steady_clock_.now() + expected_time_;
    if (current_joint_positions_.size() == 0 ||
        current_joint_positions_.size() > total_joint_num_)
    {
        RCLCPP_ERROR(logger_,
                     "joints num wrong cmds%ld current_joint_positions_%ld "
                     "total joint num %d",
                     cmds_.size(), current_joint_positions_.size(),
                     total_joint_num_);
        return Status::FAILED;
    }
    for (int i = 0; i < command->cmd.data.size(); i++)
    {
        cmds_[i].index=command->cmd.data[i].index;
        cmds_remaining_[i].index=command->cmd.data[i].index;
        if(command->is_absolute)
        {
            cmds_[i].position=command->cmd.data[i].position;
            cmds_remaining_[i].position=command->cmd.data[i].position- current_joint_positions_[cmds_[i].index];
        }
        else
        {
            cmds_[i].position=command->cmd.data[i].position+current_joint_positions_[command->cmd.data[i].index];
            cmds_remaining_[i].position=command->cmd.data[i].position;
        }
        RCLCPP_DEBUG(logger_, "index %d target pos %.2f remaining %.2f", command->cmd.data[i].index,
                    command->cmd.data[i].position,cmds_remaining_[i].position);
    }

    return Status::SUCCEEDED;
    // RCLCPP_INFO(logger_, "checkpoint1");
}

Status RevolveJoints::onCycleUpdate()
{
    // 没考虑弧度范围-pi到pi
    rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
    // if (time_remaining.seconds() < 0.0 && expected_time_.seconds() > 0.0)
    // {
    //     RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the "
    //                          "RevolveJoints goal - Exiting RevolveJoints");
    //     return Status::FAILED;
    // }
    // RCLCPP_INFO(logger_, "checkpoint2");
    sensor_msgs::msg::JointState joint_cmd;
    std::vector<rc2024_interfaces::msg::JointInfo>::iterator it = cmds_.begin();
    std::vector<rc2024_interfaces::msg::JointInfo>::iterator it2 = cmds_remaining_.begin();
    for (int i = 0; i < total_joint_num_; i++)
    {
        double res;
       
        if (i == it->index&&it != cmds_.end())
        {   
            
            // double pos_remaining = it->position - current_joint_positions_[i];
            // if (std::abs(pos_remaining) > 0.05)
            //     is_done = false;
            // res = current_joint_positions_[i] +
            //         copysign(dt * joint_rotational_vel_, pos_remaining);
            // RCLCPP_INFO(logger_, "checkpoint2 joint %d res%.4f ", i, res);

            // if ((res > it->position && pos_remaining > 0) ||
            //     (res < it->position && pos_remaining < 0))
            // {
            //     res = it->position;
            // }
            res =it->position - time_remaining.seconds()/expected_time_.seconds()*it2->position;
            // RCLCPP_INFO(logger_, "checkpoint3 joint %d time raining %.2f res%.4f", i,time_remaining.seconds(), res);
            it++;
            it2++;
        }
        else
        {
            res = current_joint_positions_[i];
        }

        joint_cmd.position.push_back(res);
    }
    // RCLCPP_INFO(logger_, "checkpoint4 %2f %2f", joint_cmd.position[2], time_remaining.seconds());
    joint_pub_->publish(joint_cmd);
    if (time_remaining.seconds()<=0.0)
    {
        RCLCPP_INFO(logger_, "RevolveJoints goal reached");
        return Status::SUCCEEDED;
    }
    // else if(time_remaining.seconds() <=0.0)
    // {
    //     RCLCPP_WARN(logger_, "Exceeded time allowance before reaching the "
    //                          "RevolveJoints goal - Exiting RevolveJoints");
    //     return Status::FAILED;
    // }

    return Status::RUNNING;
}

void RevolveJoints::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_positions_ = msg->position;
}
} // namespace r2v2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(r2v2_behaviors::RevolveJoints, nav2_core::Behavior)
