
#ifndef R2V2_BEHAVIORS__REVOLVE_JOINTS_HPP_
#define R2V2_BEHAVIORS__REVOLVE_JOINTS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rc2024_interfaces/action/revolve_joints.hpp"

namespace r2v2_behaviors
{
using namespace nav2_behaviors;
using Action_T = rc2024_interfaces::action::RevolveJoints;

/**
 * @class r2v2_behaviors::SpinTo
 * @brief An action server behavior for spinning in
 */
class RevolveJoints : public TimedBehavior<Action_T>
{
  public:
    /**
     * @brief A constructor for r2v2_behaviors::Spin
     */
    RevolveJoints();
    ~RevolveJoints();

    /**
     * @brief Initialization to run behavior
     * @param command Goal to execute
     * @return Status of behavior
     */
    Status onRun(
        const std::shared_ptr<const Action_T::Goal> command) override;

    /**
     * @brief Configuration of behavior action
     */
    void onConfigure() override;

    /**
     * @brief Loop function to run behavior
     * @return Status of behavior
     */
    Status onCycleUpdate() override;

  protected:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_pos_sub_;

    Action_T::Feedback::SharedPtr feedback_;
    std::vector<double> target_joint_positions_;
    std::vector<double> current_joint_positions_;
    int joint_num_;
    double joint_rotational_vel_;
    double min_rotational_vel_;
    double rotational_acc_lim_;
    rclcpp::Duration expected_time_{0,0};
    rclcpp::Time end_time_;
};

} // namespace r2v2_behaviors

#endif // NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
