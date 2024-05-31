
#ifndef R2V2_BEHAVIORS__SPIN_TO_HPP_
#define R2V2_BEHAVIORS__SPIN_TO_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "rc2024_interfaces/action/spin_to.hpp"

namespace r2v2_behaviors
{
using namespace nav2_behaviors;
using SpinAction = rc2024_interfaces::action::SpinTo;

/**
 * @class r2v2_behaviors::SpinTo
 * @brief An action server behavior for spinning in
 */
class SpinTo : public TimedBehavior<SpinAction>
{
  public:
    /**
     * @brief A constructor for r2v2_behaviors::Spin
     */
    SpinTo();
    ~SpinTo();

    /**
     * @brief Initialization to run behavior
     * @param command Goal to execute
     * @return Status of behavior
     */
    Status onRun(
        const std::shared_ptr<const SpinAction::Goal> command) override;

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
    /**
     * @brief Check if pose is collision free
     * @param distance Distance to check forward
     * @param cmd_vel current commanded velocity
     * @param pose2d Current pose
     * @return is collision free or not
     */
    bool isCollisionFree(const double &distance,
                         geometry_msgs::msg::Twist *cmd_vel,
                         geometry_msgs::msg::Pose2D &pose2d);

    SpinAction::Feedback::SharedPtr feedback_;

    double min_rotational_vel_;
    double max_rotational_vel_;
    double rotational_acc_lim_;
    double cmd_yaw_;
    double prev_yaw_;
    double relative_yaw_;
    double simulate_ahead_time_;
    rclcpp::Duration command_time_allowance_{0, 0};
    rclcpp::Time end_time_;
};

} // namespace r2v2_behaviors

#endif // NAV2_BEHAVIORS__PLUGINS__SPIN_HPP_
