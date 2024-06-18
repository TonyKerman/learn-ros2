# 获取当前位姿
## 官网方法
[来源:https://docs.ros.org](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)
现在，让我们看一下与访问帧转换相关的代码。包含 tf2_ros 一个 TransformListener 类，该类使接收转换的任务更容易。
```cpp
#include "tf2_ros/transform_listener.h"
```

在这里，我们创建一个 TransformListener 对象。创建侦听器后，它将开始通过网络接收 tf2 转换，并缓冲它们长达 10 秒。
```cpp
tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
tf_listener_ =
  std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

```
最后，我们向侦听器查询特定的转换。我们使用以下参数调用 lookup_transform 方法：

Target frame 目标帧

Source frame 源帧

The time at which we want to transform
我们想要转型的时刻

提供 tf2::TimePointZero 只会让我们获得最新的可用转换。所有这些都封装在一个 try-catch 块中，以处理可能的异常。
```cpp
    std::string fromFrameRel = "map";
    std::string toFrameRel = "base_link";
void timer_callback()
{
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);    
}

```
生成的变换表示目标龟相对于 turtle2 的位置和方向。然后使用海龟之间的角度来计算跟随目标海龟的速度命令。有关 tf2 的更多常规信息，另请参阅“概念”部分中的 tf2 页面。
## nav2 实现
.h
```cpp
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
class a
{
    private:
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
}
```
.cpp:首先定义两个函数(或`#include "nav2_util/node_utils.hpp"`)
```cpp
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
/**
* @brief get the current pose of the robot
* @param global_pose Pose to transform
* @param tf_buffer TF buffer to use for the transformation
* @param global_frame Frame to transform into
* @param robot_frame Frame to transform from
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout,
  const rclcpp::Time stamp)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  global_pose.header.frame_id = robot_frame;
  global_pose.header.stamp = stamp;

  return transformPoseInTargetFrame(global_pose, global_pose, tf_buffer, global_frame, transform_timeout);
}
/**
* @brief get an arbitrary pose in a target frame
* @param input_pose Pose to transform
* @param transformed_pose Output transformation
* @param tf_buffer TF buffer to use for the transformation
* @param target_frame Frame to transform into
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose,
  tf2_ros::Buffer & tf_buffer, const std::string target_frame,
  const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose = tf_buffer.transform(
      input_pose, target_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str());
  }

  return false;
}
```
使用
```cpp
int main(){
    tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_->setUsingDedicatedThread(true);
    tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
    for(;;)
    {
        if (!getCurrentPose(current_pose, *tf_, "map", "base_link", 0.1))
        RCLCPP_ERROR(this->get_logger(), "Current robot pose is not available.\n\n\n");
    }
}

```
