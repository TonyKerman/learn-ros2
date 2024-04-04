[behaviortree doc](https://www.behaviortree.dev/docs/category/tutorials-basic)
[nav2:using_groot](https://navigation.ros.org/tutorials/docs/using_groot.html)
[nav2 bt nodes](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html)
## Groot2
[下载](https://www.behaviortree.dev/groot)
## 导入nav2 Bt节点
1. 在编辑器模式下打开 Groot。
2. 通过上下文菜单或菜单栏顶部中间的导入图标选择“从文件加载调色板”选项（文件夹形状图标）
3. 打开文件 /path/to/navigation2/nav2_behavior_tree/nav2_tree_nodes.xml 以导入用于导航的所有自定义行为树节点。这是 Nav2 自定义行为树节点的调色板。
## 给行为树blackboard添加常量
[ref](https://www.behaviortree.dev/docs/guides/scripting/)

### 使用xml
```xml
    <Script code=" msg:='hello world' " />
    <Script code=" A:=THE_ANSWER; B:=3.14; color:=RED " />
```
### Groot2
使用Ation->Script

## 具有通用类型的端口
[ref](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_03_generic_ports)
在前面的教程中，我们介绍了输入和输出端口，其中端口的类型是 
接下来，我们将展示如何将通用 C++ 类型分配给您的端口。
### 解析字符串
例如：
```cpp
// We want to use this custom type
struct Position2D 
{ 
  double x;
  double y; 
};
```
为了允许 XML 加载器从字符串实例化 Position2D ，我们需要提供 BT::convertFromString<Position2D>(StringView) 的模板特化。

Position2D 如何序列化为字符串由您决定；在这种情况下，我们只需用分号分隔两个数字。

```cpp
// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline Position2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            return output;
        }
    }
} // end namespace BT
```
### nav2 提供的解析方法

path:`navigation2\nav2_behavior_tree\include\nav2_behavior_tree\bt_utils.hpp`

nav2提供了以下几种消息的解析方法
* `geometry_msgs::msg::Point`
* `geometry_msgs::msg::Quaternion`
* `geometry_msgs::msg::PoseStamped`
* `std::chrono::milliseconds`
* `std::set<int>`
简而言之，只需要将各个元素用`;`分割就行

eg:
```cpp
template<>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 9) {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }
}
```