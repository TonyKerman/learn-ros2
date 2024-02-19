# 控制urdf关节转动

原理：发送关节位姿给robot_state_pubsher，robot_state_publisher发送tf控制机器人的关节转动。

demo：learnROS2/src/urdf_state_publisher

## 创建pkg

```bash
ros2 pkg create {example_urdf_description} --build-type ament_python --destination-directory src --dependencies rclpy
cd src/{example_urdf_description}
mkdir urdf launch
touch urdf/robot.urdf launch/display_rviz2.launch.py
```

## 编写urdf

{pkg_name}/urdf/robot.urdf
```xml  
...
  <!--注意关节名，以后要用-->
  <joint name="left_wheel_joint" type="continuous">
      <parent link="base_link" />
      <child link="left_wheel_link" />
      <origin xyz="-0.02 0.10 -0.06" />
      <axis xyz="0 1 0" />
  </joint>

```

## 编写launch

{pkg_name}/launch/mv_wheel.launch.py

```python
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #注意要改
    package_name = 'urdf_state_publisher'
    urdf_name = "robot.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    #自己控制机器人，需要发布关节数据，这个节点要改为自己的节点
    joint_state_publisher_node = Node(
        package='urdf_state_publisher',
        executable='state_publisher',
        #name='state_publisher',
        arguments=[urdf_model_path]
        )
    #robot_state_publisher_node负责发布机器人模型信息robot_description，并将joint_states数据转换tf信息发布
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
    #rviz2_node负责显示机器人的信息
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld

```

## 编写setup.py
```python
from glob import glob
import os
...
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
...
'console_scripts': [
            "state_publisher= urdf_state_publisher.state_publisher:main"
```

## 编写 state_publish.py

joint_states有一个头和四个数组需要赋值（可通过ros2 interface指令查询）

    # 这是一个持有数据的信息，用于描述一组扭矩控制的关节的状态。
    #
    # 每个关节（渐进式或棱柱式）的状态由以下因素定义。
    # #关节的位置（rad或m）。
    # #关节的速度（弧度/秒或米/秒）和
    # #在关节上施加的力（Nm或N）。
    #
    # 每个关节都由其名称来唯一标识
    # 头部规定了记录关节状态的时间。所有的联合状态
    # 必须是在同一时间记录的。
    #
    # 这个消息由多个数组组成，每个部分的联合状态都有一个数组。
    # 目标是让每个字段都是可选的。例如，当你的关节没有
    # 扭矩与它们相关，你可以让扭矩数组为空。
    #
    # 这个信息中的所有数组都应该有相同的大小，或者为空。
    # 这是唯一能将关节名称与正确的
    # 状态。
    std_msgs/Header header #时间戳信息 和 frame_id
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort

其中 stdmsg/msg/Header

    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data
    # in a particular coordinate frame.

    # Two-integer timestamp that is expressed as seconds and nanoseconds.
    builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec

    # Transform frame with which this data is associated.
    string frame_id

## 运行