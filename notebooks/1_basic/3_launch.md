# launch:同时启动多个节点

## 使用python编写launch
### 1 在一个package下：
```bash
    mkdir launch
    touch launchname.py
```
### 2 编写launchname.py

```python
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #配置package信息
    package_name = 'urdf_state_publisher'
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    #当你使用了一个urdf时
    urdf_name = "robot.urdf"
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    #运行一个命令行任务
    ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    #常用node
    #joint_state_publisher_gui 负责发布机器人关节数据信息，通过joint_states话题发布如果要自己控制机器人，需要发布关节数据，这个节点要改为自己的节点
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
    
    #启动
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
```

### 3编写 setup.py 将launch文件拷贝到安装目录

```python
from setuptools import setup
from glob import glob
import os

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    },
)
```


### 4运行

```bash
colcon build ,source install/setup.bash
ros2 launch robot_startup example_action.launch.py
```

## 进阶
[examples](../ros2_examples/launch_testing/)