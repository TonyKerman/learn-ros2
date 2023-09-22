# launch:同时启动多个节点

## 使用python编写launch

### 1

在一个package下：

    mkdir launch
    touch launchname.py

### 2

编写launchname.py

```python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
# 定一个名字叫做generate_launch_description的函数，ROS2会对该函数名字做识别。
def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    action_robot_01 = Node(
        package="example_action_rclcpp",
        executable="action_robot_01"
    )
    action_control_01 = Node(
        package="example_action_rclcpp",
        executable="action_control_01"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [action_robot_01, action_control_01])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
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
