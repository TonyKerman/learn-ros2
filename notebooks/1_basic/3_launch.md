# launch:同时启动多个节点

## launch基础
### 1 在一个package下：
```bash
    mkdir launch
    touch launchname.py
```
### 2 编写xxx.launch.py

```python
import os
from launch import LaunchDescription
#from launch.substitutions import LaunchConfiguration
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

### 3 编写 setup.py 将launch文件拷贝到安装目录

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
## launch.py 文件详解

文件一般分为4部分：
1. import部分 
2. 配置信息部分：配置相关路径，声明参数等参数
3. 声明部分：声明要启动的节点，命令行任务，子launch文件等
4. 启动部分：将刚声明的加入启动序列
```python
#import部分
import os
...
#import部分

def generate_launch_description():
    ld = LaunchDescription()
    #配置部分
    package_name = 'urdf_state_publisher'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    ...
    #配置部分

    #声明部分
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        )
    #声明部分

    #启动部分
    ld.add_action(rviz2_node)
    return ld
    #启动部分
```
### 必要的import
```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node # 用于启动Node
from ament_index_python.packages import get_package_share_directory # 用于找到本package的路径
#from launch_ros.substitutions import FindPackageShare # 和上面那一个功能相同
```
其他 import 在下面和其他部分一起介绍

### 配置本package名称，路径

```py
#import部分
from ament_index_python.packages import get_package_share_directory
#配置部分
    pkg_dir = get_package_share_directory('pkg_name')
```
或者
```py
#import部分
from launch_ros.substitutions import FindPackageShare
#配置部分
    pkg_share = FindPackageShare(package='pkg_name').find('pkg_name')
```
### 配置文件路径
配置要用的 map,urdf,config 等文件的路径

常用有：
配置部分
* 导入map`map_yaml_file = os.path.join(pkg1_dir, 'maps', 'map.yaml')`
* 导入urdf模型`default_model_path = os.path.join(pkg2_dir, 'urdf','xxx.urdf')` 
* 导入rviz配置文件`default_rviz_config_path = os.path.join(pkg3_dir, 'rviz','urdf_config.rviz')` 
...
**别忘了在setup.py中包含文件夹**
### 配置命令行参数 
我们在命令行中启动launch文件时，可以使用参数
```bash
ros2 launch pkg_name xxx.launch.py  param1:=False param2:=xxx.urdf
```
我们要在launch文件中声明参数才可以使用
#### 例程
假如要加入一个控制是否启动rviz的参数
```py
#import部分

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
#声明部分
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
#启动部分
    ld.add_action(declare_use_rviz_cmd)
```
我们可以在rviz_node中添加启动条件
```py
#import部分
from launch.conditions import IfCondition
#声明部分
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')) 
    )
#启动部分
    ld.add_action(rviz_node)
```
`LaunchConfiguration('use_rviz')`根据`declare_use_rviz_cmd`,默认为`True`
### 启动一个Node
launch最常用的就是启动一个Node
```py
#import部分
from launch_ros.actions import Node
#声明部分
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')) 
    )
#启动部分
    ld.add_action(rviz_node)
```
参数分析
取自源码
#### `package` 可以找到节点可执行文件的软件包
#### `executable` 可执行文件的名称
如果提供了软件包，则指要查找的可执行文件的名称。
则为可执行文件的名称，否则为要运行的可执行文件的路径。
#### `name` 节点名称
在rqt,log中显示的节点名，如果未给出名称（或名称为 None），则在创建节点时不会将名称传递给
节点的代码中指定的默认名称。
#### `namespace` 该节点的 ROS 命名空间
可以是绝对的（即以 / 开头），也可以是相对的。
相对命名空间。
如果是绝对命名空间，则不考虑其他因素，直接将其传递给节点，以设置命名空间。
直接传递给节点来设置命名空间。
如果是相对的，"ros_namespace "LaunchConfiguration 中的命名空间
中的命名空间将被预输入给定的相对节点命名空间。
如果没有给定命名空间，则默认命名空间`/`将被假定为
将被默认。
#### `exec_name` 用来表示进程的标签。
        默认为节点可执行文件的基名。
#### `parameters` 含有参数规则的 yaml 文件名列表、或参数字典。
#### `remappings` 重映射规则
有序的 "to "和 "from "字符串对列表。作为 ROS 重映射规则传递给节点的 "to "和 "from "字符串对的有序列表
#### `ros_arguments` 节点的 ROS 参数列表
#### `arguments` 节点的额外参数列表
因节点而异 
eg:
```py
    rviz_node = launch_ros.actions.Node(
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
```

* 参数以列表形式传递，每个元素都是一个 yaml
文件（字符串或 pathlib.Path 文件的完整
字符串或 pathlib.Path 文件的完整路径），或指定参数规则的字典。
字典的键可以是字符串，也可以是将展开为字符串的
的迭代。
字典中的值可以是字符串、整数、浮点数或将展开为字符串的
的元组。
此外，字典中的值可以是上述类型的列表
类型的列表，或具有相同属性的另一个字典。
包含字典中生成的参数的 yaml 文件将被写入一个临时文件，路径为
写入一个临时文件，其路径将传递给
节点。
可以传递多个参数字典/文件：每个文件路径
将依次传递到节点（最后一个参数定义在节点上生效）。
参数的最后一个定义生效）。
不过，即使在前面指定了节点名，节点名的完全限定也会优先于通配符。
在前面指定。
如果未指定 "命名空间"，字典将以
通配符名称空间 (`/**`)，其他特定参数声明
可能会覆盖它。

### launch执行命令行任务
我们可以在launch中执行命令行任务,例如：打开`gazebo`
```py
#import部分
from launch.actions import ExecuteProcess
#声明部分
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world], 
        cwd=[warehouse_dir],
        output='screen',
    )
#启动部分
    ld.add_action(start_gazebo_server_cmd)
```
* `cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world]`逗号相当于空格
* `cwd`:在哪执行命令
### 执行子launch
launch可以嵌套

## 进阶
[examples](../ros2_examples/launch_testing/)