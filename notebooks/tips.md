# 创建功能包

## eg1:创建一个包含 action 的包
```bash
cd {WORKSPACE}/
ros2 pkg create use_my_action --build-type ament_python --dependencies rclpy my_action_interfaces --destination-directory src --node-name action_robot_02 --maintainer-name "TonyKerman" --maintainer-email "2676239430@qq.com"
# 手动再创建action_control_02节点文件
touch src/use_my_action/use_my_action/action_control_02.py
```


## eg2:创建一个自定义接口功能包
```bash
ros2 pkg create example_ros2_interfaces --build-type ament_cmake --dependencies rosidl_default_generators --destination-directory src 
```


## usage: ros2 pkg create

[-h] [--package-format {2,3}] [--description DESCRIPTION] [--license LICENSE] [--destination-directory DESTINATION_DIRECTORY] [--build-type {cmake,ament_cmake,ament_python}] [--dependencies DEPENDENCIES [DEPENDENCIES ...]] [--maintainer-email MAINTAINER_EMAIL] [--maintainer-name MAINTAINER_NAME] [--node-name NODE_NAME] [--library-name LIBRARY_NAME] package_name

Create a new ROS 2 package

### positional arguments 参数

package_name          The package name

#### options

-

    -h, --help            

show this help message and exit

-

    --destination-directory DESTINATION_DIRECTORY

Directory where to create the package directory 创建包的位置，一般在工作区/src

-

    --build-type {cmake,ament_cmake,ament_python}

The build type to process the package with 包类型

-

    --dependencies DEPENDENCIES [DEPENDENCIES ...]

list of dependencies 依赖

-

    --node-name NODE_NAME

name of the empty executable 初始化一个节点，具体是修改setup.py和新建一个python文件

不太重要的：

- --maintainer-email MAINTAINER_EMAIL

email address of the maintainer of this package

- --maintainer-name MAINTAINER_NAME

name of the maintainer of this package

- --package-format {2,3}, --package_format {2,3}
                        The package.xml format.

- --description DESCRIPTION
                        The description given in the package.xml

- --license LICENSE     The license attached to this package; this can be an arbitrary string, but a LICENSE file will only be generated if
                        it is one of the supported licenses (pass '?' to get a list)

- --library-name LIBRARY_NAME
                        name of the empty library

# colcon 编译
```bash
colcon build --packages-select {pkg_name}
```

# ros2 cli 

## topic
### 查看目前的topic
```bash
ros2 topic list
```
或用rqt查看

### 查看topic来源
```bash
ros2 topic info /joint_states
```
返回：

    Type: sensor_msgs/msg/JointState
    Publisher count: 1
    Subscription count: 1

### 查看topic-msg 内容
```bash
ros2 interfaces show sensor_msgs/msg/JointState
```
返回

    std_msgs/Header header

    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
##