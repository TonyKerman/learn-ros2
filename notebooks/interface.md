# interface 接口

接口是ros2定义的一种规范

## ros2常见接口cli命令

查看接口列表

    ros2 interface list
查看接口详细内容

    ros2 interface show std_msgs/msg/String

## 接口形式

话题接口格式：xxx.msg

    int64 num

服务接口格式：xxx.srv

    int64 a
    int64 b
    ---
    int64 sum

动作接口格式：xxx.action

    int32 xxx #命令
    ---
    int32[] xxx #Result:结果
    ---
    int32[] xxx #Feedback:反馈

## 接口数据类型

根据引用方式不同可以分为基础类型和包装类型两类。

基础类型有（同时后面加上[]可形成数组）

    bool
    byte
    char
    float32,float64
    int8,uint8
    int16,uint16
    int32,uint32
    int64,uint64
    string

包装类型

即在已有的接口类型上进行包含，比如

    uint32 id
    string image_name
    sensor_msgs/Image

## 自定义接口

### 创建功能包

    ros2 pkg create {xxx_ros2_interfaces} --build-type ament_cmake --dependencies rosidl_default_generators --destination-directory src
    cd  src/{xxx_ros2_interfaces}
    mkdir msg action srv

(删除{})

注意功能包类型必须为：ament_cmake

依赖rosidl_default_generators必须添加，geometry_msgs视内容情况添加（我们这里有geometry_msgs/Pose pose所以要添加）。

接着创建文件夹和文件将3.2中文件写入，注意话题接口放到msg文件夹下，以.msg结尾。服务接口放到srv文件夹下，以srv结尾。

    .
    ├── CMakeLists.txt
    ├── msg
    │   ├── RobotPose.msg
    │   └── RobotStatus.msg
    ├── package.xml
    └── srv
        └── MoveRobot.srv

### 接着修改CMakeLists.txt

    find_package(rosidl_default_generators REQUIRED)
    find_package(geometry_msgs REQUIRED)
    #添加下面的内容
    rosidl_generate_interfaces(${PROJECT_NAME}
    #选择添加
    "msg/xxx.msg"
    "action/xxx.action"
    "srv/xxx.srv"
    # 当使用其他包中的数据类型时 DEPENDENCIES geometry_msgs
    )

### 接着修改package.xml

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rosidl_default_generators</depend>
    <depend>geometry_msgs</depend>
    
    <member_of_group>rosidl_interface_packages</member_of_group> #添加这一行

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

保存即可编译

    colcon build --packages-select example_ros2_interfaces

编译完成后在install/example_ros2_interfaces/include下你应该可以看到C++的头文件。
在install/example_ros2_interfaces/local/lib/python3.10/dist-packages下应该可以看到Python版本的头文件。

接下来的代码里我们就可以通过头文件导入和使用我们定义的接口了。