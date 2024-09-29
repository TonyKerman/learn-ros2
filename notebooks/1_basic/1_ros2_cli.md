# rosdep 安装依赖
更推荐fishros的`rosdepc`
```bash
rosdep install -v --rosdistro=humble --from-paths src/package_name
```
前提是package.xml里有类似的行：
`<depend>rclpy</depend>`

# colcon 编译
```bash
colcon build --packages-select {pkg_name}
```
## 常用参数
* `--symbol-install`
* `--ament-cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1` 设置编译变量，配合`Clang`插件使用

# tf2 发布静态变换
```
ros2 run tf2_ros static_transform_publisher 0 0 3 0 0 3.14 parent child
```

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


