# 创建一个工作区
# 创建一个节点

## 1. 在工作区根目录下 

        cd src/

## 2. 创建节点

        ros2 pkg create  minalnode --build-type ament_cmake

一般来说，要添加依赖

对于cpp节点

        ros2 pkg create minalnode --build-type ament_cmake --dependencies rclcpp
        
对于py节点 rclcpp改为rclpy

## 3. 创建编写源文件

在minalnode下

        cd/src
        touch node01.cpp
编写cpp文件

[node01.cpp](../src/learning_node/minalnode/src/node01.cpp)


## 4.修改CMakeListstxt
 
[CMakeListstxt](../src/learning_node/minalnode/CMakeLists.txt)

        find_package(rclcpp REQUIRED) #对应了创建节点时 --dependencies rclcpp
        add_executable(firstnode src/code.cpp) #添加执行入口
        ament_target_dependencies(node01 rclcpp) #找rclcpp
        install(TARGETS
        node01
        DESTINATION lib/${PROJECT_NAME}
        )

## 5. 编译并运行

        colcon build --packages-select minalnode
        source install/setup.bash
        ros2 run minalnode node01


# write and run in python 写代码并构建运行 py

1. create a package

        cd ros2_ws/src
        ros2 pkg create first_node_py  --build-type ament_python --dependencies rclpy 

2. example steps of creating a node:
     1. 导入库文件
     2. 初始化客户端库
     3. 新建节点
     4. spin循环节点
     5. 关闭客户端库 
 
3. edit first_node_py/first_node_py/code.py
4. edit setup.py 添加入口点

         entry_points={
            'console_scripts': [
        >>        "node02 = first_node_py.code:main"
            ],
        }, 
5. build and run 构建执行

        colcon build
        source install/setup.bash #执行一次就行
        ros2 run first_node_py node02
