# 安装gazebo
目前gazebo+ros2有多种版本可以使用
## Gazebo-ROS2插件(Gazebo-classic)
教程来源: [fishros](https://fishros.com/d2lros2/#/humble/chapt9/get_started/3.%E5%9C%A8Gazebo%E5%8A%A0%E8%BD%BD%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%A8%A1%E5%9E%8B)

### 优点
1. 直接作为ros的一个package存在,可直接发送topic
2. 直接使用.xml文件,方便与urdf同时使用
### 缺点
1. 版本老,ui简陋
2. gazebo官方不再更新
3. 没有详细ros2教程

## gz sim
教程来源 [gazebo官方网站](https://gazebosim.org/docs/harmonic)
### 优点
1. 官方ros2教程
2. 版本更新支持
3. 可作为单独软件运行
### 缺点
1. 使用.sdf文件格式(与.xml大致类似)
2. 需要掌握相关 命令行命令
3. 使用`gazebo-ros-brige` 与ros2通信

因为 Gazebo-classic 没有详细教程,故使用 gz sim