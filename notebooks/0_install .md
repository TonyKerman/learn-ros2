# ROS2 Study Note

## Install ROS2 with docker

1. install docker via apt
    <https://www.runoob.com/docker/ubuntu-docker-install.html>
then
    sudo groupadd docker
    sudo gpasswd -a ${USER} docker
    newgrp docker
    sudo service docker restart
2. set a workspace

3. pull  ros2 image

4. install docker-compose
    sudo apt install docker-compose
5. write a Dockerfile and docker-compose.yml

[Dockerfile](../Dockerfile)

[docker-compose.yml](../docker-compose.yml)

* [https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/]
* [https://imhuwq.com/2018/12/02/Clion%20%E4%BD%BF%E7%94%A8%20Docker%20%E4%BD%9C%E4%B8%BA%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83/]
* [https://zhuanlan.zhihu.com/p/520752548]

# Install micro ros

## 1.配置环境

micro_ros 官方教程地址：[Overview](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/)

主要内容是：

First micro-ROS application on Linux 链接

First micro-ROS Application on FreeRTOS 链接

这个只是通用教程，可以先看看。把 micro_ros_agent 安装好
(跳过Configuring the firmware部分)

## 2.micro_ros_stm32cubemx_utils

https://github.com/micro-ROS/micro_ros_stm32cubemx_utils

# 解决RVIZ2黑屏问题
[https://github.com/ros2/rviz/issues/948]

    add-apt-repository ppa:kisak/kisak-mesa && apt install -y mesa-utils && glxgears

其中 glxgears 是测试3d功能的，不安装也行

如果提示add-apt-repository commond not found
    
    sudo apt install --reinstall software-properties-common

PPA换源：

    gedit sources.list.d/kisak-ubuntu-kisak-mesa-jammy.list

    deb https://launchpad.proxy.ustclug.org/kisak/kisak-mesa/ubuntu/ jammy main
    # deb-src https://ppa.launchpadcontent.net/kisak/kisak-mesa/ubuntu/ jammy main

    apt update -y && apt upgrade