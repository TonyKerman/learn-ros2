
# 一键安装
```bash
wget http://fishros.com/install -O fishros && . fishros
```
先安装ros2，再安装rosdepc，再执行
```
rosdepc init
rosdecp update
```
完成rosdepc安装

# 常见安装问题

## RVIZ2黑屏问题
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

## VsCode终端无法运行rqt,rviz

### 方法一
如果VsCode是通过snap安装的，删除VsCode并通过官网下载.deb安装
`sudo snap remove code`
<!-- ### 方法二
不使用VsCode集成终端，使用外部终端
1. 更改设置`Features->Terminal->Explorer Kind`:external
2. 更改设置`Features->Terminal->Linux Exec`: gnome-terminal
3. 可以右键`vscode`中的文件夹打开外部终端，或者使用快捷键`shift+ctrl+c` -->
## rviz2 报错：RenderingAPIException: Invalid parentWindowHandle (....)
qt问题，要设置环境变量
1. 在`.bahsrc`添加`export QT_QPA_PLATFORM xcb`
2. 在VsCode设置`Terminal->Integrated->Env:Linux`添加一行 "QT_QPA_PLATFORM": "xcb"

# Install ROS2 with docker

1. install docker via apt
    <https://www.runoob.com/docker/ubuntu-docker-install.html>
then
    sudo groupadd docker
    sudo gpasswd -a ${USER} docker
    newgrp docker
    sudo service docker restart
1. set a workspace

2. pull  ros2 image

3. install docker-compose
    sudo apt install docker-compose
4. write a Dockerfile and docker-compose.yml

[Dockerfile](../Dockerfile)

[docker-compose.yml](../docker-compose.yml)

* [https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/]
* [https://imhuwq.com/2018/12/02/Clion%20%E4%BD%BF%E7%94%A8%20Docker%20%E4%BD%9C%E4%B8%BA%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83/]
* [https://zhuanlan.zhihu.com/p/520752548]