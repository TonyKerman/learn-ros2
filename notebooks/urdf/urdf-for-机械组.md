by TonyKerman 2676239430@qq.com
# URDF

一种构建可视化机器人模型的文件

使用xml语言编写
- [URDF](#urdf)
  - [快速查看模型](#快速查看模型)
  - [URDF的编写](#urdf的编写)
  - [两种关键组件(Joint\&Link)](#两种关键组件jointlink)
    - [Link：部件](#link部件)
      - [link的子标签列表](#link的子标签列表)
    - [Joint：关节](#joint关节)
  - [eg.创建图形](#eg创建图形)
  - [solidworks导出urdf](#solidworks导出urdf)
    - [下载sw插件](#下载sw插件)
    - [使用](#使用)
    - [Tips:](#tips)

## 快速查看模型

一个不用ros查看urdf的软件

[urdf-viewer](https://github.com/openrr/urdf-viz)（tonyKerman：没打开过）

[在线看](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/)条件：urdf文件不引用stl文件（从sw导入的看不了，自己写的urdf文件应该可以看）

## URDF的编写

一般情况下，URDF由声明信息和两种关键组件共同组成

声明信息包含两部分，第一部分是xml的声明信息，放在第一行 第二部分是机器人的声明，通过robot标签就可以声明一个机器人模型

```xml
<?xml version="1.0"?>
<robot name="fishbot">
    <link></link>
    ....
    <joint></joint>
......
</robot>
```

## 两种关键组件(Joint&Link)

### Link：部件

我们把左轮，右轮、支撑轮子，IMU和雷达部件称为机器人的Link

声明一个 Link
```xml
<link name="base_link">

</link>
```

通过两行代码就可以定义好base_link，但现在的base_link是空的，我们还要声明我们的base_link长什么样，通过visual子标签就可以声明出来机器人的visual形状

```xml
<!-- base link -->
<link name="base_link">
    <visual>
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <geometry>
        <cylinder length="0.12" radius="0.10"/>
    </geometry>
    </visual>
</link>
```

#### link的子标签列表

    <visual> 显示形状
    <geometry> (几何形状)
        <box> 长方体
        标签属性: size-长宽高
        举例：<box size="1 1 1" />

        <cylinder> 圆柱体
        标签属性:radius -半径 length-高度
        举例：<cylinder radius="1" length="0.5"/>

        <sphere> 球体
        属性：radius -半径
        举例：<sphere radius="0.015"/>

        <mesh> 第三方导出的模型文件
        属性：filename
        举例: <mesh filename="package://robot_description/meshes/base_link.DAE"/>
    
    <origin> (可选：默认在物体几何中心)
    属性 xyz默认为零矢量 rpy 弧度 表示的翻滚、俯仰、偏航
    举例：<origin xyz="0 0 0" rpy="0 0 0" />

    <material> 材质
    属性 name 名字

    <color>
    属性 rgba a代表透明度
    举例：<material name="white"><color rgba="1.0 1.0 1.0 0.5" /> </material>

    <collision> 碰撞属性，仿真章节中讲解
    <inertial> 惯性参数 质量等，仿真章节中讲解

### Joint：关节

而Link和Link之间的连接部分称之为Joint关节
joint为机器人关节，机器人关节用于连接两个机器人部件，主要写明父子关系

* 父子之间的连接类型，包括是否固定的，可以旋转的等
* 父部件名字
* 子部件名字
* 父子之间相对位置
* 父子之间的旋转轴，绕哪个轴转


## eg.创建图形

创建一个圆柱形

```xml
<?xml version="1.0"?>
<robot name="myfirst">
<link name="base_link">
    <visual>
    <geometry>
        <cylinder length="0.6" radius="0.2"/>
    </geometry>
    </visual>
</link>
</robot>
```

创建一个多link模型

```xml
<?xml version="1.0"?>
<robot name="bot">
    
  <!-- base link -->
  <link name="base_link">
      <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!--<cylinder length="0.12" radius="0.10"/> -->
        <box size="0.7 0.7 0.2"/>
      </geometry>
    </visual>
  </link>
    
  <!-- laser link -->
  <link name="laser_link">
      <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="black">
          <color rgba="0.0 0.0 0.0 0.5" /> 
      </material>
    </visual>
  </link>
    
  <!-- laser joint -->
    <joint name="laser_joint" type="fixed">
        <parent link="base_link" />
        <child link="laser_link" />
        <origin xyz="0.3 0 0.125" />
    </joint>

</robot>

```

## solidworks导出urdf

### 下载sw插件
[solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter)
下载按钮：页面右下角Releases下方
### 使用

[文字教程](https://www.guyuehome.com/44161)

[视频教程](https://www.bilibili.com/video/BV1Tx411o7rH/?share_source=copy_web&vd_source=83e4e90b05831aae4c03fbe4df8cd338)

### Tips:

1. 导出的时候起名不要有汉字，最好叫 "modelname",方便电控组之后修改

2. 导出的时候选"export urdf and meshes",导出后文件夹格式大致是
```
    -meshes
    -urdf
    ...
```
   

把meshes里xxx.stl移动到到urdf里，然后只需要把urdf文件夹打包发给电控组了

3. 选择link的时候，只需要选轮廓就行了，比如说底盘，把底盘的外框生成为一个link就行，越简单越好 
4. 只需要生成电控需要的部分，比如底盘（电控需要知道底盘的轮廓），传感器（需要知道传感器相对底盘的位置）作为child_link，部件越少越好
5. 为joint设置limit（旋转限位）,除了云台等必要旋转部件，其他部件可以把所有的设置为0（固定）
