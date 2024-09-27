```xml
<!-- base link -->
<link name="base_link">
    <visual>
    <!--设置原点-->
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <!--设置形状-->
    <geometry>
        <!-- 长方体 -->
        <box size="1.0 1.0 1.0" />
        <!-- 球体 -->
        <sphere radius="1.0"/>
        <!-- 圆柱体 -->
        <cylinder length="1.0" radius="1.0"/>
        <!-- 导入模型 -->
        <mesh filename="package://robot_description/meshes/base_link.STL"/>
    </geometry>
    <!-- 设置材料 -->
    <material name="white">
        <color rgba="1.0 1.0 1.0 0.5" /> 
    </material>
    </visual>
</link>
```

# 关节

1. 固定关节 type="fixed" 不设置limit
2. 轮轴关节 type="continuous" 不设置limit
3. 旋转关节 type="revolute"
4. 平移关节 type="prismatic"
5. 平面移动关节
6. 球铰链关节
```xml
<joint name="joint1" type="revolute"> 
  <parent link="parent_link"/>
  <child link="child_link"/>
  <!--定义旋转轴-->
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 1"/>
  <!--设置最大扭矩，最小，最大范围，最大速度-->
  <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
</joint>
```