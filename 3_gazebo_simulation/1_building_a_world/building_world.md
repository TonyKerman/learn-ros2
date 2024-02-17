# Building your own robot

In this tutorial we will learn how to build our own robot in SDFormat. We will build a simple two wheeled robot.

在本教程中，我们将学习如何用 SDFormat 制作自己的机器人。我们将制作一个简单的双轮机器人。

 You can find the finished SDF file for the tutorial [here](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/building_robot/building_robot.sdf).
 您可以在 [此处](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/building_robot/building_robot.sdf) 找到本教程的 SDF 文件。

## What is SDF

[SDFormat](http://sdformat.org/) (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

[SDFormat](http://sdformat.org/)（仿真描述格式），有时缩写为 SDF，是一种 XML 格式，用于描述机器人仿真器、可视化和控制的对象和环境。

## Building a world

We will start by building a simple world and then build our robot in it. Open a new file called `building_robot.sdf` and copy the following code to it.
我们将首先构建一个简单的世界，然后在其中构建我们的机器人。打开一个名为 "building_robot.sdf "的新文件，并复制以下代码。

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```

Save the file, navigate to the directory where you saved the file and launch the simulator:

保存文件，导航到保存文件的目录并启动模拟器：

`gz sim building_robot.sdf`

**Note**: You can name your file any name and save it anywhere on your computer.

You should see an empty world with just a ground plane and a sun light. Check [World demo](sdf_worlds) to learn how to build your own world.

**注意**： 您可以将文件命名为任何名称，并保存在计算机上的任何位置。

您应该看到一个只有地平面和太阳光的空世界。查看 [世界演示](sdf_worlds) 了解如何构建自己的世界。
## Building a model

Under the `</model>` tag we will add our robot model as follows:

### Defining the model

```xml
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
```

Here we define the name of our model `vehicle_blue`, which should be a unique name among its siblings (other tags or models on the same level).
Each model may have one link designated as the `canonical_link`, the implicit frame of the model is attached to this link. If not defined, the first `<link>` will be chosen as the canonical link.
The `<pose>` tag is used to define the position and orientation of our model and the `relative_to` attribute is used to define the pose of the model relative to any other frame.
If `relative_to` is not defined, the model's `<pose>` will be relative to the world.

在这里，我们定义模型的名称 "vehicle_blue"，它应该是其同级（同级上的其他标记或模型）中唯一的名称。
每个模型都可以有一个指定为 `canonical_link` 的链接，该模型的隐式框架就附在这个链接上。如果没有定义，第一个 `<link>` 将被选为规范链接。
<pose>`标记用于定义模型的位置和方向，`relative_to`属性用于定义模型相对于其他帧的姿势。
如果没有定义 `relative_to` 属性，模型的 `<pose>` 将是相对于世界的。

Let's make our pose relative to the `world`. The values inside the pose tag are as follows: `<pose>X Y Z R P Y</pose>`, where the `X Y Z` represent the position of the frame and `R P Y` represent the orientation in roll pitch yaw.
We set them to zeros which makes the two frames (the model and the world) identical.


让我们把姿势设置为相对于 "世界"。姿势标签内的值如下： 其中，"X Y Z "代表帧的位置，"R P Y "代表滚动俯仰偏航的方向。
我们将它们设置为零，这样两个帧（模型和世界）就完全相同了。

## Links forming our robot

Every model is a group of `links` (can be just one link) connected together with `joints`.

每个模型都是一组用 "关节 "连接起来的 "链接"（可以只有一个链接）。

### Chassis

```xml
    <link name='chassis'>
        <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
```

We define the first link, the `chassis` of our car and it's pose relative to the `model`.

我们定义了第一个链接，即汽车的 "底盘 "和它相对于 "模型 "的姿态。

#### Inertial properties 惯性属性

```xml
    <inertial> <!--inertial properties of the link mass, inertia matix-->
        <mass>1.14395</mass>
        <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
        </inertia>
    </inertial>
```

Here we define the inertial properties of the chassis like the `<mass>` and the `<inertia>` matrix. The values of the inertia matrix for primitive shapes can be calculated using this [tool](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx).

在这里，我们定义了底盘的惯性属性，如"<质量>"和"<惯性>"矩阵。使用此 [工具](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx)可以计算原始形状的惯性矩阵值。


#### Visual and collision 可视化和碰撞

```xml
    <visual name='visual'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
        <!--let's add color to our link-->
        <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
        </material>
    </visual>
```

As the name suggests, the `<visual>` tag is responsible for how our link will look.
We define the shape of our link inside the `<geometry>` tag as a `<box>` (cuboid) and then specify the three dimensions (in meters) of this box inside the `<size>` tag.
Then, inside the `<material>` tag we define the material of our link.
Here we defined the `<ambient>`, `<diffuse>` and `<specular>` colors in a set of four numbers red/green/blue/alpha each in range [0, 1].

顾名思义，`<visual>`标记负责显示链接的外观。
我们在 `<geometry>` 标签中将链接的形状定义为一个 `<box>`（长方体），然后在 `<size>` 标签中指定这个盒子的三维尺寸（以米为单位）。
然后，我们在 `<material>` 标记中定义链接的材料。
在这里，我们用一组红/绿/蓝/α四个数字定义了 "环境"、"漫反射 "和 "镜面 "颜色，每个数字的范围为 [0, 1]。

```xml
        <collision name='collision'>
            <geometry>
                <box>
                    <size>2.0 1.0 0.5</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

The `<collision>` tag defines the collision properties of the link, how our link will react with other objects and the effect of the physics engine on it.

`<碰撞>`标签定义了链接的碰撞属性、链接与其他对象的反应以及物理引擎对链接的影响。

**Note**: `<collision>` can be different from the visual properties, for example, simpler collision models are often used to reduce computation time.

**注意**： 例如，为了减少计算时间，通常会使用更简单的碰撞模型。

After copying all the parts above into the world file in order, run the world again:

将上述所有部分按顺序复制到世界文件后，再次运行世界：

`gz sim building_robot.sdf`



In the top left toolbar, click the Translate icon, then select your model.
You should see three axes like this:

在左上角的工具栏中，单击 "Translate "图标，然后选择模型。
你应该会看到这样的三个轴：

These are the axes of our model where red is the x-axis, green is the y-axis and blue is the z-axis.

这些是我们模型的轴线，其中红色为 x 轴，绿色为 y 轴，蓝色为 z 轴。

### Left wheel

Let's add wheels to our robot. The following code goes after the `</link>` tag and before the `</model>` tag. All the links and joints belonging to the same model should be defined before the `</model>`.

让我们为机器人添加轮子。以下代码位于`</link>`标签之后、`</model>`标签之前。属于同一模型的所有链接和关节都应在 `</model>` 之前定义。

```xml
<link name='left_wheel'>
    <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
```

We defined the name of our link `left_wheel` and then defined its `<pose>` `relative_to` the `chassis` link.
The wheel needed to be placed on the left to the back of the `chassis` so that's why we chose the values for `pose` as `-0.5 0.6 0`.
Also, our wheel is a cylinder, but on its side.
That's why we defined the orientation value as `-1.5707 0 0` which is a `-90` degree rotation around the x-axis (the angles are in radians).
Then we defined the `inertial` properties of the wheel, the `mass` and the `inertia` matrix.

我们定义了链接的名称 `left_wheel`，然后定义了其`<pose>`与 `chassis `链接的 `relative_to `关系。
车轮需要放在 `底盘 `的左后方，因此我们选择的 `pose `值为`-0.5 0.6 0`。
另外，我们的车轮是一个圆柱体，但在侧面。
因此，我们将方向值定义为 `-1.5707 0 0`，即绕 x 轴旋转 `-90` 度（角度单位为弧度）。
然后，我们定义了车轮的 `惯性`属性、`质量`和 `惯性`矩阵。

#### Visualization and Collision

```xml
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

The `<visual>` and  the `<collision>` properties are similar to the previous link, except the shape of our link has the shape of `<cylinder>` that requires two attributes: the `<radius>` and the `<length>` of the cylinder.
Save the file and run the world again, our model should look like this:

`<视觉>`属性和`<碰撞>`属性与之前的链接类似，只是我们的链接形状为`<圆柱体>`，需要两个属性：圆柱体的`<半径>`和`<长度>`。
保存文件并再次运行世界，我们的模型应该是这样的：

### Right wheel

```xml
<!--The same as left wheel but with different position-->
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose> <!--angles are in radian-->
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

The right wheel is similar to the left wheel except for its position.

右轮与左轮相似，只是位置不同。

### Defining an arbitrary frame 定义任意框架

As of SDF 1.7 (Fortress uses SDF 1.8), we can define arbitrary frames. It takes two attributes:

从 SDF 1.7（Fortress 使用 SDF 1.8）开始，我们可以定义任意框架。它有两个属性

* `name`: the name of the frame
* `attached_to`: the name of the frame or the link to which this frame is attached.

* `name`: 框架名称
* `attached_to`: 框架名称或该框架所连接的链接。


Let's add a frame for our caster wheel as follows:

让我们为脚轮添加一个框架，如下所示：

```xml
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

We gave our frame name `caster_frame` and attached it to the `chassis` link, then the `<pose>` tag to define the position and orientation of the frame.
We didn't use the `relative_to` attribute so the pose is with respect to the frame named in the `attached_to` attribute, `chassis` in our case.

我们将框架命名为 `caster_frame`，并将其连接到 `chassis`链接，然后使用`<pose>`标记来定义框架的位置和方向。
我们没有使用 `relative_to`属性，因此姿势是相对于 `attached_to`属性中命名的框架而言的，在我们的例子中是 `chassis`

### Caster wheel

```xml
<!--caster wheel-->
<link name='caster'>
    <pose relative_to='caster_frame'/>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016</iyy>
            <iyz>0</iyz>
            <izz>0.016</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
        <material>
            <ambient>0.0 1 0.0 1</ambient>
            <diffuse>0.0 1 0.0 1</diffuse>
            <specular>0.0 1 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
    </collision>
</link>
```

Our last link is the `caster` and its pose is with respect to the frame `caster_frame` we defined above.
As you could notice we closed the `pose` tag without defining the position or the orientation; in this case the pose of the link is the same as (identity) the frame in `relative_to`.

我们的最后一个链接是 `caster`，它的姿态是相对于我们上面定义的帧 `caster_frame `而言的。
正如你所注意到的，我们关闭了 `pose` 标签，但没有定义位置或方向；在这种情况下，链接的姿势与 `relative_to` 中的帧相同。

In the `<visual>` and `<collision>` tags we defined a different shape `<sphere>` which requires the `<radius>` of the sphere.

在 `<visual>` 和 `<collision>` 标记中，我们定义了一个不同的形状 `<sphere>`，它需要球体的 `<radius>`。

### Connecting links together (joints)

We need to connect these links together; here comes the job of the `<joint>` tag.
The joint tag connects two links together and defines how they will move with respect to each other.
Inside the `<joint>` tag we need to define the two links to connect and their relations (way of movement).

我们需要将这些链接连接在一起；这就是`<joint>`"标记的工作。
连接标签将两个链接连接在一起，并定义它们如何相对移动。
在 `<joint>` 标记中，我们需要定义要连接的两个链接及其关系（运动方式）。


#### Left wheel joint

```xml
<joint name='left_wheel_joint' type='revolute'>
    <pose relative_to='left_wheel'/>
```

Our first joint is the `left_wheel_joint`.
It takes two attributes: the name `name='left_wheel_joint'` and the type `type='revolute'`.
the `revolute` type gives 1 rotational degree of freedom with joint limits.
The pose of the joint is the same as the child link frame, which is the `left_wheel` frame.

我们的第一个关节是 `left_wheel_joint`。
它有两个属性：名称 `name='left_wheel_joint'` 和类型 `type='revolute'` 。
revolute "类型提供 1 个具有关节限制的旋转自由度。
关节的姿态与子链接框架（即 `left_wheel` 框架）相同。

```xml
    <parent>chassis</parent>
    <child>left_wheel</child>
```

Every joint connects two links (bodies) together.
Here we connect the `chassis` with the `left_wheel`.
`chassis` is the parent link and `left_wheel` is the child link.

每个关节都将两个链接（车身）连接在一起。
在这里，我们将 `chassis` 与 `left_wheel` 连接起来。
底盘"是父链接，"左轮 "是子链接。

```xml
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz> <!--can be defined as any frame or even arbitrary frames-->
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

Here we define the axis of rotation.
The axis of rotation can be any frame, not just the `parent` or the `child` link.
We chose the y-axis with respect to the `model` frame so we put `1` in the y element and zeros in the others.
For the revolute joint we need to define the `<limits>` of our rotation angle in the `<lower>` and `<upper>` tags.

这里我们定义旋转轴。
旋转轴可以是任何框架，而不仅仅是 "父 "或 "子 "链接。
我们选择了相对于 "模型 "框架的 y 轴，因此在 y 元素中输入 "1"，在其他元素中输入 0。
对于旋转接头，我们需要在 `<lower>` 和 `<upper>` 标记中定义旋转角度的 `<limits>`。

**Note**: The angles are in radians.
**注意**： 角度以弧度为单位。
#### Right wheel joint

The `right_wheel_joint` is very similar except for the pose of the joint.
This joint connects the `right_wheel` with the `chassis`.

右车轮连接 "与 "底盘 "连接的姿势非常相似。
该关节将`右轮`与`底盘`连接起来。

```xml
<joint name='right_wheel_joint' type='revolute'>
    <pose relative_to='right_wheel'/>
    <parent>chassis</parent>
    <child>right_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>    <!--negative infinity-->
            <upper>1.79769e+308</upper>     <!--positive infinity-->
        </limit>
    </axis>
</joint>
```

#### Caster wheel joint

For the caster we need a different type of joint (connection).
We used `type='ball'` which gives 3 rotational degrees of freedom.

对于脚轮，我们需要不同类型的关节（连接）。
我们使用了`type='ball'`，它提供了 3 个旋转自由度。
```xml
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
```

## Conclusion

Run the world:

`gz sim building_robot.sdf`


Hurray! We build our first robot. You can learn more details about SDFormat tags [here](http://sdformat.org/spec). In the next [tutorial](moving_robot) we will learn how to move our robot around.

万岁 我们造出了第一个机器人。有关 SDFormat 标签的更多详情，请参阅 [此处](http://sdformat.org/spec)。在下一个 [教程]（moving_robot）中，我们将学习如何移动我们的机器人。
