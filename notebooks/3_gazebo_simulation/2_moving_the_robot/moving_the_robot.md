# Moving the robot

In this tutorial we will learn how to move our robot. We will use the
robot we built in the [Build your own robot](building_robot)
tutorial. You can download the robot from [here](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/building_robot/building_robot.sdf).
You can also find the finished world of this tutorial [here](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/moving_robot/moving_robot.sdf).

在本教程中，我们将学习如何移动机器人。我们将使用我们在构建您自己的机器人教程中构建的机器人。您可以从这里下载机器人。您还可以在这里找到本教程的完成世界。



## What is a plugin 什么是插件

To make our robot move we will use the `diff_drive` plugin. But before doing so let's answer the question "What is a plugin?" A plugin is a chunk of code that is compiled as a shared library and inserted into the simulation. Plugins make us control many aspects of the simulation like world, models, etc.

为了让我们的机器人移动，我们将使用 diff_drive 插件。但在此之前，让我们先回答“什么是插件？”这个问题。插件是被编译为共享库并插入到模拟中的一段代码。插件使我们能够控制模拟的许多方面，例如世界、模型等。

### Diff_drive plugin

`diff_drive` plugin helps us control our robot, specifically a robot that
can be differentially driven. Let's setup the plugin on our robot. Open
the `moving_the_robot.sdf` and add the following code within the `vehicle_blue`
model tags.

diff_drive 插件帮助我们控制我们的机器人，特别是可以差动驱动的机器人。让我们在机器人上设置插件。打开 building_robot.sdf 并在 vehicle_blue 模型标记中添加以下代码。

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

The `<plugin>` tag has two attributes, `filename` which takes the library file name and `name` which takes the name of the plugin.
In the `<left_joint>` and `<right_joint>` tags we define the joints which connect the left and the right wheel with the body of the robot, in our case `left_wheel_joint` and `right_wheel_joint`. `<wheel_separation>` takes the distance between the two wheels.
Our robot has its `left_wheel` at 0.6 m and the `right_wheel` at -0.6 m in y-axis with respect to the `chassis`, so the `wheel_separation` is 1.2 m.
`<wheel_radius>` takes the radius of the wheel which was defined in the `<radius>` tag under the wheel link.
`<odom_publish_frequency>` sets the frequency at which the odometry is published at `/model/vehicle_blue/odometry`.
`cmd_vel` is the input `<topic>` to the `DiffDrive` plugin.

<plugin> 标签有两个属性， filename 接受库文件名， name 接受插件名称。在 <left_joint> 和 <right_joint> 标签中，我们定义将左右轮与机器人身体连接的关节，在我们的例子中是 left_wheel_joint 和 right_wheel_joint 。 <wheel_separation> 取两个轮子之间的距离。我们的机器人的 left_wheel 位于 0.6 m 处， right_wheel 位于 y 轴上相对于 chassis 的 -0.6 m 处，因此 wheel_separation 获取轮子半径，该半径在轮子链接下的 <radius> 标签中定义。 <odom_publish_frequency> 设置在 /model/vehicle_blue/odometry 处发布里程计的频率。 cmd_vel 是 DiffDrive 插件的输入 <topic> 。

## Topics and Messages

Now our model is ready. We just need to send commands (messages) to it.
These messages will be published (sent) on the `cmd_vel` topic defined above.

现在我们的模型已经准备好了。我们只需要向它发送命令（消息）即可。这些消息将在上面定义的 cmd_vel 主题上发布（发送）。

A topic is simply a name for grouping a specific set of messages or a particular service.
Our model will subscribe (listen) to the messages sent on the `cmd_vel` topic.

主题只是用于对一组特定消息或特定服务进行分组的名称。我们的模型将订阅（侦听）在 cmd_vel 主题上发送的消息。

Launch the robot world:

启动机器人世界：

`gz sim moving_the_robot.sdf`

In another terminal let's send a message to to our robot:

在另一个终端中，我们向机器人发送一条消息：

`gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`

Now you should have your robot moving in the simulation.

现在您应该让机器人在模拟中移动。

**Note:** Don't forget to press the play button in the simulation.

注意：不要忘记在模拟中按播放按钮。

The command specifies the topic to publish to after the `-t` option.
After the `-m` we specify the message type.
Our robot expects messages of type `Twist` which consists of two components, `linear` and `angular`.
After the `-p` option we specify the content (value) of the message: linear speed `x: 0.5` and angular speed `z: 0.05`.

该命令在 -t 选项后指定要发布到的主题。在 -m 之后我们指定消息类型。我们的机器人需要 Twist 类型的消息，它由两个组件组成： linear 和 angular 。在 -p 选项之后，我们指定消息的内容（值）：线速度 x: 0.5 和角速度 z: 0.05 。

**Hint:** You can know what every topic option does using this command: `gz topic -h`

提示：您可以使用以下命令了解每个主题选项的作用： gz topic -h

For more information about `Topics` and `Messages` in Gazebo check the [Transport library tutorials](https://gazebosim.org/api/transport/9.0/tutorials.html)

有关 Gazebo 中的 Topics 和 Messages 的更多信息，请查看传输库教程

## Moving the robot using the keyboard

Instead of sending messages from the terminal we will send messages using the keyboard keys. To do so we will add two new plugins: `KeyPublisher` and `TriggeredPublisher`.

我们将使用键盘按键发送消息，而不是从终端发送消息。为此，我们将添加两个新插件： KeyPublisher 和 TriggeredPublisher 。

### KeyPublisher

`KeyPublisher` is an `gz-gui` plugin that reads the keyboard's keystrokes and sends them on a default topic `/keyboard/keypress`.
Let's try this plugin as follows:

KeyPublisher 是一个 gz-gui 插件，它读取键盘的击键并将其发送到默认主题 /keyboard/keypress 。让我们尝试一下这个插件，如下所示：

* In one terminal type 在一种终端类型中

    `gz sim moving_the_robot.sdf`

* In the top right corner click on the plugins dropdown list (vertical ellipsis), click the Key Publisher. 在右上角单击插件下拉列表（垂直省略号），单击发布者。

* In another terminal type 在另一种终端类型中
 
    `gz topic -e -t /keyboard/keypress`

The last command will display all messages sent on `/keyboard/keypress` topic.

最后一个命令将显示在 /keyboard/keypress 主题上发送的所有消息。

In the Gazebo window press different keys and you should see data (numbers) on the terminal where you run the `gz topic -e -t /keyboard/keypress` command.

在 Gazebo 窗口中按不同的键，您应该在运行 gz topic -e -t /keyboard/keypress 命令的终端上看到数据（数字）。

We want to map these keystrokes into messages of type `Twist` and publish them to the `/cmd_vel` topic which our model listens to.
The `TriggeredPublisher` plugin will do this.

我们希望将这些击键映射为 Twist 类型的消息，并将它们发布到我们的模型监听的 /cmd_vel 主题。 TriggeredPublisher 插件将执行此操作。

### Triggered Publisher

The `TriggeredPublisher` plugin publishes a user specified message on an output topic in response to an input message that matches user specified criteria.
Let's add the following code under the `<world>` tag:

TriggeredPublisher 插件在输出主题上发布用户指定的消息，以响应与用户指定条件匹配的输入消息。让我们在 <world> 标记下添加以下代码：

```xml
<!-- Moving Forward-->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

This code defines the `triggered-publisher` plugin.
It accepts messages of type `gz.msgs.Int32` on the `/keyboard/keypress` topic and if the value in the `data` field matches `16777235`(Up arrow key) it outputs a `Twist` message on the `cmd_vel` topic with values `x: 0.5`, `z: 0.0`.

此代码定义 triggered-publisher 插件。它接受 /keyboard/keypress 主题上 gz.msgs.Int32 类型的消息，如果 data 字段中的值与 16777235 匹配（向上箭头键），则它在 cmd_vel 主题上输出一条 Twist 消息，其值为 x: 0.5 、 z: 0.0 。

Now launch `moving_the_robot.sdf` then add the Key Publisher plugin and our robot should move forward as we press the Up arrow key &#8593; (make sure you start the simulation by pressing the play button to see the robot move forward after pressing the Up arrow key).

现在启动 building_robot.sdf 然后添加 Key Publisher 插件，当我们按下向上键 ↑ 时，我们的机器人应该向前移动（确保您通过按下播放按钮开始模拟，以在按下向上箭头键）。

There is a demo explaining how the [Triggered Publisher](https://github.com/gazebosim/gz-sim/blob/gz-sim7/tutorials/triggered_publisher.md) works.

有一个演示解释了触发发布器的工作原理。

### Moving using arrow keys

To see what values are sent on the `/keyboard/keypress` topic when pressing the arrows we can use the `--echo` or `-e` option

要查看按箭头时在 /keyboard/keypress 主题上发送的值，我们可以使用 --echo 或 -e 选项

* Run the model in one terminal: 在一个终端中运行模型：


    `gz sim moving_the_robot.sdf`

* In the top right corner click on the plugins dropdown list (vertical ellipsis), click the Key Publisher. 在右上角单击插件下拉列表（垂直省略号），单击密钥发布者。


* In another terminal run the following command: 在另一个终端中运行以下命令：

    `gz topic -e -t /keyboard/keypress`

Start pressing the arrows keys and see what values they give: 开始按箭头键并查看它们给出的值：


* Left &#8592;  : 16777234
* Up  &#8593;   : 16777235
* Right &#8594; : 16777236
* Down &#8595;  : 16777237

We will add the `Triggered publisher` plugin for each arrow key.
For example, the Down arrow:

我们将为每个箭头键添加 Triggered publisher 插件。例如，向下箭头：

```xml
<!-- Moving Backward-->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <input type="gz.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777237</match>
    </input>
    <output type="gz.msgs.Twist" topic="/cmd_vel">
        linear: {x: -0.5}, angular: {z: 0.0}
    </output>
</plugin>
```

Map each arrow (key stroke) with the desired message (movement) as we did with the backward arrow:

将每个箭头（击键）映射到所需的消息（移动），就像我们对向后箭头所做的那样：

* Left &#10142; 16777234 &#10142; linear: {x: 0.0}, angular: {z: 0.5}
* Up &#10142; 16777235 &#10142; linear: {x: 0.5}, angular: {z: 0.0}
* Right &#10142; 16777236 &#10142; linear: {x: 0.0}, angular: {z: -0.5}
* Down &#10142; 16777237 &#10142; linear: {x: -0.5}, angular: {z: 0.0}

Now it's your turn try to make the robot move using different keys.

现在轮到你尝试使用不同的按键让机器人移动。

