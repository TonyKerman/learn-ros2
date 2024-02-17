
# 使用publisher
## 1. 声明：
```c
rcl_publisher_t publisher;
//配套的msg
std_msgs__msg__Int32 msg;
```
## 2. 初始化，在`rclc_node_init_default`之后：
```c
rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "publish_name"
            );
```
其中，`ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32)`的作用是检测消息类型,其检测的消息类型名称格式不是类似`std_msgs__msg__Int32`的，而是ros2中消息本来的名称格式
## 3. 发布
### 直接发布（不常用）
```c
    if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK)
    {
        printf("Error publishing (line %d)\n", __LINE__);
    }
```
### 通过执行器发布
见执行器一节

# 使用subscriber
