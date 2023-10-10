
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
其中，`ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32)`检测的消息类型不是类似`std_msgs__msg__Int32`的类型，而是ros2中消息本来的名称
## 3. 发布
### 直接发布（不常用）
```c
rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
        if (ret != RCL_RET_OK)
        {
            printf("Error publishing (line %d)\n", __LINE__);
        }
```
### 通过执行器发布
执行器见

