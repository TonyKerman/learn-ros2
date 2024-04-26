
[官方demo](https://github.com/micro-ROS/micro-ROS-demos)
# 创建rclc 节点

创建一个最小的rclc节点,由于没有使用executor,所以该节点只能使用 publisher

这是stm32 FreeRTOS环境下的rclc,使用esp32语法会有不同
```c
// include rclc
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
//include 使用的msgs
#include <std_msgs/msg/int32_multi_array.h>

//当本文件是.cpp文件时
#ifdef __cplusplus
extern "C"
{
#endif

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

//当本文件是.cpp文件时
#ifdef __cplusplus
}
#endif

// micro-ROS 节点线程
void UserStartDefaultTask(void *argument)
{
    // micro-ROS configuration
    if(rmw_uros_set_custom_transport(
            true,
            (void *) &huart6, //修改为通信串口
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read) == RMW_RET_ERROR)

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        //printf("Error on default allocators (line %d)\n", __LINE__);
    }
    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();
    //create init_options  Note:这里卡死，可能原因：没打开串口中断
    rclc_support_init(&support, 0, NULL, &allocator);
    // 创建 node
    rclc_node_init_default(&node, "stm32_node", "", &support);
    for(;;)
    {
        HAL_GPIO_TogglePin(LDG_GPIO_Port, LDG_Pin);
        osDelay(300);
    }
    
}

```
# rclc Msg 消息使用
消息分为两种 不带`MultiArray`的和带`MultiArray`的,初始化方法有所不同
示例
## Int32
不带`MultiArray`的msg,例如 `Int32` `Pose2d`等
使用方法比较方便
```c
//创建 
std_msgs__msg__Int32 msg;
//使用 
msg.data++;
```
## Int32MultiArray
带`MultiArray`的msg在使用前要手动分配内存
```c
// 创建 
std_msgs__msg__Int32MultiArray msg;
//rclc_publisher_init_default()放在这里
// 初始化 Int32MultiArray 消息
std_msgs__msg__Int32MultiArray__init(&multi_array_msg);
//为数组类消息分配内存
multi_array_msg.data.capacity = 5;
multi_array_msg.data.size = 5;
multi_array_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 5);
``` 
### 内存分配方式
其中 `multi_array_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 5);`
也可以改成
```
char array_buf[5];
multi_array_msg.data.data=array_buf;
```
或使用FreeRTOS 提供的`pvPortMalloc`
### capacity 和 size
.data.capacity 和.data.size是两个与数组管理相关的字段，它们有着不同的含义和用途：

#### .data.capacity：

    这个字段表示分配给.data.data数组的最大空间，即数组可以存储的最大元素数量。
    在动态数组中，capacity决定了内存分配的大小，它通常是在数组创建时设定的，或者在需要更多空间时增加。
    在这个例子中，.data.capacity = 5;意味着数组有足够的空间来存储5个int32_t类型的元素。
#### .data.size：

    这个字段表示数组中当前存储的元素数量。
    它反映了数组实际使用的大小，而不是最大容量。
    在这个例子中，.data.size = 5;表示数组中有5个元素被初始化，这与capacity相同，意味着数组已满。
    在实际使用中，.data.size应当始终不大于.data.capacity。如果尝试添加超过capacity的元素，则需要先增加capacity以避免内存越界。当从数组中移除元素时，size会减少，但capacity保持不变，除非显式地减少它来释放内存。

## 常用：JointState
### 消息定义
```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

### 分配内存
```c

    #define ARRAY_LEN 30 // String 数组长度
    #define JOINT_DOUBLE_LEN 10 //关节数量
	char joint_states_msg_buffer[ARRAY_LEN];
	joint_states_msg.header.frame_id.data = joint_states_msg_buffer;
	joint_states_msg.header.frame_id.size = 20;
	joint_states_msg.header.frame_id.capacity = ARRAY_LEN;

	rosidl_runtime_c__String string_buffer[JOINT_DOUBLE_LEN];
	joint_states_msg.name.data = string_buffer;
	joint_states_msg.name.size = 0;
	joint_states_msg.name.capacity = JOINT_DOUBLE_LEN;

	for(int i = 0; i < JOINT_DOUBLE_LEN; i++){
		joint_states_msg.name.data[i].data = (char*) malloc(ARRAY_LEN);
		joint_states_msg.name.data[i].size = 0;
		joint_states_msg.name.data[i].capacity = ARRAY_LEN;
	}

	double joint_states_position_buffer[JOINT_DOUBLE_LEN];
	joint_states_msg.position.data = joint_states_position_buffer;
	joint_states_msg.position.size= 7;
	joint_states_msg.position.capacity = JOINT_DOUBLE_LEN;

	double joint_states_velocity_buffer[JOINT_DOUBLE_LEN];	
	joint_states_msg.velocity.data = joint_states_velocity_buffer;
	joint_states_msg.velocity.size = 7;
	joint_states_msg.velocity.capacity = JOINT_DOUBLE_LEN;
	
	double joint_states_effort_buffer[JOINT_DOUBLE_LEN];	
	joint_states_msg.effort.data = joint_states_effort_buffer;
	joint_states_msg.effort.size = 7;
	joint_states_msg.effort.capacity = JOINT_DOUBLE_LEN;
```
# Tips
1. 可以使用Freertos创建其他线程，但是不要在其中使用micro_ros的内容，包括使用publisher等，会卡死
2. 其他线程不运行,可能是microros任务占用了太多的内存导致其他任务初始化失败,方法:在cubeMX->FreeRTOS中增大TOTAL_HEAP_SIZE