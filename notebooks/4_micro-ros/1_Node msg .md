
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
其中 `multi_array_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 5);`
也可以改成
```
char array_buf[5];
multi_array_msg.data.data=array_buf;
```
或使用FreeRTOS 提供的`pvPortMalloc`

# Tips
1. 可以使用Freertos创建其他线程，但是不要在其中使用micro_ros的内容，包括使用publisher等，会卡死
2. 其他线程不运行,可能是microros任务占用了太多的内存导致其他任务初始化失败,方法:在cubeMX->FreeRTOS中增大TOTAL_HEAP_SIZE