注意,这是stm32 FreeRTOS环境下的rclc,使用esp32语法会有不同
# 创建rclc 节点
```c
//main.c
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
//include msgs
#include <std_msgs/msg/int32_multi_array.h>
/* USER CODE END Includes */
...
/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
void UserStartDefaultTask(void *argument)
{
    // micro-ROS configuration
    if(rmw_uros_set_custom_transport(
            true,
            (void *) &huart6,
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
        printf("Error on default allocators (line %d)\n", __LINE__);

    }
    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();
    //create init_options  Note:这里卡死，可能原因：没打开串口中断
    rclc_support_init(&support, 0, NULL, &allocator);
    // create node
    rclc_node_init_default(&node, "stm32_node", "", &support);
    for(;;)
    {
        HAL_GPIO_TogglePin(LDG_GPIO_Port, LDG_Pin);
        osDelay(300);
    }
    /* USER CODE END 5 */
}
/* USER CODE END 4 */

```
# rclc Msg消息
这里是使用std_msg的示例
## Int32
```c
//创建 
std_msgs__msg__Int32 msg;
//使用 
msg.data++;
```

## Int32MultiArray
```c
// 创建 
std_msgs__msg__Int32MultiArray msg;
//rclc_publisher_init_default()放在这里
// 初始化 Int32MultiArray 消息
std_msgs__msg__Int32MultiArray__init(&multi_array_msg);

// 设置 Int32MultiArray 消息的数据
multi_array_msg.data.capacity = 5;
multi_array_msg.data.size = 5;
multi_array_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * 5);
multi_array_msg.data.data[0] = 1;
multi_array_msg.data.data[1] = 2;
multi_array_msg.data.data[2] = 3;
multi_array_msg.data.data[3] = 4;
multi_array_msg.data.data[4] = 5;
``` 