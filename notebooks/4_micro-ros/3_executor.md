# 执行器
相当于rclpy中的spin()负责完成具体的执行动作
## 使用 executor
### 1. 声明
```c
rclc_executor_t executor;
```
### 2.初始化
注意,执行器的初始化要在publisher,subscriber,timer等初始化后进行
```c
rclc_executor_t executor;
executor = rclc_executor_get_zero_initialized_executor();
unsigned int num_handles = 1 + 1;
rclc_executor_init(&executor, &support.context, num_handles, &allocator);
//将一个subscriber添加到excutor
rclc_executor_add_subscription(&executor, &my_sub, &sub_msg, &my_subscriber_callback, ON_NEW_DATA);
```
其中`num_handles`是`timer`和`subscriber`的总数

选项 `ON_NEW_DATA` 选择spin-method的执行方式.在此示例中,仅当有新数据可用时才会调用订阅 my_sub 的回调.另一种执行语义是 `ALWAYS` ,这意味着,当执行器的 `spin-method` 被调用时,订阅回调总是被执行.如果回调应以固定速率执行,无论新数据是否可用,此选项可能很有用.如果您选择此选项,则如果没有新数据可用,将使用消息参数 `NULL` 执行回调.因此,您需要确保您的回调也接受 `NULL` 作为消息参数.

也可以添加一个计时器`rclc_executor_add_timer(&executor, &my_timer);`
### 3.spin

`rclc_executor_spin(&executor);`该函数永远运行而不返回.

如果只想执行10次可以这样:
```c
for (unsigned int i = 0; i < 10; i++) {
  // timeout specified in nanoseconds (here 1s)
  rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
}
```

## 影响反馈时延的主要因素

### 时延测试:
上位机定时发布带timestamp的控制消息,下位机接受消息并在定时器中发布反馈消息,其中含有最后一次控制消息的timestamp
### 实验
1. 上位机周期10ms,下位机10ms:
    
    0
    10
    0
    10
    0
    ...

2. 上位机周期5ms,下位机11ms:

    5
    5
    5
    5
    ...
3. 上位机周期30ms,下位机10ms:

    0
    0
    30
    0
    0
    30

### 结论
1. 上位机周期大于下位机周期时,会出现间隔延迟
2. 上位机周期 约<0.5 下位机周期,时延约=上位机周期
3. 反馈周期非常大时,考虑下位机其他线程时间片占用过多,或microros线程优先级太低
