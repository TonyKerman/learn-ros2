# debug rclcpp 单个节点

## 1 .vscode/launch.json
```json
        "version": "0.2.0",
        "configurations": [
            {
                "name": "C++ Debugger",
                "request": "launch",
                "type": "cppdbg",
                "miDebuggerServerAddress": "localhost:3000",
                "cwd": "/",
                "program": "/home/wtr2023/ros_ws/rc2024/RC24_R2V2/build/package_name/executable_name"
            }
        ]
```

## 2 运行节点

* 使用 ros2 run 运行单个节点debug
```bash
ros2 run --prefix 'gdbserver localhost:3000' package_name executable_name
```
* 或者在launch.py里增加参数 `prefix`
```py
    r2_bt_commander =Node(
        package='r2v2_bt_commander',
        executable='bt_commander',
        name='bt_commander',
        output='screen',
        prefix=["gdbserver localhost:3000"],
        emulate_tty=True,
        parameters=[nav2_params]
    )
```
## 3 debug
首先运行节点,然后使用vscode debug