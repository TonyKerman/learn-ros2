# microros_agent
## 安装
源码安装：[WTR_micro_ros_stm32cubemx_utils](https://github.com/WTR2023/WTR_micro_ros_stm32cubemx_utils)
snap安装： `sudo snap install micro-ros-agent`

## 命令行
```
Available arguments (per transport):
  * COMMON
    -h/--help.
    -m/--middleware <value> (ced, dds, rtps) [default: 'dds'].
    -r/--refs <value>.
    -v/--verbose <value> ( - ) [default: ''].
    -d/--discovery <value> [default: '7400'].
  * IPvX (udp4, udp6, tcp4, tcp6)
    -p/--port <value>.
  * SERIAL (serial, multiserial, pseudoterminal)
    -b/--baudrate <value> [default: '115200'].
    -D/--dev <value>.  * CAN FD (canfd)
    -D/--dev <value>.
```
### verbose 等级
默认 -v4
* -v5 显示接收/发送时间
* -v6 显示接收/发送时间及消息十六进制内容
## vscode 任务
```json
    "version": "2.0.0",
    "tasks": [
        {
            "label": "microRos agent",
            "type": "shell",
            "command": " micro-ros-agent serial --dev /dev/ttyACM0 -b 1800000",
            "options": {
                "cwd": "/home/wtr2023/ros_ws/microros_ws"
            },
            "presentation": {
                "echo": true,
                "reveal": "always",
                "focus": false,
                "panel": "dedicated",
                "showReuseMessage": true,
                "clear": true
            },
            "problemMatcher": []
        }
    ]
```