# microros_agent
## 安装
源码安装：[WTR_micro_ros_stm32cubemx_utils](https://github.com/WTR2023/WTR_micro_ros_stm32cubemx_utils)
snap安装： `sudo snap install micro-ros-agent`
## 说明 (service mode)

  Micro-ROS, whose default implementation is based on eProsima's
  Micro XRCE-DDS middleware, is composed of client applications
  which interact with the ROS 2 world by means of an Agent.
  This agent keeps tracks of the entities created by means of
  requests performed on the microcontroller side, and uses them
  to communicate with the ROS 2 dataspace.

  Being an extension of the Micro XRCE-DDS Agent, the micro-ROS
  agent supports being run by the user like this:

      $ micro-ros-agent --help

  In addition, the Agent supports running as a service that can be
  enabled with:

      $ snap set micro-ros-agent daemon=true

  If the service is enabled, by default it uses the `udp4` transport on
  port 8888. The following parameters can be changed **(these are
  specific to the service, the `micro-ros-agent` command simply
  takes command-line arguments, but the capabilities are the same)**:

  * `transport`. Supported transports are `udp4`, `udp6`, `tcp4`,
    `tcp6`, `serial`, and `pseudoterminal`. Default is `udp4`. Change
    with:

        $ snap set micro-ros-agent transport="new transport"

  * `middleware`. Supported kinds of middleware are `ced`, `rtps`, and
    `dds`. Default is `dds`. Change with:

        $ snap set micro-ros-agent middleware="new middleware"

  * `verbosity`. Supported verbosity levels are 0-6, defaulting to 4.
    Change with:

        $ snap set micro-ros-agent verbosity="selected verbosity"

  * `discovery`. Enable or disable the discovery server. Defaults to
    "false". Change with:

        $ snap set micro-ros-agent discovery="true or false"

  * `discovery-port`. Port on which the discovery server (see above)
    listens. Defaults to 7400. Change with:

        $ snap set micro-ros-agent discovery-port="selected port"

  * `p2p-port`. Port to use for the P2P profile. Change with:

        $ snap set micro-ros-agent p2p-port="selected port"

  * `port`. Port on which the agent listens. Only applicable to one of
    the UDP or TCP transports (see above). Defaults to 8888. Change with:

        $ snap set micro-ros-agent port="selected port"

  * `baudrate`. Baud rate to use when accessing serial ports. Only
    applicable when using the `serial` or `pseudoterminal` transport.
    Defaults to 115200. Change with:

        $ snap set micro-ros-agent baudrate="baud rate"

  * `device`. The serial device to use. Only applicable when using the
    `serial` or `pseudoterminal` transport. Change with:

        $ snap set micro-ros-agent device="device path"

  When using the snap with a serial device, some steps need to be taken
  in order to establish a successful connection:

  * Refresh your local installation of the snap `core` package:

        $ sudo snap refresh core --edge

  * Enable the hotplug feature and restart the `snapd` daemon:

        $ sudo snap set core experimental.hotplug=true
        $ sudo systemctl restart snapd

  * After plugging the microcontroller to the serial port, execute
    the `snap interface serial-port` command. You should see something
    like this:

          name:    serial-port
          summary: allows accessing a specific serial port
          plugs:
            - micro-ros-agent
          slots:
          - snapd:cp2102cp2109uartbrid (allows accessing a specific serial port)

  * Connect your snap image to the desired serial port (replace accordingly):

        $ snap connect micro-ros-agent:serial-port snapd:cp2102cp2109uartbrid

  After this, you can execute your snap as usual,
  using `sudo micro-ros-agent serial -d <serial-dev>`.

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