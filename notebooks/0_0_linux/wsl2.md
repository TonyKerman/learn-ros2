# WSL2
[链接：WSL2文档](https://learn.microsoft.com/zh-cn/windows/wsl/)
## 基本命令
### 安装发行版
在Microsoft Store搜素，如“Ubuntu22.04”
### 安装
```bash
wsl --install
```
### 更新（解决一些奇怪的问题）
```bash
wsl --update
```
### 列出现有发行版
```bash
wsl --list
```
### 重启WSL2
```bash
wsl --shutdown
wsl
```
### 终止指定的发行版
```bash
wsl --terminate <Distribution Name>
```
### 设置
开始菜单搜索“WSL Setting”
## 网络
[网络注意事项](https://learn.microsoft.com/zh-cn/windows/wsl/networking)
[镜像网络模式](https://learn.microsoft.com/zh-cn/windows/wsl/networking#mirrored-mode-networking)
使用网络应用和 WSL 时需要了解一些注意事项。 默认情况下，WSL 使用基于 NAT 的体系结构。建议尝试新的镜像网络模式以获得最新的功能和改进。
### 为什么使用径向网络模式？
```
以下是启用此模式的当前优势：
IPv6 支持
使用 localhost 地址 127.0.0.1 从 Linux 内部连接到 Windows 服务器。 不支持 IPv6 localhost 地址 ::1
改进了 VPN 的网络兼容性
多播支持
直接从局域网 (LAN) 连接到 WSL
```
具体的来说，如果不使用镜像网络模式，其他机器要访问一台机器上的wsl发行版比较麻烦。
但是镜像网络模式开启后，其他机器可以通过<IP>:<端口>的方式访问wsl。**仍然需要Docker的端口映射**
### 开启wsl
* “WSL Setting” 中网络->mirrored networking
* 使用管理员权限在 PowerShell 窗口中运行以下命令，以配置 Hyper-V 防火墙设置，从而允许入站连接：
```bash
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```
* 重启WSL2

