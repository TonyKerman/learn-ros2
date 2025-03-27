DESKTOP_FILE="$HOME/Desktop/close_terminals.desktop"
TARGET_SCRIPT="/home/wtr2023/ros_ws/rc2024/RC24_R2V2/rc24_r2_v2_ros2/launch/close_terminals.sh"

# 创建 .desktop 文件
echo "[Desktop Entry]
Version=1.0
Name=Close Terminals Script # 应用程序名
Exec=$TARGET_SCRIPT
Icon=utilities-terminal
Terminal=true # 是否使用终端
Type=Application
Categories=Utility;Application;" > $DESKTOP_FILE

# 设置执行权限
chmod +x $DESKTOP_FILE

echo "Desktop shortcut created at $DESKTOP_FILE"