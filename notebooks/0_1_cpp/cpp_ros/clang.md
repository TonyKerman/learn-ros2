# vscode clang
在vscode中使用clang插件的代码补全，有两种方法。
## 修改CMakeLists.txt
添加一行
```
set(DCMAKE_EXPORT_COMPILE_COMMANDS ON)
```
首选方法，但是有时不好使。
## 编译时添加参数
```
colcon build --amentcmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

在`build`文件夹下看见`compile_commands.json`就是成功了。