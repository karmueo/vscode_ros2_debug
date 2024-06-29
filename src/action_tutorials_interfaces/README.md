# Vscode调试

## 目录

- [教程](#教程)

## 教程 <a name = "教程"></a>

### 1. 创建包
```bash
ros2 pkg create action_tutorials_interfaces
```

### 2. 创建一个action文件夹：

### 3. 在action文件夹中创建一个名为 Fibonacci.action 的文件，其中包含以下内容 

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

目标请求是我们要计算的Fibonacci数列的阶数`order`，结果是最终的数列`sequence`，反馈是目前为止计算出的部分数列`partial_sequence`。

### 4. build

在 CMakeLists.txt 中 `ament_package()` 行之前添加以下几行:

```sh
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

将所需的依赖项添加到 `package.xml` 中：

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

执行build
```bash
# 
cd /workspaces/vscode_ros2_workspace
# Build
colcon build
```

### 5. 检查是否成功
按约定，action类型将以其包名称和单词`action`作为前缀。因此，当想要引用的新action时，它将具有全名`action_tutorials_interfaces/action/Fibonacci`。

可以使用命令行工具检查的action是否成功构建：

```bash
# Source setup.bash
# 如果是Windows系统: call install/setup.bat
. install/setup.bash
# 测试action定义是否存在
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```