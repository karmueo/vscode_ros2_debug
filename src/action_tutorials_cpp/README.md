# Vscode调试

## 目录

- [教程](#教程)

## 教程 <a name = "教程"></a>

### 1. 前置条件
- 详见`action_tutorials_interfaces`包，创建action中定义的 `Fibonacci.action` 接口。

### 2. 添加可见性控制
- 为了使该包能够在 Windows 上编译并运行，需要添加一些“可见性控制”。详见`include/action_tutorials_cpp/visibility_control.h,`

### 3. 编译服务端
- 打开 CMakeLists.txt，并在 find_package 调用之后添加以下内容：
  ```cmake
  add_library(action_server SHARED
    src/fibonacci_action_server.cpp)
  target_include_directories(action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_server
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_server
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
  install(TARGETS
    action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  ```

### 4. 编译客户端
- 打开CMakeLists.txt，并在 find_package 调用之后添加以下内容：

  ```cmake
  add_library(action_client SHARED
    src/fibonacci_action_client.cpp)
  target_include_directories(action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_client
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_client
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
  install(TARGETS
    action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  ```
