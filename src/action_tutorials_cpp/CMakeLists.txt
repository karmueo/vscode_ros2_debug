cmake_minimum_required(VERSION 3.8)
project(action_tutorials_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# 定义并创建一个库
add_library(action_server SHARED
    src/fibonacci_action_server.cpp)
add_library(action_client SHARED
    src/fibonacci_action_client.cpp)

# 依赖头文件，其中
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # $<BUILD_INTERFACE:...>表示在编译时使用的头文件 
  $<INSTALL_INTERFACE:include>) # $<INSTALL_INTERFACE:...>表示在安装时使用的头文件
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 添加编译时定义。在编译过程中提供了一种方式来指示编译器对源代码进行条件编译。这可以用于启用或禁用代码段，或者为代码提供编译时配置信息。
# PRIVATE意味着这些定义仅用于编译指定的目标，即action_server。这些定义不会被传递给依赖于action_server的其他目标
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

# 为action_server的目标添加依赖，后面每一个参数都是一个依赖的目标
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

# 注册一个节点作为组件
# 第一个参数action_server是目标的名称
# 第二个参数PLUGIN指定了要注册的组件的完整类名
# 第三个参数EXECUTABLE指定了要注册的组件的可执行文件名，这意味着在构建项目时，将会生成一个名为fibonacci_action_server的可执行文件，它包含了action_tutorials_cpp::FibonacciActionServer节点的实现。
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)

# TARGETS关键字后面跟随的是需要安装的目标名称
# ARCHIVE DESTINATION表示静态库（.a或.lib文件）应该被安装到的目录
# LIBRARY DESTINATION表示动态库（.so、.dll或.dylib文件）应该被安装到的目录
# RUNTIME DESTINATION表示可执行文件应该被安装到的目录
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
