cmake_minimum_required(VERSION 3.8)
project(action_tutorials_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# rosidl_default_generators是ROS2中的一个包，提供了一套默认的接口定义语言（IDL）生成器，这些生成器能够将ROS2接口文件（如.msg、.srv和.action文件）转换成不同编程语言的源代码。
find_package(rosidl_default_generators REQUIRED)

# 使用rosidl_default_generators包为指定的ROS2接口文件生成源代码。这个命令接受两个参数：第一个参数${PROJECT_NAME}代表当前项目的名称；第二个参数是一个字符串列表，列出了需要生成代码的ROS2接口文件的相对路径。
# CMake将为这个.action文件生成相应的C++源代码，这些代码将包含定义Fibonacci动作所需的目标、反馈和结果的消息类型。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

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
