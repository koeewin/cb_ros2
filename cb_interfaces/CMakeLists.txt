cmake_minimum_required(VERSION 3.5)
project(cb_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(cb_interfaces
  "msg/SideButtons.msg"
  "msg/MainButtons.msg"
  "msg/Joystick.msg"
  "msg/Panel.msg"
  "msg/Flags.msg"
  "srv/EndPath.srv"
  "srv/StartPath.srv"
  "srv/RewritePath.srv"
  "srv/FollowPath.srv"
  "srv/TurnAround.srv"
  "srv/OdomOffset.srv"
  "srv/ChangeCtrlmode.srv"
  "srv/ShowLed.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
