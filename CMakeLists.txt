cmake_minimum_required(VERSION 3.8)
project(assignment2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetAvgVel.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(user_drive src/user_drive.cpp)
ament_target_dependencies(user_drive rclcpp geometry_msgs)

add_executable(safety src/safety.cpp)
ament_target_dependencies(safety rclcpp geometry_msgs sensor_msgs nav_msgs)

rosidl_target_interfaces(user_drive ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(safety ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  user_drive
  safety
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
