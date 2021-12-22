cmake_minimum_required(VERSION 3.5)
project(robotarm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/${PROJECT_NAME} include/${PROJECT_NAME}/app include/${PROJECT_NAME}/demo include/${PROJECT_NAME}/dll)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Setup custom messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Position.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# Low level driver
add_library(dll SHARED src/dll/Serial.cpp src/dll/SSC32.cpp src/ServoCommand.cpp)
ament_export_targets(dll HAS_LIBRARY_TARGET)
ament_export_dependencies(dll rclcpp std_msgs Boost)
install(TARGETS dll
        EXPORT dll
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

# Robot application/high level driver
find_package(ament_lint_auto REQUIRED)
add_executable(robot src/app/robot.cpp src/app/Robotarm.cpp src/app/Logger.cpp src/ServoCommand.cpp)
ament_target_dependencies(robot rclcpp)
ament_lint_auto_find_test_dependencies()
target_link_libraries(robot dll)

# Demo application
find_package(ament_lint_auto REQUIRED)
add_executable(demo src/demo/demonstrator.cpp src/demo/Demo.cpp)
ament_target_dependencies(demo rclcpp std_msgs)
ament_lint_auto_find_test_dependencies()

# Make robot and demo available for ROS2
install(TARGETS
        robot
        demo
        DESTINATION lib/${PROJECT_NAME})

# Register custom ROS2 messages
rosidl_target_interfaces(demo
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(robot
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
