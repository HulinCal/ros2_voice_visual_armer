cmake_minimum_required(VERSION 3.8)
project(llama_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(llama_ros_msgs REQUIRED)
find_package(arm_controller REQUIRED)

find_library(LLAMA_LIB llama REQUIRED)
find_library(MTMD_LIB mtmd REQUIRED)

include_directories(
  /usr/local/include
)

add_executable(llama_action_server
  src/llama_action_server.cpp
)

ament_target_dependencies(llama_action_server
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  cv_bridge
  llama_ros_msgs
  arm_controller
)

target_link_libraries(llama_action_server
  ${LLAMA_LIB}
  ${MTMD_LIB}
)

install(TARGETS
  llama_action_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
