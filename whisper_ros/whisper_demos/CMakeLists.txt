cmake_minimum_required(VERSION 3.8)
project(whisper_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(llama_ros_msgs REQUIRED)
find_package(paddle_speech_msgs REQUIRED)

include_directories(
  include
)

# whisper_demo_nonstop_node
add_executable(whisper_demo_nonstop_node
  src/whisper_demo_nonstop_node.cpp
)
ament_target_dependencies(whisper_demo_nonstop_node
  rclcpp
  rclcpp_action
  std_msgs
  whisper_msgs
  llama_ros_msgs
  paddle_speech_msgs
)

# Install executables
install(TARGETS
  whisper_demo_nonstop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python script
install(PROGRAMS
  whisper_demos/whisper_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME whisper_demo_node
)

ament_package()