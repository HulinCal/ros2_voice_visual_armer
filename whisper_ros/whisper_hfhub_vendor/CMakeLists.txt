cmake_minimum_required(VERSION 3.8)
project(whisper_hfhub_vendor)

include(FetchContent)
find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)  # Add CURL as a required package

# 定义本地下载目录
set(DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../download")
set(HFHUB_LOCAL_DIR "${DOWNLOAD_DIR}/huggingface-hub-cpp")

# 检查本地是否已有huggingface-hub-cpp源码
if(EXISTS "${HFHUB_LOCAL_DIR}/CMakeLists.txt")
  message(STATUS "Using local huggingface-hub-cpp from: ${HFHUB_LOCAL_DIR}")
  set(FETCHCONTENT_SOURCE_DIR_HFHUB "${HFHUB_LOCAL_DIR}")
else()
  message(STATUS "huggingface-hub-cpp not found locally, will download from GitHub")
  message(STATUS "Download directory: ${HFHUB_LOCAL_DIR}")
endif()

# Declare hfhub from Git repository
FetchContent_Declare(
  hfhub
  GIT_REPOSITORY https://github.com/agonzc34/huggingface-hub-cpp
  GIT_TAG        1.1.4
  GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(hfhub)

# 如果是从网络下载的，保存到本地download目录
if(NOT EXISTS "${HFHUB_LOCAL_DIR}/CMakeLists.txt" AND EXISTS "${hfhub_SOURCE_DIR}/CMakeLists.txt")
  message(STATUS "Saving huggingface-hub-cpp to local directory: ${HFHUB_LOCAL_DIR}")
  file(COPY "${hfhub_SOURCE_DIR}/" DESTINATION "${HFHUB_LOCAL_DIR}")
endif()

# Export targets and include directories
file(GLOB COMMON_HEADERS 
  ${hfhub_SOURCE_DIR}/include/*.h
)
install(
  FILES ${COMMON_HEADERS}
  DESTINATION include
)

install(
  TARGETS hfhub
  EXPORT export_hfhub
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Export include directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(CURL)
ament_export_targets(export_hfhub HAS_LIBRARY_TARGET)
ament_package()