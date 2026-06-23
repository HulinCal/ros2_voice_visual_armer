cmake_minimum_required(VERSION 3.8)
project(whisper_cpp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
find_package(ament_cmake REQUIRED)

# 定义本地下载目录
set(DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../download")
set(WHISPER_LOCAL_DIR "${DOWNLOAD_DIR}/whisper.cpp")

# 检查本地是否已有whisper.cpp源码
if(EXISTS "${WHISPER_LOCAL_DIR}/CMakeLists.txt")
  message(STATUS "Using local whisper.cpp from: ${WHISPER_LOCAL_DIR}")
  set(FETCHCONTENT_SOURCE_DIR_WHISPER "${WHISPER_LOCAL_DIR}")
else()
  message(STATUS "whisper.cpp not found locally, will download from GitHub")
  message(STATUS "Download directory: ${WHISPER_LOCAL_DIR}")
endif()

FetchContent_Declare(
  whisper
  GIT_REPOSITORY https://github.com/ggerganov/whisper.cpp.git
  GIT_TAG        v1.8.4
)

FetchContent_MakeAvailable(whisper)

# 如果是从网络下载的，保存到本地download目录
if(NOT EXISTS "${WHISPER_LOCAL_DIR}/CMakeLists.txt" AND EXISTS "${whisper_SOURCE_DIR}/CMakeLists.txt")
  message(STATUS "Saving whisper.cpp to local directory: ${WHISPER_LOCAL_DIR}")
  file(COPY "${whisper_SOURCE_DIR}/" DESTINATION "${WHISPER_LOCAL_DIR}")
endif()

# ggml
set_target_properties(
  ggml PROPERTIES
  OUTPUT_NAME "whisper_ggml"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

set_target_properties(
  ggml-base PROPERTIES
  OUTPUT_NAME "whisper_ggml_base"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

set_target_properties(
  ggml-cpu PROPERTIES
  OUTPUT_NAME "whisper_ggml_cpu"
  INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
  CXX_STANDARD 17
)

if(GGML_CUDA)
  set_target_properties(
    ggml-cuda PROPERTIES
    OUTPUT_NAME "whisper_ggml_cuda"
    INTERFACE_INCLUDE_DIRECTORIES "$<BUILD_INTERFACE:${whisper_SOURCE_DIR}/ggml/include>"
    CXX_STANDARD 17
  )
endif()

# whisper
set_target_properties(
  whisper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES $<INSTALL_INTERFACE:.>
  CXX_STANDARD 17
)

# grammar
add_library(grammar
  ${whisper_SOURCE_DIR}/examples/grammar-parser.cpp
)

target_include_directories(grammar
  PUBLIC
    $<BUILD_INTERFACE:${whisper_SOURCE_DIR}/examples;${whisper_SOURCE_DIR}/include;${whisper_SOURCE_DIR}/ggml/include>
    $<INSTALL_INTERFACE:include>
)

# export
file(GLOB COMMON_HEADERS 
  ${whisper_SOURCE_DIR}/ggml/include/*.h
  ${whisper_SOURCE_DIR}/include/*.h
  ${whisper_SOURCE_DIR}/examples/*.h
)
install(
  FILES ${COMMON_HEADERS}
  DESTINATION include
)

set(INSTALL_TARGETS 
  ggml
  ggml-base
  ggml-cpu
  whisper
  grammar
)

if(GGML_CUDA)
  list(APPEND INSTALL_TARGETS ggml-cuda)
endif()

install(
  TARGETS ${INSTALL_TARGETS}
  EXPORT export_whisper
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_whisper HAS_LIBRARY_TARGET)
ament_package()
