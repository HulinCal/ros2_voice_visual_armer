cmake_minimum_required(VERSION 3.8)
project(whisper_onnxruntime_vendor)

option(ONNX_GPU "Use GPU for inference" OFF)

set(GPU "")

# Determine the architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ARCHITECTURE "x64")
    if(ONNX_GPU)
        set(GPU "-gpu")
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ARCHITECTURE "aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Set variables for the package
set(ONNXRUNTIME_VERSION "1.20.0")  # Specify the desired ONNX Runtime version
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ARCHITECTURE}${GPU}-${ONNXRUNTIME_VERSION}.tgz")

# ROS 2 package configuration
find_package(ament_cmake REQUIRED)

# 定义本地下载目录
set(DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../download")
set(ONNXRUNTIME_LOCAL_TGZ "${DOWNLOAD_DIR}/onnxruntime-linux-${ARCHITECTURE}${GPU}-${ONNXRUNTIME_VERSION}.tgz")

# Define a vendor package installation directory
set(ONNXRUNTIME_INSTALL_DIR "${CMAKE_BINARY_DIR}/onnxruntime-linux-${ARCHITECTURE}${GPU}-${ONNXRUNTIME_VERSION}")

# Add a custom target to download and extract the prebuilt ONNX Runtime
find_program(CURL_EXECUTABLE curl REQUIRED)

if(NOT CURL_EXECUTABLE)
    message(FATAL_ERROR "curl is required to download ONNX Runtime but was not found.")
endif()

# 检查本地是否已有ONNX Runtime包
if(EXISTS "${ONNXRUNTIME_LOCAL_TGZ}")
    message(STATUS "Using local ONNX Runtime from: ${ONNXRUNTIME_LOCAL_TGZ}")
    # 使用本地的tgz文件
    add_custom_command(
        OUTPUT ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so
        COMMENT "Extracting local ONNX Runtime ${ONNXRUNTIME_VERSION}"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy "${ONNXRUNTIME_LOCAL_TGZ}" "${CMAKE_BINARY_DIR}/onnxruntime.tgz"
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_BINARY_DIR}/onnxruntime.tgz
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        VERBATIM
    )
else()
    message(STATUS "ONNX Runtime not found locally, will download from: ${ONNXRUNTIME_URL}")
    message(STATUS "Will save to: ${ONNXRUNTIME_LOCAL_TGZ}")
    # 从网络下载并保存到本地
    add_custom_command(
        OUTPUT ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so
        COMMENT "Downloading and extracting ONNX Runtime ${ONNXRUNTIME_VERSION}"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DOWNLOAD_DIR}"
        COMMAND ${CURL_EXECUTABLE} -L -o ${CMAKE_BINARY_DIR}/onnxruntime.tgz ${ONNXRUNTIME_URL}
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/onnxruntime.tgz" "${ONNXRUNTIME_LOCAL_TGZ}"
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_BINARY_DIR}/onnxruntime.tgz
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        VERBATIM
    )
endif()

# Then create a target that depends on that output
add_custom_target(download_onnxruntime ALL
    DEPENDS ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so
)

# Import the library and link it to the download target
add_library(onnxruntime SHARED IMPORTED GLOBAL)
add_dependencies(onnxruntime download_onnxruntime)
set_property(TARGET onnxruntime APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(onnxruntime PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${ONNXRUNTIME_INSTALL_DIR}/include
    IMPORTED_LOCATION_RELEASE ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so
    IMPORTED_LOCATION ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so
)

# Create an interface library that can be exported
add_library(whisper_onnxruntime INTERFACE)

# Specify the full path to the library instead of just the library name
target_link_libraries(whisper_onnxruntime INTERFACE 
    ${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so  # Use full path instead of "onnxruntime"
)

target_include_directories(whisper_onnxruntime INTERFACE 
    $<BUILD_INTERFACE:${ONNXRUNTIME_INSTALL_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Create a library directory in the install space
install(DIRECTORY
    ${ONNXRUNTIME_INSTALL_DIR}/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*"
)

# Install includes
install(DIRECTORY
    ${ONNXRUNTIME_INSTALL_DIR}/include/
    DESTINATION include
)

# Install and export the interface target
install(TARGETS whisper_onnxruntime
        EXPORT export_whisper_onnxruntime_vendor)

# Export the targets
ament_export_targets(export_whisper_onnxruntime_vendor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()