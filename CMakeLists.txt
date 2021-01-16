cmake_minimum_required(VERSION "3.16")

# DepthAI Library
add_subdirectory(depthai-core EXCLUDE_FROM_ALL)

# Project declaration
project(oakd_orbslam3)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 3)

include_directories(
    ${OpenCV_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/depthai-core/include
)

# Output dirs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Target name
set(TARGET_NAME oakd_kimera)

# Add main target
add_executable(${TARGET_NAME}
    src/main.cpp
    src/util.cpp
)
target_link_libraries(${TARGET_NAME} PUBLIC
    ${OpenCV_LIBS}
    depthai-core
)