cmake_minimum_required(VERSION "3.16")

# DepthAI Library
add_subdirectory(depthai-core EXCLUDE_FROM_ALL)

# Project declaration
project(oakd_orbslam3)
set(CMAKE_BUILD_TYPE Debug)

# Make sure that custom modules like FindXXXX are found
# Prefer to use system installation of gflags/glog
set(GFLAGS_PREFER_EXPORTED_GFLAGS_CMAKE_CONFIGURATION TRUE)
set(GLOG_PREFER_EXPORTED_GLOG_CMAKE_CONFIGURATION TRUE)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-z,now")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog 0.3.5 REQUIRED)
find_package(GTSAM 4.0.2 REQUIRED)
find_package(opengv REQUIRED)
find_package(DBoW2 REQUIRED)
if(NOT TARGET DBoW2::DBoW2)
add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
set_target_properties(DBoW2::DBoW2 PROPERTIES
INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()
find_package(KimeraRPGO REQUIRED)
find_package(kimera_vio REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/depthai-core/include
    ${EIGEN3_INCLUDE_DIRS}
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
    kimera_vio
)