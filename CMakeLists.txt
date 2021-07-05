# Set the minimum required version of cmake for a project.
cmake_minimum_required(VERSION 3.1)

# Set Variables
SET(CMAKE_CXX_FLAGS "-std=c++0x") # C++11
SET(PROJECT_NAME "table_tracker")

# Project
PROJECT(${PROJECT_NAME} CXX)

# Find library package
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4abt REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(~/data/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes/)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
include(${VTK_USE_FILE})

# File
FILE(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*)
FILE(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*)
#ADD_SUBDIRECTORY(extern)
#ADD_SUBDIRECTORY(helper)

# The main program
add_executable(${PROJECT_NAME} table_main.cc ${SRC_FILES} ${HEADER_FILES})

# Link
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${VTK_LIBRARIES} k4a::k4a Eigen3::Eigen)

