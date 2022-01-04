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
FIND_PACKAGE(k4arecord REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
#find_package(Python2 COMPONENTS Development NumPy)
add_subdirectory(libjpeg-turbo)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(~/data/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes/)
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
include(${VTK_USE_FILE})
#include_directories(/home/hurel/data/matplotlib-cpp)



# File
FILE(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*)
FILE(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*)
#ADD_SUBDIRECTORY(extern)
#ADD_SUBDIRECTORY(helper)

# The main program
add_executable(${PROJECT_NAME} table_main.cc ${SRC_FILES} ${HEADER_FILES})

# target_include_directories(${PROJECT_NAME} PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})

# Link
target_link_libraries( ${PROJECT_NAME} 
	${OpenCV_LIBS} 
	${VTK_LIBRARIES}
       	k4a::k4a
	k4a::k4arecord
       	Eigen3::Eigen
	libjpeg-turbo::libjpeg-turbo
)
#	Python2::Python 
#	Python2::NumPy)

