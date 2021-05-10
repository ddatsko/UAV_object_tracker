cmake_minimum_required(VERSION 3.0.2)
project(yolo_tracker)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  mrs_lib
  mrs_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES yolo_tracker
  CATKIN_DEPENDS mrs_lib mrs_msgs roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Set all the warnings if compiler is G++ or Clang
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Set build type to release if no other was specified in command line arguments
if (NOT CMAKE_BUILD_TYPE)
	message(DEBUG "No Cmake build type specified. Using Release as default")
	set(CMAKE_BUILD_TYPE Release)
endif()


# Main executable
add_executable(tracker src/main.cpp src/ObjectDetector.cpp src/UAVController.cpp src/utils.cpp)

# Find OpenCV and link it
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(tracker ${OpenCV_LIBS})

# Find and link cv_bridge
find_library(cv_bridge REQUIRED)
target_link_libraries(tracker cv_bridge)


add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(tracker
   ${catkin_LIBRARIES}
 )
