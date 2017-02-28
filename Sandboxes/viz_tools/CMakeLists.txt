cmake_minimum_required(VERSION 2.8.3)
project(viz_tools)

add_definitions(-O3)
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  gazebo_ros
  #motion_ctrl
  interactive_markers
)

# Depend on system install of Gazebo
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS})

add_subdirectory(marker_pub)
add_subdirectory(utils)

catkin_package(
  INCLUDE_DIRS
	 ${PROJECT_SOURCE_DIR}
  LIBRARIES
    SJutils_dreamer
    marker_pub
  DEPENDS 
    roscpp 
    tf
  	sensor_msgs
  	geometry_msgs
  	gazebo_msgs
  	gazebo_ros
    #motion_ctrl
    interactive_markers
)


MESSAGE(STATUS ${PROJECT_SOURCE_DIR})
#include_directories(${PROJECT_SOURCE_DIR}/../Motion_Ctrl)
#include_directories(${PROJECT_SOURCE_DIR}/../)
#include_directories(TaskManager)
#include_directories(include)


add_executable(gaze_markers src/gaze_markers.cpp)
target_link_libraries(gaze_markers ${catkin_LIBRARIES} marker_pub SJutils_dreamer)

add_executable(test_node_dreamer src/test_node4.cpp)
target_link_libraries(test_node_dreamer ${catkin_LIBRARIES} marker_pub SJutils_dreamer)