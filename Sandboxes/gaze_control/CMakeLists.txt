cmake_minimum_required(VERSION 2.8.3)
project(gaze_control)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
 	message_generation
 	cmake_modules
 	roscpp
 	rospy
# 	sensor_msgs
 )

find_package(TinyXML REQUIRED)




## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   	sensor_msg.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  HeadJointCmd.srv
  RunProgram.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  LIBRARIES   gaze_control
#  INCLUDE_DIRS include
  
  DEPENDS 
  	TinyXML
#  	sensor_msg
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(  ${EIGEN3_INCLUDE_DIR}  )

## Declare a C++ library
# add_library(gaze_control
#   src/${PROJECT_NAME}/gaze_control.cpp
# )

include_directories(include ${catkin_INCLUDE_DIRS})

set(${DREAMER_CONTROLLER}_SOURCES
	src/gaze_control_cpp/dreamerHighLevelController.cpp
	src/gaze_control_cpp/dreamerController.cpp 
	src/gaze_control_cpp/headKinematics.cpp 
	src/gaze_control_cpp/modernRobotics.cpp 
	src/gaze_control_cpp/utilQuat.cpp
	src/gaze_control_cpp/dreamerJointPublisher.cpp
	src/gaze_control_cpp/Waypoint.cpp
	)


add_executable(dreamerHighLevelController ${${DREAMER_CONTROLLER}_SOURCES})
# add_executable(UART_com src/gaze_control_cpp/UART_com.cpp)


target_link_libraries(dreamerHighLevelController ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

