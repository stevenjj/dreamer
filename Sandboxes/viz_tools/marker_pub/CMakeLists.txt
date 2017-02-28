FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

include_directories(${PROJECT_SOURCE_DIR}/)

add_library(marker_pub SHARED ${sources} ${headers} ${cppheaders})
target_link_libraries(marker_pub SJutils_dreamer ${catkin_LIBRARIES} )