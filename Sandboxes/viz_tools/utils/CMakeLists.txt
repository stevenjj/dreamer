FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

include_directories (${catkin_INCLUDE_DIRS}) #"${PROJECT_SOURCE_DIR}/../../comm_comm/include" )

add_library(SJutils_dreamer SHARED ${sources} ${headers} ${cppheaders})
target_link_libraries(SJutils_dreamer pthread)
#add_dependencies(SJutils comm_comm)