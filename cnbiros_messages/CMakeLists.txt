set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_messages)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation roscpp grid_map_msgs)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} include)

add_message_files(
  FILES
)

generate_messages(
   DEPENDENCIES
   std_msgs
   grid_map_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    LIBRARIES cnbiros_messages grid_map_ros
)

add_library(cnbiros_messages SHARED src/cnbiros_msg.cpp)
add_dependencies(cnbiros_messages ${${PROJECT_NAME}_EXPORTED_TARGETS})
