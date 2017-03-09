set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_services)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation roscpp)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} include)

add_service_files(
  FILES
  RosInterfaceState.srv
  GridMapReset.srv
  OdometryReset.srv
  NavParameterSet.srv
  SetBaseVelocity.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    LIBRARIES cnbiros_services
)

add_library(cnbiros_services SHARED src/cnbiros_services.cpp)
add_dependencies(cnbiros_services ${${PROJECT_NAME}_EXPORTED_TARGETS})
