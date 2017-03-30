set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_common)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation cnbiros_core cnbiros_messages cnbiros_services grid_map_ros)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_messages_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  	cnbiros_messages
	cnbiros_services
	cnbiros_core
)

