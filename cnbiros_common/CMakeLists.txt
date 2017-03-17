set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_common)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation cnbiros_core cnbiros_messages cnbiros_services grid_map_ros)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${cnbiros_messages_INCLUDE_DIRS} 
		    ${Eigen3_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
	${Eigen3_INCLUDE_DIRS} 
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  	cnbiros_messages
	cnbiros_services
	cnbiros_core
  DEPENDS
  	Eigen3
)

add_library(cnbiros_common 
			src/KinectScan.cpp)
target_link_libraries(cnbiros_common ${catkin_LIBRARIES} ${Eigen3_LIBRARY})

# Kinectscan node
add_executable(kinectscan_node_base bin/kinectscan_node_base.cpp)
target_link_libraries(kinectscan_node_base cnbiros_common ${Eigen3_LIBRARY})
